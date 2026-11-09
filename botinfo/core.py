"""The BotInfo engine: every agent request goes through here, and so does every stop.

Order of checks for an action call (each refusal is logged with the rule that made it):
  1. botinfo_stop      -> stop now. Never refused, never rate-limited, never queued.
  2. overall rate    -> [rate_limits].all, counted on every call except botinfo_stop
  3. known action    -> only actions in the policy exist; anything else is "unknown_action"
  4. meta fields     -> request_id (replay), issued_at (stale / from the future)
  5. parameters      -> exact fields, types, bounds (refused, never clamped)
  6. stop latch      -> after a stop, motion stays refused until the OWNER resumes
  7. motion rate     -> [rate_limits].motion and the action's own rate
  8. confirmation    -> the owner confirms on the local channel; the request expires after command_ttl_s
  9. execution       -> continuous-motion limit (renewals and back-to-back commands count together), workspace
                        check, then start the motion on a lease (keep-alives, stop at the end)

Threads: the MCP server calls call_tool() from its reader thread, tick() from a timer thread, and the owner
channel from a third. Two locks: _state (fast, never held during robot I/O) and _io (serialises robot I/O so a
stop is always the LAST thing sent, after any keep-alive already on the wire). Lock order: _io, then _state.
A stop sets the latch under _state first, so no new motion can start while it waits for _io.

Stop receipts: a stop that was sent is not a robot at rest. When the binding says which status fields read the
robot's speed ([settle]), tick() reads the robot's status after every stop and every lease end until those speeds
stay under a threshold, then logs one "stop_receipt" audit event: when the robot came to rest, or that it did not
within the timeout, or that its status could not be read. Nothing is read on the stop path itself.
A receipt is only as good as those fields: odometry that echoes the commanded speed (open-loop) or a status that
stopped updating can read zero while the body still moves, so "at_rest" means the robot REPORTED rest.

Last motion: every stop that cuts a lease, every lease end and every replace of a leased motion records which
motion BotInfo ended (or tried to end), its lease time (ran_s: from accepting the call to sending the stop, of of_s
planned; BotInfo's clock, not the robot's motion time) and what ended it, in last_motion (botinfo_status and
botinfo_wait). The "stop" and "lease_end" audit lines carry ran_s and of_s; a replace whose stop was sent writes a
"replaced" line with them, and how the new motion's start went is in last_motion and its "decision" line. A stop
that cuts a lease also returns the record as stopped_motion. A stop that did not reach the robot (the motion may
still be running), or a replacing motion that was never sent, failed to start or was cut by a stop as it started,
says so in "ended". One-shot actions have no lease and are not recorded. An agent told "stopped" then reads the
time BotInfo logged instead of subtracting two timestamps.

In the benchmark everything runs on one thread with a VirtualClock.
"""
from __future__ import annotations

import itertools
import math
import threading
import time
from collections import OrderedDict, defaultdict, deque

from .audit import AuditLog
from .backends.base import BackendError, refusal, sanitize
from .clock import RealClock
from .policy import PolicyError, check_params, needs_confirmation, split_meta

MAX_SEEN_IDS = 10000
MAX_READ_ERRORS = 3                         # a stop receipt gives up after this many failed status reads in a row
STOP_NOTE = ("This is a software stop sent over a network link. If the robot did not stop, use its local "
             "stop (power switch or hardware e-stop).")


class BotInfo:
    def __init__(self, policy, backend, clock=None, audit: AuditLog | None = None, on_pending=None,
                 strict_binding: bool = True, sleep=None):
        problems = [p for p in backend.check_policy(policy) if not p.startswith("warning:")]
        if problems and strict_binding:
            raise PolicyError("policy and backend do not fit:\n  - " + "\n  - ".join(problems))
        self.policy, self.backend = policy, backend
        self.clock = clock or RealClock()
        self.audit = audit or AuditLog(clock=self.clock)
        self.on_pending = on_pending
        # botinfo_wait sleeps with this; a virtual clock has no real time to wait in unless the harness passes one
        self.sleep = sleep or (None if getattr(self.clock, "virtual", False) else time.sleep)
        self._state = threading.RLock()
        self._io = threading.RLock()
        self.latched = False
        self.latch_reason = ""
        self.lease: dict | None = None
        self.busy_until = -math.inf
        self.busy_action = ""
        self.pending: dict = {}
        self.seen_ids: OrderedDict = OrderedDict()
        self.calls_all: deque = deque()
        self.calls_motion: deque = deque()
        self.calls_action: dict = defaultdict(deque)
        self.stop_gen = 0
        self.chain_start = None                 # when the current stretch of motion began
        self.last_motion_end = -math.inf
        self.next_ws_check = -math.inf
        self.connected = True
        self._ids = itertools.count(1)
        settle = getattr(backend, "settle_spec", None)
        self.settle = settle() if callable(settle) else None
        self.watch: dict | None = None          # the stop receipt being measured
        self.last_stop: dict | None = None
        self.last_motion: dict | None = None    # the latest leased motion BotInfo ended: which lease, lease time, why
        self._stop_ids = itertools.count(1)
        self.audit.write("start", policy=policy.name, policy_file=policy.source, backend=backend.describe(),
                         actions=sorted(policy.actions), stop_receipts=self.settle is not None)

    # ================================================================== agent-facing tools
    def list_tools(self) -> list:
        tools = []
        for a in self.policy.actions.values():
            props = {p.name: p.json_schema() for p in a.params.values()}
            props["request_id"] = {"type": "string", "description": "Optional. A unique id; a repeated id is "
                                                                    "refused as a replay."}
            props["issued_at"] = {"type": "number", "description": "Optional. Unix time the command was issued; "
                                  f"older than {self.policy.command_ttl_s:g} s is refused as stale."}
            required = [p.name for p in a.params.values() if not p.has_default]
            desc = a.description or a.name
            if a.kind == "motion":
                desc += (f" Runs on a lease of at most {self.policy.max_lease_s:g} s, then stops by itself."
                         + (" Renew with botinfo_renew." if self.policy.renewable else "")
                         + " The call returns at once, while the robot moves; runs_for_s says for how long.")
            if a.kind in ("motion", "oneshot"):
                desc += (" A new motion command stops the running one early and replaces it (the answer says how "
                         "far it got); call botinfo_wait first if the running one should finish."
                         if self.policy.on_busy == "replace" else
                         " A new motion command is refused while one is running.")
            if a.confirm or any(p.confirm_above is not None or p.confirm_values for p in a.params.values()):
                desc += (" Some calls need the owner's confirmation; those return pending_confirmation. A newer "
                         "motion command cancels a request still waiting for it.")
            if a.uninterruptible:
                desc += " Cannot be interrupted by a stop once it starts."
            tools.append({"name": a.name, "description": desc,
                          "inputSchema": {"type": "object", "properties": props, "required": required,
                                          "additionalProperties": False},
                          "annotations": {"readOnlyHint": False, "destructiveHint": a.kind != "instant"}})
        tools.append({"name": "botinfo_stop", "description": "Stop the robot now. Always allowed. Afterwards motion "
                      "stays blocked until the owner resumes.",
                      "inputSchema": {"type": "object", "properties": {"reason": {"type": "string"}}},
                      "annotations": {"readOnlyHint": False, "destructiveHint": False}})
        tools.append({"name": "botinfo_status", "description": "BotInfo state (stopped?, running lease, pending "
                      "confirmations, last_stop, last_motion) and the robot's status fields. last_stop.rest says "
                      "whether the robot's own speed readings fell to rest after the latest stop, when its speed can "
                      "be read. last_motion is the leased motion BotInfo ended (or tried to end) last: its lease "
                      "time (ran_s of of_s seconds, from accepting the call to sending the stop, by BotInfo's clock) "
                      "and what ended it. If its 'ended' says the stop was not delivered, the motion may still be "
                      "running. One-shot actions are not recorded there.",
                      "inputSchema": {"type": "object", "properties": {}}, "annotations": {"readOnlyHint": True}})
        if any(a.kind in ("motion", "oneshot") for a in self.policy.actions.values()):
            tools.append({"name": "botinfo_wait", "description": "Wait until the running motion ends, or at most max_s "
                          f"seconds (default {self.policy.max_lease_s:g}), then return the same state as botinfo_status. "
                          "It returns once BotInfo has sent the stop at the end of the motion (the robot may still be "
                          "slowing down, and last_stop.rest may still say watching). When a leased motion ended, "
                          "last_motion gives its lease time (ran_s of of_s, by BotInfo's clock) and what ended it. "
                          "Use it between moves so each one "
                          "finishes. A stop (yours or the owner's) ends the wait at once, and botinfo_stop is never "
                          "delayed by it.",
                          "inputSchema": {"type": "object", "properties": {"max_s": {
                              "type": "number", "minimum": 0, "maximum": self.policy.max_lease_s}},
                              "additionalProperties": False},
                          "annotations": {"readOnlyHint": True}})
        tools.append({"name": "botinfo_policy", "description": "What this robot's owner allows: actions, bounds, limits.",
                      "inputSchema": {"type": "object", "properties": {}}, "annotations": {"readOnlyHint": True}})
        if self.policy.renewable:
            tools.append({"name": "botinfo_renew", "description": "Extend the running motion lease (heartbeat). "
                          f"At most {self.policy.max_lease_s:g} s from now and {self.policy.max_total_s:g} s in total.",
                          "inputSchema": {"type": "object", "properties": {
                              "lease_id": {"type": "string"},
                              "extend_s": {"type": "number", "minimum": 0.1, "maximum": self.policy.max_lease_s}},
                              "required": ["lease_id", "extend_s"], "additionalProperties": False},
                          "annotations": {"readOnlyHint": False}})
        return tools

    def call_tool(self, name, args=None) -> dict:
        args = {} if args is None else args
        self.audit.write("request", tool=name, args=args)
        if name == "botinfo_stop":
            reason = args.get("reason", "") if isinstance(args, dict) else ""
            return self.stop(by="agent", reason=str(reason)[:200])
        now = self.clock.now()
        with self._state:
            if not self._take(self.calls_all, self.policy.rate_all, now):
                return self._deny(name, "rate_limit_all", f"more than {self.policy.rate_all.max} calls in "
                                  f"{self.policy.rate_all.window_s:g} s")
        if name == "botinfo_status":
            return self._status_result()
        if name == "botinfo_wait":
            return self._wait(args)
        if name == "botinfo_policy":
            return self._ok(name, "policy", policy=self.policy.summary())
        if name == "botinfo_renew" and self.policy.renewable:
            return self._renew(args)
        action = self.policy.actions.get(name) if isinstance(name, str) else None
        if action is None:
            return self._deny(name, "unknown_action", f"{str(name)[:80]!r} is not an action this robot's owner "
                              f"allows; allowed: {sorted(self.policy.actions)}")
        args, meta, why = split_meta(args)
        if why:
            return self._deny(name, "bad_meta", why)
        why = self._check_meta(meta, now)
        if why:
            return self._deny(name, why[0], why[1])
        params, why = check_params(action, args)
        if why:
            return self._deny(name, "bad_params", why)
        with self._state:
            if self.latched:
                return self._deny(name, "stopped", f"the robot was stopped ({self.latch_reason}); only the owner "
                                  "can resume")
            if action.kind in ("motion", "oneshot"):
                if not self._take(self.calls_motion, self.policy.rate_motion, now):
                    return self._deny(name, "rate_limit_motion", f"more than {self.policy.rate_motion.max} motion "
                                      f"commands in {self.policy.rate_motion.window_s:g} s")
            if action.rate and not self._take(self.calls_action[name], action.rate, now):
                return self._deny(name, "rate_limit_action", f"more than {action.rate.max} {name} calls in "
                                  f"{action.rate.window_s:g} s")
            gone = []
            if action.kind in ("motion", "oneshot"):     # a newer motion makes an older request obsolete
                gone = sorted(c for c, p in self.pending.items()
                              if self.policy.actions[p["action"]].kind in ("motion", "oneshot"))
                for c in gone:
                    del self.pending[c]
                if gone:
                    self.audit.write("pending_superseded", confirmation_ids=gone, by=name)
            why_confirm = needs_confirmation(action, params)
            if why_confirm:
                cid = f"c-{next(self._ids)}"
                p = {"id": cid, "action": name, "params": params, "created": now,
                     "expires": now + self.policy.command_ttl_s, "why": why_confirm}
                self.pending[cid] = p
        if why_confirm:
            self.audit.write("decision", tool=name, decision="pending_confirmation", rule="confirm",
                             reason=why_confirm, confirmation_id=cid, params=params)
            if self.on_pending:
                try:
                    self.on_pending(dict(p))
                except Exception:  # a notification problem must not break the request path
                    pass
            out = {"ok": True, "decision": "pending_confirmation", "confirmation_id": cid, "reason": why_confirm,
                   "expires_in_s": self.policy.command_ttl_s,
                   "note": "The owner must confirm on the robot's local channel. The agent cannot confirm."}
        else:
            out = self._execute(action, params, origin="agent")
        if gone:
            out["cancelled_requests"] = gone
        return out

    # ================================================================== owner channel
    def owner_stop(self, reason: str = "owner stop") -> dict:
        return self.stop(by="owner", reason=reason)

    def owner_resume(self) -> dict:
        with self._state:
            was = self.latched
            self.latched, self.latch_reason = False, ""
        self.audit.write("owner", command="resume", was_stopped=was)
        return {"ok": True, "resumed": was}

    def owner_confirm(self, cid: str) -> dict:
        now = self.clock.now()
        with self._state:
            p = self.pending.pop(cid, None)
        if p is None:
            self.audit.write("owner", command="confirm", confirmation_id=cid, result="unknown or already handled")
            return {"ok": False, "reason": f"no pending confirmation {cid!r}"}
        if now > p["expires"]:
            self.audit.write("owner", command="confirm", confirmation_id=cid, result="expired",
                             late_by_s=round(now - p["expires"], 3))
            return {"ok": False, "reason": f"{cid} expired {now - p['expires']:.1f} s ago; ask the agent to send it again"}
        self.audit.write("owner", command="confirm", confirmation_id=cid, result="confirmed", action=p["action"])
        return self._execute(self.policy.actions[p["action"]], p["params"], origin=f"owner-confirmed {cid}")

    def owner_deny(self, cid: str) -> dict:
        with self._state:
            p = self.pending.pop(cid, None)
        self.audit.write("owner", command="deny", confirmation_id=cid, found=p is not None)
        return {"ok": p is not None}

    def owner_status(self) -> dict:
        return {"ok": True, **self._state_view(include_pending_params=True)}

    def client_disconnected(self):
        with self._state:
            self.connected = False
        self.audit.write("client_disconnect", action=self.policy.on_client_disconnect)
        if self.policy.on_client_disconnect == "stop":
            self.stop(by="botinfo", reason="the agent disconnected")

    # ================================================================== stop
    def stop(self, by: str = "agent", reason: str = "") -> dict:
        with self._state:
            self.stop_gen += 1
            if self.policy.stop_latch:
                self.latched = True
                self.latch_reason = f"stop by {by}" + (f": {reason}" if reason else "")
            lease, self.lease = self.lease, None
            motion = None
            if lease is not None:
                now = self.clock.now()
                self.last_motion_end = now
                motion = self.last_motion = _motion(lease, now, f"stop by {by}")
            cancelled = sorted(self.pending)
            self.pending.clear()
        with self._io:
            try:
                self._send_stop()
                delivered, err = True, None
            except BackendError as e:
                delivered, err = False, str(e)
        sid = self._open_receipt(by)
        if motion is not None:
            with self._state:
                motion["stop_id"] = sid
                if not delivered:
                    motion["ended"] += ", stop not delivered"
        self.audit.write("stop", by=by, reason=reason, delivered=delivered, error=err, latched=self.latched,
                         cancelled_lease=lease["id"] if lease else None, cancelled_pending=cancelled, stop_id=sid,
                         **({"ran_s": motion["ran_s"], "of_s": motion["of_s"]} if motion else {}))
        out = {"ok": delivered, "decision": "stopped", "delivered": delivered, "latched": self.latched,
               "stop_id": sid, "note": STOP_NOTE}
        if motion is not None:
            out["stopped_motion"] = dict(motion)
        if err:
            out["error"] = err
        return out

    def _send_stop(self):
        """backend.stop(), with a refusal the backend RETURNS turned into the BackendError it means. A connector that
        answers a stop with {"ok": false} or {"status": "error"} did not stop the robot, so the stop is logged as not
        delivered and, at a lease end or a replace, reaches _backend_failed and the latch."""
        why = refusal(self.backend.stop())
        if why:
            raise BackendError(f"stop refused: {why}")

    # ================================================================== stop receipts
    def _open_receipt(self, by: str) -> str:
        """Number this stop and, when the binding says how to read the robot's speed, start watching for the robot
        to come to rest. tick() does the reading, so the stop itself is never slowed by it. A receipt still open
        from an earlier stop is closed as superseded."""
        sid = f"s-{next(self._stop_ids)}"
        now = self.clock.now()
        with self._state:
            old, self.watch = self.watch, None
            if self.settle is not None:
                self.watch = {"id": sid, "by": by, "t0": now, "next": now, "reads": 0, "errors": 0,
                              "errors_in_row": 0, "first": None, "last": None, "still_since": None, "peak": {}}
            self.last_stop = {"id": sid, "by": by, "rest": "watching" if self.watch else "not_measured"}
        if old is not None:
            self._close_receipt(old, "superseded", now, note=f"a newer stop ({sid}) was sent")
        return sid

    def _end_receipt(self, note: str):
        with self._state:
            w, self.watch = self.watch, None
        if w is not None:
            self._close_receipt(w, "superseded", self.clock.now(), note=note)

    def _watch_tick(self):
        """One status read for the open stop receipt; tick() calls it with _io held. The robot counts as at rest
        once every [settle] field has read under `below` for hold_s. The receipt closes then, after timeout_s, or
        after MAX_READ_ERRORS failed reads in a row. A failed read here never triggers on_backend_error."""
        spec = self.settle
        now = self.clock.now()
        with self._state:
            w = self.watch
            if w is None or now < w["next"] - 1e-9:
                return
            w["next"] = now + spec["every_s"]
        try:
            raw, err = self.backend.status(), None
        except BackendError as e:
            raw, err = {}, str(e)
        clean, red = sanitize(raw, self.backend.status_schema())
        for r in red:
            self.audit.write("output_redacted", **r)
        speeds = [clean.get(f) for f in spec["fields"]]
        good = err is None and all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in speeds)
        outcome = None
        with self._state:
            if self.watch is not w:                 # a newer stop or a new motion took over during the read
                return
            w["reads"] += 1
            if good:
                w["errors_in_row"] = 0
                snap = {"after_stop_s": round(now - w["t0"], 3), **clean}
                if w["first"] is None:
                    w["first"] = snap
                w["last"] = snap
                for f, v in zip(spec["fields"], speeds):
                    w["peak"][f] = max(w["peak"].get(f, 0.0), abs(v))
                if all(abs(v) < spec["below"] for v in speeds):
                    if w["still_since"] is None:
                        w["still_since"] = now
                    if now - w["still_since"] >= spec["hold_s"] - 1e-9:
                        outcome = "at_rest"
                else:
                    w["still_since"] = None
            else:
                w["errors"] += 1
                w["errors_in_row"] += 1
                w["still_since"] = None
                w["last_error"] = err or "a speed field was missing or not a number"
                if w["errors_in_row"] >= MAX_READ_ERRORS:
                    outcome = "unreadable"
            if outcome is None and now - w["t0"] >= spec["timeout_s"] - 1e-9:
                outcome = "not_at_rest" if w["first"] is not None else "unreadable"
            if outcome:
                self.watch = None
        if outcome:
            self._close_receipt(w, outcome, now)

    def _close_receipt(self, w: dict, outcome: str, now: float, note: str = ""):
        rec = {"stop_id": w["id"], "by": w["by"], "outcome": outcome, "watched_s": round(now - w["t0"], 3),
               "reads": w["reads"], "read_errors": w["errors"],
               "peak_after_stop": {f: round(v, 4) for f, v in w["peak"].items()},
               "first_read": w["first"], "last_read": w["last"]}
        if outcome == "at_rest":
            rec["at_rest_after_s"] = round(w["still_since"] - w["t0"], 3)
        rec.update(_moved(w["first"], w["last"], self.settle.get("pose")))
        if w.get("last_error"):
            rec["last_error"] = w["last_error"][:200]
        if note:
            rec["note"] = note
        with self._state:
            if self.last_stop is not None and self.last_stop["id"] == w["id"]:
                self.last_stop = {"id": w["id"], "by": w["by"], "rest": outcome,
                                  **({"at_rest_after_s": rec["at_rest_after_s"]} if outcome == "at_rest" else {})}
        self.audit.write("stop_receipt", **rec)

    # ================================================================== time
    def tick(self):
        now = self.clock.now()
        ws = self.policy.workspace
        with self._io:
            with self._state:
                for cid in [c for c, p in self.pending.items() if now > p["expires"]]:
                    p = self.pending.pop(cid)
                    self.audit.write("pending_expired", confirmation_id=cid, action=p["action"])
                lease, todo, ended = self.lease, None, None
                if lease is not None:
                    if now >= lease["end"] - 1e-9:
                        self.lease, todo = None, "end"
                        self.last_motion_end = now
                        ended = self.last_motion = _motion(lease, now, "lease_end")
                    elif now >= lease["next_ka"] - 1e-9:
                        lease["next_ka"] = now + self.policy.keepalive_s
                        lease["keepalives"] += 1
                        todo = "keepalive"
                ws_due = ws is not None and self.lease is not None and now >= self.next_ws_check
                if ws_due:
                    self.next_ws_check = now + ws.check_every_s
            if todo == "end":
                try:
                    self._send_stop()
                    sid = self._open_receipt("lease_end")
                    with self._state:
                        ended["stop_id"] = sid
                    self.audit.write("lease_end", lease_id=lease["id"], action=lease["action"], stop_sent=True,
                                     keepalives=lease["keepalives"], ran_s=ended["ran_s"], of_s=ended["of_s"],
                                     stop_id=sid)
                except BackendError as e:
                    with self._state:
                        ended["ended"] = "lease_end, stop not delivered"
                    self.audit.write("lease_end", lease_id=lease["id"], action=lease["action"], stop_sent=False,
                                     ran_s=ended["ran_s"], of_s=ended["of_s"], error=str(e))
                    self._backend_failed(e, "stop at lease end")
            elif todo == "keepalive":
                try:
                    self.backend.keepalive(self.policy.actions[lease["action"]], lease["params"])
                except BackendError as e:
                    self._backend_failed(e, "keep-alive")
            if ws_due and self.lease is not None:
                try:
                    pose = self.backend.pose()
                except BackendError as e:
                    pose = None
                    self._backend_failed(e, "pose read")
                if pose is not None and not _inside(pose[:2], ws, margin=0.0):
                    self.stop(by="botinfo", reason=f"left the workspace at x={pose[0]:.2f} y={pose[1]:.2f}")
            if self.settle is not None:
                self._watch_tick()
            try:
                self.backend.tick()
            except BackendError as e:
                self._backend_failed(e, "backend tick")

    def _backend_failed(self, err, during: str):
        self.audit.write("backend_error", during=during, error=str(err))
        if self.policy.on_backend_error == "stop":
            self.stop(by="botinfo", reason=f"robot link failed during {during}")

    # ================================================================== internals
    def _execute(self, action, params, origin: str) -> dict:
        name = action.name
        with self._io:
            with self._state:
                if self.latched:
                    return self._deny(name, "stopped", f"the robot was stopped ({self.latch_reason})")
                gen = self.stop_gen
                now = self.clock.now()
                busy = self.lease is not None or now < self.busy_until
                replace = action.kind in ("motion", "oneshot") and busy
                if replace and self.policy.on_busy == "reject":
                    left = (self.lease["end"] if self.lease is not None else self.busy_until) - now
                    return self._deny(name, "busy", f"another motion is running ({max(0.0, left):.2f} s left); "
                                      "stop it first or wait")
            replaced, stop_failed = None, False      # the last_motion record of a leased motion this call replaces
            try:
                if action.kind == "motion":
                    duration = self.backend.motion_duration(action, params)
                    if not (0 < duration <= self.policy.max_lease_s + 1e-9):
                        return self._deny(name, "lease_too_long", f"{duration:.2f} s is outside 0..{self.policy.max_lease_s:g} s")
                    with self._state:
                        moving = self.lease is not None or now - self.last_motion_end < self.policy.min_rest_s
                        chain = self.chain_start if (moving and self.chain_start is not None) else now
                    if now + duration - chain > self.policy.max_total_s + 1e-9:
                        return self._deny(name, "continuous_limit", f"the robot has been moving for {now - chain:.1f} s; "
                                          f"at most {self.policy.max_total_s:g} s of motion without "
                                          f"{self.policy.min_rest_s:g} s of rest")
                    why = self._workspace_problem(action, params)
                    if why:
                        return self._deny(name, "workspace", why)
                cut = None
                if replace:
                    with self._state:
                        old, self.lease = self.lease, None
                        t = self.clock.now()
                        cut = ({"lease_id": old["id"], "action": old["action"], "ran_s": round(t - old["start"], 3),
                                "of_s": round(old["end"] - old["start"], 3)} if old else {"action": self.busy_action})
                        if old:
                            replaced = self.last_motion = {**cut, "ended": f"stopping for {name}"}
                    try:
                        self._send_stop()
                    except BackendError:
                        stop_failed = True
                        self._mark(replaced, f"stop for {name} not delivered; {name} was not sent")
                        raise
                    self.audit.write("replaced", old=old["id"] if old else self.busy_action, by=name,
                                     **{k: v for k, v in cut.items() if k.endswith("_s")})
                if self._stopped_since(gen):         # a stop latched while we waited (e.g. for the replace stop)
                    self._mark(replaced, f"stopped for {name}; a stop arrived before {name} was sent")
                    return self._said(self._deny(name, "stopped", "a stop arrived while the motion was starting"), cut)
                if action.kind in ("motion", "oneshot"):
                    self._end_receipt(f"{name} started a new motion")
                if action.kind == "motion":
                    self.backend.start_motion(action, params)
                    with self._state:
                        if self.stop_gen != gen:
                            started = False
                        else:
                            started = True
                            lid = f"l-{next(self._ids)}"
                            self.chain_start = chain
                            self.lease = {"id": lid, "action": name, "params": params, "start": now,
                                          "end": now + duration, "next_ka": now + self.policy.keepalive_s,
                                          "keepalives": 0}
                    if not started:                  # the start was sent; the stop that landed meanwhile goes after it
                        self._mark(replaced, f"replaced by {name}, which a stop cut as it started")
                        return self._said(self._deny(name, "stopped", "a stop arrived while the motion was starting"),
                                          cut)
                    self._mark(replaced, f"replaced by {name}")
                    self.audit.write("decision", tool=name, decision="allowed", origin=origin, params=params,
                                     lease_id=lid, runs_for_s=round(duration, 3))
                    return self._said({"ok": True, "decision": "allowed", "lease_id": lid,
                                       "runs_for_s": round(duration, 3), "renewable": self.policy.renewable}, cut)
                result = self.backend.execute(action, params)
                if action.kind == "oneshot":
                    with self._state:
                        self.busy_until = now + self.backend.oneshot_duration(action, params)
                        self.busy_action = name
                self._mark(replaced, f"replaced by {name}")
                self.audit.write("decision", tool=name, decision="allowed", origin=origin, params=params)
                return self._said({"ok": True, "decision": "allowed", "result": result,
                                   **({"uninterruptible": True} if action.uninterruptible else {})}, cut)
            except BackendError as e:
                if not stop_failed:                  # a connector can fail after the command went out
                    self._mark(replaced, f"stopped for {name}; starting {name} failed ({str(e)[:120]}), and the "
                                         "command may have reached the robot")
                self.audit.write("decision", tool=name, decision="failed", origin=origin, error=str(e))
                self._backend_failed(e, f"{name}")
                return {"ok": False, "decision": "failed", "rule": "backend_error", "reason": str(e)}

    def _mark(self, replaced, ended: str):
        """Say in a replaced motion's last_motion record how the replace turned out."""
        if replaced is not None:
            with self._state:
                replaced["ended"] = ended

    @staticmethod
    def _said(out: dict, cut) -> dict:
        """Tell the agent when its call stopped the running motion early. Motion calls return at once, so an agent
        that sends its next move too soon would otherwise believe the last one ran in full."""
        if cut:
            out["replaced"] = cut
            out["note"] = (f"This stopped the previous {cut['action']} ({cut['lease_id']}) early, after "
                           f"{cut['ran_s']:g} of {cut['of_s']:g} s." if "lease_id" in cut else
                           f"This stopped the previous {cut['action']} early.")
        return out

    def _stopped_since(self, gen) -> bool:
        """A stop sets stop_gen under _state at once, even while it waits for _io (held by this request). Checking
        it right before anything is sent keeps a request that was already past the latch check, and then waited
        (for a slow connector to finish the motion being replaced), from starting a motion after the stop."""
        with self._state:
            return self.stop_gen != gen

    def _workspace_problem(self, action, params) -> str | None:
        ws = self.policy.workspace
        if ws is None or action.kind != "motion":
            return None
        pose = self.backend.pose()
        if pose is None:
            return "the robot's position is unknown, so the workspace cannot be checked"
        path = self.backend.predict_path(action, params, pose)
        if path is None:
            return f"{action.name} has no path model, so the workspace cannot be checked"
        d0 = _outside_by(path[0], ws)
        for pt in path:
            if _outside_by(pt, ws) > d0 + 1e-6:
                return (f"the predicted path reaches x={pt[0]:.2f} y={pt[1]:.2f}, outside the workspace "
                        f"x {ws.x[0]:g}..{ws.x[1]:g}, y {ws.y[0]:g}..{ws.y[1]:g} (margin {ws.margin_m:g} m)")
        return None

    def _check_meta(self, meta: dict, now: float):
        if self.policy.require_request_id and "request_id" not in meta:
            return "missing_request_id", "this policy requires a request_id on every action call"
        rid = meta.get("request_id")
        if rid is not None:
            with self._state:
                if rid in self.seen_ids:
                    return "replay", f"request_id {rid!r} was already used"
                self.seen_ids[rid] = now
                while len(self.seen_ids) > MAX_SEEN_IDS:
                    self.seen_ids.popitem(last=False)
        ia = meta.get("issued_at")
        if ia is not None:
            age = self.clock.wall() - ia
            if age > self.policy.command_ttl_s + self.policy.max_clock_skew_s:
                return "stale", f"issued {age:.1f} s ago; commands expire after {self.policy.command_ttl_s:g} s"
            if age < -self.policy.max_clock_skew_s:
                return "future", f"issued_at is {-age:.1f} s in the future"
        return None

    @staticmethod
    def _take(dq: deque, rate, now: float) -> bool:
        while dq and dq[0] <= now - rate.window_s:
            dq.popleft()
        if len(dq) >= rate.max:
            return False
        dq.append(now)
        return True

    def _deny(self, tool, rule, reason) -> dict:
        self.audit.write("decision", tool=tool, decision="denied", rule=rule, reason=reason)
        return {"ok": False, "decision": "denied", "rule": rule, "reason": reason}

    def _ok(self, tool, what, **data) -> dict:
        self.audit.write("decision", tool=tool, decision="allowed", rule=what)
        return {"ok": True, **data}

    def _state_view(self, include_pending_params=False) -> dict:
        now = self.clock.now()
        with self._state:
            lease = self.lease
            return {"stopped": self.latched, "stop_reason": self.latch_reason,
                    "lease": None if lease is None else {"id": lease["id"], "action": lease["action"],
                                                         "ends_in_s": round(max(0.0, lease["end"] - now), 3)},
                    "busy_with": self.busy_action if now < self.busy_until else None,
                    "pending": [{"id": p["id"], "action": p["action"], "why": p["why"],
                                 "expires_in_s": round(max(0.0, p["expires"] - now), 3),
                                 **({"params": p["params"]} if include_pending_params else {})}
                                for p in self.pending.values()],
                    "last_stop": None if self.last_stop is None else dict(self.last_stop),
                    "last_motion": None if self.last_motion is None else dict(self.last_motion)}

    def _status_result(self, tool: str = "botinfo_status", rule: str = "status", **extra) -> dict:
        robot, err = {}, None
        with self._io:
            try:
                raw = self.backend.status()
            except BackendError as e:
                raw, err = {}, str(e)
        clean, red = sanitize(raw, self.backend.status_schema())
        for r in red:
            self.audit.write("output_redacted", **r)
        robot = clean if err is None else {"unreachable": err}
        self.audit.write("decision", tool=tool, decision="allowed", rule=rule, redacted=len(red), **extra)
        return {"ok": True, **extra, "botinfo": self._state_view(), "robot": robot,
                **({"redacted_fields": [r["field"] for r in red]} if red else {})}

    def _wait(self, args) -> dict:
        """Block the CALLER until the running motion ends, a stop lands, the agent goes away or max_s passes. It
        holds no lock while it sleeps, so tick() keeps ending leases and every stop goes through at once. The MCP
        server runs it on its own thread (StdioServer slow_tools), so the agent's botinfo_stop is never queued behind
        it."""
        cap = self.policy.max_lease_s
        if not isinstance(args, dict) or set(args) - {"max_s"}:
            return self._deny("botinfo_wait", "bad_params", "expected only max_s")
        m = args.get("max_s", cap)
        if isinstance(m, bool) or not isinstance(m, (int, float)) or not math.isfinite(m) or not 0 <= m <= cap:
            return self._deny("botinfo_wait", "bad_params", f"max_s must be 0..{cap:g}")
        t0 = self.clock.now()
        while True:
            now = self.clock.now()
            with self._state:
                moving = self.lease is not None or now < self.busy_until
                why = ("stopped" if self.latched else "agent gone" if not self.connected
                       else "idle" if not moving else "max_s" if now - t0 >= m - 1e-9 else None)
            if why or self.sleep is None:
                break
            self.sleep(min(0.05, max(0.0, m - (now - t0))))
        return self._status_result("botinfo_wait", "wait", waited_s=round(self.clock.now() - t0, 3), moving=moving,
                                   ended_by=why or "clock does not run")

    def _renew(self, args) -> dict:
        if not isinstance(args, dict) or set(args) != {"lease_id", "extend_s"}:
            return self._deny("botinfo_renew", "bad_params", "expected exactly lease_id and extend_s")
        lid, ext = args["lease_id"], args["extend_s"]
        if isinstance(ext, bool) or not isinstance(ext, (int, float)) or not math.isfinite(ext) \
                or not 0.1 <= ext <= self.policy.max_lease_s:
            return self._deny("botinfo_renew", "bad_params", f"extend_s must be 0.1..{self.policy.max_lease_s:g}")
        now = self.clock.now()
        with self._state:
            if self.latched:
                return self._deny("botinfo_renew", "stopped", "the robot was stopped; only the owner can resume")
            lease = self.lease
            if lease is None or lease["id"] != lid:
                return self._deny("botinfo_renew", "no_lease", f"no running lease {str(lid)[:40]!r}")
            new_end = now + float(ext)
            chain = self.chain_start if self.chain_start is not None else lease["start"]
            if new_end - chain > self.policy.max_total_s + 1e-9:
                return self._deny("botinfo_renew", "continuous_limit", f"at most {self.policy.max_total_s:g} s of "
                                  f"motion without {self.policy.min_rest_s:g} s of rest")
            lease["end"] = max(lease["end"], new_end)
            ends_in = lease["end"] - now
        self.audit.write("decision", tool="botinfo_renew", decision="allowed", lease_id=lid, ends_in_s=round(ends_in, 3))
        return {"ok": True, "decision": "renewed", "lease_id": lid, "ends_in_s": round(ends_in, 3)}


def _motion(lease: dict, now: float, ended: str) -> dict:
    """The record of a motion BotInfo just ended: which lease, its lease time so far (accepted to stop sent) of the
    time planned, and why."""
    return {"lease_id": lease["id"], "action": lease["action"], "ran_s": round(now - lease["start"], 3),
            "of_s": round(lease["end"] - lease["start"], 3), "ended": ended}


def _moved(first, last, pose) -> dict:
    """How far the robot went between the first and the last status read of a stop receipt, when the binding names
    pose fields. The first read comes after the stop was sent, so this is a lower bound on the travel after it."""
    if not pose or not first or not last:
        return {}
    try:
        x0, y0, h0 = (float(first[pose[k]]) for k in ("x", "y", "heading"))
        x1, y1, h1 = (float(last[pose[k]]) for k in ("x", "y", "heading"))
    except (KeyError, TypeError, ValueError):
        return {}
    turn = (h1 - h0 + math.pi) % (2 * math.pi) - math.pi
    return {"moved_m_after_first_read": round(math.hypot(x1 - x0, y1 - y0), 4),
            "turned_rad_after_first_read": round(abs(turn), 4)}


def _inner(ws):
    m = ws.margin_m
    return (ws.x[0] + m, ws.x[1] - m), (ws.y[0] + m, ws.y[1] - m)


def _outside_by(pt, ws) -> float:
    (x0, x1), (y0, y1) = _inner(ws)
    dx = max(x0 - pt[0], 0.0, pt[0] - x1)
    dy = max(y0 - pt[1], 0.0, pt[1] - y1)
    return math.hypot(dx, dy)


def _inside(pt, ws, margin=0.0) -> bool:
    return ws.x[0] + margin <= pt[0] <= ws.x[1] - margin and ws.y[0] + margin <= pt[1] <= ws.y[1] - margin
