"""The Reins engine: every agent request goes through here, and so does every stop.

Order of checks for an action call (each refusal is logged with the rule that made it):
  1. reins_stop      -> stop now. Never refused, never rate-limited, never queued.
  2. overall rate    -> [rate_limits].all, counted on every call except reins_stop
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

In the benchmark everything runs on one thread with a VirtualClock.
"""
from __future__ import annotations

import itertools
import math
import threading
from collections import OrderedDict, defaultdict, deque

from .audit import AuditLog
from .backends.base import BackendError, sanitize
from .clock import RealClock
from .policy import PolicyError, check_params, needs_confirmation, split_meta

MAX_SEEN_IDS = 10000
STOP_NOTE = ("This is a software stop sent over a network link. If the robot did not stop, use its local "
             "stop (power switch or hardware e-stop).")


class Reins:
    def __init__(self, policy, backend, clock=None, audit: AuditLog | None = None, on_pending=None,
                 strict_binding: bool = True):
        problems = [p for p in backend.check_policy(policy) if not p.startswith("warning:")]
        if problems and strict_binding:
            raise PolicyError("policy and backend do not fit:\n  - " + "\n  - ".join(problems))
        self.policy, self.backend = policy, backend
        self.clock = clock or RealClock()
        self.audit = audit or AuditLog(clock=self.clock)
        self.on_pending = on_pending
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
        self.audit.write("start", policy=policy.name, policy_file=policy.source, backend=backend.describe(),
                         actions=sorted(policy.actions))

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
                         + (" Renew with reins_renew." if self.policy.renewable else "")
                         + " The call returns at once, while the robot moves; runs_for_s says for how long.")
            if a.kind in ("motion", "oneshot"):
                desc += (" A new motion command stops the running one early and replaces it (the answer says how "
                         "far it got); wait for it to end first if it should finish."
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
        tools.append({"name": "reins_stop", "description": "Stop the robot now. Always allowed. Afterwards motion "
                      "stays blocked until the owner resumes.",
                      "inputSchema": {"type": "object", "properties": {"reason": {"type": "string"}}},
                      "annotations": {"readOnlyHint": False, "destructiveHint": False}})
        tools.append({"name": "reins_status", "description": "Reins state (stopped?, running lease, pending "
                      "confirmations) and the robot's status fields.",
                      "inputSchema": {"type": "object", "properties": {}}, "annotations": {"readOnlyHint": True}})
        tools.append({"name": "reins_policy", "description": "What this robot's owner allows: actions, bounds, limits.",
                      "inputSchema": {"type": "object", "properties": {}}, "annotations": {"readOnlyHint": True}})
        if self.policy.renewable:
            tools.append({"name": "reins_renew", "description": "Extend the running motion lease (heartbeat). "
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
        if name == "reins_stop":
            reason = args.get("reason", "") if isinstance(args, dict) else ""
            return self.stop(by="agent", reason=str(reason)[:200])
        now = self.clock.now()
        with self._state:
            if not self._take(self.calls_all, self.policy.rate_all, now):
                return self._deny(name, "rate_limit_all", f"more than {self.policy.rate_all.max} calls in "
                                  f"{self.policy.rate_all.window_s:g} s")
        if name == "reins_status":
            return self._status_result()
        if name == "reins_policy":
            return self._ok(name, "policy", policy=self.policy.summary())
        if name == "reins_renew" and self.policy.renewable:
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
            self.stop(by="reins", reason="the agent disconnected")

    # ================================================================== stop
    def stop(self, by: str = "agent", reason: str = "") -> dict:
        with self._state:
            self.stop_gen += 1
            if self.policy.stop_latch:
                self.latched = True
                self.latch_reason = f"stop by {by}" + (f": {reason}" if reason else "")
            lease, self.lease = self.lease, None
            if lease is not None:
                self.last_motion_end = self.clock.now()
            cancelled = sorted(self.pending)
            self.pending.clear()
        with self._io:
            try:
                self.backend.stop()
                delivered, err = True, None
            except BackendError as e:
                delivered, err = False, str(e)
        self.audit.write("stop", by=by, reason=reason, delivered=delivered, error=err, latched=self.latched,
                         cancelled_lease=lease["id"] if lease else None, cancelled_pending=cancelled)
        out = {"ok": delivered, "decision": "stopped", "delivered": delivered, "latched": self.latched,
               "note": STOP_NOTE}
        if err:
            out["error"] = err
        return out

    # ================================================================== time
    def tick(self):
        now = self.clock.now()
        ws = self.policy.workspace
        with self._io:
            with self._state:
                for cid in [c for c, p in self.pending.items() if now > p["expires"]]:
                    p = self.pending.pop(cid)
                    self.audit.write("pending_expired", confirmation_id=cid, action=p["action"])
                lease, todo = self.lease, None
                if lease is not None:
                    if now >= lease["end"] - 1e-9:
                        self.lease, todo = None, "end"
                        self.last_motion_end = now
                    elif now >= lease["next_ka"] - 1e-9:
                        lease["next_ka"] = now + self.policy.keepalive_s
                        lease["keepalives"] += 1
                        todo = "keepalive"
                ws_due = ws is not None and self.lease is not None and now >= self.next_ws_check
                if ws_due:
                    self.next_ws_check = now + ws.check_every_s
            if todo == "end":
                try:
                    self.backend.stop()
                    self.audit.write("lease_end", lease_id=lease["id"], action=lease["action"], stop_sent=True,
                                     keepalives=lease["keepalives"], ran_s=round(now - lease["start"], 3))
                except BackendError as e:
                    self.audit.write("lease_end", lease_id=lease["id"], action=lease["action"], stop_sent=False,
                                     error=str(e))
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
                    self.stop(by="reins", reason=f"left the workspace at x={pose[0]:.2f} y={pose[1]:.2f}")
            try:
                self.backend.tick()
            except BackendError as e:
                self._backend_failed(e, "backend tick")

    def _backend_failed(self, err, during: str):
        self.audit.write("backend_error", during=during, error=str(err))
        if self.policy.on_backend_error == "stop":
            self.stop(by="reins", reason=f"robot link failed during {during}")

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
                    self.backend.stop()
                    self.audit.write("replaced", old=old["id"] if old else self.busy_action, by=name,
                                     **{k: v for k, v in cut.items() if k.endswith("_s")})
                if self._stopped_since(gen):         # a stop latched while we waited (e.g. for the replace stop)
                    return self._said(self._deny(name, "stopped", "a stop arrived while the motion was starting"), cut)
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
                    if not started:
                        return self._said(self._deny(name, "stopped", "a stop arrived while the motion was starting"),
                                          cut)
                    self.audit.write("decision", tool=name, decision="allowed", origin=origin, params=params,
                                     lease_id=lid, runs_for_s=round(duration, 3))
                    return self._said({"ok": True, "decision": "allowed", "lease_id": lid,
                                       "runs_for_s": round(duration, 3), "renewable": self.policy.renewable}, cut)
                result = self.backend.execute(action, params)
                if action.kind == "oneshot":
                    with self._state:
                        self.busy_until = now + self.backend.oneshot_duration(action, params)
                        self.busy_action = name
                self.audit.write("decision", tool=name, decision="allowed", origin=origin, params=params)
                return self._said({"ok": True, "decision": "allowed", "result": result,
                                   **({"uninterruptible": True} if action.uninterruptible else {})}, cut)
            except BackendError as e:
                self.audit.write("decision", tool=name, decision="failed", origin=origin, error=str(e))
                self._backend_failed(e, f"{name}")
                return {"ok": False, "decision": "failed", "rule": "backend_error", "reason": str(e)}

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
                                for p in self.pending.values()]}

    def _status_result(self) -> dict:
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
        self.audit.write("decision", tool="reins_status", decision="allowed", rule="status", redacted=len(red))
        return {"ok": True, "reins": self._state_view(), "robot": robot,
                **({"redacted_fields": [r["field"] for r in red]} if red else {})}

    def _renew(self, args) -> dict:
        if not isinstance(args, dict) or set(args) != {"lease_id", "extend_s"}:
            return self._deny("reins_renew", "bad_params", "expected exactly lease_id and extend_s")
        lid, ext = args["lease_id"], args["extend_s"]
        if isinstance(ext, bool) or not isinstance(ext, (int, float)) or not math.isfinite(ext) \
                or not 0.1 <= ext <= self.policy.max_lease_s:
            return self._deny("reins_renew", "bad_params", f"extend_s must be 0.1..{self.policy.max_lease_s:g}")
        now = self.clock.now()
        with self._state:
            if self.latched:
                return self._deny("reins_renew", "stopped", "the robot was stopped; only the owner can resume")
            lease = self.lease
            if lease is None or lease["id"] != lid:
                return self._deny("reins_renew", "no_lease", f"no running lease {str(lid)[:40]!r}")
            new_end = now + float(ext)
            chain = self.chain_start if self.chain_start is not None else lease["start"]
            if new_end - chain > self.policy.max_total_s + 1e-9:
                return self._deny("reins_renew", "continuous_limit", f"at most {self.policy.max_total_s:g} s of "
                                  f"motion without {self.policy.min_rest_s:g} s of rest")
            lease["end"] = max(lease["end"], new_end)
            ends_in = lease["end"] - now
        self.audit.write("decision", tool="reins_renew", decision="allowed", lease_id=lid, ends_in_s=round(ends_in, 3))
        return {"ok": True, "decision": "renewed", "lease_id": lid, "ends_in_s": round(ends_in, 3)}


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
