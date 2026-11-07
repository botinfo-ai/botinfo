"""What BotInfo needs from a backend (the adapter in front of one robot connector).

A backend executes ALREADY-CHECKED actions. It never decides what is allowed; core.py does. It must:
  - start_motion / keepalive / stop for kind="motion" actions (stop must work at any time, from any thread);
  - execute for kind="oneshot" and kind="instant" actions;
  - status() -> a dict of typed fields, described by status_schema() so BotInfo can redact anything else;
  - optionally pose() and predict_path() for the workspace check;
  - optionally settle_spec(), naming the status fields that read the robot's speed, for stop receipts.
Errors reaching the robot raise BackendError. stop() may instead return the connector's reply: a reply that says the
stop failed (refusal() below) counts as not delivered, exactly like a raised BackendError.
"""
from __future__ import annotations

import math
import re

TOKEN_RE = re.compile(r"^[A-Za-z0-9_.:/ -]*$")
INJECTION_HINTS = re.compile(
    r"(ignore (all |any |your )?(previous|prior|above)|system prompt|you are now|new instructions|"
    r"\bCALL\s+[a-z_]+|tool[ _]?call|override|maintenance mode|disregard|do not tell)", re.I)
REDACTED = "[redacted]"


class BackendError(Exception):
    pass


class ToolError(Exception):
    """A connector refused or failed a tool call (unknown tool, bad arguments, robot unreachable)."""


class Link:
    """The wire between a connector (or BotInfo) and the robot. The benchmark cuts it to simulate a lost
    connection; while it is down, nothing reaches the robot in either direction."""

    def __init__(self):
        self.up = True

    def check(self):
        if not self.up:
            raise BackendError("robot unreachable (link down)")


class Backend:
    name = "backend"

    def check_policy(self, policy) -> list:
        """Problems that make this policy unusable with this backend; entries starting "warning:" are advice."""
        return []

    def describe(self) -> dict:
        return {"backend": self.name}

    # motion
    def motion_duration(self, action, params) -> float:
        if action.duration_param:
            return float(params[action.duration_param])
        raise BackendError(f"{action.name}: no duration")

    def start_motion(self, action, params) -> None:
        raise BackendError(f"{action.name}: not supported")

    def keepalive(self, action, params) -> None:
        pass

    def stop(self):
        """Stop now. Return None, or the connector's reply so core.py can tell a refusal from a stop."""
        raise NotImplementedError

    # one-shot and instant
    def execute(self, action, params) -> dict:
        raise BackendError(f"{action.name}: not supported")

    def oneshot_duration(self, action, params) -> float:
        return 0.0

    # reading
    def status(self) -> dict:
        return {}

    def status_schema(self) -> dict:
        return {}

    def settle_spec(self):
        """How BotInfo tells that the robot came to rest after a stop, or None (no stop receipts):
        {"fields": [number fields of status() that read the robot's speed], "below": at rest when every |value|
        is under this, "hold_s": for this long, "timeout_s": give up after this, "every_s": read this often,
        "pose": {"x": field, "y": field, "heading": field} or None}."""
        return None

    def pose(self):
        """(x, y, heading_rad) or None."""
        return None

    def predict_path(self, action, params, pose):
        """List of (x, y) the action would pass through, or None if unknown."""
        return None

    def tick(self) -> None:
        pass

    def close(self) -> None:
        pass


FAILED_FLAGS = ("ok", "success", "successful", "delivered")     # False here = the call failed
ERROR_FLAGS = ("isError", "error")                               # truthy here = the call failed
FAILED_STATUS = {"error", "failed", "failure", "refused", "rejected"}


def refusal(reply) -> str | None:
    """Why a reply that came back WITHOUT an exception still says the call failed, or None.

    Public robot connectors report failure as data in several shapes: False, {"ok": false},
    {"success": false}, {"isError": true}, {"status": "error", "message": ...}, a truthy "error". None, True and
    any other reply read as carried out; text a connector does not structure cannot be judged here."""
    if reply is False:
        return "the backend reported the call failed"
    if not isinstance(reply, dict):
        return None
    for k in FAILED_FLAGS:
        if reply.get(k) is False:
            return f"{k} is false" + _detail(reply)
    for k in ERROR_FLAGS:
        if reply.get(k):
            return f"{k}: {str(reply[k])[:200]}" if isinstance(reply[k], str) else f"{k} is set" + _detail(reply)
    st = reply.get("status")
    if isinstance(st, str) and st.strip().lower() in FAILED_STATUS:
        return f"status is {st.strip().lower()!r}" + _detail(reply)
    return None


def _detail(reply: dict) -> str:
    for k in ("message", "reason", "error", "detail"):
        if isinstance(reply.get(k), str) and reply[k]:
            return f" ({reply[k][:200]})"
    return ""


def sanitize(values: dict, schema: dict) -> tuple:
    """Keep only schema fields with values of the declared type. -> (clean, redactions).

    schema: {field: {"type": "number"|"boolean"|"enum"|"token", "values": [...], "max_len": n}}
    A value that does not fit is replaced by "[redacted]" and reported, with a hint if it looks like an
    instruction aimed at a model. Fields not in the schema are dropped (and reported)."""
    clean, red = {}, []
    for k, v in (values or {}).items():
        spec = schema.get(k)
        if spec is None:
            red.append(_redaction(k, v, "field not in the status schema"))
            continue
        t = spec["type"]
        ok = False
        if t == "number":
            ok = isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
        elif t == "boolean":
            ok = isinstance(v, bool)
        elif t == "enum":
            ok = isinstance(v, str) and v in spec["values"]
        elif t == "token":
            ok = isinstance(v, str) and len(v) <= spec.get("max_len", 32) and bool(TOKEN_RE.match(v))
        if ok:
            clean[k] = v
        else:
            clean[k] = REDACTED
            red.append(_redaction(k, v, f"not a valid {t}"))
    return clean, red


def _redaction(k, v, why) -> dict:
    text = v if isinstance(v, str) else repr(v)
    return {"field": k, "why": why, "length": len(text), "preview": text[:120],
            "looks_like_instructions": bool(INJECTION_HINTS.search(text))}
