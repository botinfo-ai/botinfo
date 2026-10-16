"""Sesame quadruped (Dorian Todd's open-source robot) through its firmware's own HTTP API.

Two things live here:
  SesameBackend         Reins' adapter. Policy actions walk / turn (motion, on a lease with keep-alives),
                        emote / posture (one-shot), face (instant). Only one-field JSON bodies the firmware knows.
  SesameDirectConnector the REFERENCE raw connector graded by the benchmark: it hands the agent the firmware API
                        the way an existing client would (send any command name, read status), with no checks.
                        It is BotInfo's model of that pattern, not a third-party product.

Transports: MockTransport (in-process, sesame_mock.MockSesame, used by the benchmark) and HttpTransport (plain
HTTP; localhost only unless allow_remote=True). Firmware facts: sesame_spec.py. The firmware has no auth and no
HTTPS, accepts any command name (stock), and cannot interrupt a pose once it starts.
"""
from __future__ import annotations

import ipaddress
import json
import urllib.error
import urllib.request
from urllib.parse import urlsplit

from . import sesame_spec as F
from .base import Backend, BackendError, Link, ToolError

HTTP_TIMEOUT_S = 1.0
SUPPORTED = {"walk": "motion", "turn": "motion", "emote": "oneshot", "posture": "oneshot", "face": "instant"}
DIRECTIONS = {"walk": ("forward", "backward"), "turn": ("left", "right")}
ENUMS = {("walk", "direction"): ("forward", "backward"), ("turn", "direction"): ("left", "right"),
         ("emote", "name"): F.POSE_COMMANDS, ("posture", "name"): F.POSTURE_COMMANDS,
         ("face", "name"): tuple(f for f in F.FIRMWARE_FACES if f not in F.FACES_WITHOUT_BITMAP)}


class MockTransport:
    """Requests go straight to an in-process MockSesame, through a Link the benchmark can cut."""

    def __init__(self, mock, link: Link | None = None):
        self.mock, self.link = mock, link or Link()

    def request(self, method: str, path: str, body: str = "") -> tuple:
        self.link.check()
        status, _, text = self.mock.handle(method, path, body)
        return status, text


def is_local_url(url: str) -> bool:
    host = urlsplit(url).hostname or ""
    if host == "localhost":
        return True
    try:
        return ipaddress.ip_address(host).is_loopback
    except ValueError:
        return False


class HttpTransport:
    def __init__(self, url: str, allow_remote: bool = False, timeout: float = HTTP_TIMEOUT_S):
        u = urlsplit(url)
        if u.scheme != "http" or not u.hostname:
            raise ValueError(f"robot URL must be plain http://host[:port], got {url!r}")
        if not allow_remote and not is_local_url(url):
            raise ValueError("refusing a non-localhost robot URL without allow_remote (v0 is mock/simulation only)")
        self.base, self.timeout = url.rstrip("/"), timeout

    def request(self, method: str, path: str, body: str = "") -> tuple:
        data = body.encode() if method == "POST" else None
        req = urllib.request.Request(self.base + path, data=data, method=method,
                                     headers={"Content-Type": "application/json"} if data else {})
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as r:
                return r.status, r.read().decode("utf-8", "replace")
        except urllib.error.HTTPError as e:
            return e.code, e.read().decode("utf-8", "replace")
        except (urllib.error.URLError, OSError, TimeoutError) as e:
            raise BackendError(f"robot unreachable: {e}") from None


class SesameBackend(Backend):
    name = "sesame-http"

    def __init__(self, transport, read_settings: bool = True):
        self.t = transport
        self.settings = dict(F.DEFAULT_SETTINGS)
        self.firmware = "unknown"
        if read_settings:
            self.refresh_settings()

    def refresh_settings(self):
        status, text = self.t.request("GET", "/getSettings")
        if status != 200:
            raise BackendError(f"/getSettings answered {status}")
        try:
            s = json.loads(text)
        except ValueError:
            raise BackendError("/getSettings is not JSON") from None
        problem = F.settings_problem(s)
        if problem:
            raise BackendError(f"refusing to time moves: {problem}")
        self.settings = {k: s[k] for k in F.DEFAULT_SETTINGS}
        self.firmware = (f"patched (motionTimeout {s['motionTimeout']} ms)" if isinstance(s.get("motionTimeout"), int)
                         else "stock (no motion timeout)")

    def check_policy(self, policy) -> list:
        problems = []
        for a in policy.actions.values():
            if a.name not in SUPPORTED:
                problems.append(f"{a.name}: the Sesame backend has no such action ({sorted(SUPPORTED)})")
                continue
            if a.kind != SUPPORTED[a.name]:
                problems.append(f"{a.name}: must be kind = \"{SUPPORTED[a.name]}\"")
            want = {"walk": {"direction", "steps"}, "turn": {"direction", "steps"}}.get(a.name, {"name"})
            if set(a.params) != want:
                problems.append(f"{a.name}: parameters must be exactly {sorted(want)}")
                continue
            for pn, p in a.params.items():
                allowed = ENUMS.get((a.name, pn))
                if allowed is not None and (p.type != "enum" or not set(p.values) <= set(allowed)):
                    problems.append(f"{a.name}.{pn}: must be an enum within {list(allowed)}")
                if pn == "steps" and (p.type != "integer" or p.min < 1):
                    problems.append(f"{a.name}.steps: must be an integer with min >= 1")
            if a.name == "emote" and not (a.uninterruptible and a.confirm):
                problems.append("emote: firmware poses cannot be interrupted by stop; "
                                "the policy must say uninterruptible = true and confirm = true")
            if a.name in ("walk", "turn"):
                hi = a.params.get("steps")
                if hi is not None and hi.type == "integer":
                    longest = max(F.move_seconds(d, int(hi.max), self.settings) for d in DIRECTIONS[a.name])
                    if longest > policy.max_lease_s:
                        problems.append(f"{a.name}: {int(hi.max)} steps take {longest:.1f} s, above "
                                        f"max_lease_s {policy.max_lease_s:g}")
        if policy.keepalive_s > 1.5:
            problems.append("lease.keepalive_s above 1.5 s would let the patched firmware's 3 s timeout fire mid-walk")
        return problems

    def describe(self) -> dict:
        return {"backend": self.name, "firmware": self.firmware, "settings": self.settings}

    def _post(self, payload: dict):
        status, text = self.t.request("POST", "/api/command", json.dumps(payload, separators=(",", ":")))
        if status != 200:
            raise BackendError(f"robot answered {status}: {text[:120]}")

    def motion_duration(self, action, params) -> float:
        return F.move_seconds(params["direction"], params["steps"], self.settings)

    def start_motion(self, action, params):
        self._post({"command": params["direction"]})

    def keepalive(self, action, params):
        self._post({"command": params["direction"]})

    def stop(self):
        self._post({"command": F.STOP_COMMAND})

    def execute(self, action, params) -> dict:
        if action.name == "face":
            self._post({"face": params["name"]})
        else:
            self._post({"command": params["name"]})
        return {"sent": params["name"]}

    def oneshot_duration(self, action, params) -> float:
        return F.pose_seconds(params["name"], self.settings)

    def status(self) -> dict:
        status, text = self.t.request("GET", "/api/status")
        if status != 200:
            raise BackendError(f"/api/status answered {status}")
        try:
            s = json.loads(text)
        except ValueError:
            raise BackendError("/api/status is not JSON") from None
        return {"current_command": s.get("currentCommand"), "current_face": s.get("currentFace"),
                "firmware": "patched" if self.firmware.startswith("patched") else "stock"}

    def status_schema(self) -> dict:
        return {"current_command": {"type": "enum", "values": ("",) + F.FIRMWARE_COMMANDS},
                "current_face": {"type": "enum", "values": F.FIRMWARE_FACES},
                "firmware": {"type": "enum", "values": ("stock", "patched")}}


class SesameDirectConnector:
    """REFERENCE raw connector: the firmware API handed to the agent as tools, with no checks (BotInfo's model of
    how existing clients use it: one command name per call, sent as is)."""

    label = "Sesame firmware API, direct (reference connector)"

    def __init__(self, transport):
        self.t = transport

    def list_tools(self) -> list:
        return [
            {"name": "send_command", "description": "Send a command to the robot (forward, backward, left, right, "
             "stop, stand, rest, wave, dance, ...). Moves repeat until stop.",
             "inputSchema": {"type": "object", "properties": {"command": {"type": "string"}},
                             "required": ["command"]}},
            {"name": "set_face", "description": "Show a face on the robot's screen.",
             "inputSchema": {"type": "object", "properties": {"face": {"type": "string"}}, "required": ["face"]}},
            {"name": "get_status", "description": "Read the robot's current command and face.",
             "inputSchema": {"type": "object", "properties": {}}},
        ]

    def call_tool(self, name: str, args: dict) -> dict:
        try:
            if name == "send_command":
                return self._send({"command": args.get("command")})
            if name == "set_face":
                return self._send({"face": args.get("face")})
            if name == "get_status":
                status, text = self.t.request("GET", "/api/status")
                return {"http_status": status, "response": _json_or_text(text)}
        except BackendError as e:
            raise ToolError(str(e)) from None
        raise ToolError(f"unknown tool {name!r}")

    def _send(self, payload) -> dict:
        status, text = self.t.request("POST", "/api/command", json.dumps(payload))
        return {"http_status": status, "response": _json_or_text(text)}

    def tick(self):
        pass

    def close(self):
        pass


def _json_or_text(text):
    try:
        return json.loads(text)
    except ValueError:
        return text
