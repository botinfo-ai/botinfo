"""Front an existing MCP connector (ros-mcp-server, or the in-process reference connector) by mapping each
policy action to a fixed tool call. The agent never sees the connector's own tools.

A binding file says, per action, which connector tool to call and with which arguments. Arguments are literal
values except whole-value placeholders: the string "$linear_mps" is replaced by the CHECKED value of that
parameter (a number stays a number). Nothing is ever spliced into a longer string, so a parameter cannot change
the topic, the message type or any other field. "$$" at the start of a string stands for a literal "$".

    [binding]
    kind = "mcp-tools"
    connector = "simrover-ros-style"            # in-process (demo, benchmark)
    # command = ["uvx", "ros-mcp", "--transport=stdio"]   # or launch an MCP server over stdio
    error_keys = ["error"]                      # a result with a truthy one of these counts as a failure
    init = [ { tool = "...", args = { } } ]      # optional calls made once at start

    [stop]
    call = { tool = "publish_once", args = { topic = "/cmd_vel", msg_type = "...", msg = { ... zeros ... } } }
    repeat = 2

    [actions.drive]                             # kind = "motion" in the policy
    start = { tool = "publish_once", args = { ..., msg = { linear = { x = "$linear_mps" } } } }
    keepalive = "start"                         # re-send start every keepalive_s, or "none"
    start_mode = "wait"                         # or "send": put start on the wire and return without its reply,
                                                # for a connector whose motion tool blocks until the motion ends
                                                # (unitree-go2-mcp-server); an error reply stops the robot later
    path = { model = "unicycle", linear = "$linear_mps", angular = "$angular_rps", duration = "$duration_s" }

    [actions.set_led]                           # kind = "instant" or "oneshot"
    call = { tool = "publish_once", args = { ... } }
    duration_s = 0.0                            # oneshot only: how long it keeps the robot busy

    [status]
    call = { tool = "subscribe_once", args = { topic = "/odom", msg_type = "nav_msgs/msg/Odometry" } }
    fields.x = { path = "msg.pose.pose.position.x", type = "number" }
    fields.heading_rad = { path = "msg.pose.pose.orientation", type = "yaw_from_quaternion" }

    [pose]
    x = "x"
    y = "y"
    heading = "heading_rad"
"""
from __future__ import annotations

import copy
import math
import threading
from pathlib import Path

from .base import Backend, BackendError, ToolError

try:
    import tomllib
except ImportError:  # Python < 3.11
    tomllib = None

FIELD_TYPES = ("number", "boolean", "enum", "token", "yaw_from_quaternion")


class BindingError(ValueError):
    pass


def load_binding(path) -> dict:
    path = Path(path)
    if tomllib is None:
        raise BindingError("TOML bindings need Python 3.11+")
    d = tomllib.loads(path.read_text(encoding="utf-8"))
    d.setdefault("binding", {})
    d["binding"]["_source"] = str(path)
    return d


def _placeholders(v, out: set) -> set:
    if isinstance(v, str) and v.startswith("$") and not v.startswith("$$"):
        out.add(v[1:])
    elif isinstance(v, dict):
        for x in v.values():
            _placeholders(x, out)
    elif isinstance(v, list):
        for x in v:
            _placeholders(x, out)
    return out


def fill(template, params: dict):
    """Replace whole-value "$name" strings by params[name]; "$$x" -> "$x"."""
    if isinstance(template, str):
        if template.startswith("$$"):
            return template[1:]
        if template.startswith("$"):
            return params[template[1:]]
        return template
    if isinstance(template, dict):
        return {k: fill(v, params) for k, v in template.items()}
    if isinstance(template, list):
        return [fill(v, params) for v in template]
    return copy.deepcopy(template)


def dig(obj, path: str):
    for part in path.split("."):
        if isinstance(obj, dict) and part in obj:
            obj = obj[part]
        elif isinstance(obj, list) and part.isdigit() and int(part) < len(obj):
            obj = obj[int(part)]
        else:
            return None
    return obj


def _call_spec(d, where) -> dict:
    if not isinstance(d, dict) or not isinstance(d.get("tool"), str) or not isinstance(d.get("args", {}), dict):
        raise BindingError(f"{where}: expected {{ tool = \"...\", args = {{ ... }} }}")
    extra = set(d) - {"tool", "args"}
    if extra:
        raise BindingError(f"{where}: unknown key(s) {sorted(extra)}")
    return {"tool": d["tool"], "args": d.get("args", {})}


class ToolTemplateBackend(Backend):
    name = "mcp-tools"

    def __init__(self, caller, binding: dict):
        """caller: anything with call_tool(name, args) -> dict that raises ToolError on failure."""
        self.caller = caller
        b = binding.get("binding", {})
        if b.get("kind", "mcp-tools") != "mcp-tools":
            raise BindingError(f"binding kind {b.get('kind')!r} is not mcp-tools")
        self.source = b.get("_source", "")
        self.label = b.get("label", "MCP connector")
        self.error_keys = tuple(b.get("error_keys", ["error"]))
        self.init_calls = [_call_spec(c, f"[binding].init[{i}]") for i, c in enumerate(b.get("init", []))]
        if "stop" not in binding or "call" not in binding["stop"]:
            raise BindingError("[stop].call is required: BotInfo must know how to stop this robot")
        self.stop_call = _call_spec(binding["stop"]["call"], "[stop].call")
        if _placeholders(self.stop_call["args"], set()):
            raise BindingError("[stop].call: the stop call cannot have placeholders")
        self.stop_repeat = int(binding["stop"].get("repeat", 1))
        if not 1 <= self.stop_repeat <= 5:
            raise BindingError("[stop].repeat: 1..5")
        self.actions = {}
        for name, a in binding.get("actions", {}).items():
            spec = {}
            if "start" in a:
                spec["start"] = _call_spec(a["start"], f"[actions.{name}].start")
                ka = a.get("keepalive", "start")
                if ka not in ("start", "none"):
                    raise BindingError(f"[actions.{name}].keepalive: \"start\" or \"none\"")
                spec["keepalive"] = ka
                mode = a.get("start_mode", "wait")
                if mode not in ("wait", "send"):
                    raise BindingError(f"[actions.{name}].start_mode: \"wait\" or \"send\"")
                spec["start_mode"] = mode
                if "path" in a:
                    p = a["path"]
                    if p.get("model") != "unicycle" or not {"linear", "angular", "duration"} <= set(p):
                        raise BindingError(f"[actions.{name}].path: model = \"unicycle\" with linear, angular, duration")
                    spec["path"] = p
            if "call" in a:
                spec["call"] = _call_spec(a["call"], f"[actions.{name}].call")
                spec["duration_s"] = a.get("duration_s", 0.0)
            if ("start" in spec) == ("call" in spec):
                raise BindingError(f"[actions.{name}]: give either start (motion) or call (instant/oneshot)")
            self.actions[name] = spec
        st = binding.get("status")
        self.status_call = _call_spec(st["call"], "[status].call") if st else None
        self.status_fields = {}
        for fname, f in (st or {}).get("fields", {}).items():
            if f.get("type") not in FIELD_TYPES or not isinstance(f.get("path"), str):
                raise BindingError(f"[status].fields.{fname}: needs path and type in {list(FIELD_TYPES)}")
            self.status_fields[fname] = f
        self.pose_map = binding.get("pose")
        if self.pose_map and not all(self.pose_map.get(k) in self.status_fields for k in ("x", "y", "heading")):
            raise BindingError("[pose]: x, y and heading must name [status].fields")
        self._started = False
        self._sent_errors: list = []
        self._sent_lock = threading.Lock()

    # -------------------------------------------------------------- plumbing
    def _call(self, spec, params=None, on_sent=None) -> dict:
        args = fill(spec["args"], params or {})
        try:
            res = (self.caller.call_tool(spec["tool"], args) if on_sent is None else
                   self.caller.call_tool(spec["tool"], args, on_sent=on_sent))
        except ToolError as e:
            raise BackendError(f"{spec['tool']}: {e}") from None
        if isinstance(res, dict):
            for k in self.error_keys:
                if res.get(k):
                    raise BackendError(f"{spec['tool']}: {str(res[k])[:200]}")
        return res if isinstance(res, dict) else {"result": res}

    def _send(self, spec, params, wait_s: float = 2.0):
        """start_mode = "send": put the call on the wire and return without its reply. The connector still gets
        every call in order. A failed reply is kept and raised at the next start or tick, so BotInfo stops."""
        self._raise_sent_errors()
        sent = threading.Event()

        def go():
            try:
                self._call(spec, params, on_sent=sent.set)
            except BackendError as e:
                with self._sent_lock:
                    self._sent_errors.append(e)
            finally:
                sent.set()
        threading.Thread(target=go, name="botinfo-send", daemon=True).start()
        sent.wait(wait_s)
        self._raise_sent_errors()

    def _raise_sent_errors(self):
        with self._sent_lock:
            errs, self._sent_errors = self._sent_errors, []
        if errs:
            raise errs[0]

    def start(self):
        if not self._started:
            for c in self.init_calls:
                self._call(c)
            self._started = True

    def check_policy(self, policy) -> list:
        problems = []
        for a in policy.actions.values():
            spec = self.actions.get(a.name)
            if spec is None:
                problems.append(f"{a.name}: no mapping in the binding")
                continue
            if a.kind == "motion" and "start" not in spec:
                problems.append(f"{a.name}: a motion needs start (and keepalive) in the binding")
            if a.kind != "motion" and "call" not in spec:
                problems.append(f"{a.name}: a {a.kind} action needs call in the binding")
            used = _placeholders(spec, set())
            unknown = used - set(a.params)
            if unknown:
                problems.append(f"{a.name}: binding uses {sorted(unknown)}, not parameters of this action")
        extra = set(self.actions) - set(policy.actions)
        if extra:
            problems.append(f"warning: binding maps actions the policy does not define: {sorted(extra)} (unused)")
        if policy.workspace and not (self.pose_map and self.status_call):
            problems.append("the policy has a workspace but the binding gives no [pose]")
        return problems

    def describe(self) -> dict:
        return {"backend": self.name, "connector": self.label, "binding": self.source}

    # -------------------------------------------------------------- Backend
    def start_motion(self, action, params):
        self.start()
        spec = self.actions[action.name]
        if spec.get("start_mode") == "send":
            self._send(spec["start"], params)
        else:
            self._call(spec["start"], params)

    def keepalive(self, action, params):
        spec = self.actions[action.name]
        if spec.get("keepalive") == "start":
            self._call(spec["start"], params)

    def stop(self):
        err = None
        for _ in range(self.stop_repeat):
            try:
                self._call(self.stop_call)
            except BackendError as e:
                err = e
        if err:
            raise err

    def execute(self, action, params) -> dict:
        self.start()
        self._call(self.actions[action.name]["call"], params)
        return {"sent": action.name}

    def oneshot_duration(self, action, params) -> float:
        d = self.actions[action.name].get("duration_s", 0.0)
        return float(fill(d, params)) if isinstance(d, (str, int, float)) else 0.0

    def status(self) -> dict:
        if not self.status_call:
            return {}
        self.start()
        res = self._call(self.status_call)
        out = {}
        for name, f in self.status_fields.items():
            v = dig(res, f["path"])
            if f["type"] == "yaw_from_quaternion":
                v = _yaw(v)
            out[name] = v
        return out

    def status_schema(self) -> dict:
        sch = {}
        for name, f in self.status_fields.items():
            t = "number" if f["type"] == "yaw_from_quaternion" else f["type"]
            sch[name] = {"type": t, **({"values": tuple(f.get("values", ()))} if t == "enum" else {}),
                         **({"max_len": int(f.get("max_len", 32))} if t == "token" else {})}
        return sch

    def pose(self):
        if not self.pose_map:
            return None
        s = self.status()
        try:
            p = tuple(float(s[self.pose_map[k]]) for k in ("x", "y", "heading"))
        except (TypeError, ValueError, KeyError):
            return None
        return p if all(math.isfinite(v) for v in p) else None

    def predict_path(self, action, params, pose):
        spec = self.actions.get(action.name, {})
        p = spec.get("path")
        if not p or pose is None:
            return None
        v, w, d = (float(fill(p[k], params)) for k in ("linear", "angular", "duration"))
        return unicycle_path(pose, v, w, d)


    def tick(self):
        self._raise_sent_errors()
        t = getattr(self.caller, "tick", None)
        if t:
            t()

    def close(self):
        c = getattr(self.caller, "close", None)
        if c:
            c()


def unicycle_path(pose, v, w, duration, step=0.05) -> list:
    x, y, th = pose
    pts = [(x, y)]
    t = 0.0
    while t < duration - 1e-9:
        dt = min(step, duration - t)
        x += v * math.cos(th) * dt
        y += v * math.sin(th) * dt
        th += w * dt
        t += dt
        pts.append((x, y))
    return pts


def _yaw(q):
    if not isinstance(q, dict):
        return None
    try:
        x, y, z, w = (float(q.get(k, 0.0)) for k in ("x", "y", "z", "w"))
    except (TypeError, ValueError):
        return None
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
