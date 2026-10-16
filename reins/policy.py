"""The owner's policy: which actions exist, their parameters and bounds, and the limits around them.

A policy is a TOML (or JSON) file. Loading is strict: an unknown key, a missing bound or a wrong type is an
error, so a typo cannot silently switch a limit off. See policies/*.toml for examples and docs/POLICY.md for
every key.

Checking a call is pure (no clock, no robot): check_params() returns the cleaned parameters or the reason they
are refused. Values out of bounds are REFUSED, never clamped: clamping would run something the caller did not ask
for. The stateful checks (rate, expiry, replay, stop latch) live in core.py.
"""
from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass, field
from pathlib import Path

try:
    import tomllib
except ImportError:  # Python < 3.11
    tomllib = None

KINDS = ("motion", "oneshot", "instant")
PARAM_TYPES = ("number", "integer", "enum", "boolean")
RESERVED = ("request_id", "issued_at")                    # meta fields any action call may carry
NAME_RE = re.compile(r"^[a-z][a-z0-9_]{0,47}$")
REQUEST_ID_RE = re.compile(r"^[A-Za-z0-9._:-]{1,128}$")
TOOL_PREFIX = "reins_"                                    # reserved for Reins' own tools


class PolicyError(ValueError):
    pass


@dataclass(frozen=True)
class Rate:
    max: int
    window_s: float


@dataclass(frozen=True)
class Param:
    name: str
    type: str
    min: float | None = None
    max: float | None = None
    values: tuple = ()
    default: object = None
    has_default: bool = False
    confirm_above: float | None = None        # |value| above this needs the owner's confirmation
    confirm_values: tuple = ()                # these enum values need the owner's confirmation
    unit: str = ""
    description: str = ""

    def json_schema(self) -> dict:
        s: dict = {}
        if self.type == "number":
            s = {"type": "number", "minimum": self.min, "maximum": self.max}
        elif self.type == "integer":
            s = {"type": "integer", "minimum": int(self.min), "maximum": int(self.max)}
        elif self.type == "enum":
            s = {"type": "string", "enum": list(self.values)}
        elif self.type == "boolean":
            s = {"type": "boolean"}
        desc = self.description
        if self.unit:
            desc = (desc + " " if desc else "") + f"({self.unit})"
        if self.confirm_above is not None:
            desc += f" Above {self.confirm_above:g} needs the owner's confirmation."
        if self.confirm_values:
            desc += f" {', '.join(self.confirm_values)} need the owner's confirmation."
        if desc.strip():
            s["description"] = desc.strip()
        if self.has_default:
            s["default"] = self.default
        return s


@dataclass(frozen=True)
class Action:
    name: str
    kind: str
    description: str
    params: dict
    confirm: bool = False
    uninterruptible: bool = False             # a stop cannot cut it short once it starts (e.g. a Sesame pose)
    duration_param: str | None = None         # motion: which parameter is the lease length in seconds
    rate: Rate | None = None


@dataclass(frozen=True)
class Workspace:
    x: tuple
    y: tuple
    margin_m: float
    check_every_s: float


@dataclass(frozen=True)
class Policy:
    name: str
    robot: str
    description: str
    actions: dict
    command_ttl_s: float = 5.0
    max_clock_skew_s: float = 2.0
    require_request_id: bool = False
    on_busy: str = "replace"
    stop_latch: bool = True
    on_client_disconnect: str = "stop"
    on_backend_error: str = "stop"
    keepalive_s: float = 0.5
    max_lease_s: float = 5.0
    renewable: bool = False
    max_total_s: float = 30.0                # longest stretch of motion: renewals AND back-to-back commands
    min_rest_s: float = 1.0                  # rest needed before a new stretch of motion starts
    rate_all: Rate = field(default_factory=lambda: Rate(30, 10.0))
    rate_motion: Rate = field(default_factory=lambda: Rate(5, 10.0))
    workspace: Workspace | None = None
    source: str = ""

    def summary(self) -> dict:
        """What the agent may know about the policy (reins_policy tool). No secrets live here."""
        return {
            "policy": self.name, "robot": self.robot, "description": self.description,
            "actions": {a.name: {"kind": a.kind, "description": a.description,
                                 "confirm": a.confirm, "uninterruptible": a.uninterruptible,
                                 "params": {p.name: p.json_schema() for p in a.params.values()}}
                        for a in self.actions.values()},
            "limits": {"command_ttl_s": self.command_ttl_s, "max_lease_s": self.max_lease_s,
                       "renewable": self.renewable, "max_total_s": self.max_total_s,
                       "min_rest_s": self.min_rest_s,
                       "rate_all": [self.rate_all.max, self.rate_all.window_s],
                       "rate_motion": [self.rate_motion.max, self.rate_motion.window_s]},
            "stop": {"latch": self.stop_latch,
                     "note": "After a stop, motion stays refused until the owner resumes. "
                             "The agent cannot resume, confirm, or change this policy."},
            "workspace": None if self.workspace is None else
            {"x": list(self.workspace.x), "y": list(self.workspace.y), "margin_m": self.workspace.margin_m},
        }


# ------------------------------------------------------------------ loading
def _take(d: dict, where: str, allowed: dict) -> dict:
    """Strict key check. allowed = {key: (types, required)}."""
    if not isinstance(d, dict):
        raise PolicyError(f"{where}: expected a table")
    unknown = sorted(set(d) - set(allowed))
    if unknown:
        raise PolicyError(f"{where}: unknown key(s) {unknown}; allowed: {sorted(allowed)}")
    out = {}
    for k, (types, required) in allowed.items():
        if k not in d:
            if required:
                raise PolicyError(f"{where}: missing required key {k!r}")
            continue
        v = d[k]
        types = types if isinstance(types, tuple) else (types,)
        if isinstance(v, bool) and bool not in types:
            raise PolicyError(f"{where}.{k}: expected {_tn(types)}, got a boolean")
        if not isinstance(v, types):
            raise PolicyError(f"{where}.{k}: expected {_tn(types)}, got {type(v).__name__}")
        if isinstance(v, float) and not math.isfinite(v):
            raise PolicyError(f"{where}.{k}: must be finite")
        out[k] = v
    return out


def _tn(types) -> str:
    return " or ".join(t.__name__ for t in (types if isinstance(types, tuple) else (types,)))


NUM = (int, float)


def _positive(where, v, allow_zero=False):
    if v < 0 or (v == 0 and not allow_zero):
        raise PolicyError(f"{where}: must be {'>= 0' if allow_zero else '> 0'}, got {v}")
    return float(v)


def _rate(d, where) -> Rate:
    r = _take(d, where, {"max": (int, True), "window_s": (NUM, True)})
    if r["max"] < 1:
        raise PolicyError(f"{where}.max: must be >= 1")
    return Rate(r["max"], _positive(f"{where}.window_s", r["window_s"]))


def _param(name, d, where) -> Param:
    if not NAME_RE.match(name) or name in RESERVED:
        raise PolicyError(f"{where}: bad parameter name {name!r}")
    p = _take(d, where, {"type": (str, True), "min": (NUM, False), "max": (NUM, False), "values": (list, False),
                         "default": ((int, float, str, bool), False), "confirm_above": (NUM, False),
                         "confirm_values": (list, False), "unit": (str, False), "description": (str, False)})
    t = p["type"]
    if t not in PARAM_TYPES:
        raise PolicyError(f"{where}.type: {t!r} is not one of {list(PARAM_TYPES)}")
    kw = {"name": name, "type": t, "unit": p.get("unit", ""), "description": p.get("description", "")}
    if t in ("number", "integer"):
        if "min" not in p or "max" not in p:
            raise PolicyError(f"{where}: a {t} parameter needs both min and max")
        lo, hi = p["min"], p["max"]
        if t == "integer" and (not isinstance(lo, int) or not isinstance(hi, int)):
            raise PolicyError(f"{where}: integer bounds must be whole numbers")
        if lo > hi:
            raise PolicyError(f"{where}: min {lo} is above max {hi}")
        kw.update(min=lo, max=hi)
        if "confirm_above" in p:
            kw["confirm_above"] = _positive(f"{where}.confirm_above", p["confirm_above"], allow_zero=True)
    elif "min" in p or "max" in p or "confirm_above" in p:
        raise PolicyError(f"{where}: min/max/confirm_above only apply to number and integer parameters")
    if t == "enum":
        vals = p.get("values")
        if not vals or not all(isinstance(x, str) and x for x in vals) or len(set(vals)) != len(vals):
            raise PolicyError(f"{where}.values: an enum needs a list of distinct non-empty strings")
        kw["values"] = tuple(vals)
        cv = p.get("confirm_values", [])
        if not all(x in vals for x in cv):
            raise PolicyError(f"{where}.confirm_values: every entry must be one of values")
        kw["confirm_values"] = tuple(cv)
    elif "values" in p or "confirm_values" in p:
        raise PolicyError(f"{where}: values/confirm_values only apply to enum parameters")
    if "default" in p:
        dv = p["default"]
        cleaned, why = check_value(Param(**kw), dv)
        if why:
            raise PolicyError(f"{where}.default: {why}")
        kw.update(default=cleaned, has_default=True)
    return Param(**kw)


def _action(name, d, where) -> Action:
    if not NAME_RE.match(name) or name.startswith(TOOL_PREFIX):
        raise PolicyError(f"{where}: bad action name {name!r} (lowercase, digits, _; not starting with reins_)")
    a = _take(d, where, {"kind": (str, True), "description": (str, False), "params": (dict, False),
                         "confirm": (bool, False), "uninterruptible": (bool, False),
                         "duration_param": (str, False), "rate": (dict, False)})
    if a["kind"] not in KINDS:
        raise PolicyError(f"{where}.kind: {a['kind']!r} is not one of {list(KINDS)}")
    params = {pn: _param(pn, pd, f"{where}.params.{pn}") for pn, pd in a.get("params", {}).items()}
    dp = a.get("duration_param")
    if dp is not None:
        if a["kind"] != "motion":
            raise PolicyError(f"{where}.duration_param: only motion actions have one")
        if dp not in params or params[dp].type != "number" or params[dp].min <= 0:
            raise PolicyError(f"{where}.duration_param: {dp!r} must be a number parameter with min > 0")
    if a.get("uninterruptible") and a["kind"] == "motion":
        raise PolicyError(f"{where}: a motion runs on a lease and is interruptible by design; "
                          "mark one-shot moves as kind = \"oneshot\"")
    return Action(name=name, kind=a["kind"], description=a.get("description", ""), params=params,
                  confirm=a.get("confirm", False), uninterruptible=a.get("uninterruptible", False),
                  duration_param=dp, rate=_rate(a["rate"], f"{where}.rate") if "rate" in a else None)


def policy_from_dict(d: dict, source: str = "") -> Policy:
    top = _take(d, "policy file", {"policy": (dict, True), "stop": (dict, False), "lease": (dict, False),
                                   "rate_limits": (dict, False), "workspace": (dict, False),
                                   "actions": (dict, True)})
    p = _take(top["policy"], "[policy]", {
        "name": (str, True), "robot": (str, True), "description": (str, False),
        "command_ttl_s": (NUM, False), "max_clock_skew_s": (NUM, False),
        "require_request_id": (bool, False), "on_busy": (str, False)})
    s = _take(top.get("stop", {}), "[stop]", {"latch": (bool, False), "on_client_disconnect": (str, False),
                                              "on_backend_error": (str, False)})
    lease = _take(top.get("lease", {}), "[lease]", {"keepalive_s": (NUM, False), "max_lease_s": (NUM, False),
                                                    "renewable": (bool, False), "max_total_s": (NUM, False),
                                                    "min_rest_s": (NUM, False)})
    rl = _take(top.get("rate_limits", {}), "[rate_limits]", {"all": (dict, False), "motion": (dict, False)})
    kw = {"name": p["name"], "robot": p["robot"], "description": p.get("description", ""), "source": source}
    if "command_ttl_s" in p:
        kw["command_ttl_s"] = _positive("[policy].command_ttl_s", p["command_ttl_s"])
    if "max_clock_skew_s" in p:
        kw["max_clock_skew_s"] = _positive("[policy].max_clock_skew_s", p["max_clock_skew_s"], allow_zero=True)
    if "require_request_id" in p:
        kw["require_request_id"] = p["require_request_id"]
    if "on_busy" in p:
        if p["on_busy"] not in ("replace", "reject"):
            raise PolicyError("[policy].on_busy: must be \"replace\" or \"reject\"")
        kw["on_busy"] = p["on_busy"]
    if "latch" in s:
        kw["stop_latch"] = s["latch"]
    for k in ("on_client_disconnect", "on_backend_error"):
        if k in s:
            if s[k] not in ("stop", "ignore"):
                raise PolicyError(f"[stop].{k}: must be \"stop\" or \"ignore\"")
            kw[k] = s[k]
    for k in ("keepalive_s", "max_lease_s", "max_total_s", "min_rest_s"):
        if k in lease:
            kw[k] = _positive(f"[lease].{k}", lease[k])
    if "renewable" in lease:
        kw["renewable"] = lease["renewable"]
    if "all" in rl:
        kw["rate_all"] = _rate(rl["all"], "[rate_limits].all")
    if "motion" in rl:
        kw["rate_motion"] = _rate(rl["motion"], "[rate_limits].motion")
    if "workspace" in top:
        w = _take(top["workspace"], "[workspace]", {"x": (list, True), "y": (list, True), "margin_m": (NUM, False),
                                                    "check_every_s": (NUM, False)})
        for ax in ("x", "y"):
            v = w[ax]
            if len(v) != 2 or not all(isinstance(z, NUM) and not isinstance(z, bool) for z in v) or v[0] >= v[1]:
                raise PolicyError(f"[workspace].{ax}: expected [low, high] with low < high")
        margin = _positive("[workspace].margin_m", w.get("margin_m", 0.0), allow_zero=True)
        if margin * 2 >= min(w["x"][1] - w["x"][0], w["y"][1] - w["y"][0]):
            raise PolicyError("[workspace].margin_m: leaves no room inside the workspace")
        kw["workspace"] = Workspace(tuple(map(float, w["x"])), tuple(map(float, w["y"])), margin,
                                    _positive("[workspace].check_every_s", w.get("check_every_s", 0.1)))
    actions = {n: _action(n, ad, f"[actions.{n}]") for n, ad in top["actions"].items()}
    if not actions:
        raise PolicyError("[actions]: a policy needs at least one action")
    pol = Policy(actions=actions, **kw)
    for a in actions.values():
        if a.kind == "motion" and a.duration_param:
            dmax = a.params[a.duration_param].max
            if dmax > pol.max_lease_s:
                raise PolicyError(f"[actions.{a.name}]: {a.duration_param} max {dmax} is above "
                                  f"[lease].max_lease_s {pol.max_lease_s}")
    if pol.max_total_s < pol.max_lease_s:
        raise PolicyError("[lease].max_total_s must be at least max_lease_s")
    if pol.keepalive_s >= pol.max_lease_s:
        raise PolicyError("[lease].keepalive_s must be shorter than max_lease_s")
    return pol


def load_policy(path) -> Policy:
    path = Path(path)
    text = path.read_text(encoding="utf-8")
    if path.suffix == ".json":
        d = json.loads(text)
    else:
        if tomllib is None:
            raise PolicyError("TOML policies need Python 3.11+ (tomllib); use a .json policy on older Pythons")
        d = tomllib.loads(text)
    return policy_from_dict(d, source=str(path))


# ------------------------------------------------------------------ checking calls
def check_value(p: Param, v) -> tuple:
    """(cleaned, None) or (None, why)."""
    if p.type == "boolean":
        return (v, None) if isinstance(v, bool) else (None, f"{p.name}: expected true or false, got {_show(v)}")
    if p.type == "enum":
        if isinstance(v, str) and v in p.values:
            return v, None
        return None, f"{p.name}: {_show(v)} is not one of {list(p.values)}"
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        return None, f"{p.name}: expected a {p.type}, got {_show(v)}"
    if isinstance(v, float) and not math.isfinite(v):
        return None, f"{p.name}: {v!r} is not a finite number"
    if p.type == "integer":
        if isinstance(v, float):
            if not v.is_integer():
                return None, f"{p.name}: {v!r} is not a whole number"
            v = int(v)
    if not p.min <= v <= p.max:
        return None, f"{p.name}: {v!r} is outside {p.min:g}..{p.max:g}" + (f" {p.unit}" if p.unit else "")
    return (float(v) if p.type == "number" else v), None


def _show(v) -> str:
    s = repr(v)
    return s if len(s) <= 60 else s[:57] + "..."


def check_params(action: Action, args: dict) -> tuple:
    """Exact fields, typed and bounded values -> (params, None) or (None, why). args has no reserved keys."""
    if not isinstance(args, dict):
        return None, "arguments must be an object"
    extra = sorted(str(k) for k in args if k not in action.params)
    if extra:
        return None, f"unknown field(s) {extra}; {action.name} takes {sorted(action.params) or 'no fields'}"
    out = {}
    for p in action.params.values():
        if p.name not in args:
            if p.has_default:
                out[p.name] = p.default
                continue
            return None, f"missing field {p.name!r}"
        v, why = check_value(p, args[p.name])
        if why:
            return None, why
        out[p.name] = v
    return out, None


def needs_confirmation(action: Action, params: dict) -> str | None:
    """Why this call needs the owner's OK, or None."""
    if action.confirm:
        return f"{action.name} always needs the owner's confirmation" + \
            (" (a stop cannot interrupt it once it starts)" if action.uninterruptible else "")
    for p in action.params.values():
        v = params.get(p.name)
        if p.confirm_above is not None and isinstance(v, (int, float)) and abs(v) > p.confirm_above:
            return f"{p.name} {v:g} is above {p.confirm_above:g}"
        if p.confirm_values and v in p.confirm_values:
            return f"{p.name} {v} needs the owner's confirmation"
    return None


def split_meta(args) -> tuple:
    """Separate request_id / issued_at from the action's own fields -> (args, meta, why)."""
    if not isinstance(args, dict):
        return None, {}, "arguments must be an object"
    args = dict(args)
    meta = {}
    if "request_id" in args:
        rid = args.pop("request_id")
        if not isinstance(rid, str) or not REQUEST_ID_RE.match(rid):
            return None, {}, "request_id must be 1-128 characters from A-Z a-z 0-9 . _ : -"
        meta["request_id"] = rid
    if "issued_at" in args:
        ia = args.pop("issued_at")
        if isinstance(ia, bool) or not isinstance(ia, (int, float)) or not math.isfinite(ia):
            return None, {}, "issued_at must be a finite number (Unix seconds)"
        meta["issued_at"] = float(ia)
    return args, meta, None
