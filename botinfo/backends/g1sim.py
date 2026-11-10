"""G1Sim: Unitree's published G1 walking policy in MuJoCo, behind a connector shaped like Unitree's G1 LocoClient.

With it an AI agent can walk a simulated Unitree G1 through BotInfo, and the owner can stop it, with no robot.

Needs numpy and mujoco (the self-check's expected numbers come from numpy 2.0.2 and mujoco 3.3.7) and a clone of
github.com/unitreerobotics/unitree_rl_gym (BSD-3-Clause; this file was checked against commit
3e70454dae9562febade2949c9ebc57440365dcb). Nothing is copied from it; these files are read from your clone:
  deploy/pre_train/g1/motion.pt                 the walking policy (TorchScript, read here without torch); a file
                                                whose sha256 is not PINNED_POLICY_SHA256 is refused
  deploy/deploy_mujoco/configs/g1.yaml          PD gains, default angles, observation scales
  deploy/deploy_real/common/command_helper.py   the kd of create_damping_cmd
  resources/robots/g1_description/scene.xml     the 12-joint G1 model (legs) and its meshes
The control loop is deploy_mujoco.py's: physics every simulation_dt, the policy every control_decimation steps,
a PD torque on each leg joint toward the policy's target, a gait phase with a 0.8 s period in the observation.

Tools (names after unitree_sdk2's G1 LocoClient, include/unitree/robot/g1/loco/g1_loco_client.hpp @69c04316):
  Move {vx, vy, vyaw}  walk at that velocity for 1 s, as Move() does without continous_move (SetVelocity with a
                       1 s duration): the robot stops walking by itself 1 s after the last Move
  StopMove {}          velocity zero (SetVelocity(0, 0, 0)); the walking policy keeps running and balancing
  Damp {}              the policy stops; kp 0, kd 8 on every joint (unitree_rl_gym's create_damping_cmd)
  ZeroTorque {}        the policy stops; no torque at all (create_zero_cmd)
  GetState {}          pose, speed, net travel over the last second, pelvis height, tilt, upright, mode
'upright' here is the pelvis above 0.6 m only; humanoid-stop-sim's 'upright' also needs a tilt under 30 degrees.
Damp and ZeroTorque here are unitree_rl_gym's low-level commands, not the robot's own FSM modes, whose behaviour
is not in public code. After either one this sim refuses Move (restart it to stand the robot again). In
github.com/botinfo-ai/humanoid-stop-sim, both put a knee of a walking G1 on the floor and left it not upright at
every gait moment tried, while velocity zero kept it upright: so bindings/g1sim-loco.toml binds BotInfo's stop to StopMove, and
bindings/g1sim-loco-damp.toml binds it to Damp so the difference can be seen.

Self-check (no BotInfo server, simulated time):  python3 -m botinfo.backends.g1sim PATH_TO_unitree_rl_gym [OUT_DIR]
A run is what one model did in simulation: data about the model, never about a real robot.
"""
from __future__ import annotations

import hashlib
import io
import json
import math
import os
import pickle
import re
import sys
import threading
import time
import zipfile
from collections import OrderedDict, deque
from pathlib import Path

try:
    from .base import ToolError
except ImportError:          # run as one file (python3 g1sim.py ...): the self-check needs no other BotInfo file
    class ToolError(Exception):
        pass

PINNED_COMMIT = "3e70454dae9562febade2949c9ebc57440365dcb"
PINNED_POLICY_SHA256 = "cf668f75b90d1abf73d2b87612a6e76bccc61ff7e083b63582d3f6aaa3c1759d"
FILES = {"policy": "deploy/pre_train/g1/motion.pt", "config": "deploy/deploy_mujoco/configs/g1.yaml",
         "helper": "deploy/deploy_real/common/command_helper.py",
         "scene": "resources/robots/g1_description/scene.xml"}
MOVE_S = 1.0          # Move() without continous_move: SetVelocity(vx, vy, omega, 1.0)
GAIT_S = 0.8          # deploy_mujoco.py's gait phase period
UPRIGHT_Z = 0.6       # pelvis higher than this = upright (standing ~0.78 m; down ~0.06-0.31 m in humanoid-stop-sim,
                      # whose 'upright' also needs pelvis tilt under 30 degrees; this one reads the height only)
REC_HZ = 25           # the sim's own record: one row every 1/25 s
MODES = ("walking", "damping", "zero_torque")


class SetupError(ValueError):
    """The G1 sim cannot start (a missing package, a missing or unreadable file, or not the pinned policy file)."""


def _deps():
    try:
        import mujoco
        import numpy as np
    except ImportError as e:
        raise SetupError(f"the G1 sim needs numpy and mujoco (pip install numpy mujoco): {e}") from None
    return np, mujoco


def find_unitree_rl_gym(path=None) -> Path:
    """The unitree_rl_gym clone: the binding's unitree_rl_gym, else $BOTINFO_UNITREE_RL_GYM."""
    p = path or os.environ.get("BOTINFO_UNITREE_RL_GYM")
    if not p:
        raise SetupError("the G1 sim needs a clone of github.com/unitreerobotics/unitree_rl_gym: set "
                         "unitree_rl_gym = \"/path\" in the binding, or BOTINFO_UNITREE_RL_GYM")
    root = Path(p).expanduser()
    missing = [f for f in FILES.values() if not (root / f).is_file()]
    if missing:
        raise SetupError(f"{root} is not a unitree_rl_gym clone: missing {missing}")
    return root


def _config(text: str) -> dict:
    """g1.yaml's numbers, read by pattern (no yaml package needed)."""
    def lst(key):
        m = re.search(rf"^{key}:\s*\[([^\]]*)\]", text, re.M | re.S)
        if not m:
            raise SetupError(f"g1.yaml: no list {key!r}")
        return [float(v) for v in m.group(1).replace("\n", " ").split(",") if v.strip()]

    def num(key):
        m = re.search(rf"^{key}:\s*([-0-9.eE]+)", text, re.M)
        if not m:
            raise SetupError(f"g1.yaml: no number {key!r}")
        return float(m.group(1))
    c = {k: lst(k) for k in ("kps", "kds", "default_angles", "cmd_scale")}
    c.update({k: num(k) for k in ("simulation_dt", "control_decimation", "ang_vel_scale", "dof_pos_scale",
                                  "dof_vel_scale", "action_scale", "num_actions", "num_obs")})
    return c


# ------------------------------------------------------------------ TorchScript weights without torch
class _TObj:
    """Stands in for any class the pickle names (TorchScript modules, torch helpers)."""
    def __new__(cls, *a, **k):
        o = object.__new__(cls)
        o.state = None
        return o

    def __init__(self, *a, **k):
        pass

    def __setstate__(self, state):
        self.state = state


class _Storage:
    def __init__(self, name):
        self.name = name


def _read_torchscript(path, np) -> dict:
    """{dotted name: numpy array} for every tensor in a TorchScript archive. The unpickler resolves only plain
    containers and tensor rebuilds, so nothing in the file can call real code, and every tensor must lie inside its
    storage. A file that cannot be read this way raises SetupError (a clean error in botinfo serve/check)."""
    try:
        return _unpickle_tensors(path, np)
    except SetupError:
        raise
    except Exception as e:                                  # a damaged or foreign file: an error, not a traceback
        raise SetupError(f"{path}: cannot read it as the walking policy ({type(e).__name__}: {e})") from None


def _unpickle_tensors(path, np) -> dict:
    z = zipfile.ZipFile(path)                               # first, so a file that is not a zip fails here
    dtypes = {"FloatStorage": np.float32, "DoubleStorage": np.float64, "HalfStorage": np.float16,
              "LongStorage": np.int64, "IntStorage": np.int32, "ShortStorage": np.int16, "CharStorage": np.int8,
              "ByteStorage": np.uint8, "BoolStorage": np.bool_}

    def rebuild(storage, offset, size, stride, *rest):
        raw, dt = storage
        base = np.frombuffer(raw, dtype=dt)
        offset, size, stride = int(offset), tuple(int(n) for n in size), tuple(int(s) for s in stride)
        if offset < 0 or len(size) != len(stride) or any(v < 0 for v in size + stride):
            raise SetupError(f"{path}: a tensor has a malformed layout")
        if 0 not in size and offset + sum((n - 1) * s for n, s in zip(size, stride)) >= base.size:
            raise SetupError(f"{path}: a tensor reaches past the end of its storage")
        if len(size) == 0:
            return np.array(base[offset])
        item = np.dtype(dt).itemsize
        return np.array(np.lib.stride_tricks.as_strided(base[offset:], shape=size,
                                                        strides=tuple(s * item for s in stride)))

    pkl = [n for n in z.namelist() if n.endswith("/data.pkl") and n.count("/") == 1]
    if not pkl:
        raise SetupError(f"{path}: not a TorchScript archive")
    root = pkl[0].split("/")[0]
    plain = {("collections", "OrderedDict"): OrderedDict, ("builtins", "set"): set,
             ("builtins", "frozenset"): frozenset, ("_codecs", "encode"): lambda s, enc="latin1": s.encode(enc)}

    class Unpickler(pickle.Unpickler):
        def find_class(self, mod, name):
            if (mod, name) in plain:
                return plain[(mod, name)]
            if mod == "torch._utils" and name in ("_rebuild_tensor_v2", "_rebuild_tensor"):
                return rebuild
            if mod == "torch._utils" and name == "_rebuild_parameter":
                return lambda data, *a: data
            if mod == "torch" and name.endswith("Storage"):
                return _Storage(name)
            if mod.startswith("torch"):
                return lambda *a, **k: a[0] if a else None
            return type(name, (_TObj,), {})

        def persistent_load(self, pid):
            stype, key = pid[1], pid[2]
            name = stype.name if isinstance(stype, _Storage) else str(stype)
            return z.read(f"{root}/data/{key}"), dtypes.get(name, np.float32)

    out, seen = {}, set()

    def walk(o, p):
        if isinstance(o, np.ndarray):
            out[p] = o
            return
        if id(o) in seen:
            return
        seen.add(id(o))
        if isinstance(o, _TObj):
            walk(o.state, p)
        elif isinstance(o, dict):
            for k, v in o.items():
                walk(v, f"{p}.{k}" if p else str(k))
        elif isinstance(o, (list, tuple)):
            for i, v in enumerate(o):
                walk(v, f"{p}[{i}]")
    walk(Unpickler(io.BytesIO(z.read(pkl[0]))).load(), "")
    return out


class WalkPolicy:
    """Unitree's G1 walking network (an LSTM, then an MLP actor with ELU), evaluated in numpy."""

    def __init__(self, path, np, num_obs: int, num_actions: int):
        self.np = np
        t = _read_torchscript(path, np)

        def pick(suffix):
            hits = [k for k in t if k.endswith(suffix) and "[" not in k]
            return t[hits[0]].astype(np.float64) if hits else None
        self.w_ih, self.w_hh, self.b_ih, self.b_hh = (pick(s) for s in ("weight_ih_l0", "weight_hh_l0",
                                                                         "bias_ih_l0", "bias_hh_l0"))
        lin = [(k, v) for k, v in t.items() if re.search(r"(^|\.)actor\.\d+\.weight$", k)] or \
              [(k, v) for k, v in t.items() if re.fullmatch(r"\d+\.weight", k)]
        lin.sort(key=lambda kv: int(kv[0].split(".")[-2]))
        self.actor = [(w.astype(np.float64), t[k[:-len("weight")] + "bias"].astype(np.float64)) for k, w in lin]
        self.recurrent = self.w_ih is not None
        if not self.actor or self.actor[-1][0].shape[0] != num_actions:
            raise SetupError("the policy's output size does not match num_actions in g1.yaml")
        if (self.w_ih.shape[1] if self.recurrent else self.actor[0][0].shape[1]) != num_obs:
            raise SetupError("the policy's input size does not match num_obs in g1.yaml")
        self.h0, self.c0 = pick("hidden_state"), pick("cell_state")    # the state torch.jit.load starts from
        self.reset()

    def reset(self):
        np = self.np
        n = self.w_hh.shape[1] if self.recurrent else 0
        self.h = self.h0.reshape(-1).copy() if self.h0 is not None and self.h0.size == n else np.zeros(n)
        self.c = self.c0.reshape(-1).copy() if self.c0 is not None and self.c0.size == n else np.zeros(n)

    def __call__(self, obs):
        np = self.np
        x = obs.astype(np.float64)
        if self.recurrent:
            g = self.w_ih @ x + self.b_ih + self.w_hh @ self.h + self.b_hh
            i, f, gg, o = np.split(g, 4)                        # PyTorch gate order: input, forget, cell, output
            sig = lambda v: 1.0 / (1.0 + np.exp(-v))           # noqa: E731
            self.c = sig(f) * self.c + sig(i) * np.tanh(gg)
            self.h = sig(o) * np.tanh(self.c)
            x = self.h
        for j, (w, b) in enumerate(self.actor):
            x = w @ x + b
            if j < len(self.actor) - 1:
                x = np.where(x > 0, x, np.expm1(np.minimum(x, 0)))    # ELU, alpha 1
        return x.astype(np.float32)


# ------------------------------------------------------------------ the simulated robot
class G1Sim:
    """The 12-joint G1 under Unitree's walking policy, stepped up to clock.now() whenever it is touched."""

    def __init__(self, clock, root, viewer: bool = False):
        np, mujoco = _deps()
        self.np, self.mj, self.clock = np, mujoco, clock
        root = Path(root)
        self.cfg = c = _config((root / FILES["config"]).read_text(encoding="utf-8"))
        helper = (root / FILES["helper"]).read_text(encoding="utf-8")
        try:
            block = helper[helper.index("def create_damping_cmd"):helper.index("def create_zero_cmd")]
            self.kd_damp = float(re.search(r"\.kd\s*=\s*([0-9.]+)", block).group(1))
        except (ValueError, AttributeError):
            raise SetupError("command_helper.py: cannot read create_damping_cmd's kd") from None
        self.policy_sha256 = hashlib.sha256((root / FILES["policy"]).read_bytes()).hexdigest()
        if self.policy_sha256 != PINNED_POLICY_SHA256:
            raise SetupError(f"{root / FILES['policy']} has sha256 {self.policy_sha256}, not the {PINNED_POLICY_SHA256} "
                             f"of the motion.pt this backend was checked against: git -C {root} checkout {PINNED_COMMIT}")
        self.na, self.nobs = int(c["num_actions"]), int(c["num_obs"])
        self.net = WalkPolicy(root / FILES["policy"], np, self.nobs, self.na)
        self.m = mujoco.MjModel.from_xml_path(str(root / FILES["scene"]))
        self.m.opt.timestep = c["simulation_dt"]
        if self.m.nu != self.na:
            raise SetupError(f"the model has {self.m.nu} actuators, the policy {self.na} actions")
        self.dt, self.dec = self.m.opt.timestep, int(c["control_decimation"])
        self.rec_every = max(1, int(round(1.0 / (REC_HZ * self.dt))))
        self.kp, self.kd = np.array(c["kps"]), np.array(c["kds"])
        self.default = np.array(c["default_angles"], dtype=np.float32)
        self.cmd_scale = np.array(c["cmd_scale"], dtype=np.float32)
        self.lock = threading.RLock()
        self.d = mujoco.MjData(self.m)                      # the model's own initial pose, as deploy_mujoco.py
        self.t0 = clock.now()
        self.k = 0                                          # physics steps taken
        self.mode = "walking"
        self.cmd = np.zeros(3, dtype=np.float32)            # vx, vy, vyaw
        self.cmd_until = 0.0                                # sim time the current Move runs out
        self.action = np.zeros(self.na, dtype=np.float32)
        self.target = self.default.copy()
        self.obs = np.zeros(self.nobs, dtype=np.float32)
        self.track = deque(maxlen=2 * REC_HZ + 1)           # (t, x, y): net travel over the last second
        self.samples = deque(maxlen=600 * REC_HZ)           # the sim's own record, last 10 min
        self.received: list = []                            # every command that reached the robot
        self.fell_at = None
        self.compute_s = 0.0                                # wall time spent stepping (real-time check)
        self._record()
        self.viewer, self.viewer_error = None, None
        if viewer:
            try:
                import mujoco.viewer
                self.viewer = mujoco.viewer.launch_passive(self.m, self.d)
            except Exception as e:                          # no display, or macOS outside mjpython
                self.viewer_error = f"{type(e).__name__}: {e}"

    def now(self) -> float:
        return self.k * self.dt

    # -------------------------------------------------------------- physics
    def step(self):
        with self.lock:
            n = int((self.clock.now() - self.t0) / self.dt + 1e-9) - self.k
            if n <= 0:
                return
            w = time.perf_counter()
            for _ in range(n):
                self._physics_step()
            self.compute_s += time.perf_counter() - w
            if self.viewer is not None and self.viewer.is_running():
                self.viewer.sync()

    def _physics_step(self):
        d = self.d
        q, dq = d.qpos[7:], d.qvel[6:]
        if self.mode == "walking":
            d.ctrl[:] = (self.target - q) * self.kp - dq * self.kd
        elif self.mode == "damping":
            d.ctrl[:] = -self.kd_damp * dq
        else:
            d.ctrl[:] = 0.0
        self.mj.mj_step(self.m, d)
        self.k += 1
        t = self.now()
        if t >= self.cmd_until - 1e-9 and self.cmd.any():
            self.cmd[:] = 0.0                               # the Move ran out: velocity zero, still balancing
        if self.mode == "walking" and self.k % self.dec == 0:
            self._policy_step(t)
        if self.k % self.rec_every == 0:
            self._record()

    def _policy_step(self, t):
        d, c, o, na = self.d, self.cfg, self.obs, self.na
        phase = (t % GAIT_S) / GAIT_S
        qw, qx, qy, qz = d.qpos[3:7]
        o[:3] = d.qvel[3:6] * c["ang_vel_scale"]
        o[3:6] = (2 * (-qz * qx + qw * qy), -2 * (qz * qy + qw * qx), 1 - 2 * (qw * qw + qz * qz))
        o[6:9] = self.cmd * self.cmd_scale
        o[9:9 + na] = (d.qpos[7:] - self.default) * c["dof_pos_scale"]
        o[9 + na:9 + 2 * na] = d.qvel[6:] * c["dof_vel_scale"]
        o[9 + 2 * na:9 + 3 * na] = self.action
        o[9 + 3 * na:9 + 3 * na + 2] = (math.sin(2 * math.pi * phase), math.cos(2 * math.pi * phase))
        self.action = self.net(o)
        self.target = self.action * c["action_scale"] + self.default

    def _record(self):
        d = self.d
        t = round(self.now(), 4)
        x, y, z = (float(v) for v in d.qpos[:3])
        if z <= UPRIGHT_Z and self.fell_at is None:
            self.fell_at = t
        self.track.append((t, x, y))
        self.samples.append((t, x, y, z, float(math.hypot(d.qvel[0], d.qvel[1])), self.mode,
                             *(float(v) for v in self.cmd)))

    # -------------------------------------------------------------- commands (from the connector)
    def _received(self, name, payload):
        self.received.append({"t": round(self.now(), 4), "name": name, "payload": payload})

    def set_velocity(self, vx: float, vy: float, vyaw: float, duration: float):
        with self.lock:
            self.step()
            self._received("SetVelocity", [vx, vy, vyaw, duration])
            if self.mode != "walking":
                raise ToolError(f"the walking policy is off ({self.mode}); restart the sim to stand the robot again")
            self.cmd[:] = (vx, vy, vyaw)
            self.cmd_until = self.now() + duration

    def stop_move(self):
        with self.lock:
            self.step()
            self._received("StopMove", None)
            self.cmd[:] = 0.0
            self.cmd_until = self.now()

    def set_mode(self, mode: str):
        with self.lock:
            self.step()
            self._received(mode, None)
            self.mode = mode
            self.cmd[:] = 0.0

    def state(self) -> dict:
        with self.lock:
            self.step()
            d, t = self.d, self.now()
            x, y, z = (float(v) for v in d.qpos[:3])
            qw, qx, qy, qz = (float(v) for v in d.qpos[3:7])
            old = next((p for p in self.track if p[0] >= t - 1.0 - 1e-6), self.track[0])
            span = t - old[0]
            travel = math.hypot(x - old[1], y - old[2]) / span if span > 1e-6 else 0.0
            return {"t": round(t, 3), "x": x, "y": y, "z": z,
                    "yaw": math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)),
                    "tilt_deg": math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (qx * qx + qy * qy))))),
                    "speed_mps": float(math.hypot(d.qvel[0], d.qvel[1])), "travel_mps": travel,
                    "upright": z > UPRIGHT_Z, "mode": self.mode, "command": [float(v) for v in self.cmd],
                    "fell_at_s": self.fell_at}

    def close(self):
        with self.lock:
            if self.viewer is not None:
                self.viewer.close()
                self.viewer = None


# ------------------------------------------------------------------ the connector BotInfo talks to
def _schema(props: dict, required: list) -> dict:
    return {"type": "object", "properties": props, "required": required}


LOCO_TOOLS = [
    {"name": "Move", "description": "Walk at this velocity for 1 s (vx, vy in m/s; vyaw in rad/s).",
     "inputSchema": _schema({"vx": {"type": "number"}, "vy": {"type": "number"}, "vyaw": {"type": "number"}},
                            ["vx", "vy", "vyaw"])},
    {"name": "StopMove", "description": "Velocity zero; the walking policy keeps balancing.", "inputSchema": _schema({}, [])},
    {"name": "Damp", "description": "Walking policy off; joints damped (kp 0, kd 8): unitree_rl_gym's low-level "
     "create_damping_cmd, not LocoClient's Damp() FSM mode.", "inputSchema": _schema({}, [])},
    {"name": "ZeroTorque", "description": "Walking policy off; no joint torque: unitree_rl_gym's low-level "
     "create_zero_cmd, not LocoClient's ZeroTorque() FSM mode.", "inputSchema": _schema({}, [])},
    {"name": "GetState", "description": "Pose, speed, travel, pelvis height, tilt, upright (pelvis above 0.6 m), mode.",
     "inputSchema": _schema({}, [])},
]


class G1LocoConnector:
    label = "G1 sim (MuJoCo + Unitree's published walking policy), LocoClient-style tools"

    def __init__(self, robot):
        self.robot = robot
        self.closed = False

    def list_tools(self) -> list:
        return [dict(t) for t in LOCO_TOOLS]

    def call_tool(self, name: str, args: dict) -> dict:
        if self.closed:
            raise ToolError("connector is closed")
        if not isinstance(args, dict):
            raise ToolError("arguments must be an object")
        if name == "Move":
            v = []
            for k in ("vx", "vy", "vyaw"):
                x = args.get(k, 0.0)
                if isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x):
                    raise ToolError(f"Move: {k} must be a finite number, not {x!r}")
                v.append(float(x))
            self.robot.set_velocity(*v, MOVE_S)
            return {"code": 0}
        if name == "StopMove":
            self.robot.stop_move()
            return {"code": 0}
        if name == "Damp":
            self.robot.set_mode("damping")
            return {"code": 0}
        if name == "ZeroTorque":
            self.robot.set_mode("zero_torque")
            return {"code": 0}
        if name == "GetState":
            return self.robot.state()
        raise ToolError(f"unknown tool {name!r}")

    def tick(self):
        self.robot.step()

    def close(self):
        self.closed = True
        self.robot.close()


# ------------------------------------------------------------------ self-check (simulated time, no server)
class _Clock:
    def __init__(self):
        self.t = 0.0

    def now(self) -> float:
        return self.t


# humanoid-stop-sim's single run (results/2026-11-08-stop_walk.md: walk at 0.5 m/s from t = 0, stop at 4.0 s):
# x walked, pelvis z 4 s after the stop, distance moved in those 4 s. The 'repro' rows must match them.
REPRO = {"StopMove": (1.756, 0.779, 0.199), "Damp": (1.756, 0.063, 1.005), "ZeroTorque": (1.756, 0.064, 0.544)}


def _scenarios():
    """(name, seconds standing first, walk velocity, walk seconds, what happens after the walk). Move is renewed
    every 0.2 s while walking (as BotInfo's lease keepalive does); 6 s follow the 'after' step."""
    return [("repro, StopMove", 0.0, (0.5, 0.0, 0.0), 4.0, "StopMove"),
            ("repro, Damp", 0.0, (0.5, 0.0, 0.0), 4.0, "Damp"),
            ("repro, ZeroTorque", 0.0, (0.5, 0.0, 0.0), 4.0, "ZeroTorque"),
            ("stand", 1.0, (0.0, 0.0, 0.0), 0.0, "nothing"),
            ("walk, StopMove", 1.0, (0.5, 0.0, 0.0), 4.0, "StopMove"),
            ("walk, Move runs out", 1.0, (0.5, 0.0, 0.0), 4.0, "nothing"),
            ("walk, Damp", 1.0, (0.5, 0.0, 0.0), 4.0, "Damp"),
            ("turn, StopMove", 1.0, (0.3, 0.0, 0.5), 4.0, "StopMove"),
            ("back, StopMove", 1.0, (-0.2, 0.0, 0.0), 4.0, "StopMove")]


def selfcheck(root, out=None) -> int:
    root = find_unitree_rl_gym(root)
    rows, ok = [], True
    for name, stand_s, vel, walk_s, after in _scenarios():
        clock = _Clock()
        sim = G1Sim(clock, root)
        conn = G1LocoConnector(sim)
        next_move = [0.0]

        def run(seconds, move=None):
            end = clock.t + seconds
            while clock.t < end - 1e-9:
                if move is not None and clock.t >= next_move[0] - 1e-9:
                    conn.call_tool("Move", {"vx": move[0], "vy": move[1], "vyaw": move[2]})
                    next_move[0] = clock.t + 0.2
                clock.t = round(clock.t + 0.02, 6)
                conn.tick()
        run(stand_s)
        s0 = conn.call_tool("GetState", {})
        if walk_s:
            next_move[0] = clock.t
            run(walk_s, vel)
        s1 = conn.call_tool("GetState", {})
        if after != "nothing":
            conn.call_tool(after, {})
        reads = {}
        for k in range(1, 7):
            run(1.0)
            reads[k] = conn.call_tool("GetState", {})
        end, four = reads[6], reads[4]
        rows.append({"scenario": name, "stand_s": stand_s, "move": list(vel), "walk_s": walk_s, "after": after,
                     "walked_m": round(math.hypot(s1["x"] - s0["x"], s1["y"] - s0["y"]), 3),
                     "walked_x_m": round(s1["x"] - s0["x"], 3),
                     "yaw_change_rad": round(s1["yaw"] - s0["yaw"], 3),
                     "pelvis_z_4s_m": round(four["z"], 3),
                     "moved_4s_m": round(math.hypot(four["x"] - s1["x"], four["y"] - s1["y"]), 3),
                     "upright_at_end": end["upright"], "fell_at_s": end["fell_at_s"],
                     "pelvis_z_end_m": round(end["z"], 3), "tilt_end_deg": round(end["tilt_deg"], 1),
                     "travel_mps_after": {k: round(r["travel_mps"], 3) for k, r in reads.items()},
                     "moved_after_m": round(math.hypot(end["x"] - s1["x"], end["y"] - s1["y"]), 3),
                     "sim_s": round(sim.now(), 3), "compute_s": round(sim.compute_s, 3),
                     "commands": len(sim.received)})
        expect_up = after in ("nothing", "StopMove")
        ok = ok and end["upright"] == expect_up
        if name.startswith("repro"):
            got = (rows[-1]["walked_x_m"], rows[-1]["pelvis_z_4s_m"], rows[-1]["moved_4s_m"])
            rows[-1]["repro_expected"], rows[-1]["repro_match"] = REPRO[after], got == REPRO[after]
            ok = ok and got == REPRO[after]
    np, mujoco = _deps()
    print("# G1Sim self-check (simulation; Unitree's published G1 walking policy in MuJoCo)")
    print(f"unitree_rl_gym: {root}\nmotion.pt sha256 {sim.policy_sha256} (the file this backend was checked against; "
          "G1Sim refuses any other)")
    print(f"numpy {np.__version__}, mujoco {mujoco.__version__} (the expected numbers come from numpy 2.0.2, "
          "mujoco 3.3.7; with other versions the repro rows may not match exactly)")
    print(f"timestep {sim.dt}, policy every {sim.dec} steps, damping kd {sim.kd_damp:g}, upright = pelvis above "
          f"{UPRIGHT_Z} m (humanoid-stop-sim's 'upright' also needs tilt under 30 deg: see the tilt column)")
    print("scenario | walked m | yaw change rad | pelvis z 4 s after m | moved in 4 s after m | upright at end (6 s) | "
          "first below upright s | pelvis z end m | tilt end deg | net travel m/s 1..6 s after | moved after m | "
          "compute s per sim s | repro (walked, z, moved) expected -> match")
    for r in rows:
        tr = ", ".join(f"{v:.3f}" for v in r["travel_mps_after"].values())
        rp = f"{r['repro_expected']} -> {r['repro_match']}" if "repro_expected" in r else "-"
        print(f"{r['scenario']} | {r['walked_m']} | {r['yaw_change_rad']} | {r['pelvis_z_4s_m']} | {r['moved_4s_m']} | "
              f"{r['upright_at_end']} | {r['fell_at_s']} | {r['pelvis_z_end_m']} | {r['tilt_end_deg']} | {tr} | "
              f"{r['moved_after_m']} | {r['compute_s'] / r['sim_s']:.3f} | {rp}")
    print("expected: upright after StopMove and after a Move runs out; down after Damp and ZeroTorque; repro rows equal "
          "humanoid-stop-sim's results/2026-11-08-stop_walk.md -> " + ("as expected" if ok else "NOT as expected"))
    if out:
        Path(out).mkdir(parents=True, exist_ok=True)
        (Path(out) / "g1sim_selfcheck.json").write_text(json.dumps({"rows": rows, "as_expected": ok,
                                                                     "policy_sha256": sim.policy_sha256,
                                                                     "numpy": np.__version__,
                                                                     "mujoco": mujoco.__version__}, indent=1))
    return 0 if ok else 1


def main(argv) -> int:
    if not argv:
        print("usage: python3 -m botinfo.backends.g1sim PATH_TO_unitree_rl_gym [OUT_DIR]", file=sys.stderr)
        return 2
    root = Path(argv[0])
    nested = root / "unitreerobotics" / "unitree_rl_gym"     # also accept a folder holding the clone as <owner>/<repo>
    try:
        return selfcheck(nested if nested.is_dir() else root, argv[1] if len(argv) > 1 else None)
    except SetupError as e:
        print(f"g1sim: {e}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
