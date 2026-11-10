"""The simulated G1 behind BotInfo: the bindings fit the policy, BotInfo's stop reaches the robot as StopMove (or as
Damp with the damp binding), and the connector refuses bad numbers. A stand-in robot replaces the physics, so these
run without numpy or mujoco; the last test runs the real sim when numpy, mujoco and a unitree_rl_gym clone
(BOTINFO_UNITREE_RL_GYM) are there."""
import importlib.util
import math
import os
import tempfile
import unittest
from pathlib import Path

from helpers import ROOT

from botinfo.audit import AuditLog
from botinfo.backends.base import ToolError
from botinfo.backends.g1sim import G1LocoConnector, SetupError, _read_torchscript
from botinfo.backends.tooltemplate import ToolTemplateBackend, load_binding
from botinfo.clock import VirtualClock
from botinfo.core import BotInfo
from botinfo.policy import load_policy

POLICY = ROOT / "policies" / "g1sim.toml"
STOPMOVE = ROOT / "bindings" / "g1sim-loco.toml"
DAMP = ROOT / "bindings" / "g1sim-loco-damp.toml"


class StandInG1:
    """G1Sim's interface without physics: a point that walks at the commanded velocity while the Move lasts."""

    def __init__(self, clock):
        self.clock, self.t = clock, clock.now()
        self.x = self.y = self.yaw = 0.0
        self.cmd, self.cmd_until, self.mode = (0.0, 0.0, 0.0), 0.0, "walking"
        self.received = []

    def step(self):
        now = self.clock.now()
        if now > self.t:
            vx, _, w = self.cmd if now <= self.cmd_until else (0.0, 0.0, 0.0)
            dt = now - self.t
            self.x += vx * math.cos(self.yaw) * dt
            self.y += vx * math.sin(self.yaw) * dt
            self.yaw += w * dt
            self.t = now

    def set_velocity(self, vx, vy, vyaw, duration):
        self.step()
        self.received.append((self.clock.now(), "SetVelocity"))
        if self.mode != "walking":
            raise ToolError(f"the walking policy is off ({self.mode})")
        self.cmd, self.cmd_until = (vx, vy, vyaw), self.clock.now() + duration

    def stop_move(self):
        self.step()
        self.received.append((self.clock.now(), "StopMove"))
        self.cmd = (0.0, 0.0, 0.0)

    def set_mode(self, mode):
        self.step()
        self.received.append((self.clock.now(), mode))
        self.mode, self.cmd = mode, (0.0, 0.0, 0.0)

    def state(self):
        self.step()
        moving = self.mode == "walking" and self.clock.now() <= self.cmd_until and any(self.cmd)
        return {"x": self.x, "y": self.y, "yaw": self.yaw, "z": 0.78 if self.mode == "walking" else 0.3,
                "speed_mps": abs(self.cmd[0]) if moving else 0.0, "travel_mps": abs(self.cmd[0]) if moving else 0.0,
                "upright": self.mode == "walking", "mode": self.mode}

    def close(self):
        pass


class G1Rig:
    def __init__(self, binding=STOPMOVE, robot=None):
        self.clock = VirtualClock()
        self.robot = robot(self.clock) if robot else StandInG1(self.clock)
        self.conn = G1LocoConnector(self.robot)
        self.backend = ToolTemplateBackend(self.conn, load_binding(binding))
        self.botinfo = BotInfo(load_policy(POLICY), self.backend, clock=self.clock, audit=AuditLog(clock=self.clock))

    def run(self, seconds):
        end = self.clock.now() + seconds
        while self.clock.now() < end - 1e-9:
            self.clock.advance(0.02)
            self.conn.tick()
            self.botinfo.tick()

    def call(self, name, args=None):
        return self.botinfo.call_tool(name, args or {})

    def names_since(self, n):
        """What reached the robot after its first n calls (a keepalive can share the clock time of the stop)."""
        return [name for (_, name) in self.robot.received[n:]]


class G1Bindings(unittest.TestCase):
    def test_both_bindings_fit_the_policy(self):
        policy = load_policy(POLICY)
        for path in (STOPMOVE, DAMP):
            backend = ToolTemplateBackend(G1LocoConnector(StandInG1(VirtualClock())), load_binding(path))
            self.assertEqual(backend.check_policy(policy), [], msg=path.name)

    def test_stop_is_velocity_zero_and_the_policy_stays_on(self):
        r = G1Rig()
        self.assertTrue(r.call("walk", {"forward_mps": 0.3, "duration_s": 4.0})["ok"])
        r.run(1.0)
        self.assertGreaterEqual(r.names_since(0).count("SetVelocity"), 5)          # Move renewed every 0.2 s
        n = len(r.robot.received)
        r.call("botinfo_stop")
        r.run(1.0)
        after = r.names_since(n)
        self.assertEqual(after[0], "StopMove")
        self.assertNotIn("SetVelocity", after)
        self.assertNotIn("damping", after)
        self.assertEqual(r.robot.mode, "walking")
        self.assertTrue(r.backend.status()["upright"])

    def test_a_walk_that_runs_out_ends_with_stopmove(self):
        r = G1Rig()
        self.assertTrue(r.call("walk", {"forward_mps": 0.3, "duration_s": 1.0})["ok"])
        r.run(1.5)
        self.assertEqual(r.robot.received[-1][1], "StopMove")
        self.assertGreater(r.robot.x, 0.2)

    def test_the_damp_binding_stops_with_damp(self):
        r = G1Rig(binding=DAMP)
        self.assertTrue(r.call("walk", {"forward_mps": 0.3, "duration_s": 4.0})["ok"])
        r.run(1.0)
        n = len(r.robot.received)
        r.botinfo.owner_stop()
        r.run(0.5)
        self.assertEqual(r.names_since(n)[0], "damping")
        self.assertEqual(r.robot.mode, "damping")
        self.assertFalse(r.backend.status()["upright"])

    def test_move_refuses_numbers_that_are_not_finite(self):
        conn = G1LocoConnector(StandInG1(VirtualClock()))
        for bad in (float("nan"), float("inf"), True, "0.3", None):
            with self.assertRaises(ToolError, msg=repr(bad)):
                conn.call_tool("Move", {"vx": bad, "vy": 0.0, "vyaw": 0.0})
        with self.assertRaises(ToolError):
            conn.call_tool("Jump", {})
        self.assertEqual(conn.call_tool("Move", {"vx": 0.2, "vy": 0.0, "vyaw": 0.0}), {"code": 0})

    def test_a_damaged_policy_file_is_a_setup_error(self):
        """botinfo serve and check report a ValueError cleanly; a file that is not a zip must not end in a traceback."""
        self.assertTrue(issubclass(SetupError, ValueError))
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "motion.pt"
            p.write_bytes(b"not a TorchScript archive")
            with self.assertRaises(SetupError) as cm:
                _read_torchscript(p, None)          # the zip is opened before numpy is needed
            self.assertIn("BadZipFile", str(cm.exception))


HAVE_SIM = all(importlib.util.find_spec(m) for m in ("numpy", "mujoco")) and bool(os.environ.get("BOTINFO_UNITREE_RL_GYM"))


@unittest.skipUnless(HAVE_SIM, "needs numpy, mujoco and BOTINFO_UNITREE_RL_GYM (a unitree_rl_gym clone)")
class G1Physics(unittest.TestCase):
    def rig(self, binding):
        from botinfo.backends.g1sim import G1Sim, find_unitree_rl_gym
        root = find_unitree_rl_gym()
        return G1Rig(binding=binding, robot=lambda clock: G1Sim(clock, root))

    def test_stopmove_keeps_the_walking_g1_upright(self):
        r = self.rig(STOPMOVE)
        r.run(1.0)
        self.assertTrue(r.call("walk", {"forward_mps": 0.3, "duration_s": 5.0})["ok"])
        r.run(3.0)
        self.assertGreater(r.backend.status()["x"], 0.3)
        r.call("botinfo_stop")
        r.run(4.0)
        s = r.backend.status()
        self.assertTrue(s["upright"])
        self.assertEqual(s["mode"], "walking")

    def test_damp_leaves_the_walking_g1_down(self):
        r = self.rig(DAMP)
        r.run(1.0)
        self.assertTrue(r.call("walk", {"forward_mps": 0.3, "duration_s": 5.0})["ok"])
        r.run(3.0)
        r.call("botinfo_stop")
        r.run(4.0)
        self.assertFalse(r.backend.status()["upright"])


if __name__ == "__main__":
    unittest.main()
