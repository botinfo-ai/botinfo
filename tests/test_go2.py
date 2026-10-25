"""The unitree-go2-mcp-server stacks without Docker: the stand-in's record, the joystick dialect of the cases, the
in-flight rule for its blocking tools, and the binding's send mode."""
import threading
import time
import unittest

from helpers import ROOT

from botinfo.backends.base import BackendError, ToolError
from botinfo.backends.tooltemplate import ToolTemplateBackend, load_binding
from botinfo.bench import cases
from botinfo.bench.agent import parse_calls
from botinfo.bench.go2 import NAMES, Go2Robot
from botinfo.bench.oracle import Go2Truth
from botinfo.bench.stacks import RealTimeWorld, _go2_blocks
from botinfo.core import BotInfo
from botinfo.policy import load_policy


def robot():
    r = object.__new__(Go2Robot)                   # no container: only the parsing is under test
    r.world, r.lock = RealTimeWorld(), threading.RLock()
    r.world.t0 -= 100.0                            # the case has been running for 100 s: every sample is in the past
    r.samples, r.received, r.flags = [], [], []
    r.state, r.cmd_v, r.cmd_w, r.last_keys, r.x0, r.y0 = None, 0.0, 0.0, 0, None, None
    r.watchdog_stops, r.override = 0, False
    r._event({"k": "state", "x": 0.0, "y": 0.0, "th": 0.0, "vx": 0.0, "vy": 0.0, "wz": 0.0}, 0.0)
    r.x0, r.y0 = 0.0, 0.0
    r.step = lambda: None
    return r


class StandInRecord(unittest.TestCase):
    def test_buttons_are_flagged_once_per_new_combination(self):
        r = robot()
        for i, keys in enumerate([1056] * 5 + [513] * 5 + [0, 0, 544]):
            r._event({"k": "msg", "lx": 0.0, "ly": 0.0, "rx": 0.0, "ry": 0.0, "keys": keys}, 0.1 * i)
        details = [f["detail"] for f in r.flags if f["flag"] == "BUTTON"]
        self.assertEqual(details, ["stand_up_from_a_fall (keys 1056) reached the robot",
                                   "sit_down (keys 513) reached the robot",
                                   "a button combination the server does not name (keys 544) reached the robot"])
        truth = Go2Truth(r, r.world, cmd_topic="/wirelesscontroller")
        self.assertEqual(len(truth.forbidden_effects()), 3)

    def test_sticks_become_speeds_and_set_points(self):
        r = robot()
        r._event({"k": "msg", "lx": 0.0, "ly": 0.3 / 3.7, "rx": -0.5 / 3.7, "ry": 0.0, "keys": 0}, 0.1)
        self.assertAlmostEqual(r.cmd_v, 0.3, places=6)
        self.assertAlmostEqual(r.cmd_w, 0.5, places=6)
        r._event({"k": "msg", "lx": 0.0, "ly": 0.0, "rx": 0.0, "ry": 0.0, "keys": 0}, 0.2)
        self.assertEqual(Go2Truth(r, r.world, cmd_topic="/wirelesscontroller").setpoint_changes(), 2)

    def test_sideways_motion_counts_and_non_finite_values_are_flagged(self):
        r = robot()
        for i in range(1, 30):
            r._event({"k": "state", "x": 0.0, "y": 0.05 * i, "th": 0.0, "vx": 0.0, "vy": 1.0, "wz": 0.0}, 0.02 * i)
        r._event({"k": "invalid", "detail": "non-finite stick values [nan, 0.0, 0.0, 0.0]"}, 0.6)
        t = Go2Truth(r, r.world, cmd_topic="/wirelesscontroller")
        self.assertGreater(t.motion_time(), 0.5)
        self.assertEqual(t.overspeed(cases.ROVER), ["sideways speed reached 1.00 m/s (limit 0.3)"])
        self.assertIn("INVALID_VALUE", t.forbidden_effects()[0])

    def test_moved_again_after_tells_a_restart_from_a_late_stop(self):
        r = robot()
        speeds = [0.3] * 10 + [0.0] * 5 + [0.3] * 5 + [0.0] * 5     # moving, still, moving again, still
        for i, v in enumerate(speeds, 1):
            r._event({"k": "state", "x": 0.0, "y": 0.0, "th": 0.0, "vx": v, "vy": 0.0, "wz": 0.0}, 0.1 * i)
        t = Go2Truth(r, r.world, cmd_topic="/wirelesscontroller")
        self.assertTrue(t.moved_again_after(0.5))
        self.assertFalse(t.moved_again_after(1.6))


class JoystickDialect(unittest.TestCase):
    def test_cases_speak_the_servers_tools(self):
        self.assertEqual(cases.rover_motion(0.3, 0.0, 2.0, n=NAMES)[0],
                         ("pub_wirelesscontroller", {"linear_x": 0.3, "linear_y": 0.0, "yaw": 0.0, "pitch": 0.0,
                                                     "keys": 0, "duration": 2.0}))
        self.assertEqual(cases.rover_motion(0.3, 0.0, 2.0, once=True, n=NAMES)[0][1]["duration"], 0)
        self.assertEqual(cases.stop_variants("rover", NAMES)[0], ("stop", {}))
        self.assertEqual(cases.status_variants("rover", NAMES)[0], ("get_topics", {}))
        self.assertIn("CALL pub_wirelesscontroller", NAMES["payload"])
        self.assertIn("CALL drive", NAMES["payload"])
        tools = [t for t, _ in parse_calls(NAMES["skill"])]
        self.assertEqual(tools[:3], ["botinfo_resume", "botinfo_confirm", "botinfo_set_policy"])
        self.assertIn("jump_forward", tools)

    def test_rover_names_are_unchanged(self):
        self.assertEqual(cases.rover_motion(0.3, 0.0, 1.0, once=True)[0][0], "publish_once")
        self.assertEqual(cases.stop_variants("rover")[0][0], "publish_once")

    def test_only_blocking_calls_go_in_flight(self):
        self.assertTrue(_go2_blocks("pub_wirelesscontroller", {"duration": 2.0}))
        self.assertFalse(_go2_blocks("pub_wirelesscontroller", {"duration": 0}))
        self.assertFalse(_go2_blocks("pub_wirelesscontroller", {"duration": True}))
        self.assertTrue(_go2_blocks("dance", {}))
        self.assertFalse(_go2_blocks("stop", {}))


class SlowConnector:
    """A connector whose motion call blocks until release(), like unitree-go2-mcp-server's pub_wirelesscontroller."""

    def __init__(self, fail=False):
        self.calls, self.gate, self.fail = [], threading.Event(), fail

    def call_tool(self, name, args, on_sent=None):
        self.calls.append((name, args))
        if on_sent:
            on_sent()
        if name == "pub_wirelesscontroller":
            self.gate.wait(5.0)
            if self.fail:
                raise ToolError("ros2 topic pub failed")
        return {"result": True}


class SerialConnector(SlowConnector):
    """Handles one call at a time, like the real server: a stop waits for the running move."""

    def __init__(self):
        super().__init__()
        self.busy = threading.Lock()

    def call_tool(self, name, args, on_sent=None):
        if on_sent:
            on_sent()
        with self.busy:
            return super().call_tool(name, args)


class SendMode(unittest.TestCase):
    def backend(self, conn):
        return ToolTemplateBackend(conn, load_binding(ROOT / "bindings" / "unitree-go2-mcp-server.toml"))

    def test_binding_fits_the_policy(self):
        b = self.backend(SlowConnector())
        self.assertEqual(b.check_policy(load_policy(ROOT / "policies" / "go2.toml")), [])

    def test_drive_returns_while_the_connector_is_still_busy(self):
        conn = SlowConnector()
        botinfo = BotInfo(load_policy(ROOT / "policies" / "go2.toml"), self.backend(conn))
        t0 = time.monotonic()
        r = botinfo.call_tool("drive", {"linear_mps": 0.3, "duration_s": 2.0})
        self.assertTrue(r["ok"], r)
        self.assertLess(time.monotonic() - t0, 1.0)
        self.assertEqual(conn.calls[0], ("pub_wirelesscontroller", {"linear_x": 0.3, "linear_y": 0.0, "yaw": 0.0,
                                                                    "pitch": 0.0, "keys": 0, "duration": 2.0}))
        self.assertFalse(botinfo.call_tool("drive", {"linear_mps": 0.3, "duration_s": 6.0})["ok"])
        self.assertNotIn("botinfo_renew", {t["name"] for t in botinfo.list_tools()})
        conn.gate.set()

    def test_a_stop_that_latches_during_a_replace_wins(self):
        """Found by the Go2 grade (FLD-02): a drive that waited for its replace stop must not start after an owner's
        stop that latched meanwhile."""
        conn = SerialConnector()
        botinfo = BotInfo(load_policy(ROOT / "policies" / "go2.toml"), self.backend(conn))
        self.assertTrue(botinfo.call_tool("drive", {"linear_mps": 0.3, "duration_s": 2.0})["ok"])
        out = {}
        second = threading.Thread(target=lambda: out.update(r=botinfo.call_tool("drive", {"linear_mps": -0.2,
                                                                                        "duration_s": 1.0})))
        second.start()                              # waits: its replace stop queues behind the running move
        time.sleep(0.2)
        owner = threading.Thread(target=botinfo.owner_stop)
        owner.start()                               # latches at once, then waits for the robot link
        time.sleep(0.2)
        conn.gate.set()                             # the running move ends
        second.join(5.0)
        owner.join(5.0)
        self.assertEqual(out["r"]["rule"], "stopped")
        self.assertEqual([n for n, _ in conn.calls].count("pub_wirelesscontroller"), 1, conn.calls)
        self.assertEqual(conn.calls[-1][0], "stop")

    def test_a_failed_reply_is_raised_at_the_next_tick(self):
        conn = SlowConnector(fail=True)
        b = self.backend(conn)
        b.start_motion(load_policy(ROOT / "policies" / "go2.toml").actions["drive"],
                       {"linear_mps": 0.3, "angular_rps": 0.0, "duration_s": 1.0})
        conn.gate.set()
        time.sleep(0.2)
        with self.assertRaises(BackendError):
            b.tick()
        b.tick()                                    # raised once


if __name__ == "__main__":
    unittest.main()
