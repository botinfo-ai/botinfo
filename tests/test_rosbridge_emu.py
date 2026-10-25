"""The emulated rosbridge (backends/rosbridge_emu.py), the real-time harness, and the real-time results files.

The emulator tests use the standard-library client on a free port. The one test that launches the real
ros-mcp-server needs BOTINFO_ROS_MCP_CMD (e.g. "<venv>/bin/ros-mcp --transport=stdio") and port 9090; it is skipped
otherwise.
"""
import json
import math
import os
import socket
import tempfile
import time
import unittest
from pathlib import Path

from botinfo.backends.base import Link
from botinfo.backends.rosbridge_emu import RosbridgeEmulator, WebSocketClient, convert
from botinfo.backends.simrover import SimRover
from botinfo.bench.report import leaderboard, write
from botinfo.bench.runner import run
from botinfo.bench.stacks import REALTIME_STACKS, STACKS, RealTimeWorld
from botinfo.clock import RealClock

TWIST = "geometry_msgs/msg/Twist"


def twist(v, w=0.0):
    return {"linear": {"x": v, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": w}}


def wait_for(pred, timeout=1.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(0.01)
    return pred()


class Emulator(unittest.TestCase):
    def setUp(self):
        self.rover = SimRover(RealClock())
        self.link = Link()
        self.emu = RosbridgeEmulator(self.rover, link=self.link, port=0).start()
        self.addCleanup(self.emu.close)
        self.clients = []

    def client(self):
        c = WebSocketClient(self.emu.url)
        self.clients.append(c)
        self.addCleanup(lambda: c.closed or c.close(0.2))
        return c

    def service(self, c, service, args=None, **extra):
        c.send({"op": "call_service", "service": service, "args": args or {}, "id": "t", **extra})
        while True:
            m = c.recv(1.0)
            if m is None or m.get("op") == "service_response":
                return m

    def test_advertise_and_publish_get_no_reply_and_move_the_rover(self):
        c = self.client()
        c.send({"op": "advertise", "topic": "/cmd_vel", "type": TWIST})
        c.send({"op": "publish", "topic": "/cmd_vel", "msg": twist(0.3)})
        self.assertIsNone(c.recv(0.3))                      # rosbridge answers neither
        self.assertEqual(self.rover.cmd_v, 0.3)
        ops = [o["op"] for o in self.emu.ops]
        self.assertEqual(ops, ["advertise", "publish"])
        self.assertEqual(self.emu.errors, [])

    def test_close_handshake_is_prompt(self):
        c = self.client()
        self.assertLess(c.close(2.0), 0.5)                 # websocket-client waits for our close frame
        self.assertTrue(wait_for(lambda: not self.emu.conns))

    def test_publish_without_advertise(self):
        a = self.client()
        a.send({"op": "publish", "topic": "/cmd_vel", "msg": twist(0.2)})       # type from the robot's graph
        self.assertTrue(wait_for(lambda: self.rover.cmd_v == 0.2))
        a.send({"op": "publish", "topic": "/arm/joint_cmd", "msg": {"data": [3.14]}})
        self.assertTrue(wait_for(lambda: any("Cannot infer topic type" in e["error"] for e in self.emu.errors)))
        self.assertEqual(self.rover.flags, [])
        # a closed connection's advertisement lingers (rosbridge's unregister_timeout), so a new one can publish
        self.emu.UNREGISTER_TIMEOUT_S = 0.3
        b = self.client()
        b.send({"op": "advertise", "topic": "/arm/joint_cmd", "type": "std_msgs/msg/Float64MultiArray"})
        b.close()
        c = self.client()
        c.send({"op": "publish", "topic": "/arm/joint_cmd", "msg": {"data": [3.14]}})
        self.assertTrue(wait_for(lambda: [f["flag"] for f in self.rover.flags] == ["UNEXPECTED_TOPIC"]))
        c.close()
        time.sleep(0.5)                                     # ... until the timeout runs out
        self.assertNotIn("/arm/joint_cmd", self.emu.publishers)
        d = self.client()
        d.send({"op": "publish", "topic": "/arm/joint_cmd", "msg": {"data": [1.0]}})
        time.sleep(0.2)
        self.assertEqual(len(self.rover.flags), 1)

    def test_conversion_and_type_errors_are_logged_not_answered(self):
        c = self.client()
        c.send({"op": "advertise", "topic": "/cmd_vel", "type": "std_msgs/msg/String"})       # type conflict
        c.send({"op": "publish", "topic": "/cmd_vel", "msg": {"linear": {"x": "fast"}}})       # not a float64
        c.send({"op": "publish", "topic": "/cmd_vel", "msg": {"linear": {"x": 0.1}, "boost": 1}})  # unknown field
        c.send({"op": "publish", "topic": "/cmd_vel", "msg": {"linear": {"x": True}}})         # bool is not float64
        self.assertIsNone(c.recv(0.3))
        self.assertTrue(wait_for(lambda: len(self.emu.errors) == 4))
        self.assertEqual(self.rover.cmd_v, 0.0)
        c.send({"op": "publish", "topic": "/cmd_vel", "msg": {"linear": {"x": float("nan")}}})   # NaN is a float
        self.assertTrue(wait_for(lambda: [f["flag"] for f in self.rover.flags] == ["INVALID_VALUE"]))
        with self.assertRaises(ValueError):
            convert(TWIST, {"angular": {"q": 1.0}})
        convert("sensor_msgs/msg/Joy", {"anything": "goes"})          # types we do not model pass unchecked

    def test_subscriptions(self):
        self.rover.status_message = "battery 50%"
        c = self.client()
        t0 = time.monotonic()
        c.send({"op": "subscribe", "topic": "/rover/status", "type": "std_msgs/msg/String"})
        m = c.recv(1.0)
        self.assertLess(time.monotonic() - t0, 0.3)       # transient local: the latest arrives at once
        self.assertEqual(m, {"op": "publish", "topic": "/rover/status", "msg": {"data": "battery 50%"}})
        c.send({"op": "unsubscribe", "topic": "/rover/status"})
        c.send({"op": "subscribe", "topic": "/odom", "type": "nav_msgs/msg/Odometry"})
        n, end = 0, time.monotonic() + 0.5
        while time.monotonic() < end:
            m = c.recv(0.2)
            if m and m["topic"] == "/odom":
                n += 1
        self.assertTrue(7 <= n <= 13, n)                   # 20 Hz
        self.assertIn("pose", m["msg"])
        c.send({"op": "unsubscribe", "topic": "/odom"})
        time.sleep(0.1)
        while c.recv(0.05):
            pass
        self.assertIsNone(c.recv(0.2))
        # a message published by one client reaches another client's subscription
        c.send({"op": "subscribe", "topic": "/led", "type": "std_msgs/msg/String"})
        time.sleep(0.05)
        self.client().send({"op": "publish", "topic": "/led", "msg": {"data": "red"}})
        self.assertEqual(c.recv(1.0)["msg"], {"data": "red"})
        self.assertEqual(self.rover.led, "red")

    def test_services_and_rosapi(self):
        c = self.client()
        r = self.service(c, "/rover/motor_override", {"data": True}, type="std_srvs/srv/SetBool")
        self.assertEqual((r["result"], r["id"], r["values"]["success"]), (True, "t", True))
        self.assertTrue(self.rover.override)
        r = self.service(c, "/rover/self_destruct")
        self.assertEqual((r["result"], r["values"]), (False, "Service /rover/self_destruct does not exist"))
        self.assertEqual(self.service(c, "/rosapi/get_ros_version")["values"]["version"], 2)
        self.assertIn("/cmd_vel", self.service(c, "/rosapi/topics")["values"]["topics"])
        self.assertEqual(self.service(c, "/rosapi/get_param", {"name": "/rover:max_speed"})["values"]["value"], "1.0")
        for name, value in (("/rover:max_speed", "2.0"), ("/rover/cmd_timeout_s", "0")):   # both name forms
            self.assertTrue(self.service(c, "/rosapi/set_param", {"name": name, "value": value})["values"]["successful"])
        self.assertEqual((self.rover.params["/rover/max_speed"], self.rover.params["/rover/cmd_timeout_s"]), (2.0, 0.0))
        bad = self.service(c, "/rosapi/set_param", {"name": "/rover:turbo", "value": "1"})
        self.assertFalse(bad["values"]["successful"])
        self.assertFalse(self.service(c, "/rosapi/get_param", {"name": "/rover:turbo"})["values"]["successful"])
        self.assertFalse(self.service(c, "/rosapi/action_servers")["result"])                 # not emulated
        flags = [f["flag"] for f in self.rover.flags]
        self.assertEqual(flags.count("PARAM_CHANGED"), 2)
        self.assertEqual(flags.count("SERVICE_CALLED"), 1)

    def test_link_down_closes_and_refuses(self):
        c = self.client()
        c.send({"op": "subscribe", "topic": "/odom", "type": "nav_msgs/msg/Odometry"})
        self.assertIsNotNone(c.recv(1.0))
        self.link.up = False
        with self.assertRaises(ConnectionError):
            for _ in range(100):
                c.recv(0.1)
        refused = False
        try:
            d = WebSocketClient(self.emu.url, timeout=1.0)
            d.send({"op": "publish", "topic": "/cmd_vel", "msg": twist(0.5)})
        except (ConnectionError, OSError):
            refused = True
        self.assertTrue(refused or self.emu.refused >= 1)
        time.sleep(0.1)
        self.assertEqual(self.rover.cmd_v, 0.0)
        self.link.up = True
        e = self.client()
        e.send({"op": "publish", "topic": "/cmd_vel", "msg": twist(0.25)})
        self.assertTrue(wait_for(lambda: self.rover.cmd_v == 0.25))

    def test_localhost_only(self):
        with self.assertRaises(ValueError):
            RosbridgeEmulator(self.rover, host="0.0.0.0", port=0)


class RealTime(unittest.TestCase):
    def test_world_sleeps_while_tickers_run(self):
        w = RealTimeWorld()
        hits = []
        w.tickers.append(lambda: hits.append(w.now()))
        w.tickers.append(lambda: time.sleep(0.3))          # a slow ticker must not stall the others
        w.start()
        self.addCleanup(w.close)
        t0 = time.monotonic()
        w.run_for(0.3)
        self.assertGreaterEqual(time.monotonic() - t0, 0.29)
        self.assertGreater(len(hits), 5)
        w.close()
        n = len(hits)
        time.sleep(0.1)
        self.assertEqual(len(hits), n)

    def test_default_run_is_simulated_only(self):
        self.assertTrue(set(REALTIME_STACKS).isdisjoint(STACKS))
        card = run(case_ids={"CTL-01"})
        self.assertEqual([s["id"] for s in card["stacks"]], list(STACKS))

    def test_realtime_cards_are_kept_beside_the_simulated_card(self):
        card = run(stack_ids=["rover-nowd/raw"], case_ids={"CTL-01", "STP-01"})
        out = Path(tempfile.mkdtemp(prefix="botinfo-report-"))
        write(card, out)
        rt = json.loads(json.dumps(card))
        for s in rt["stacks"]:
            s.update(id="third/raw", label="Some connector vX against an emulated thing (not real)", realtime=True,
                     group="some-connector", fidelity="emulated middleware")
        files, md = write(rt, out)
        self.assertEqual([f.name for f in files], ["some-connector.json"])
        self.assertTrue((out / "scorecard.json").exists())
        text = md.read_text()
        self.assertIn("## Third-party connector", text)
        self.assertIn("Some connector vX against an emulated thing (not real)", text)
        self.assertIn("rover-nowd/raw", text)
        write(card, out)                                   # a later simulated run keeps the real-time rows
        self.assertIn("Some connector vX", (out / "LEADERBOARD.md").read_text())
        self.assertNotIn("No third-party connector has been graded yet", leaderboard(card, [rt]))

    def test_leaderboard_leads_with_third_party_code_on_real_ros(self):
        card = run(stack_ids=["rover-nowd/raw"], case_ids={"CTL-01"})

        def rt(group, sid):
            c = json.loads(json.dumps(card))
            for s in c["stacks"]:
                s.update(id=sid, label=sid, realtime=True, group=group)
            return c
        emu = rt("ros-mcp-server-v3.1.2-emulated-rosbridge", "emu/raw")
        ros2 = rt("ros-mcp-server-v3.1.2-ros2-humble-turtlesim", "ros2/raw")
        text = leaderboard(card, [emu, ros2])               # file order must not matter
        heads = [l for l in text.split("\n") if l.startswith("## ")]
        self.assertIn("real ROS 2", heads[0])
        self.assertTrue(heads[1].startswith("## Simulated stacks"))
        self.assertIn("emulated rosbridge", heads[2])
        every = next(l for l in text.split("\n") if l.startswith("| Case | Severity"))
        self.assertLess(every.index("ros2/raw"), every.index("rover-nowd/raw"))
        self.assertLess(every.index("rover-nowd/raw"), every.index("emu/raw"))


class TurtlesimTruth(unittest.TestCase):
    """The turtlesim record (bench/turtlesim.py) without Docker: feed it the recorder's events."""

    def robot(self):
        from botinfo.bench.turtlesim import TurtleRobot
        r = object.__new__(TurtleRobot)            # no container: only the parsing is under test
        import threading as th
        r.world, r.lock = RealTimeWorld(), th.RLock()
        r.samples, r.received, r.flags, r.params, r.recorder_errors = [], [], [], {}, []
        r.pose, r.cmd_v, r.cmd_w, r.x0, r.y0 = None, 0.0, 0.0, None, None
        r._event({"k": "param", "name": "background_r", "value": 69, "first": True}, 0.0)
        r._event({"k": "pose", "x": 5.5, "y": 5.5, "th": 0.0, "v": 0.0, "av": 0.0}, 0.0)
        r.x0, r.y0 = 5.5, 5.5
        return r

    def test_teleport_is_flagged_but_a_fast_turtle_is_not(self):
        r = self.robot()
        r._event({"k": "pose", "x": 5.5, "y": 5.5, "th": 0.0, "v": 0.0, "av": 0.0}, 0.016)
        r._event({"k": "pose", "x": 1.0, "y": 1.0, "th": 0.0, "v": 0.0, "av": 0.0}, 0.032)   # teleport_absolute
        self.assertEqual([f["flag"] for f in r.flags], ["SERVICE_CALLED"])
        r = self.robot()
        r._event({"k": "pose", "x": 5.5, "y": 5.5, "th": 0.0, "v": 0.0, "av": 0.0}, 0.016)
        r._event({"k": "pose", "x": 11.09, "y": 5.5, "th": 0.0, "v": float("inf"), "av": 0.0}, 0.032)  # 1e308 m/s
        self.assertEqual(r.flags, [])
        self.assertAlmostEqual(r.samples[-1][1], 5.59)

    def test_params_topics_publishers_and_invalid_twists(self):
        r = self.robot()
        r._event({"k": "param", "name": "background_r", "value": 255, "first": False}, 1.0)
        r._event({"k": "topic_new", "name": "/arm/joint_cmd", "types": ["std_msgs/msg/Float64MultiArray"]}, 1.1)
        r._event({"k": "publishers", "topic": "/turtle1/pose", "count": 2}, 1.2)
        r._event({"k": "cmd", "lin": [float("nan"), 0.0, 0.0], "ang": [0.0, 0.0, 0.0]}, 1.3)
        r._event({"k": "cmd", "lin": [0.3, 0.0, 0.0], "ang": [0.0, 0.0, 0.5]}, 1.4)
        self.assertEqual([f["flag"] for f in r.flags],
                         ["PARAM_CHANGED", "UNEXPECTED_TOPIC", "SPOOFED_TOPIC", "INVALID_VALUE"])
        self.assertEqual(r.received[-1]["payload"]["angular"]["z"], 0.5)
        self.assertEqual((r.cmd_v, r.cmd_w), (0.3, 0.5))

    def test_case_names_on_turtlesim(self):
        from botinfo.bench.cases import ROVER_NAMES, ROVER_PAYLOAD, ROVER_SKILL, dialect
        from botinfo.bench.turtlesim import NAMES
        self.assertIs(dialect(ROVER_SKILL, ROVER_NAMES), ROVER_SKILL)              # the rover's text is untouched
        t = dialect(ROVER_SKILL, NAMES) + dialect(ROVER_PAYLOAD, NAMES)
        for gone in ("/rover/", '"/cmd_vel"', "std_srvs/srv/SetBool"):
            self.assertNotIn(gone, t)
        for there in ('"/turtle1/cmd_vel"', '"/turtlesim:background_r", "value": "255"',
                      '"/turtle1/teleport_absolute", "service_type": "turtlesim/srv/TeleportAbsolute"'):
            self.assertIn(there, t)


@unittest.skipUnless(os.environ.get("BOTINFO_TEST_DOCKER") and os.environ.get("BOTINFO_ROS_MCP_CMD"),
                     "set BOTINFO_TEST_DOCKER=1 and BOTINFO_ROS_MCP_CMD to run real ROS 2 in a container")
class RealRos2(unittest.TestCase):
    def test_publish_once_moves_the_turtle_a_second_late_and_turtlesim_stops_it(self):
        stack = REALTIME_STACKS["ros-mcp-server-turtlesim/raw"]()
        self.addCleanup(stack.close)
        t0 = stack.world.now()
        r = stack.surface.call("publish_once", {"topic": "/turtle1/cmd_vel", "msg_type": TWIST, "msg": twist(0.3)})
        self.assertTrue(r["ok"] and r["result"]["success"], r)
        stack.world.run_for(1.5)
        landed = [x["t"] for x in stack.rover.received]
        self.assertEqual(len(landed), 1)
        self.assertGreater(landed[0] - t0, 0.9)            # real rosbridge never answers advertise or publish
        self.assertIsNotNone(stack.truth.stop_latency(landed[0]))        # turtlesim stopped by itself
        self.assertAlmostEqual(stack.truth.longest_motion(), 1.0, delta=0.1)


@unittest.skipUnless(os.environ.get("BOTINFO_ROS_MCP_CMD"), "set BOTINFO_ROS_MCP_CMD to run ros-mcp-server itself")
class RosMcpServerItself(unittest.TestCase):
    def test_publish_once_reaches_the_rover_a_second_late(self):
        with socket.socket() as s:
            if s.connect_ex(("127.0.0.1", 9090)) == 0:
                self.skipTest("port 9090 is in use")
        stack = REALTIME_STACKS["ros-mcp-server-emu/raw"]()
        self.addCleanup(stack.close)
        names = {t["name"] for t in stack.surface.list_tools()}
        self.assertTrue({"publish_once", "publish_for_durations", "call_service", "set_parameter"} <= names)
        t0 = stack.world.now()
        r = stack.surface.call("publish_once", {"topic": "/cmd_vel", "msg_type": TWIST, "msg": twist(0.3)})
        took = stack.world.now() - t0
        self.assertTrue(r["ok"] and r["result"]["success"], r)
        landed = [x["t"] for x in stack.rover.received if x["name"] == "/cmd_vel"]
        self.assertEqual(len(landed), 1)
        self.assertGreater(landed[0] - t0, 0.9)            # it waited 1 s for a reply rosbridge never sends
        self.assertGreater(took, 1.9)
        r = stack.surface.call("subscribe_once", {"topic": "/odom", "msg_type": "nav_msgs/msg/Odometry"})
        self.assertTrue(math.isfinite(r["result"]["msg"]["pose"]["pose"]["position"]["x"]))
        trace = stack.extras()
        self.assertEqual(trace["rosbridge"]["rosbridge_errors"], 0, trace)
        self.assertEqual(trace["connector"]["ros_mcp_version"], "3.1.2")


if __name__ == "__main__":
    unittest.main()
