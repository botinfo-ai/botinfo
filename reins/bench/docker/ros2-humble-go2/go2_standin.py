#!/usr/bin/env python3
"""A stand-in Go2 for the Reins Safety Benchmark. Runs INSIDE the "robot" container as a ROS 2 node of its own, on
the far side of a network link from the connector under test. It is not a model of how a Go2 walks.

What it implements, and only that: the robot's end of the interface unitree-go2-mcp-server uses. It subscribes to
/wirelesscontroller (unitree_go/msg/WirelessController, the joystick message from unitree_ros2) and turns the
sticks into body velocity with the server's own stated scale: a stick at 1.0 is 3.7 m/s or 3.7 rad/s, because the
server divides by 3.7 (server.py, convert_velocity_to_wirelesscontroller). So a call for 0.3 m/s drives at
0.3 m/s. Velocity changes at once (no inertia, no gait). Named moves arrive as button combinations (keys); they are
recorded, not acted out.

What happens when the messages stop is unknown for a real Go2. --stick-timeout says which way this stand-in goes:
  0   (default) hold the last stick input until the next message, like a joystick held where it was left;
  S   treat the sticks as centred once S seconds pass without a message (a robot-side watchdog).

Ground truth comes from here, never from the connector: stdout gets one JSON object per line, each with "wall"
(this machine's time.time()):
  {"k": "ready"}                                            once the node is up
  {"k": "state", "x", "y", "th", "vx", "vy", "wz"}          every 20 ms (world frame, from the start pose)
  {"k": "msg", "lx", "ly", "rx", "ry", "keys"}              every WirelessController that arrived
  {"k": "button", "keys", "name"}                           a message whose keys are not 0 (a button combination)
  {"k": "invalid", "detail"}                                a non-finite stick value arrived (ignored)
  {"k": "timeout"}                                          the stick timeout centred the sticks
stdin: {"plant": "text"} sets what /go2/status says (std_msgs/String, 10 Hz): someone else's text on the robot's
network.
"""
import argparse
import json
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from unitree_go.msg import WirelessController

SCALE = 3.7                      # m/s or rad/s at full stick: the server's own conversion
# The button combinations server.py names (msgs/wirelesscontroller.py); anything else is reported by its code.
KEYS = {1056: "stand_up_from_a_fall", 272: "stretch", 528: "shake_hands", 2064: "love", 1025: "pounce",
        257: "jump_forward", 513: "sit_down", 258: "greet", 514: "dance"}
STEP_S = 0.01
EMIT_EVERY = 2


class StandIn(Node):
    def __init__(self, stick_timeout_s: float):
        super().__init__("go2_standin")
        self.lock = threading.Lock()
        self.out = threading.Lock()
        self.stick_timeout_s = stick_timeout_s
        self.sticks = (0.0, 0.0, 0.0, 0.0)          # lx, ly, rx, ry
        self.last_msg = None
        self.timed_out = False
        self.x = self.y = self.th = 0.0
        self.t_last = time.monotonic()
        self.n = 0
        self.status = "go2 ok"
        self.create_subscription(WirelessController, "/wirelesscontroller", self.on_msg, 10)
        self.status_pub = self.create_publisher(String, "/go2/status", 10)
        self.create_timer(STEP_S, self.step)
        self.create_timer(0.1, self.publish_status)

    def emit(self, d):
        d["wall"] = time.time()
        line = json.dumps(d)
        with self.out:
            sys.stdout.write(line + "\n")
            sys.stdout.flush()

    def on_msg(self, m):
        vals = (m.lx, m.ly, m.rx, m.ry)
        self.emit({"k": "msg", "lx": m.lx, "ly": m.ly, "rx": m.rx, "ry": m.ry, "keys": m.keys})
        if m.keys:
            self.emit({"k": "button", "keys": m.keys, "name": KEYS.get(m.keys, f"keys {m.keys}")})
        if not all(math.isfinite(v) for v in vals):
            self.emit({"k": "invalid", "detail": f"non-finite stick values {list(vals)}"})
            return
        with self.lock:
            self.integrate()
            self.sticks = tuple(max(-1.0, min(1.0, v)) for v in vals)   # a real stick cannot go past full
            self.last_msg = time.monotonic()
            self.timed_out = False

    def velocity(self):
        lx, ly, rx, _ = self.sticks
        return ly * SCALE, lx * SCALE, -rx * SCALE     # inverse of the server's mapping: vx, vy, wz

    def integrate(self):
        now = time.monotonic()
        dt = now - self.t_last
        self.t_last = now
        vx, vy, wz = self.velocity()
        self.x += (vx * math.cos(self.th) - vy * math.sin(self.th)) * dt
        self.y += (vx * math.sin(self.th) + vy * math.cos(self.th)) * dt
        self.th = math.atan2(math.sin(self.th + wz * dt), math.cos(self.th + wz * dt))

    def step(self):
        with self.lock:
            self.integrate()
            if (self.stick_timeout_s > 0 and self.last_msg is not None and not self.timed_out
                    and time.monotonic() - self.last_msg > self.stick_timeout_s):
                self.sticks = (0.0, 0.0, 0.0, 0.0)
                self.timed_out = True
                self.emit({"k": "timeout"})
            vx, vy, wz = self.velocity()
            self.n += 1
            if self.n % EMIT_EVERY == 0:
                self.emit({"k": "state", "x": self.x, "y": self.y, "th": self.th, "vx": vx, "vy": vy, "wz": wz})

    def publish_status(self):
        msg = String()
        msg.data = self.status
        self.status_pub.publish(msg)


def read_stdin(node):
    for line in sys.stdin:
        try:
            d = json.loads(line)
        except ValueError:
            continue
        if isinstance(d, dict) and isinstance(d.get("plant"), str):
            node.status = d["plant"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stick-timeout", type=float, default=0.0)
    a = ap.parse_args()
    rclpy.init()
    node = StandIn(a.stick_timeout)
    threading.Thread(target=read_stdin, args=(node,), daemon=True).start()
    node.emit({"k": "ready", "stick_timeout_s": a.stick_timeout})
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
