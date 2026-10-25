#!/usr/bin/env python3
"""Ground truth for the BotInfo Benchmark on turtlesim. Runs INSIDE the ROS container, as a ROS 2 node of its
own, so what it sees never passes through rosbridge (the middleware under test).

stdout: one JSON object per line, each with "wall" (this machine's time.time()):
  {"k": "ready", "topics": [...]}                      after the graph baseline is taken
  {"k": "pose", "x", "y", "th", "v", "av"}             every /turtle1/pose (turtlesim publishes ~62.5 Hz)
  {"k": "cmd", "lin": [x,y,z], "ang": [x,y,z]}         every Twist that reached /turtle1/cmd_vel
  {"k": "param", "name", "value", "first": bool}       turtlesim's background_* parameters, on change
  {"k": "topic_new", "name", "types"}                  a topic that was not in the graph at start
  {"k": "publishers", "topic", "count"}                publishers on the robot's own output topic /turtle1/pose
stdin: {"plant": "text"} sets what /turtle1/status says (std_msgs/String, published at 10 Hz).
"""
import json
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from std_msgs.msg import String
from turtlesim.msg import Pose

PARAMS = ["background_r", "background_g", "background_b"]
IGNORE = {"/rosout", "/parameter_events", "/client_count", "/connected_clients"}
BASELINE_AFTER_S = 2.0


class Recorder(Node):
    def __init__(self):
        super().__init__("botinfo_truth_recorder")
        self.lock = threading.Lock()
        self.t_start = time.monotonic()
        self.status = "turtle1 ok"
        self.baseline = None
        self.seen_new = set()
        self.params = {}
        self.pose_pubs = None
        self.pending = None
        self.create_subscription(Pose, "/turtle1/pose", self.on_pose, 100)
        self.create_subscription(Twist, "/turtle1/cmd_vel", self.on_cmd, 100)
        self.status_pub = self.create_publisher(String, "/turtle1/status", 10)
        self.param_cli = self.create_client(GetParameters, "/turtlesim/get_parameters")
        self.create_timer(0.1, self.publish_status)
        self.create_timer(0.1, self.poll)

    def emit(self, d):
        d["wall"] = time.time()
        line = json.dumps(d)
        with self.lock:
            sys.stdout.write(line + "\n")
            sys.stdout.flush()

    def on_pose(self, m):
        self.emit({"k": "pose", "x": m.x, "y": m.y, "th": m.theta, "v": m.linear_velocity, "av": m.angular_velocity})

    def on_cmd(self, m):
        self.emit({"k": "cmd", "lin": [m.linear.x, m.linear.y, m.linear.z], "ang": [m.angular.x, m.angular.y,
                                                                                  m.angular.z]})

    def publish_status(self):
        msg = String()
        msg.data = self.status
        self.status_pub.publish(msg)

    def poll(self):
        if self.param_cli.service_is_ready() and self.pending is None:
            req = GetParameters.Request()
            req.names = PARAMS
            self.pending = self.param_cli.call_async(req)
            self.pending.add_done_callback(self.on_params)
        names = {n: t for n, t in self.get_topic_names_and_types()}
        if self.baseline is None:
            if time.monotonic() - self.t_start >= BASELINE_AFTER_S and self.params:
                self.baseline = set(names)
                self.emit({"k": "ready", "topics": sorted(names)})
        else:
            for n in sorted(set(names) - self.baseline - IGNORE - self.seen_new):
                self.seen_new.add(n)
                self.emit({"k": "topic_new", "name": n, "types": names[n]})
        c = self.count_publishers("/turtle1/pose")
        if c != self.pose_pubs:
            self.pose_pubs = c
            self.emit({"k": "publishers", "topic": "/turtle1/pose", "count": c})

    def on_params(self, fut):
        self.pending = None
        try:
            values = fut.result().values
        except Exception as e:  # report, keep polling
            self.emit({"k": "error", "what": f"get_parameters: {e!r}"})
            return
        for name, v in zip(PARAMS, values):
            value = v.integer_value if v.type == 2 else v.double_value if v.type == 3 else None
            if self.params.get(name, "unset") != value:
                first = name not in self.params
                self.params[name] = value
                self.emit({"k": "param", "name": name, "value": value, "first": first})


def read_stdin(node):
    for line in sys.stdin:
        try:
            d = json.loads(line)
        except ValueError:
            continue
        if isinstance(d, dict) and isinstance(d.get("plant"), str):
            node.status = d["plant"]


def main():
    rclpy.init()
    node = Recorder()
    threading.Thread(target=read_stdin, args=(node,), daemon=True).start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
