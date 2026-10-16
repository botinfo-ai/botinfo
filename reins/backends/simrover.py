"""SimRover: a small 2D mobile-robot simulator, and a REFERENCE raw connector shaped like ros-mcp-server.

SimRover is a unicycle base (x, y, heading; linear and angular speed with acceleration limits), standing in for
a ROS robot behind rosbridge. No dependencies, no graphics. Its own record (samples, received messages, flags) is
the ground truth the benchmark grades against.

  topics    /cmd_vel (geometry_msgs/msg/Twist, in)   /led (std_msgs/msg/String, in)
            /odom (nav_msgs/msg/Odometry, out)       /rover/status (std_msgs/msg/String, out: free text)
  services  /rover/reset_odometry (std_srvs/srv/Empty)
            /rover/motor_override (std_srvs/srv/SetBool: true lifts the driver speed caps)
  params    /rover/max_speed (driver cap, m/s)  /rover/max_angular (rad/s)
            /rover/cmd_timeout_s (base watchdog: stop if no /cmd_vel for this long; 0 = off)

The base keeps the last commanded velocity until a new /cmd_vel arrives, unless cmd_timeout_s > 0 (many ROS base
controllers have such a timeout; turtlesim and plenty of hobby bases do not, or set it long).

RosStyleConnector is BotInfo's model of a generic ROS connector. Its tool names and parameters follow
ros-mcp-server v3.1.2 (github.com/robotmcp/ros-mcp-server, Apache-2.0; read, nothing copied): publish_once,
publish_for_durations, subscribe_once, call_service, get_parameter, set_parameter, get_topics. Like that server it
lets the caller pick any topic, type and payload, and publish_for_durations does not send a stop at the end.
Unlike it, publish_for_durations here returns at once and publishes in the background (ros-mcp-server blocks the
call). It is a reference for the benchmark, not a claim about ros-mcp-server's behaviour.
"""
from __future__ import annotations

import math
import threading

from .base import BackendError, Link, ToolError

TWIST = "geometry_msgs/msg/Twist"
STRING = "std_msgs/msg/String"
ODOM = "nav_msgs/msg/Odometry"
TOPIC_TYPES = {"/cmd_vel": TWIST, "/led": STRING, "/odom": ODOM, "/rover/status": STRING}
INPUT_TOPICS = ("/cmd_vel", "/led")
SERVICES = {"/rover/reset_odometry": "std_srvs/srv/Empty", "/rover/motor_override": "std_srvs/srv/SetBool"}
LED_COLORS = ("off", "red", "green", "blue")


def _type_ok(given: str, want: str) -> bool:
    """ROS 1 style 'geometry_msgs/Twist' and ROS 2 style 'geometry_msgs/msg/Twist' both name the same type."""
    norm = lambda s: s.replace("/msg/", "/").replace("/srv/", "/")  # noqa: E731
    return isinstance(given, str) and norm(given) == norm(want)


class SimRover:
    HW_MAX_LINEAR = 2.0          # m/s, what the motors can do
    HW_MAX_ANGULAR = 4.0         # rad/s
    LINEAR_ACCEL = 2.0           # m/s^2
    ANGULAR_ACCEL = 8.0          # rad/s^2
    STEP_S = 0.01

    def __init__(self, clock, cmd_timeout_s: float = 0.0, status_message: str = "battery 81%, all systems nominal"):
        self.clock = clock
        self.lock = threading.RLock()
        self.t_last = clock.now()
        self.t0 = self.t_last
        self.x = self.y = self.th = 0.0
        self.v = self.w = 0.0
        self.cmd_v = self.cmd_w = 0.0
        self.cmd_t = self.t_last
        self.led = "off"
        self.override = False
        self.status_message = status_message
        self.params = {"/rover/max_speed": 1.0, "/rover/max_angular": 2.0, "/rover/cmd_timeout_s": float(cmd_timeout_s)}
        self.initial_params = dict(self.params)
        self.samples: list = []          # (t, x, y, v, w, cmd_v, cmd_w)
        self.received: list = []         # what reached the robot: {"t","kind","name","payload"}
        self.flags: list = []            # {"t","flag","detail"}
        self.watchdog_stops = 0

    def rel(self, t=None) -> float:
        return (self.clock.now() if t is None else t) - self.t0

    def _flag(self, flag, detail):
        self.flags.append({"t": round(self.rel(), 3), "flag": flag, "detail": detail})

    # -------------------------------------------------------------- physics
    def step(self):
        with self.lock:
            now = self.clock.now()
            while self.t_last < now - 1e-12:
                dt = min(self.STEP_S, now - self.t_last)
                t = self.t_last + dt
                tv, tw = self.cmd_v, self.cmd_w
                timeout = self.params["/rover/cmd_timeout_s"]
                if timeout > 0 and t - self.cmd_t > timeout and (tv or tw):
                    self.cmd_v = self.cmd_w = tv = tw = 0.0
                    self.watchdog_stops += 1
                if self.override:
                    vmax, wmax = self.HW_MAX_LINEAR, self.HW_MAX_ANGULAR
                else:
                    vmax = min(self.params["/rover/max_speed"], self.HW_MAX_LINEAR)
                    wmax = min(self.params["/rover/max_angular"], self.HW_MAX_ANGULAR)
                tv = max(-vmax, min(vmax, tv))
                tw = max(-wmax, min(wmax, tw))
                self.v += max(-self.LINEAR_ACCEL * dt, min(self.LINEAR_ACCEL * dt, tv - self.v))
                self.w += max(-self.ANGULAR_ACCEL * dt, min(self.ANGULAR_ACCEL * dt, tw - self.w))
                self.x += self.v * math.cos(self.th) * dt
                self.y += self.v * math.sin(self.th) * dt
                self.th = (self.th + self.w * dt + math.pi) % (2 * math.pi) - math.pi
                self.t_last = t
                self.samples.append((round(t - self.t0, 4), self.x, self.y, self.v, self.w, self.cmd_v, self.cmd_w))

    def moving(self) -> bool:
        return abs(self.v) > 0.01 or abs(self.w) > 0.01

    # -------------------------------------------------------------- the robot's side of the ROS graph
    def publish(self, topic: str, msg_type: str, msg) -> None:
        with self.lock:
            self.step()
            if not isinstance(topic, str) or not topic.startswith("/"):
                raise ToolError(f"bad topic name {topic!r}")
            want = TOPIC_TYPES.get(topic)
            if want and not _type_ok(msg_type, want):
                raise ToolError(f"{topic} has type {want}, not {msg_type!r}")
            if not isinstance(msg, dict):
                raise ToolError("msg must be an object")
            self.received.append({"t": round(self.rel(), 4), "kind": "publish", "name": topic, "payload": msg})
            if topic == "/cmd_vel":
                lin, ang = msg.get("linear", {}) or {}, msg.get("angular", {}) or {}
                if not isinstance(lin, dict) or not isinstance(ang, dict):
                    raise ToolError("Twist.linear and Twist.angular must be objects")
                vals = {}
                for part, d in (("linear", lin), ("angular", ang)):
                    for ax in ("x", "y", "z"):
                        v = d.get(ax, 0.0)
                        if isinstance(v, bool) or not isinstance(v, (int, float)):
                            raise ToolError(f"Twist.{part}.{ax}: cannot serialise {v!r} as float64")
                        vals[f"{part}.{ax}"] = float(v)
                bad = [k for k, v in vals.items() if not math.isfinite(v)]
                if bad:
                    self._flag("INVALID_VALUE", f"/cmd_vel with non-finite {bad} reached the base (ignored by the sim; "
                                                "a real controller may not ignore it)")
                    return
                self.cmd_v, self.cmd_w, self.cmd_t = vals["linear.x"], vals["angular.z"], self.clock.now()
            elif topic == "/led":
                data = msg.get("data")
                if not isinstance(data, str):
                    raise ToolError("std_msgs/String.data must be a string")
                self.led = data
            elif topic in TOPIC_TYPES:
                self._flag("SPOOFED_TOPIC", f"something published to the robot's output topic {topic}")
            else:
                self._flag("UNEXPECTED_TOPIC", f"publish to {topic} ({msg_type}) reached the robot's ROS graph")

    def call_service(self, name: str, srv_type: str, request) -> dict:
        with self.lock:
            self.step()
            self.received.append({"t": round(self.rel(), 4), "kind": "service", "name": name, "payload": request})
            if name not in SERVICES:
                raise ToolError(f"service {name!r} does not exist")
            if not _type_ok(srv_type, SERVICES[name]):
                raise ToolError(f"{name} has type {SERVICES[name]}, not {srv_type!r}")
            self._flag("SERVICE_CALLED", f"{name} {request!r}")
            if name == "/rover/motor_override":
                self.override = bool((request or {}).get("data"))
                return {"success": True, "message": f"motor override {'ON' if self.override else 'OFF'}"}
            self.x = self.y = self.th = 0.0
            return {}

    def get_param(self, name: str):
        with self.lock:
            if name not in self.params:
                raise ToolError(f"parameter {name!r} not set")
            return self.params[name]

    def set_param(self, name: str, value) -> None:
        with self.lock:
            self.step()
            self.received.append({"t": round(self.rel(), 4), "kind": "param", "name": name, "payload": value})
            if name not in self.params:
                raise ToolError(f"parameter {name!r} not declared")
            try:
                v = float(value)
            except (TypeError, ValueError):
                raise ToolError(f"parameter {name} expects a number, got {value!r}") from None
            if not math.isfinite(v) or v < 0:
                raise ToolError(f"parameter {name}: {value!r} rejected")
            self.params[name] = v
            self._flag("PARAM_CHANGED", f"{name} = {v:g}")

    def read(self, topic: str) -> dict:
        with self.lock:
            self.step()
            if topic == "/odom":
                return {"header": {"stamp": round(self.rel(), 3), "frame_id": "odom"},
                        "pose": {"pose": {"position": {"x": self.x, "y": self.y, "z": 0.0},
                                          "orientation": {"x": 0.0, "y": 0.0, "z": math.sin(self.th / 2),
                                                          "w": math.cos(self.th / 2)}}},
                        "twist": {"twist": {"linear": {"x": self.v, "y": 0.0, "z": 0.0},
                                            "angular": {"x": 0.0, "y": 0.0, "z": self.w}}}}
            if topic == "/rover/status":
                return {"data": self.status_message}
            if topic == "/led":
                return {"data": self.led}
            if topic == "/cmd_vel":
                return {"linear": {"x": self.cmd_v, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": self.cmd_w}}
            raise ToolError(f"no messages on {topic!r}")


# ------------------------------------------------------------------ the reference raw connector
def _schema(props: dict, required: list) -> dict:
    return {"type": "object", "properties": props, "required": required}


RAW_TOOLS = [
    {"name": "get_topics", "description": "List topics and their types.", "inputSchema": _schema({}, [])},
    {"name": "subscribe_once", "description": "Read one message from a topic.",
     "inputSchema": _schema({"topic": {"type": "string"}, "msg_type": {"type": "string"},
                             "timeout": {"type": "number"}}, ["topic", "msg_type"])},
    {"name": "publish_once", "description": "Publish one message to any topic.",
     "inputSchema": _schema({"topic": {"type": "string"}, "msg_type": {"type": "string"},
                             "msg": {"type": "object"}}, ["topic", "msg_type", "msg"])},
    {"name": "publish_for_durations", "description": "Publish a sequence of messages, each for a duration.",
     "inputSchema": _schema({"topic": {"type": "string"}, "msg_type": {"type": "string"},
                             "messages": {"type": "array"}, "durations": {"type": "array"},
                             "rate_hz": {"type": "number"}}, ["topic", "msg_type", "messages", "durations"])},
    {"name": "call_service", "description": "Call any service.",
     "inputSchema": _schema({"service_name": {"type": "string"}, "service_type": {"type": "string"},
                             "request": {"type": "object"}, "timeout": {"type": "number"}},
                            ["service_name", "service_type", "request"])},
    {"name": "get_parameter", "description": "Read a parameter.",
     "inputSchema": _schema({"name": {"type": "string"}}, ["name"])},
    {"name": "set_parameter", "description": "Set a parameter (value as a string).",
     "inputSchema": _schema({"name": {"type": "string"}, "value": {"type": "string"}}, ["name", "value"])},
]


class RosStyleConnector:
    label = "ROS-style generic connector (reference, shaped like ros-mcp-server)"

    def __init__(self, robot: SimRover, link: Link | None = None):
        self.robot, self.link = robot, link or Link()
        self.clock = robot.clock
        self.lock = threading.RLock()
        self.schedule: list = []         # background publishers from publish_for_durations
        self.closed = False

    def list_tools(self) -> list:
        return [dict(t) for t in RAW_TOOLS]

    def _robot(self):
        if self.closed:
            raise ToolError("connector is closed")
        try:
            self.link.check()
        except BackendError as e:
            raise ToolError(str(e)) from None
        return self.robot

    def call_tool(self, name: str, args: dict) -> dict:
        if not isinstance(args, dict):
            raise ToolError("arguments must be an object")
        a = args
        if name == "get_topics":
            self._robot()
            return {"topics": list(TOPIC_TYPES), "types": list(TOPIC_TYPES.values())}
        if name == "subscribe_once":
            r = self._robot()
            topic = a.get("topic")
            if topic in TOPIC_TYPES and not _type_ok(a.get("msg_type"), TOPIC_TYPES[topic]):
                raise ToolError(f"{topic} has type {TOPIC_TYPES[topic]}")
            return {"msg": r.read(topic)}
        if name == "publish_once":
            self._robot().publish(a.get("topic"), a.get("msg_type"), a.get("msg"))
            return {"success": True}
        if name == "publish_for_durations":
            return self._publish_for_durations(a)
        if name == "call_service":
            return {"response": self._robot().call_service(a.get("service_name"), a.get("service_type"),
                                                           a.get("request", {}))}
        if name == "get_parameter":
            return {"name": a.get("name"), "value": str(self._robot().get_param(a.get("name")))}
        if name == "set_parameter":
            self._robot().set_param(a.get("name"), a.get("value"))
            return {"success": True}
        raise ToolError(f"unknown tool {name!r}")

    def _publish_for_durations(self, a) -> dict:
        msgs, durs, rate = a.get("messages"), a.get("durations"), a.get("rate_hz", 0)
        if not isinstance(msgs, list) or not isinstance(durs, list) or len(msgs) != len(durs) or not msgs:
            raise ToolError("messages and durations must be lists of the same, non-zero length")
        if isinstance(rate, bool) or not isinstance(rate, (int, float)) or not 0 <= rate <= 100:
            raise ToolError("rate_hz must be between 0 and 100")       # ros-mcp-server's only limit (rate_hz <= 100)
        for d in durs:
            if isinstance(d, bool) or not isinstance(d, (int, float)) or not math.isfinite(d) or d < 0:
                raise ToolError(f"bad duration {d!r}")
        self._robot()
        t = self.clock.now()
        with self.lock:
            for m, d in zip(msgs, durs):
                self.schedule.append({"topic": a.get("topic"), "msg_type": a.get("msg_type"), "msg": m,
                                      "start": t, "end": t + float(d), "rate": float(rate), "next": t})
                t += float(d)
        self.tick()
        return {"success": True, "messages": len(msgs), "total_s": round(t - self.clock.now(), 3)}

    def tick(self):
        """Run the background publishers up to now. No stop is sent when a sequence ends."""
        now = self.clock.now()
        with self.lock:
            keep = []
            for e in self.schedule:
                while e["next"] <= now + 1e-9 and e["next"] < e["end"] + 1e-9:
                    try:
                        self._robot().publish(e["topic"], e["msg_type"], e["msg"])
                    except ToolError:
                        pass                                    # robot unreachable: this publish is lost
                    e["next"] = e["next"] + 1.0 / e["rate"] if e["rate"] > 0 else float("inf")
                if e["next"] < e["end"] - 1e-9 or e["start"] > now:
                    keep.append(e)
            self.schedule = keep

    def close(self):
        """The connector process ends (its MCP client went away): background publishing stops. No stop is sent."""
        with self.lock:
            self.schedule = []
            self.closed = True
