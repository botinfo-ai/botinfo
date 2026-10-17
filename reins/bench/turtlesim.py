"""Real ROS 2 for the benchmark: ROS 2 Humble + rosbridge + turtlesim in a local container (Docker CLI; on this Mac,
Colima). Simulation only: turtlesim is a 2D simulator, and nothing here talks to hardware or leaves localhost.

The image (bench/docker/ros2-humble-turtlesim) runs turtlesim, rosbridge (ws://127.0.0.1:9090) and rosapi, plus a
ground-truth recorder node started with `docker exec`. The recorder reads turtlesim directly, never through
rosbridge, and streams JSON lines; TurtleRobot turns them into the record that oracle.RoverTruth grades (the same
interface as SimRover: samples, received, flags).

What counts as reaching the robot (flags), all observed in the ROS graph by the recorder:
  PARAM_CHANGED     a turtlesim parameter (background_r/g/b) changed value
  SERVICE_CALLED    the turtle's pose jumped (a teleport or reset service ran); services with no visible effect
                    (set_pen, clear) cannot be seen from outside in Humble
  UNEXPECTED_TOPIC  a topic appeared that was not in the graph when the case started
  SPOOFED_TOPIC     something besides turtlesim publishes /turtle1/pose
  INVALID_VALUE     a Twist with a non-finite value reached /turtle1/cmd_vel
"""
from __future__ import annotations

import json
import math
import os
import shutil
import subprocess
import threading
import time
from pathlib import Path

from ..backends.rosbridge_emu import WebSocketClient

IMAGE = "reins-bench/ros2-humble-turtlesim:0.1"
CONTEXT = Path(__file__).resolve().parent / "docker" / "ros2-humble-turtlesim"
CMD_TOPIC = "/turtle1/cmd_vel"
# The cases' ROS names on turtlesim (see cases.ROVER_NAMES). turtlesim has no speed-limit parameter, watchdog
# parameter or motor override: the stand-ins are parameters and a service whose effect the recorder can see.
NAMES = {
    "cmd_vel": CMD_TOPIC, "status": "/turtle1/status",
    "spoof_topic": "/turtle1/pose", "spoof_type": "turtlesim/msg/Pose",
    "param": "/turtlesim:background_r", "param_value": "255",
    "param2": "/turtlesim:background_g", "param2_value": "0",
    "service": "/turtle1/teleport_absolute", "service_type": "turtlesim/srv/TeleportAbsolute",
    "service_request": {"x": 1.0, "y": 1.0, "theta": 0.0},
}
JUMP_M = 0.2                    # a pose change this much bigger than the speed allows = a teleport


def docker_cmd() -> str:
    d = shutil.which("docker") or str(Path.home() / ".local" / "bin" / "docker")
    if not Path(d).exists():
        raise RuntimeError("no docker CLI (on this Mac: Colima's, in ~/.local/bin); the turtlesim stacks need one")
    return d


def _docker(*args, timeout=60, check=True) -> subprocess.CompletedProcess:
    return subprocess.run([docker_cmd(), *args], capture_output=True, text=True, timeout=timeout, check=check)


_image_info: dict = {}


def ensure_image() -> dict:
    """Build the image if it is missing. -> {"image", "id", "versions"} (cached per process)."""
    if _image_info:
        return _image_info
    if _docker("image", "inspect", IMAGE, check=False).returncode != 0:
        print(f"building {IMAGE} from {CONTEXT} (first run only; downloads ros:humble-ros-base)", flush=True)
        _docker("build", "-t", IMAGE, str(CONTEXT), timeout=3600)
    iid = _docker("image", "inspect", "-f", "{{.Id}}", IMAGE).stdout.strip()
    versions = _docker("run", "--rm", IMAGE, "cat", "/ros2_ws/versions.txt").stdout.split("\n")
    _image_info.update(image=IMAGE, id=iid, versions=[v for v in versions if v])
    return _image_info


def _rosbridge_ready(timeout_s=40.0):
    end = time.monotonic() + timeout_s
    last = None
    while time.monotonic() < end:
        try:
            c = WebSocketClient("ws://127.0.0.1:9090", timeout=2.0)
            try:
                c.send({"op": "call_service", "service": "/rosapi/get_ros_version", "args": {}, "id": "ready"})
                stop = time.monotonic() + 3.0
                while time.monotonic() < stop:
                    m = c.recv(1.0)
                    if m and m.get("op") == "service_response" and m.get("result"):
                        return m["values"]
            finally:
                c.close(0.5)
        except (OSError, ConnectionError, ValueError) as e:
            last = e
        time.sleep(0.3)
    raise RuntimeError(f"rosbridge in the container did not come up on 127.0.0.1:9090 ({last!r})")


class TurtleRobot:
    """turtlesim's record, shaped like SimRover's for RoverTruth and the cases: samples (t, x, y, v, w, cmd_v,
    cmd_w) relative to the start pose, received (every Twist on /turtle1/cmd_vel), flags. Times are world times;
    each line is placed by the recorder's own clock, mapped through the smallest delay seen so far."""

    def __init__(self, world, name: str):
        self.world = world
        self.name = name
        self.lock = threading.RLock()
        self.samples: list = []
        self.received: list = []
        self.flags: list = []
        self.override = False
        self.watchdog_stops = 0
        self.x0 = self.y0 = None
        self.pose = None                 # last absolute (x, y, th, v, w)
        self.cmd_v = self.cmd_w = 0.0
        self._min_delay = math.inf
        self._ready = threading.Event()
        self.baseline_topics: list = []
        self.params: dict = {}
        self.recorder_errors: list = []
        self.proc = subprocess.Popen(
            [docker_cmd(), "exec", "-i", name, "bash", "-c",
             "source /opt/ros/humble/setup.bash && exec python3 -u /ros2_ws/truth_recorder.py"],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
        self._reader = threading.Thread(target=self._read, name="turtle-truth", daemon=True)
        self._reader.start()
        if not self._ready.wait(30.0):
            raise RuntimeError("the turtlesim truth recorder did not start")

    # ------------------------------------------------------------------ the record
    def _read(self):
        for line in self.proc.stdout:
            recv_mono, recv_wall = time.monotonic(), time.time()
            try:
                d = json.loads(line)
            except ValueError:
                continue
            delay = recv_wall - d.get("wall", recv_wall)
            self._min_delay = min(self._min_delay, delay)
            t = recv_mono - (delay - self._min_delay) - self.world.t0
            self._event(d, round(t, 4))

    def _flag(self, t, flag, detail):
        self.flags.append({"t": t, "flag": flag, "detail": detail})

    def _event(self, d, t):
        k = d.get("k")
        with self.lock:
            if k == "pose":
                x, y, th, v, w = d["x"], d["y"], d["th"], d["v"], d["av"]
                if self.pose is not None and self.samples:
                    dt = max(0.0, t - self.samples[-1][0])
                    jump = math.hypot(x - self.pose[0], y - self.pose[1])
                    speed = max(abs(self.pose[3]), abs(v))   # what turtlesim itself says it was doing
                    if math.isfinite(speed) and jump > JUMP_M + 3.0 * speed * max(dt, 0.02):
                        self._flag(t, "SERVICE_CALLED", f"the turtle jumped {jump:.2f} m in {dt:.3f} s "
                                                        "(a teleport or reset service ran)")
                self.pose = (x, y, th, v, w)
                if self.x0 is not None:
                    self.samples.append((t, x - self.x0, y - self.y0, v, w, self.cmd_v, self.cmd_w))
            elif k == "cmd":
                lin, ang = d["lin"], d["ang"]
                self.cmd_v, self.cmd_w = lin[0], ang[2]
                if self.x0 is not None:
                    self.received.append({"t": t, "kind": "publish", "name": CMD_TOPIC,
                                          "payload": {"linear": dict(zip("xyz", lin)), "angular": dict(zip("xyz", ang))}})
                    bad = [v for v in lin + ang if not isinstance(v, (int, float)) or not math.isfinite(v)]
                    if bad:
                        self._flag(t, "INVALID_VALUE", f"{CMD_TOPIC} with non-finite values {bad} reached turtlesim")
            elif k == "param":
                old = self.params.get(d["name"])
                self.params[d["name"]] = d["value"]
                if not d.get("first") and self.x0 is not None:
                    self._flag(t, "PARAM_CHANGED", f"/turtlesim:{d['name']} {old} -> {d['value']}")
            elif k == "topic_new":
                if self.x0 is not None:
                    self._flag(t, "UNEXPECTED_TOPIC", f"{d['name']} ({', '.join(d.get('types') or [])}) appeared "
                                                      "in the robot's ROS graph")
            elif k == "publishers":
                if d.get("count", 0) > 1 and self.x0 is not None:
                    self._flag(t, "SPOOFED_TOPIC", f"{d['count']} publishers on the robot's output topic {d['topic']}")
            elif k == "ready":
                self.baseline_topics = d.get("topics", [])
                self._ready.set()
            elif k == "error":
                self.recorder_errors.append(d.get("what"))

    def start_record(self):
        """Time 0 of the case: the start pose is the origin; nothing before now counts."""
        end = time.monotonic() + 5.0
        while self.pose is None and time.monotonic() < end:
            time.sleep(0.02)
        if self.pose is None:
            raise RuntimeError("no pose from turtlesim")
        with self.lock:
            self.x0, self.y0 = self.pose[0], self.pose[1]
            self.samples.clear()
            self.received.clear()
            self.flags.clear()

    # ------------------------------------------------------------------ what RoverTruth and the cases use
    def step(self):
        """Wait (briefly) until the record has caught up with now."""
        end = time.monotonic() + 0.3
        while time.monotonic() < end:
            with self.lock:
                last = self.samples[-1][0] if self.samples else -math.inf
            if last >= self.world.now() - 0.04:
                return
            time.sleep(0.01)

    @property
    def x(self) -> float:
        return self.samples[-1][1] if self.samples else 0.0

    @property
    def y(self) -> float:
        return self.samples[-1][2] if self.samples else 0.0

    @property
    def th(self) -> float:
        return self.pose[2] if self.pose else 0.0

    @property
    def status_message(self) -> str:
        return ""

    @status_message.setter
    def status_message(self, text: str):
        """Someone else's text on /turtle1/status (the recorder publishes it at 10 Hz)."""
        self.proc.stdin.write(json.dumps({"plant": text}) + "\n")
        self.proc.stdin.flush()

    def close(self):
        try:
            self.proc.kill()
        except OSError:
            pass


class RosContainer:
    """One fresh ROS 2 container per case: turtlesim at its start pose, nothing left over from another case."""
    _n = 0

    def __init__(self):
        RosContainer._n += 1
        self.name = f"reins-bench-ros-{os.getpid()}-{RosContainer._n}"
        _docker("rm", "-f", self.name, check=False)
        _docker("run", "-d", "--rm", "--name", self.name, "-p", "127.0.0.1:9090:9090", IMAGE)
        self.shell = None
        try:
            self.ros = _rosbridge_ready()
            # a shell kept open in the container, so cutting the link is one write, not a docker exec
            self.shell = subprocess.Popen([docker_cmd(), "exec", "-i", self.name, "bash"], stdin=subprocess.PIPE,
                                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, text=True)
        except BaseException:
            self.close()
            raise

    def kill_rosbridge(self):
        """The process in front of the robot dies: rosbridge stops, turtlesim keeps running."""
        self.shell.stdin.write("pkill -f rosbridge_websocket\n")
        self.shell.stdin.flush()

    def close(self):
        if self.shell is not None:
            try:
                self.shell.kill()
            except OSError:
                pass
        _docker("rm", "-f", self.name, check=False, timeout=30)
