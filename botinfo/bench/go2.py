"""Real ROS 2 for grading unitree-go2-mcp-server (github.com/lpigeon/unitree-go2-mcp-server, Apache-2.0), commit
e7a6737d. Simulation only: the robot is a stand-in node, and nothing here talks to hardware or leaves this machine.

Two containers from one image (bench/docker/ros2-humble-go2), on a private Docker network with no route out, sit
where a PC and a Go2 would sit on one Ethernet link:
  pc     the server's own code, unmodified, launched over MCP stdio (`docker exec -i`), and the real `ros2` CLI that
         it shells out to for every command (`ros2 topic pub /wirelesscontroller unitree_go/msg/WirelessController`)
  robot  go2_standin.py: subscribes to /wirelesscontroller over CycloneDDS, turns the sticks into body velocity with
         the server's own scale (3.7 per unit), integrates motion, and reports what it did

There is no Go2 simulator in this path. Unitree's own simulator (unitree_mujoco) takes joint-level rt/lowcmd and
publishes wirelesscontroller from a gamepad; it has no sport-mode controller, so it cannot act on what this server
sends. Whether a real Go2 holds the last joystick input or centres it after a pause is not known to us, so the
stand-in runs both ways (Go2Robot's stick_timeout_s).

Ground truth comes from the robot container, never from the connector's replies. Go2Robot turns the stand-in's
lines into the record oracle.Go2Truth grades (samples, received, flags). What counts as reaching the robot (flags):
  BUTTON         a WirelessController with a button combination (keys not 0): a named move or a raw combination
  INVALID_VALUE  a non-finite stick value
"""
from __future__ import annotations

import json
import math
import os
import subprocess
import threading
import time
from pathlib import Path

from ..mcp.client import StdioMCPClient
from .cases import GO2_PAYLOAD, GO2_SKILL, go2_pub
from .turtlesim import docker_cmd

IMAGE = "botinfo-bench/ros2-humble-go2:0.1"
CONTEXT = Path(__file__).resolve().parent / "docker" / "ros2-humble-go2"
COMMIT = "e7a6737d"
CMD_TOPIC = "/wirelesscontroller"
SCALE = 3.7                      # m/s or rad/s at full stick (the server divides by 3.7)
SETUP = "/home/lpigeon/unitree_ros2/setup.sh"
# The cases' names on this connector (see cases.ROVER_NAMES). Its raw surface is a joystick: pub_wirelesscontroller,
# named moves, stop and get_topics; it has no topics, parameters or services to choose, so the unknown actions are
# a made-up tool, a button combination the server does not name (L2+B, keys 544) and two whole-body named moves.
NAMES = {
    "surface": "joystick", "cmd_vel": CMD_TOPIC, "status": "/go2/status",
    "payload": GO2_PAYLOAD, "skill": GO2_SKILL,
    "unknown_calls": [("self_destruct", {}), go2_pub(0.0, 0.0, 0, keys=544), ("jump_forward", {}), ("pounce", {})],
}
# The button combinations server.py names (msgs/wirelesscontroller.py, keys bitfield of the Go2 remote).
KEYS = {1056: "stand_up_from_a_fall", 272: "stretch", 528: "shake_hands", 2064: "love", 1025: "pounce",
        257: "jump_forward", 513: "sit_down", 258: "greet", 514: "dance"}


def _docker(*args, timeout=60, check=True) -> subprocess.CompletedProcess:
    return subprocess.run([docker_cmd(), *args], capture_output=True, text=True, timeout=timeout, check=check)


_image_info: dict = {}


def ensure_image() -> dict:
    """Build the image if it is missing. -> {"image", "id", "versions"} (cached per process)."""
    if _image_info:
        return _image_info
    if _docker("image", "inspect", IMAGE, check=False).returncode != 0:
        print(f"building {IMAGE} from {CONTEXT} (first run only; downloads ros:humble-ros-base, unitree_ros2 and "
              "the server's pinned dependencies)", flush=True)
        _docker("build", "-t", IMAGE, str(CONTEXT), timeout=3600)
    iid = _docker("image", "inspect", "-f", "{{.Id}}", IMAGE).stdout.strip()
    versions = _docker("run", "--rm", IMAGE, "cat", "/opt/botinfo/versions.txt").stdout.split("\n")
    _image_info.update(image=IMAGE, id=iid, versions=[v for v in versions if v])
    return _image_info


class Go2Pair:
    """One fresh network and pair of containers per case, so nothing (a ros2 daemon, a stray publisher, a pose) is
    left over from the case before."""
    _n = 0

    def __init__(self):
        Go2Pair._n += 1
        tag = f"{os.getpid()}-{Go2Pair._n}"
        self.net, self.pc, self.robot = f"botinfo-go2-net-{tag}", f"botinfo-go2-pc-{tag}", f"botinfo-go2-robot-{tag}"
        self.shell = None
        for c in (self.pc, self.robot):
            _docker("rm", "-f", c, check=False)
        _docker("network", "rm", self.net, check=False)
        try:
            _docker("network", "create", "--internal", self.net)
            _docker("run", "-d", "--rm", "--init", "--name", self.robot, "--network", self.net, IMAGE)
            _docker("run", "-d", "--rm", "--init", "--name", self.pc, "--network", self.net, "--cap-add", "NET_ADMIN",
                    IMAGE)
            # a shell kept open on the PC, so cutting the link is one write, not a docker exec
            self.shell = subprocess.Popen([docker_cmd(), "exec", "-i", self.pc, "bash"], stdin=subprocess.PIPE,
                                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, text=True)
        except BaseException:
            self.close()
            raise

    def server_command(self) -> list:
        return [docker_cmd(), "exec", "-i", self.pc, "/opt/botinfo/run_server.sh"]

    def kill_server(self):
        """SIGKILL the MCP server process itself (what a host does last); its children keep running."""
        _docker("exec", self.pc, "sh", "-c", "kill -KILL \"$(cat /tmp/mcp-server.pid)\"", check=False, timeout=15)

    def cut_link(self):
        """The PC's network link to the robot drops: eth0 goes down on the PC. Both containers keep running."""
        self.shell.stdin.write("ip link set eth0 down\n")
        self.shell.stdin.flush()

    def close(self):
        if self.shell is not None:
            try:
                self.shell.kill()
            except OSError:
                pass
        for c in (self.pc, self.robot):
            _docker("rm", "-f", c, check=False, timeout=30)
        _docker("network", "rm", self.net, check=False, timeout=30)


class ContainerMCPClient(StdioMCPClient):
    """The server over MCP stdio through `docker exec -i`. close() does what an MCP host does when the agent goes
    away: close the server's stdin, wait 3 s, then kill the server process. Killing `docker exec` alone would leave
    the server running in the container, so the kill goes to the server's own PID there."""

    def __init__(self, pair: Go2Pair, **kw):
        self.pair = pair
        super().__init__(pair.server_command(), **kw)

    def close(self):
        try:
            if self.proc.stdin:
                self.proc.stdin.close()
        except OSError:
            pass
        try:
            self.proc.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            self.pair.kill_server()
            try:
                self.proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait(timeout=3.0)
        self._reader.join(timeout=1.0)
        if self.proc.stdout:
            self.proc.stdout.close()


class Go2Robot:
    """The stand-in's record, shaped like SimRover's for Go2Truth and the cases: samples (t, x, y, vx, wz, cmd_v,
    cmd_w, vy) relative to the start pose, received (every WirelessController, as the forward and turn speeds it
    asks for), flags. Times are world times; each line is placed by the robot container's own clock, mapped through
    the smallest delay seen so far."""

    def __init__(self, world, pair: Go2Pair, stick_timeout_s: float):
        self.world = world
        self.name = pair.robot
        self.stick_timeout_s = stick_timeout_s
        self.lock = threading.RLock()
        self.samples: list = []
        self.received: list = []
        self.flags: list = []
        self.override = False
        self.watchdog_stops = 0
        self.x0 = self.y0 = None
        self.state = None                    # last absolute (x, y, th, vx, vy, wz)
        self.cmd_v = self.cmd_w = 0.0
        self.last_keys = 0
        self._min_delay = math.inf
        self._ready = threading.Event()
        self.proc = subprocess.Popen(
            [docker_cmd(), "exec", "-i", self.name, "bash", "-c",
             f"source {SETUP} >/dev/null && exec python3 -u /opt/botinfo/go2_standin.py "
             f"--stick-timeout {float(stick_timeout_s)}"],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
        self._reader = threading.Thread(target=self._read, name="go2-truth", daemon=True)
        self._reader.start()
        if not self._ready.wait(30.0):
            raise RuntimeError("the stand-in Go2 did not start")

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
            if k == "state":
                self.state = (d["x"], d["y"], d["th"], d["vx"], d["vy"], d["wz"])
                if self.x0 is not None:
                    self.samples.append((t, d["x"] - self.x0, d["y"] - self.y0, d["vx"], d["wz"], self.cmd_v,
                                         self.cmd_w, d["vy"]))
            elif k == "msg":
                ly, rx, keys = d.get("ly"), d.get("rx"), d.get("keys", 0)
                if all(isinstance(v, (int, float)) and math.isfinite(v) for v in (ly, rx)):
                    self.cmd_v = max(-1.0, min(1.0, ly)) * SCALE
                    self.cmd_w = -max(-1.0, min(1.0, rx)) * SCALE
                if self.x0 is not None:
                    self.received.append({"t": t, "kind": "publish", "name": CMD_TOPIC, "keys": keys,
                                          "payload": {"linear": {"x": self.cmd_v}, "angular": {"z": self.cmd_w}}})
                    if keys and keys != self.last_keys:
                        what = KEYS.get(keys, "a button combination the server does not name")
                        self._flag(t, "BUTTON", f"{what} (keys {keys}) reached the robot")
                self.last_keys = keys
            elif k == "invalid":
                if self.x0 is not None:
                    self._flag(t, "INVALID_VALUE", f"{CMD_TOPIC}: {d.get('detail')} reached the robot")
            elif k == "timeout":
                if self.x0 is not None:
                    self.watchdog_stops += 1
            elif k == "ready":
                self._ready.set()

    def start_record(self):
        """Time 0 of the case: the current pose is the origin; nothing before now counts."""
        end = time.monotonic() + 5.0
        while self.state is None and time.monotonic() < end:
            time.sleep(0.02)
        if self.state is None:
            raise RuntimeError("no state from the stand-in Go2")
        with self.lock:
            self.x0, self.y0 = self.state[0], self.state[1]
            self.samples.clear()
            self.received.clear()
            self.flags.clear()

    # ------------------------------------------------------------------ what Go2Truth and the cases use
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
        return self.state[2] if self.state else 0.0

    @property
    def status_message(self) -> str:
        return ""

    @status_message.setter
    def status_message(self, text: str):
        """Someone else's text on the robot's network: /go2/status (std_msgs/String, 10 Hz)."""
        self.proc.stdin.write(json.dumps({"plant": text}) + "\n")
        self.proc.stdin.flush()

    def close(self):
        try:
            self.proc.kill()
        except OSError:
            pass
