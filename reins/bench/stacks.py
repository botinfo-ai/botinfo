"""The stacks the benchmark grades: a robot (simulated or mocked), a connector, and optionally Reins in front.

STACKS run in simulated time (VirtualClock) on one thread, so each run is exact and repeatable. A stack gives the
hostile agent a tool surface (list_tools / call), and gives the harness the hooks a test needs: cut the robot
link, disconnect the agent, the owner's stop / resume / confirm, plant text in the robot's status.

Rows marked reference=True use BotInfo's own reference connectors (sesame.SesameDirectConnector,
simrover.RosStyleConnector). They model how existing connectors expose robots; they are not third-party code.

REALTIME_STACKS run a real third-party connector as a separate process, so they run in real time
(RealTimeWorld): the case script sleeps while ticker threads step the simulator, Reins and the agent's
behaviours. Today: ros-mcp-server v3.1.2 (github.com/robotmcp/ros-mcp-server, Apache-2.0) over MCP stdio, against
real ROS 2 + rosbridge + turtlesim in a container (bench/turtlesim.py) and against an emulated rosbridge in front
of SimRover (backends/rosbridge_emu.py); and unitree-go2-mcp-server at commit e7a6737
(github.com/lpigeon/unitree-go2-mcp-server, Apache-2.0) with the real ros2 CLI on ROS 2 Humble, in front of a
stand-in Go2 node in a second container (bench/go2.py). Real time is not repeatable to the millisecond; results
near a deadline can differ between runs.
"""
from __future__ import annotations

import json
import os
import shlex
import shutil
import subprocess
import tempfile
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

from ..audit import AuditLog
from ..backends.base import Link, ToolError
from ..backends.rosbridge_emu import RosbridgeEmulator
from ..backends.sesame import MockTransport, SesameBackend, SesameDirectConnector
from ..backends.sesame_mock import MockSesame
from ..backends.simrover import RosStyleConnector, SimRover
from ..backends.tooltemplate import ToolTemplateBackend, load_binding
from ..clock import RealClock, VirtualClock
from ..core import Reins
from ..mcp.client import StdioMCPClient
from ..policy import load_policy
from .cases import ROVER_NAMES, stop_variants
from .oracle import Go2Truth, RoverTruth, SesameTruth

ROOT = Path(__file__).resolve().parents[2]
POLICIES = ROOT / "policies"
BINDINGS = ROOT / "bindings"


class World:
    DT = 0.02

    def __init__(self):
        self.clock = VirtualClock()
        self.t0 = self.clock.now()
        self.tickers: list = []

    def now(self) -> float:
        return self.clock.now() - self.t0

    def run_for(self, seconds: float):
        self.run_until(self.now() + seconds)

    def run_until(self, t: float):
        while self.now() < t - 1e-9:
            self.clock.advance(min(self.DT, t - self.now()))
            for f in list(self.tickers):
                f()

    def at(self, t: float, fn):
        """Run fn once, at time t, on the world's own timeline (a harness event that must not wait for the
        agent's calls, e.g. the owner's stop). Here it runs after the other tickers of the step that reaches t."""
        def once():
            if self.now() >= t - 1e-9:
                self.tickers.remove(once)
                fn()
        self.tickers.append(once)

    def close(self):
        pass


class RealTimeWorld:
    """Real time on a RealClock. run_until() sleeps; each ticker (the simulator, Reins, the agent's behaviours)
    runs every DT on its own thread, so a connector call that blocks one of them never stalls the others.
    SimRover integrates its physics from its own record, so its ground truth stays exact even if a step is late."""
    DT = 0.02
    realtime = True

    def __init__(self):
        self.clock = RealClock()
        self.t0 = self.clock.now()
        self.tickers: list = []
        self.tick_errors: list = []
        self._running: dict = {}
        self._stop = threading.Event()
        self._sup = None

    def now(self) -> float:
        return self.clock.now() - self.t0

    def start(self):
        self._sup = threading.Thread(target=self._supervise, name="world-supervisor", daemon=True)
        self._sup.start()

    def _supervise(self):
        while not self._stop.is_set():
            for f in list(self.tickers):
                if id(f) not in self._running and not self._stop.is_set():
                    t = threading.Thread(target=self._run_ticker, args=(f,), name="world-ticker", daemon=True)
                    t.start()
                    self._running[id(f)] = (f, t)
            self._stop.wait(self.DT)

    def _run_ticker(self, f):
        nxt = time.monotonic()
        while not self._stop.is_set():
            try:
                f()
            except Exception as e:  # a ticker failure is recorded and reported, never hidden
                if len(self.tick_errors) < 20:
                    self.tick_errors.append(repr(e)[:300])
            nxt += self.DT
            delay = nxt - time.monotonic()
            if delay < 0:
                nxt, delay = time.monotonic(), 0.0
            self._stop.wait(delay)

    def run_for(self, seconds: float):
        self.run_until(self.now() + seconds)

    def run_until(self, t: float):
        while True:
            d = t - self.now()
            if d <= 0:
                return
            time.sleep(min(d, 0.05))

    def at(self, t: float, fn):
        """Run fn once at time t on its own thread, whatever the case script (and the agent) is doing then."""
        def go():
            while not self._stop.is_set():
                d = t - self.now()
                if d <= 0:
                    try:
                        fn()
                    except Exception as e:  # recorded like a ticker failure
                        self.tick_errors.append(repr(e)[:300])
                    return
                self._stop.wait(min(d, 0.005))
        th = threading.Thread(target=go, name="world-at", daemon=True)
        self._running[id(th)] = (go, th)
        th.start()

    def close(self):
        self._stop.set()
        if self._sup is not None:
            self._sup.join(timeout=0.5)
        for _, t in list(self._running.values()):
            t.join(timeout=0.5)


class Surface:
    """What the agent can call. Uniform result: {"ok": bool, ...}.

    background: tool names (or a function (name, args) -> bool) whose calls the agent's client keeps in flight on
    their own thread instead of waiting (real-time stacks only). ros-mcp-server's publish_for_durations, and
    unitree-go2-mcp-server's timed pub_wirelesscontroller and named moves, block for the whole motion; the case
    still has to cut the link or stop at its scheduled time. Requests still reach the connector in the order sent."""

    def __init__(self, target, is_reins: bool, background=()):
        self.target, self.is_reins = target, is_reins
        names = set(() if callable(background) else background)
        self.in_flight = background if callable(background) else (lambda name, args: name in names)
        self.background_log: list = []
        self.disconnected = False

    def list_tools(self) -> list:
        return self.target.list_tools()

    def call(self, name, args) -> dict:
        if self.disconnected:
            return {"ok": False, "error": "agent disconnected"}
        if self.is_reins:
            return self.target.call_tool(name, args)
        if self.in_flight(name, args):
            return self._in_flight(name, args)
        try:
            return {"ok": True, "result": self.target.call_tool(name, args)}
        except ToolError as e:
            return {"ok": False, "error": str(e)}

    def _in_flight(self, name, args) -> dict:
        sent = threading.Event()
        rec = {"tool": name, "result": None}

        def go():
            try:
                rec["result"] = self.target.call_tool(name, args, on_sent=sent.set)
            except ToolError as e:
                rec["result"] = {"error": str(e)[:200]}
            finally:
                sent.set()
        self.background_log.append(rec)
        threading.Thread(target=go, name="agent-in-flight", daemon=True).start()
        sent.wait(2.0)
        return {"ok": True, "result": {"in_flight": True}}


@dataclass
class Stack:
    id: str
    label: str
    robot: str                       # "sesame" | "rover"
    layer: str                       # "raw" | "reins"
    robot_config: str
    connector_label: str
    reference: bool
    world: World
    surface: Surface
    truth: object
    link: Link
    reins: Reins | None = None
    connector: object = None
    mock: object = None
    rover: object = None
    agent: object = None
    planted: list = field(default_factory=list)
    notes: str = ""
    realtime: bool = False
    group: str = ""                  # results/realtime/<group>.json for real-time stacks
    fidelity: str = ""
    extras: object = None            # () -> dict, added to each result as connector_trace
    closers: list = field(default_factory=list)
    names: dict = None               # the ROS names the cases use (cases.ROVER_NAMES when None)
    on_cut: object = None            # what cutting the link does besides link.up = False (real ROS: kill rosbridge)

    # ---------------------------------------------------------------- harness hooks
    def agent_disconnect(self):
        """The agent (MCP client) goes away. A stdio connector process exits with it."""
        if self.agent is not None:
            self.agent.gone()
        self.surface.disconnected = True
        if self.reins is not None:
            self.reins.client_disconnected()
        elif self.connector is not None:
            self.connector.close()

    def cut_link(self):
        """The wire to the robot drops (or the process in front of the robot dies) and stays down."""
        self.link.up = False
        if self.on_cut is not None:
            self.on_cut()

    def owner_stop(self):
        """The owner's software stop. Behind Reins: the owner channel. Raw: the connector's own stop route."""
        if self.reins is not None:
            return self.reins.owner_stop("benchmark owner stop")
        name, args = stop_variants(self.robot, self.names or ROVER_NAMES)[0]
        if self.realtime:                # the owner is not the agent: their stop must not wait in the script
            sent = threading.Event()

            def go():
                try:
                    self.connector.call_tool(name, args, on_sent=sent.set)
                except ToolError:
                    pass
                finally:
                    sent.set()
            threading.Thread(target=go, name="owner-stop", daemon=True).start()
            sent.wait(2.0)
            return
        try:
            self.connector.call_tool(name, args)
        except ToolError:
            pass

    def owner_resume(self):
        if self.reins is not None:
            self.reins.owner_resume()

    def owner_confirm_all(self) -> int:
        if self.reins is None:
            return 0
        ids = [p["id"] for p in self.reins.owner_status()["pending"]]
        for cid in ids:
            self.reins.owner_confirm(cid)
        return len(ids)

    def plant_status_text(self, text: str) -> bool:
        """Someone else puts text where the robot reports status. Sesame: any client on the robot's open access
        point can set currentCommand to any string (stock firmware). SimRover: its /rover/status string."""
        if self.robot == "sesame":
            body = json.dumps({"command": text})
            status, _, _ = self.mock.handle("POST", "/api/command", body)
            self.planted.append(text)
            return status == 200
        self.rover.status_message = text
        return True

    def finish(self):
        if self.mock is not None:
            self.mock.advance()
        if self.rover is not None:
            self.rover.step()

    def close(self):
        """Tear the stack down (processes, sockets, threads). Calling it twice does nothing more."""
        self.world.close()
        closers, self.closers = self.closers, []
        for c in closers:
            try:
                c()
            except Exception:  # teardown must not hide the case result
                pass


# -------------------------------------------------------------------- factories
def _sesame(stack_id, patched: bool, with_reins: bool) -> Stack:
    w = World()
    mock = MockSesame(clock=w.clock, safety_patch=patched)
    link = Link()
    transport = MockTransport(mock, link)
    cfg = "BotInfo motion-timeout patch (3 s)" if patched else "stock firmware d106cbc8 (no timeout)"
    reins = None
    if with_reins:
        reins = Reins(load_policy(POLICIES / "sesame.toml"), SesameBackend(transport), clock=w.clock,
                      audit=AuditLog(clock=w.clock))
        surface = Surface(reins, True)
        w.tickers.append(reins.tick)
        connector = None
    else:
        connector = SesameDirectConnector(transport)
        surface = Surface(connector, False)
    w.tickers.append(mock.advance)
    label = ("Reins -> " if with_reins else "") + f"Sesame firmware API ({'patched' if patched else 'stock'})"
    return Stack(id=stack_id, label=label, robot="sesame", layer="reins" if with_reins else "raw", robot_config=cfg,
                 connector_label=SesameDirectConnector.label if not with_reins else "Reins + Sesame firmware API",
                 reference=not with_reins, world=w, surface=surface, truth=SesameTruth(mock, w), link=link,
                 reins=reins, connector=connector, mock=mock)


def _rover(stack_id, watchdog_s: float, with_reins: bool) -> Stack:
    w = World()
    rover = SimRover(w.clock, cmd_timeout_s=watchdog_s)
    link = Link()
    connector = RosStyleConnector(rover, link)
    cfg = f"base cmd_vel timeout {watchdog_s:g} s" if watchdog_s else "no base watchdog"
    reins = None
    w.tickers.append(connector.tick)
    if with_reins:
        backend = ToolTemplateBackend(connector, load_binding(BINDINGS / "simrover-ros-style.toml"))
        reins = Reins(load_policy(POLICIES / "simrover.toml"), backend, clock=w.clock, audit=AuditLog(clock=w.clock))
        surface = Surface(reins, True)
        w.tickers.append(reins.tick)
    else:
        surface = Surface(connector, False)
    w.tickers.append(rover.step)
    label = ("Reins -> " if with_reins else "") + f"ROS-style connector, SimRover ({cfg})"
    return Stack(id=stack_id, label=label, robot="rover", layer="reins" if with_reins else "raw", robot_config=cfg,
                 connector_label=RosStyleConnector.label, reference=True, world=w, surface=surface,
                 truth=RoverTruth(rover, w), link=link, reins=reins, connector=connector, rover=rover)


STACKS = {
    "sesame-stock/raw": lambda: _sesame("sesame-stock/raw", False, False),
    "sesame-patched/raw": lambda: _sesame("sesame-patched/raw", True, False),
    "sesame-stock/reins": lambda: _sesame("sesame-stock/reins", False, True),
    "sesame-patched/reins": lambda: _sesame("sesame-patched/reins", True, True),
    "rover-nowd/raw": lambda: _rover("rover-nowd/raw", 0.0, False),
    "rover-wd/raw": lambda: _rover("rover-wd/raw", 0.5, False),
    "rover-nowd/reins": lambda: _rover("rover-nowd/reins", 0.0, True),
    "rover-wd/reins": lambda: _rover("rover-wd/reins", 0.5, True),
}


# -------------------------------------------------------------------- real time: ros-mcp-server (third-party code)
ROS_MCP_VERSION = "3.1.2"
ROS_MCP_LABEL = f"ros-mcp-server v{ROS_MCP_VERSION} against an emulated rosbridge (not real ROS)"
ROS_MCP_GROUP = f"ros-mcp-server-v{ROS_MCP_VERSION}-emulated-rosbridge"
ROSBRIDGE_PORT = 9090            # ros-mcp-server's built-in default (ros_mcp/main.py); we never call connect_to_robot
ROS_MCP_FIDELITY = (
    "Real ros-mcp-server v3.1.2 code (PyPI ros-mcp 3.1.2, FastMCP 2.x), launched over MCP stdio. The robot is "
    "SimRover; ROS is replaced by an emulated rosbridge (reins/backends/rosbridge_emu.py) that follows rosbridge's "
    "ros2 branch where it matters here (no reply to advertise or publish, implicit publisher registration, 10 s "
    "unregister timeout). No DDS, QoS or discovery. Real time; see docs/GRADING-EXISTING-CONNECTORS.md.")
_SERVER_ENV_KEYS = ("PATH", "HOME", "TMPDIR", "LANG", "LC_ALL", "UV_CACHE_DIR", "UV_TOOL_DIR", "UV_PYTHON_INSTALL_DIR",
                    "XDG_CACHE_HOME")
_version_cache: dict = {}


def ros_mcp_command() -> list:
    """REINS_ROS_MCP_CMD (a command line) if set, else uvx with the version pinned."""
    cmd = os.environ.get("REINS_ROS_MCP_CMD")
    if cmd:
        return shlex.split(cmd)
    return ["uvx", "--from", f"ros-mcp=={ROS_MCP_VERSION}", "ros-mcp", "--transport=stdio"]


def ros_mcp_version(cmd: list) -> str:
    """The installed ros-mcp version, checked before a row is labelled with it."""
    key = tuple(cmd)
    if key in _version_cache:
        return _version_cache[key]
    if any(c == f"ros-mcp=={ROS_MCP_VERSION}" for c in cmd):
        v = ROS_MCP_VERSION
    else:
        exe = shutil.which(cmd[0]) or cmd[0]
        py = Path(exe).with_name("python")
        if not py.exists():
            raise RuntimeError(f"cannot check which ros-mcp version {cmd[0]!r} runs; pin it (uvx --from "
                               f"ros-mcp=={ROS_MCP_VERSION}) or point REINS_ROS_MCP_CMD at a venv's bin/ros-mcp")
        out = subprocess.run([str(py), "-I", "-c", "import importlib.metadata as m; print(m.version('ros-mcp'))"],
                             capture_output=True, text=True, timeout=30)
        v = out.stdout.strip()
    if v != ROS_MCP_VERSION:
        raise RuntimeError(f"ros-mcp {v or '?'} found; these rows are for v{ROS_MCP_VERSION}")
    _version_cache[key] = v
    return v


def _server_env() -> dict:
    """A small environment for third-party code: no tokens or keys from ours. FastMCP's banner would also check
    pypi.org for a newer FastMCP; both are switched off so the server only ever talks to localhost."""
    env = {k: os.environ[k] for k in _SERVER_ENV_KEYS if k in os.environ}
    env.update(FASTMCP_CHECK_FOR_UPDATES="off", FASTMCP_SHOW_CLI_BANNER="false", PYTHONUNBUFFERED="1")
    return env


def _kill(client: StdioMCPClient):
    try:
        client.proc.kill()
    except OSError:
        pass
    client.close()


def _launch_ros_mcp(closers: list):
    """ros-mcp-server as its own process over MCP stdio, in a small environment and a scratch working directory."""
    cmd = ros_mcp_command()
    if not shutil.which(cmd[0]) and not Path(cmd[0]).exists():
        raise RuntimeError(f"{cmd[0]!r} not found: install uv, or set REINS_ROS_MCP_CMD to a ros-mcp command")
    version = ros_mcp_version(cmd)
    keep = os.environ.get("REINS_BENCH_LOGS")
    workdir = Path(tempfile.mkdtemp(prefix="reins-rosmcp-", dir=keep or None))
    err = open(workdir / "ros-mcp-server.stderr.log", "w")
    client = StdioMCPClient(cmd, env=_server_env(), cwd=str(workdir), timeout_s=15.0, stderr=err)
    closers.append(lambda: _kill(client))
    closers.append(err.close)
    if not keep:
        closers.append(lambda: shutil.rmtree(workdir, ignore_errors=True))
    return client, version


def _front(client, with_reins: bool, binding: str, policy: str, w):
    """Raw: the agent gets ros-mcp-server's own tools. Reins: the agent gets Reins, which drives ros-mcp-server."""
    if not with_reins:
        return None, Surface(client, False, background=("publish_for_durations",))
    backend = ToolTemplateBackend(client, load_binding(BINDINGS / binding))
    reins = Reins(load_policy(POLICIES / policy), backend, clock=w.clock, audit=AuditLog(clock=w.clock))
    w.tickers.append(reins.tick)
    return reins, Surface(reins, True)


def _teardown(w, closers):
    for c in [w.close] + closers:
        try:
            c()
        except Exception:  # teardown must not hide the original error
            pass


def _extras(w, surface, more: dict):
    def extras():
        d = {k: (v() if callable(v) else v) for k, v in more.items()}
        if w.tick_errors:
            d["tick_errors"] = list(w.tick_errors)
        if surface.background_log:
            d["in_flight_calls"] = [{"tool": r["tool"], "result": r["result"]} for r in surface.background_log][:5]
        return d
    return extras


def _ros_mcp_emulated(stack_id: str, with_reins: bool) -> Stack:
    w = RealTimeWorld()
    rover = SimRover(w.clock, cmd_timeout_s=0.0)
    link = Link()
    closers = []
    try:
        emu = RosbridgeEmulator(rover, link=link, port=ROSBRIDGE_PORT).start()
        client, version = _launch_ros_mcp(closers)
        closers.append(emu.close)
        reins, surface = _front(client, with_reins, "ros-mcp-server-simrover.toml", "simrover.toml", w)
        w.tickers.append(rover.step)
    except BaseException:
        _teardown(w, closers)
        raise
    with rover.lock:                     # the case's time 0 is now, after the server has started
        rover.step()
        t = rover.t_last
        rover.t0 = rover.cmd_t = t
        rover.samples.clear()
        rover.received.clear()
        rover.flags.clear()
    w.t0 = t
    emu.reset_record()
    w.start()
    info = {"server": client.server_info, "ros_mcp_version": version, "tools": len(client.tools)}
    label = ("Reins -> " if with_reins else "") + ROS_MCP_LABEL
    return Stack(id=stack_id, label=label, robot="rover", layer="reins" if with_reins else "raw",
                 robot_config="no base watchdog", connector_label=f"ros-mcp-server v{version} (third-party)",
                 reference=False, world=w, surface=surface, truth=RoverTruth(rover, w), link=link, reins=reins,
                 connector=client, rover=rover, realtime=True, group=ROS_MCP_GROUP, fidelity=ROS_MCP_FIDELITY,
                 extras=_extras(w, surface, {"rosbridge": emu.summary, "connector": info}), closers=closers)


ROS2_LABEL = f"ros-mcp-server v{ROS_MCP_VERSION} + rosbridge + turtlesim on real ROS 2 Humble (in a container)"
ROS2_GROUP = f"ros-mcp-server-v{ROS_MCP_VERSION}-ros2-humble-turtlesim"


def _ros2_fidelity(img: dict) -> str:
    return (f"Real ros-mcp-server v{ROS_MCP_VERSION} code (PyPI ros-mcp {ROS_MCP_VERSION}) over MCP stdio, real ROS 2 "
            f"Humble ({'; '.join(img['versions'])}) in a local container ({img['image']}, Colima on macOS arm64), "
            "fresh for every case. Ground truth comes from a ROS 2 node in the same container that reads turtlesim "
            "directly, not through rosbridge. The robot is turtlesim, a simulator: it stops by itself 1 s after the "
            "last Twist. Real time; see docs/GRADING-EXISTING-CONNECTORS.md.")


def _ros_mcp_turtlesim(stack_id: str, with_reins: bool) -> Stack:
    from . import turtlesim as ts
    img = ts.ensure_image()
    w = RealTimeWorld()
    link = Link()
    closers = []
    ctr = turtle = None
    try:
        ctr = ts.RosContainer()
        turtle = ts.TurtleRobot(w, ctr.name)
        client, version = _launch_ros_mcp(closers)
        reins, surface = _front(client, with_reins, "ros-mcp-server-turtlesim.toml", "turtlesim.toml", w)
    except BaseException:
        _teardown(w, closers + [c.close for c in (turtle, ctr) if c is not None])
        raise
    closers += [turtle.close, ctr.close]  # the container goes last
    w.t0 = w.clock.now()                 # the case's time 0 is now, after everything has started
    turtle.start_record()
    w.start()
    info = {"server": client.server_info, "ros_mcp_version": version, "tools": len(client.tools)}
    ros = {"image": img["image"], "image_id": img["id"], "packages": img["versions"], "rosapi": ctr.ros}
    label = ("Reins -> " if with_reins else "") + ROS2_LABEL
    return Stack(id=stack_id, label=label, robot="rover", layer="reins" if with_reins else "raw",
                 robot_config="turtlesim's own stop 1 s after the last Twist",
                 connector_label=f"ros-mcp-server v{version} (third-party)", reference=False, world=w,
                 surface=surface, truth=RoverTruth(turtle, w, cmd_topic=ts.CMD_TOPIC), link=link, reins=reins,
                 connector=client, rover=turtle, realtime=True, group=ROS2_GROUP, fidelity=_ros2_fidelity(img),
                 names=ts.NAMES, on_cut=ctr.kill_rosbridge,
                 extras=_extras(w, surface, {"connector": info, "ros": ros,
                                             "recorder": lambda: {"baseline_topics": len(turtle.baseline_topics),
                                                                  "errors": turtle.recorder_errors[:5]}}),
                 closers=closers)


# -------------------------------------------------------------------- real time: unitree-go2-mcp-server (third-party)
GO2_GROUP = "unitree-go2-mcp-server-e7a6737-ros2-humble-standin"
GO2_LABEL = "unitree-go2-mcp-server (commit e7a6737) + ros2 CLI on ROS 2 Humble, stand-in Go2"
GO2_NAMED_MOVES = ("stand_up_from_a_fall", "stretch", "shake_hands", "love", "pounce", "jump_forward", "sit_down",
                   "greet", "dance")


def _go2_blocks(name, args) -> bool:
    """The server's calls that block for a whole motion: a timed pub_wirelesscontroller and the named moves."""
    if name in GO2_NAMED_MOVES:
        return True
    if name == "pub_wirelesscontroller" and isinstance(args, dict):
        d = args.get("duration", 0)
        return isinstance(d, (int, float)) and not isinstance(d, bool) and d > 0
    return False


def _go2_config(stick_timeout_s: float) -> str:
    return (f"stand-in centres the sticks {stick_timeout_s:g} s after the last message" if stick_timeout_s
            else "stand-in holds the last joystick input (no robot-side timeout)")


def _go2_fidelity(img: dict) -> str:
    return ("Real unitree-go2-mcp-server code at commit e7a6737 (unmodified; mcp 1.7.1 from its uv.lock) over MCP "
            "stdio, and the real ros2 CLI it runs for every command, on ROS 2 Humble with CycloneDDS "
            f"({'; '.join(img['versions'])}). Two local containers ({img['image']}, Colima on macOS arm64) on a "
            "private network stand for the PC and the robot, fresh for every case. The robot is NOT a Go2 or a Go2 "
            "simulator: it is BotInfo's stand-in node that takes the server's /wirelesscontroller messages, uses "
            "the server's own scale (3.7 m/s or rad/s at full stick), changes speed instantly and records named "
            "moves as button presses without acting them out. Ground truth comes from that node, never from the "
            "connector. How a real Go2 treats this topic, and whether it holds the last input, is not known to us; "
            "both behaviours are graded. Real time; see docs/GRADING-EXISTING-CONNECTORS.md.")


def _go2(stack_id: str, with_reins: bool, stick_timeout_s: float) -> Stack:
    from . import go2
    img = go2.ensure_image()
    w = RealTimeWorld()
    link = Link()
    closers = []
    pair = robot = None
    try:
        pair = go2.Go2Pair()
        robot = go2.Go2Robot(w, pair, stick_timeout_s)
        keep = os.environ.get("REINS_BENCH_LOGS")
        workdir = Path(tempfile.mkdtemp(prefix="reins-go2mcp-", dir=keep or None))
        err = open(workdir / "go2-mcp-server.stderr.log", "w")
        closers.append(err.close)
        if not keep:
            closers.append(lambda: shutil.rmtree(workdir, ignore_errors=True))
        client = go2.ContainerMCPClient(pair, timeout_s=15.0, stderr=err)
        closers.insert(0, lambda: _kill(client))
        if with_reins:
            backend = ToolTemplateBackend(client, load_binding(BINDINGS / "unitree-go2-mcp-server.toml"))
            reins = Reins(load_policy(POLICIES / "go2.toml"), backend, clock=w.clock, audit=AuditLog(clock=w.clock))
            w.tickers.append(reins.tick)
            surface = Surface(reins, True)
        else:
            reins, surface = None, Surface(client, False, background=_go2_blocks)
    except BaseException:
        _teardown(w, closers + [c.close for c in (robot, pair) if c is not None])
        raise
    closers += [robot.close, pair.close]   # the containers go last
    w.t0 = w.clock.now()                   # the case's time 0 is now, after everything has started
    robot.start_record()
    w.start()
    cfg = _go2_config(stick_timeout_s)
    info = {"server": client.server_info, "commit": go2.COMMIT, "tools": len(client.tools)}
    ros = {"image": img["image"], "image_id": img["id"], "packages": img["versions"]}
    short = (f"centres the sticks {stick_timeout_s:g} s after the last message" if stick_timeout_s
             else "holds the last joystick input")
    label = ("Reins -> " if with_reins else "") + f"{GO2_LABEL} that {short}"
    return Stack(id=stack_id, label=label, robot="rover", layer="reins" if with_reins else "raw", robot_config=cfg,
                 connector_label=f"unitree-go2-mcp-server {go2.COMMIT} (third-party)", reference=False, world=w,
                 surface=surface, truth=Go2Truth(robot, w, cmd_topic=go2.CMD_TOPIC), link=link, reins=reins,
                 connector=client, rover=robot, realtime=True, group=GO2_GROUP, fidelity=_go2_fidelity(img),
                 names=go2.NAMES, on_cut=pair.cut_link,
                 extras=_extras(w, surface, {"connector": info, "ros": ros,
                                             "standin": lambda: {"stick_timeout_s": stick_timeout_s,
                                                                 "timeouts": robot.watchdog_stops}}),
                 closers=closers)


REALTIME_STACKS = {
    "go2-hold/raw": lambda: _go2("go2-hold/raw", False, 0.0),
    "go2-hold/reins": lambda: _go2("go2-hold/reins", True, 0.0),
    "go2-timeout/raw": lambda: _go2("go2-timeout/raw", False, 0.5),
    "go2-timeout/reins": lambda: _go2("go2-timeout/reins", True, 0.5),
    "ros-mcp-server-turtlesim/raw": lambda: _ros_mcp_turtlesim("ros-mcp-server-turtlesim/raw", False),
    "ros-mcp-server-turtlesim/reins": lambda: _ros_mcp_turtlesim("ros-mcp-server-turtlesim/reins", True),
    "ros-mcp-server-emu/raw": lambda: _ros_mcp_emulated("ros-mcp-server-emu/raw", False),
    "ros-mcp-server-emu/reins": lambda: _ros_mcp_emulated("ros-mcp-server-emu/reins", True),
}
ALL_STACKS = {**STACKS, **REALTIME_STACKS}
