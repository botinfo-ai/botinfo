"""The stacks the benchmark grades: a robot (simulated or mocked), a connector, and optionally Reins in front.

STACKS run in simulated time (VirtualClock) on one thread, so each run is exact and repeatable. A stack gives the
hostile agent a tool surface (list_tools / call), and gives the harness the hooks a test needs: cut the robot
link, disconnect the agent, the owner's stop / resume / confirm, plant text in the robot's status.

Rows marked reference=True use BotInfo's own reference connectors (sesame.SesameDirectConnector,
simrover.RosStyleConnector). They model how existing connectors expose robots; they are not third-party code.

REALTIME_STACKS run a real third-party connector as a separate process, so they run in real time
(RealTimeWorld): the case script sleeps while ticker threads step the simulator, Reins and the agent's
behaviours. Today that is ros-mcp-server v3.1.2 (github.com/robotmcp/ros-mcp-server, Apache-2.0), launched over
MCP stdio and pointed at an emulated rosbridge (backends/rosbridge_emu.py) in front of SimRover. Real time is
not repeatable to the millisecond; results near a deadline can differ between runs.
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
from .oracle import RoverTruth, SesameTruth

ROOT = Path(__file__).resolve().parents[2]
POLICIES = ROOT / "policies"
BINDINGS = ROOT / "bindings"
ZERO_TWIST = {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}


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

    background: tool names whose calls the agent's client keeps in flight on their own thread instead of waiting
    (real-time stacks only). ros-mcp-server's publish_for_durations blocks for the whole motion; the case still
    has to cut the link or stop at its scheduled time. Requests still reach the connector in the order sent."""

    def __init__(self, target, is_reins: bool, background=()):
        self.target, self.is_reins = target, is_reins
        self.background = set(background)
        self.background_log: list = []
        self.disconnected = False

    def list_tools(self) -> list:
        return self.target.list_tools()

    def call(self, name, args) -> dict:
        if self.disconnected:
            return {"ok": False, "error": "agent disconnected"}
        if self.is_reins:
            return self.target.call_tool(name, args)
        if name in self.background:
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

    def owner_stop(self):
        """The owner's software stop. Behind Reins: the owner channel. Raw: the connector's own stop route."""
        if self.reins is not None:
            return self.reins.owner_stop("benchmark owner stop")
        if self.robot == "sesame":
            name, args = "send_command", {"command": "stop"}
        else:
            name, args = "publish_once", {"topic": "/cmd_vel", "msg_type": "geometry_msgs/msg/Twist", "msg": ZERO_TWIST}
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


def _ros_mcp(stack_id: str, with_reins: bool) -> Stack:
    cmd = ros_mcp_command()
    if not shutil.which(cmd[0]) and not Path(cmd[0]).exists():
        raise RuntimeError(f"{cmd[0]!r} not found: install uv, or set REINS_ROS_MCP_CMD to a ros-mcp command")
    version = ros_mcp_version(cmd)
    w = RealTimeWorld()
    rover = SimRover(w.clock, cmd_timeout_s=0.0)
    link = Link()
    closers = []
    keep = os.environ.get("REINS_BENCH_LOGS")
    workdir = Path(tempfile.mkdtemp(prefix="reins-rosmcp-", dir=keep or None))
    try:
        emu = RosbridgeEmulator(rover, link=link, port=ROSBRIDGE_PORT).start()
        closers.append(emu.close)
        err = open(workdir / "ros-mcp-server.stderr.log", "w")
        closers.append(err.close)
        if not keep:
            closers.append(lambda: shutil.rmtree(workdir, ignore_errors=True))
        client = StdioMCPClient(cmd, env=_server_env(), cwd=str(workdir), timeout_s=15.0, stderr=err)
        closers.insert(0, lambda: _kill(client))
        reins = None
        if with_reins:
            backend = ToolTemplateBackend(client, load_binding(BINDINGS / "ros-mcp-server-simrover.toml"))
            reins = Reins(load_policy(POLICIES / "simrover.toml"), backend, clock=w.clock,
                          audit=AuditLog(clock=w.clock))
            surface = Surface(reins, True)
            w.tickers.append(reins.tick)
        else:
            surface = Surface(client, False, background=("publish_for_durations",))
        w.tickers.append(rover.step)
    except BaseException:
        for c in [w.close] + closers:
            try:
                c()
            except Exception:
                pass
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

    def extras():
        d = {"rosbridge": emu.summary(), "connector": info}
        if w.tick_errors:
            d["tick_errors"] = list(w.tick_errors)
        if surface.background_log:
            d["in_flight_calls"] = [{"tool": r["tool"], "result": r["result"]} for r in surface.background_log][:5]
        return d
    label = ("Reins -> " if with_reins else "") + ROS_MCP_LABEL
    return Stack(id=stack_id, label=label, robot="rover", layer="reins" if with_reins else "raw",
                 robot_config="no base watchdog", connector_label=f"ros-mcp-server v{version} (third-party)",
                 reference=False, world=w, surface=surface, truth=RoverTruth(rover, w), link=link, reins=reins,
                 connector=client, rover=rover, realtime=True, group=ROS_MCP_GROUP, fidelity=ROS_MCP_FIDELITY,
                 extras=extras, closers=closers)


REALTIME_STACKS = {
    "ros-mcp-server/raw": lambda: _ros_mcp("ros-mcp-server/raw", False),
    "ros-mcp-server/reins": lambda: _ros_mcp("ros-mcp-server/reins", True),
}
ALL_STACKS = {**STACKS, **REALTIME_STACKS}
