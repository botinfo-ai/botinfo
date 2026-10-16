"""The stacks the benchmark grades: a robot (simulated or mocked), a connector, and optionally Reins in front.

Every stack runs in simulated time (VirtualClock) on one thread, so each run is exact and repeatable. A stack
gives the hostile agent a tool surface (list_tools / call), and gives the harness the hooks a test needs:
cut the robot link, disconnect the agent, the owner's stop / resume / confirm, plant text in the robot's status.

Rows marked reference=True use BotInfo's own reference connectors (sesame.SesameDirectConnector,
simrover.RosStyleConnector). They model how existing connectors expose robots; they are not third-party code.
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

from ..audit import AuditLog
from ..backends.base import Link, ToolError
from ..backends.sesame import MockTransport, SesameBackend, SesameDirectConnector
from ..backends.sesame_mock import MockSesame
from ..backends.simrover import RosStyleConnector, SimRover
from ..backends.tooltemplate import ToolTemplateBackend, load_binding
from ..clock import VirtualClock
from ..core import Reins
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


class Surface:
    """What the agent can call. Uniform result: {"ok": bool, ...}."""

    def __init__(self, target, is_reins: bool):
        self.target, self.is_reins = target, is_reins
        self.disconnected = False

    def list_tools(self) -> list:
        return self.target.list_tools()

    def call(self, name, args) -> dict:
        if self.disconnected:
            return {"ok": False, "error": "agent disconnected"}
        if self.is_reins:
            return self.target.call_tool(name, args)
        try:
            return {"ok": True, "result": self.target.call_tool(name, args)}
        except ToolError as e:
            return {"ok": False, "error": str(e)}


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
        try:
            if self.robot == "sesame":
                self.connector.call_tool("send_command", {"command": "stop"})
            else:
                self.connector.call_tool("publish_once", {"topic": "/cmd_vel", "msg_type": "geometry_msgs/msg/Twist",
                                                          "msg": ZERO_TWIST})
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
