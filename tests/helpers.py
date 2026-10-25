from __future__ import annotations

from pathlib import Path

from botinfo.audit import AuditLog
from botinfo.backends.base import Link
from botinfo.backends.sesame import MockTransport, SesameBackend
from botinfo.backends.sesame_mock import MockSesame
from botinfo.backends.simrover import RosStyleConnector, SimRover
from botinfo.backends.tooltemplate import ToolTemplateBackend, load_binding
from botinfo.clock import VirtualClock
from botinfo.core import BotInfo
from botinfo.policy import load_policy, policy_from_dict

ROOT = Path(__file__).resolve().parents[1]


class RoverRig:
    """BotInfo -> reference connector -> SimRover, in simulated time."""

    def __init__(self, watchdog_s=0.0, policy=None, **botinfo_kw):
        self.clock = VirtualClock()
        self.rover = SimRover(self.clock, cmd_timeout_s=watchdog_s)
        self.link = Link()
        self.conn = RosStyleConnector(self.rover, self.link)
        self.backend = ToolTemplateBackend(self.conn, load_binding(ROOT / "bindings" / "simrover-ros-style.toml"))
        self.policy = policy or load_policy(ROOT / "policies" / "simrover.toml")
        self.botinfo = BotInfo(self.policy, self.backend, clock=self.clock, audit=AuditLog(clock=self.clock), **botinfo_kw)

    def run(self, seconds):
        end = self.clock.now() + seconds
        while self.clock.now() < end - 1e-9:
            self.clock.advance(0.02)
            self.conn.tick()
            self.botinfo.tick()
            self.rover.step()

    def call(self, name, args=None):
        return self.botinfo.call_tool(name, args or {})


class SesameRig:
    def __init__(self, patched=True, policy=None):
        self.clock = VirtualClock()
        self.mock = MockSesame(clock=self.clock, safety_patch=patched)
        self.link = Link()
        self.backend = SesameBackend(MockTransport(self.mock, self.link))
        self.botinfo = BotInfo(policy or load_policy(ROOT / "policies" / "sesame.toml"), self.backend, clock=self.clock,
                           audit=AuditLog(clock=self.clock))

    def run(self, seconds):
        end = self.clock.now() + seconds
        while self.clock.now() < end - 1e-9:
            self.clock.advance(0.02)
            self.botinfo.tick()
            self.mock.advance()

    def call(self, name, args=None):
        return self.botinfo.call_tool(name, args or {})

    def sent(self):
        return [(round(t, 2), k, v) for t, k, v in self.mock.sequence()]


def tiny_policy(**over):
    d = {"policy": {"name": "t", "robot": "test"},
         "actions": {"drive": {"kind": "motion", "duration_param": "duration_s",
                               "params": {"linear_mps": {"type": "number", "min": -0.3, "max": 0.5},
                                          "angular_rps": {"type": "number", "min": -1, "max": 1, "default": 0.0},
                                          "duration_s": {"type": "number", "min": 0.1, "max": 5.0}}},
                     "set_led": {"kind": "instant", "params": {"color": {"type": "enum", "values": ["off", "red"]}}}}}
    for k, v in over.items():
        d[k] = v
    return policy_from_dict(d)
