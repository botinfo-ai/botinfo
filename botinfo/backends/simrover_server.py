"""The reference ROS-style connector + SimRover as a standalone MCP server over stdio (real time).

    python3 -m botinfo.backends.simrover_server [--watchdog 0.5]

This is the RAW connector, no BotInfo: every generic tool is exposed. It exists so BotInfo can be tested fronting an
MCP connector in a separate process, exactly as it would front ros-mcp-server.
"""
from __future__ import annotations

import argparse
import sys

from ..clock import RealClock
from ..mcp.server import StdioServer
from .base import ToolError
from .simrover import RosStyleConnector, SimRover


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--watchdog", type=float, default=0.0, help="base cmd_vel timeout in seconds (0 = none)")
    a = ap.parse_args(argv)
    rover = SimRover(RealClock(), cmd_timeout_s=a.watchdog)
    conn = RosStyleConnector(rover)

    def call(name, args):
        try:
            return conn.call_tool(name, args), False
        except ToolError as e:
            return {"error": str(e)}, True

    def tick():
        conn.tick()
        rover.step()

    StdioServer(conn.list_tools, call, name="simrover-ros-style", tick=tick, tick_s=0.02,
                instructions="SimRover reference connector (simulation only, no safety layer).").serve()
    return 0


if __name__ == "__main__":
    sys.exit(main())
