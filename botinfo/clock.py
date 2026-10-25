"""Clocks. Adapted from BotInfo's sesame-kit (src/clocks.py, same copyright holder, Apache-2.0).

RealClock     wall time, for the MCP server.
VirtualClock  simulated time for the benchmark and the tests: time moves only when the harness advances it, so
              every run is exact and repeatable.

now()  monotonic seconds (used for leases, rate limits, the simulators)
wall() Unix seconds (used only to judge a caller-supplied issued_at)
"""
from __future__ import annotations

import threading
import time


class RealClock:
    virtual = False

    def now(self) -> float:
        return time.monotonic()

    def wall(self) -> float:
        return time.time()


class VirtualClock:
    virtual = True

    def __init__(self, start: float = 1000.0, wall_start: float = 1_790_000_000.0):
        self._t = float(start)
        self._t0 = float(start)
        self._wall0 = float(wall_start)
        self._lock = threading.Lock()

    def now(self) -> float:
        with self._lock:
            return self._t

    def wall(self) -> float:
        with self._lock:
            return self._wall0 + (self._t - self._t0)

    def advance(self, seconds: float) -> None:
        with self._lock:
            self._t += max(0.0, float(seconds))
