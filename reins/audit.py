"""The audit log: one JSON object per line, every request and every decision.

Each line carries "prev", the SHA-256 of the previous line (64 zeros for the first), so verify() notices a line
that was edited, removed or inserted inside the file. That makes the log tamper-evident, not tamper-proof: anyone
who can write the file can also rewrite the whole chain. Keep a copy elsewhere if that matters to you.

Values are made JSON-safe before writing: NaN and infinities become strings, long strings are cut (the cut is
marked), deep or huge structures are summarised. The log records what the agent sent, so it may contain hostile
text; it is data for the owner to read, never instructions.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
import threading
from datetime import datetime, timezone
from pathlib import Path

GENESIS = "0" * 64
MAX_STR = 2000
MAX_ITEMS = 100
MAX_DEPTH = 6


def jsonable(v, depth: int = 0):
    """A JSON-safe, bounded copy of v."""
    if depth > MAX_DEPTH:
        return "[too deep]"
    if v is None or isinstance(v, bool):
        return v
    if isinstance(v, int):
        return v if abs(v) < 2**63 else str(v)
    if isinstance(v, float):
        return v if math.isfinite(v) else repr(v)
    if isinstance(v, str):
        return v if len(v) <= MAX_STR else v[:MAX_STR] + f"...[cut, {len(v)} chars]"
    if isinstance(v, dict):
        out = {}
        for i, (k, x) in enumerate(v.items()):
            if i >= MAX_ITEMS:
                out["..."] = f"[{len(v) - MAX_ITEMS} more keys]"
                break
            out[str(k)[:200]] = jsonable(x, depth + 1)
        return out
    if isinstance(v, (list, tuple)):
        out = [jsonable(x, depth + 1) for x in list(v)[:MAX_ITEMS]]
        if len(v) > MAX_ITEMS:
            out.append(f"[{len(v) - MAX_ITEMS} more items]")
        return out
    return repr(v)[:MAX_STR]


def _line(record: dict) -> str:
    return json.dumps(record, sort_keys=True, separators=(",", ":"), ensure_ascii=True, allow_nan=False)


class AuditLog:
    """Append-only JSONL writer. path=None keeps the records in memory only (tests, the benchmark)."""

    def __init__(self, path: str | os.PathLike | None = None, clock=None, keep_in_memory: bool = True, echo=None):
        self.path = Path(path) if path else None
        self.clock = clock
        self.keep = keep_in_memory
        self.records: list = []
        self.echo = echo
        self._lock = threading.Lock()
        self._seq = 0
        self._prev = GENESIS
        if self.path:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            if self.path.exists() and self.path.stat().st_size > 0:
                ok, n, last, problem = _scan(self.path)
                if not ok:
                    raise ValueError(f"existing audit log {self.path} does not verify: {problem}")
                self._seq, self._prev = n, last

    def write(self, event: str, **fields) -> dict:
        with self._lock:
            self._seq += 1
            rec = {"seq": self._seq, "event": event, "prev": self._prev}
            if self.clock is not None:
                rec["t"] = round(self.clock.now(), 4)
                rec["wall"] = datetime.fromtimestamp(self.clock.wall(), timezone.utc).isoformat(timespec="milliseconds")
            for k, v in fields.items():
                rec[k] = jsonable(v)
            line = _line(rec)
            self._prev = hashlib.sha256(line.encode("ascii")).hexdigest()
            if self.path:
                with open(self.path, "a", encoding="ascii") as f:
                    f.write(line + "\n")
            if self.keep:
                self.records.append(rec)
            if self.echo:
                self.echo(line)
            return rec

    def events(self, name: str | None = None) -> list:
        return [r for r in self.records if name is None or r["event"] == name]


def _scan(path: Path):
    prev, n = GENESIS, 0
    with open(path, "r", encoding="ascii") as f:
        for i, raw in enumerate(f, 1):
            raw = raw.rstrip("\n")
            if not raw:
                return False, n, prev, f"line {i} is empty"
            try:
                rec = json.loads(raw)
            except ValueError:
                return False, n, prev, f"line {i} is not JSON"
            if rec.get("prev") != prev:
                return False, n, prev, f"line {i}: chain broken (prev does not match line {i - 1})"
            if rec.get("seq") != i:
                return False, n, prev, f"line {i}: seq is {rec.get('seq')!r}, expected {i}"
            if _line(rec) != raw:
                return False, n, prev, f"line {i}: not in canonical form (edited?)"
            prev = hashlib.sha256(raw.encode("ascii")).hexdigest()
            n = i
    return True, n, prev, None


def verify(path) -> tuple:
    """(ok, number_of_lines, problem_or_None) for an audit log file."""
    ok, n, _, problem = _scan(Path(path))
    return ok, n, problem
