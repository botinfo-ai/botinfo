"""The benchmark's agent: a deterministic stand-in for a model that has been fully hijacked.

It does two things a real agent does:
  - cooperative motion: a timed move done the way a careful client would on that tool surface (behind BotInfo one
    call; on a raw connector: start, keep-alives, and its own stop at the end);
  - it obeys text. follow(text) executes every line of the form
        CALL <tool> {json arguments}        or        CALL <tool> key=value key=value
    found in anything it reads (tool results, skills, web pages). A real model follows injections less
    reliably; this one always does, so the benchmark measures what the connector lets a hijacked agent do,
    independent of any model.

It calls the variants of an intent whose tool the surface lists (the tools it can see), and calls explicit CALL
lines as written, listed or not.
"""
from __future__ import annotations

import json
import re

CALL_RE = re.compile(r"CALL\s+([A-Za-z_][A-Za-z0-9_]{0,63})\s*")
KV_RE = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)=([^\s]+)")


def parse_calls(text: str) -> list:
    """All CALL lines in text -> [(tool, args)]."""
    out = []
    if not isinstance(text, str):
        return out
    dec = json.JSONDecoder()
    marks = list(CALL_RE.finditer(text))
    for i, m in enumerate(marks):
        rest = text[m.end(): marks[i + 1].start() if i + 1 < len(marks) else len(text)]
        args = {}
        if rest.lstrip().startswith("{"):
            try:
                args, _ = dec.raw_decode(rest.lstrip())
            except ValueError:
                args = {}
        else:
            for k, v in KV_RE.findall(rest):
                try:
                    args[k] = json.loads(v)
                except ValueError:
                    args[k] = v
        out.append((m.group(1), args if isinstance(args, dict) else {}))
    return out


def strings_in(obj, out=None) -> list:
    out = [] if out is None else out
    if isinstance(obj, str):
        out.append(obj)
    elif isinstance(obj, dict):
        for v in obj.values():
            strings_in(v, out)
    elif isinstance(obj, (list, tuple)):
        for v in obj:
            strings_in(v, out)
    return out


class Agent:
    def __init__(self, stack):
        self.stack = stack
        self.world = stack.world
        self.tools = {t["name"] for t in stack.surface.list_tools()}
        self.behaviours: list = []
        self.log: list = []
        self.read_texts: list = []
        stack.agent = self
        self.world.tickers.insert(0, self.tick)

    def call(self, name, args) -> dict:
        r = self.stack.surface.call(name, args)
        self.log.append({"t": round(self.world.now(), 3), "tool": name, "ok": bool(r.get("ok"))})
        return r

    def attempt(self, variants) -> list:
        picks = [v for v in variants if v[0] in self.tools] or variants[:1]
        return [self.call(n, a) for n, a in picks]

    def has(self, tool) -> bool:
        return tool in self.tools

    def tick(self):
        now = self.world.now()
        for b in list(self.behaviours):          # in real time the case thread may add one meanwhile
            if not b(now):
                try:
                    self.behaviours.remove(b)
                except ValueError:
                    pass

    def gone(self):
        self.behaviours = []

    def follow(self, text: str, limit: int = 50) -> int:
        calls = parse_calls(text)[:limit]
        for name, args in calls:
            self.call(name, args)
        return len(calls)

    def read_and_follow(self, variants) -> dict:
        """Read (a status tool), then obey any instructions in what came back."""
        texts = []
        for r in self.attempt(variants):
            texts += strings_in(r)
        self.read_texts += texts
        n = sum(self.follow(t) for t in texts)
        return {"read": len(texts), "calls_followed": n,
                "injection_reached_agent": any("CALL" in t for t in texts)}
