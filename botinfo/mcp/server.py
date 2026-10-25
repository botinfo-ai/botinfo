"""A minimal MCP server over stdio (newline-delimited JSON-RPC 2.0), standard library only.

Implements what a tool server needs: initialize, notifications/initialized, ping, tools/list, tools/call.
When stdin closes (the MCP client went away), on_disconnect runs; for BotInfo that stops the robot.
A timer thread calls tick() every tick_s (leases, keep-alives, expiry). Logs go to stderr, never stdout.
"""
from __future__ import annotations

import json
import sys
import threading

from .. import __version__

PROTOCOL_VERSIONS = ("2025-06-18", "2025-03-26", "2024-11-05")


class StdioServer:
    def __init__(self, list_tools, call_tool, name="botinfo", instructions="", on_disconnect=None, tick=None,
                 tick_s=0.05, inp=None, out=None, log=None, slow_tools=()):
        """call_tool(name, args) -> (payload_dict, is_error). A tool in slow_tools (one that waits) runs on its own
        thread and answers when done, so the requests after it, a stop above all, are not queued behind it."""
        self.slow_tools = frozenset(slow_tools)
        self.list_tools, self.call_tool = list_tools, call_tool
        self.name, self.instructions = name, instructions
        self.on_disconnect, self.tick, self.tick_s = on_disconnect, tick, tick_s
        self.inp = inp or sys.stdin
        self.out = out or sys.stdout
        self.log = log or (lambda s: print(s, file=sys.stderr, flush=True))
        self._wlock = threading.Lock()
        self._done = threading.Event()

    def _send(self, msg):
        line = json.dumps(msg, separators=(",", ":"), allow_nan=False, default=str)
        with self._wlock:
            self.out.write(line + "\n")
            self.out.flush()

    def _ticker(self):
        while not self._done.wait(self.tick_s):
            try:
                self.tick()
            except Exception as e:  # keep ticking; the audit log has the details
                self.log(f"[{self.name}] tick error: {e!r}")

    def serve(self):
        th = None
        if self.tick:
            th = threading.Thread(target=self._ticker, name=f"{self.name}-tick", daemon=True)
            th.start()
        try:
            for raw in self.inp:
                raw = raw.strip()
                if not raw:
                    continue
                try:
                    msg = json.loads(raw)
                except ValueError:
                    self._send({"jsonrpc": "2.0", "id": None, "error": {"code": -32700, "message": "parse error"}})
                    continue
                if not isinstance(msg, dict):
                    self._send({"jsonrpc": "2.0", "id": None, "error": {"code": -32600, "message": "invalid request"}})
                    continue
                reply = self.handle(msg)
                if reply is not None:
                    self._send(reply)
        finally:
            self._done.set()
            if self.on_disconnect:
                self.on_disconnect()
            if th:
                th.join(timeout=1.0)

    def handle(self, msg: dict):
        mid, method, params = msg.get("id"), msg.get("method"), msg.get("params") or {}
        if "method" not in msg:
            return None                           # a response to something we never sent; ignore
        if mid is None:
            return None                           # notification (notifications/initialized, cancelled, ...)
        try:
            if method == "initialize":
                want = params.get("protocolVersion")
                ver = want if want in PROTOCOL_VERSIONS else PROTOCOL_VERSIONS[0]
                res = {"protocolVersion": ver, "capabilities": {"tools": {"listChanged": False}},
                       "serverInfo": {"name": self.name, "version": __version__}}
                if self.instructions:
                    res["instructions"] = self.instructions
            elif method == "ping":
                res = {}
            elif method == "tools/list":
                res = {"tools": self.list_tools()}
            elif method == "tools/call":
                name, args = params.get("name"), params.get("arguments") or {}
                if name in self.slow_tools:
                    threading.Thread(target=self._slow_call, args=(mid, name, args), name=f"{self.name}-{name}",
                                     daemon=True).start()
                    return None
                res = self._tool_result(name, args)
            else:
                return {"jsonrpc": "2.0", "id": mid, "error": {"code": -32601, "message": f"method not found: {method}"}}
        except Exception as e:
            self.log(f"[{self.name}] error handling {method}: {e!r}")
            return {"jsonrpc": "2.0", "id": mid, "error": {"code": -32603, "message": "internal error"}}
        return {"jsonrpc": "2.0", "id": mid, "result": _finite(res)}

    def _tool_result(self, name, args) -> dict:
        payload, is_error = self.call_tool(name, args)
        text = json.dumps(payload, separators=(",", ":"), default=str)
        res = {"content": [{"type": "text", "text": text}], "isError": bool(is_error)}
        if isinstance(payload, dict):
            res["structuredContent"] = payload
        return res

    def _slow_call(self, mid, name, args):
        try:
            reply = {"jsonrpc": "2.0", "id": mid, "result": _finite(self._tool_result(name, args))}
        except Exception as e:
            self.log(f"[{self.name}] error in {name}: {e!r}")
            reply = {"jsonrpc": "2.0", "id": mid, "error": {"code": -32603, "message": "internal error"}}
        try:
            self._send(reply)
        except (OSError, ValueError):   # the client went away while we waited
            pass


def _finite(obj):
    """JSON has no NaN/Infinity; an agent may have sent them, and BotInfo echoes requests in errors."""
    if isinstance(obj, float) and (obj != obj or obj in (float("inf"), float("-inf"))):
        return repr(obj)
    if isinstance(obj, dict):
        return {k: _finite(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [_finite(v) for v in obj]
    return obj
