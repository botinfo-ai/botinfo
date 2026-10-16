"""A minimal MCP client over stdio, standard library only. Reins uses it to front an existing MCP connector
(launch it as a child process, then call its tools through a binding). Also used by the tests to drive the
Reins server the way an agent's client would.

Requests the server sends to us (sampling, roots, elicitation) are answered "method not found": Reins never
lets an upstream connector talk to the agent's model.
"""
from __future__ import annotations

import json
import subprocess
import sys
import threading

from ..backends.base import ToolError

PROTOCOL_VERSION = "2025-06-18"


class StdioMCPClient:
    def __init__(self, command: list, env=None, cwd=None, timeout_s: float = 10.0, stderr=None):
        command = [sys.executable if c == "{python}" else c for c in command]
        self.proc = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     stderr=stderr if stderr is not None else sys.stderr,
                                     text=True, bufsize=1, env=env, cwd=cwd)
        self.timeout_s = timeout_s
        self._next = 1
        self._lock = threading.Lock()
        self._cv = threading.Condition()
        self._replies: dict = {}
        self._closed = False
        self._reader = threading.Thread(target=self._read, name="mcp-client-reader", daemon=True)
        self._reader.start()
        init = self.request("initialize", {"protocolVersion": PROTOCOL_VERSION, "capabilities": {},
                                           "clientInfo": {"name": "reins", "version": "0.1.0"}})
        self.server_info = init.get("serverInfo", {})
        self.notify("notifications/initialized")
        self.tools = self.request("tools/list", {}).get("tools", [])

    def _write(self, msg):
        with self._lock:
            if self.proc.poll() is not None:
                raise ToolError("connector process has exited")
            try:
                self.proc.stdin.write(json.dumps(msg, separators=(",", ":")) + "\n")
                self.proc.stdin.flush()
            except (BrokenPipeError, OSError):
                raise ToolError("connector process is gone") from None

    def _read(self):
        for line in self.proc.stdout:
            try:
                msg = json.loads(line)
            except ValueError:
                continue
            if not isinstance(msg, dict):
                continue
            if "method" in msg and "id" in msg:          # a request from the server: refuse
                try:
                    self._write({"jsonrpc": "2.0", "id": msg["id"],
                                 "error": {"code": -32601, "message": "not supported by Reins"}})
                except ToolError:
                    pass
                continue
            if "id" in msg:
                with self._cv:
                    self._replies[msg["id"]] = msg
                    self._cv.notify_all()
        with self._cv:
            self._closed = True
            self._cv.notify_all()

    def request(self, method, params=None) -> dict:
        with self._lock:
            mid = self._next
            self._next += 1
        self._write({"jsonrpc": "2.0", "id": mid, "method": method, "params": params or {}})
        with self._cv:
            ok = self._cv.wait_for(lambda: mid in self._replies or self._closed, timeout=self.timeout_s)
            msg = self._replies.pop(mid, None)
        if msg is None:
            raise ToolError(f"{method}: no answer from the connector" + ("" if ok else f" within {self.timeout_s:g} s"))
        if "error" in msg:
            raise ToolError(f"{method}: {msg['error'].get('message', msg['error'])}")
        return msg.get("result", {})

    def notify(self, method, params=None):
        self._write({"jsonrpc": "2.0", "method": method, **({"params": params} if params else {})})

    def list_tools(self) -> list:
        return list(self.tools)

    def call_tool(self, name: str, args: dict) -> dict:
        res = self.request("tools/call", {"name": name, "arguments": args})
        payload = res.get("structuredContent")
        if payload is None:
            texts = [c.get("text", "") for c in res.get("content", []) if c.get("type") == "text"]
            joined = "\n".join(texts)
            try:
                payload = json.loads(joined)
            except ValueError:
                payload = {"text": joined}
        if res.get("isError"):
            raise ToolError(json.dumps(payload)[:300] if not isinstance(payload, dict) else
                            str(payload.get("error") or payload.get("reason") or payload)[:300])
        return payload if isinstance(payload, dict) else {"result": payload}

    def close(self):
        try:
            if self.proc.stdin:
                self.proc.stdin.close()
        except OSError:
            pass
        try:
            self.proc.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            self.proc.wait(timeout=3.0)
        self._reader.join(timeout=1.0)
        if self.proc.stdout:
            self.proc.stdout.close()
