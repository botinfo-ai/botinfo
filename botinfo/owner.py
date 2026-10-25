"""The owner's local channel: stop, resume, confirm, deny, status. Never exposed to the agent.

A Unix domain socket in a directory only the current user can open (0700), with the socket itself 0600. One JSON
object per line in, one out:

    {"cmd": "stop"} {"cmd": "resume"} {"cmd": "confirm", "id": "c-3"} {"cmd": "deny", "id": "c-3"} {"cmd": "status"}

Limits, stated plainly: anything running as the same OS user can open this socket. If the agent has a shell on
your account, it can reach this channel (and the robot) directly; BotInfo assumes it does not (docs/DESIGN.md).
"""
from __future__ import annotations

import json
import os
import socket
import socketserver
import stat
import threading
from pathlib import Path

DEFAULT_DIR = Path.home() / ".botinfo"
MAX_LINE = 4096


def default_socket(name: str) -> Path:
    return DEFAULT_DIR / f"{name}.sock"


class OwnerChannel:
    def __init__(self, botinfo, path):
        self.botinfo = botinfo
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        os.chmod(self.path.parent, 0o700)
        if self.path.exists():
            if not stat.S_ISSOCK(self.path.stat().st_mode):
                raise ValueError(f"{self.path} exists and is not a socket")
            self.path.unlink()
        botinfo_ref = botinfo

        class Handler(socketserver.StreamRequestHandler):
            def handle(self):
                line = self.rfile.readline(MAX_LINE)
                try:
                    req = json.loads(line)
                    reply = dispatch(botinfo_ref, req)
                except (ValueError, TypeError) as e:
                    reply = {"ok": False, "error": f"bad request: {e}"}
                self.wfile.write((json.dumps(reply, default=str) + "\n").encode())

        old = os.umask(0o177)
        try:
            self.server = socketserver.ThreadingUnixStreamServer(str(self.path), Handler)
        finally:
            os.umask(old)
        os.chmod(self.path, 0o600)
        self.thread = threading.Thread(target=self.server.serve_forever, name="botinfo-owner", daemon=True)

    def start(self):
        self.thread.start()
        return self

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        try:
            self.path.unlink()
        except FileNotFoundError:
            pass


def dispatch(botinfo, req: dict) -> dict:
    if not isinstance(req, dict):
        return {"ok": False, "error": "expected an object"}
    cmd = req.get("cmd")
    if cmd == "stop":
        return botinfo.owner_stop(str(req.get("reason", "owner stop"))[:200])
    if cmd == "resume":
        return botinfo.owner_resume()
    if cmd == "confirm":
        return botinfo.owner_confirm(str(req.get("id", "")))
    if cmd == "deny":
        return botinfo.owner_deny(str(req.get("id", "")))
    if cmd == "status":
        return botinfo.owner_status()
    return {"ok": False, "error": f"unknown command {cmd!r}; use stop, resume, confirm, deny, status"}


def send(path, req: dict, timeout_s: float = 3.0) -> dict:
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as s:
        s.settimeout(timeout_s)
        s.connect(str(path))
        s.sendall((json.dumps(req) + "\n").encode())
        data = b""
        while not data.endswith(b"\n"):
            chunk = s.recv(65536)
            if not chunk:
                break
            data += chunk
    return json.loads(data or b"{}")
