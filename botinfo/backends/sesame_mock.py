"""A local MOCK of the Sesame firmware's HTTP API. Not the robot: no servos, no WiFi, localhost only.

Adapted from BotInfo's sesame-kit v0.2 (src/mock_firmware.py, same copyright holder, Apache-2.0) on 2026-10-16:
imports changed, and the audit no longer checks sesame-kit's own allowlist (BotInfo grades behaviour in bench/).

It answers the same routes, parses request bodies the same (string-search) way, gives the same status codes and
messages, and simulates what the firmware's loop() would do with the robot over time:
  - forward/backward/left/right repeat until a "stop" (or another command) arrives; the stop is noticed at the
    next animation frame and the robot stands up;
  - the 13 one-shot poses run to the end even if "stop" arrives meanwhile (the firmware does not interrupt them);
  - an unknown command is accepted with 200 "Command executed" and then ignored (the firmware does not validate);
  - an unknown face falls back to "default".
Timings come from the firmware's own delays (firmware_spec.py). Behaviour is cited in API-NOTES.md.

With --safety-patch (MockSesame(safety_patch=True)) it imitates the firmware WITH BotInfo's proposed patch
(firmware-patch/, compiled but never run on a robot) instead: a forward/backward/left/right sent to /api/command
stops by itself (stand-up at the next frame) if no /api/command arrives for motionTimeout ms (3000; any accepted
/api/command resets it, so re-sending the move is the keep-alive); unknown command names get 400
{"error":"Unknown command",...} and change nothing; /getSettings also reports motionTimeout.

Separately from imitating the firmware, the mock flags commands the firmware ignores (IGNORED_BY_FIRMWARE) and
moves that keep going with no stop (RUNAWAY). With the patch it also flags every time the robot had to stop itself
(WATCHDOG_STOP: the controller went quiet mid-move). The eval counts those flags; the target is zero.

    python3 -m botinfo.backends.sesame_mock                    # serves http://127.0.0.1:8765, prints a log
    python3 -m botinfo.backends.sesame_mock --safety-patch     # the same, imitating the patched firmware
"""
from __future__ import annotations

import argparse
import json
import signal
import sys
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

from ..clock import RealClock
from . import sesame_spec as F

STOP_NOTICE_S = 0.04          # a stop is seen at the next pressingCheck: within ~0-80 ms of servo writes
RUNAWAY_S = 15.0              # a walk/turn longer than this with no stop is flagged (the kit's max is ~9 s)
SERVO_NAMES = ("R1", "R2", "L1", "L2", "R4", "R3", "L3", "L4")   # movement-sequences.h:5-16 order


def arduino_to_int(s: str) -> int:
    """Arduino String.toInt(): leading integer, else 0."""
    s = (s or "").lstrip()
    sign, i = 1, 0
    if s[:1] in "+-" and s[:1]:
        sign, i = (-1 if s[0] == "-" else 1), 1
    j = i
    while j < len(s) and s[j].isdigit():
        j += 1
    return sign * int(s[i:j]) if j > i else 0


def json_escape(s: str) -> str:
    """jsonEscape(), sesame-firmware-main.ino:392-406: backslash, double quote and control characters only."""
    out = []
    for c in s:
        if c == "\\":
            out.append("\\\\")
        elif c == '"':
            out.append('\\"')
        elif ord(c) < 0x20:
            out.append(f"\\u{ord(c):04x}")
        else:
            out.append(c)
    return "".join(out)


def parse_api_command(body: str) -> dict:
    """handleApiCommand's body parsing, line for line (sesame-firmware-main.ino:309-362). It is substring
    search, not a JSON parser: '"command":"x"' and '"command": "x"' work; '"command" : "x"' does not."""
    fs = body.find('"face":"')
    if fs == -1:
        fs = body.find('"face": "')
    face_only = fs > 0 and body.find('"command":') == -1 and body.find('"command": ') == -1
    command = face = ""
    if fs > 0:
        fs = body.find('"', fs + 6) + 1
        fe = body.find('"', fs)
        if fe > fs:
            face = body[fs:fe]
    if not face_only:
        cs = body.find('"command":"')
        if cs == -1:
            cs = body.find('"command": "')
        if cs == -1:
            return {"status": 400, "body": {"error": "Missing command field"}, "face": "", "command": None}
        cs = body.find('"', cs + 10) + 1
        ce = body.find('"', cs)
        if ce <= cs:
            return {"status": 400, "body": {"error": "Invalid command format"}, "face": "", "command": None}
        command = body[cs:ce]
    return {"status": 200, "face": face, "command": None if face_only else command, "face_only": face_only}


class MockSesame:
    """The simulated robot. Thread-safe; time comes from `clock` (RealClock or clocks.VirtualClock)."""

    def __init__(self, clock=None, settings=None, echo=None, safety_patch: bool = False):
        self.clock = clock or RealClock()
        self.lock = threading.RLock()
        self.t0 = self.clock.now()
        self.safety_patch = safety_patch
        self.settings = {**F.DEFAULT_SETTINGS, "faceFps": 8,
                         **({"motionTimeout": F.PATCH_MOTION_TIMEOUT_MS} if safety_patch else {}), **(settings or {})}
        self.wd_armed = False                 # patch only: a move sent over /api/command is being watched
        self.wd_last = 0.0                    # patch only: when the last accepted /api/command arrived
        self.current_command = ""
        self.cmd_set_at = 0.0
        self.current_face = "rest"            # setup() shows "rest" without moving motors (:746-747)
        self.posture = "boot (servos attached, not moved)"
        self.activity: dict | None = None
        self.free_since = 0.0
        self.requests: list = []              # every HTTP request and the firmware's answer
        self.events: list = []                # what the simulated robot did
        self.flags: list = []                 # audit findings
        self.echo = echo                      # optional callable(str) for a live log

    # -------------------------------------------------------------- time
    def t(self) -> float:
        return self.clock.now() - self.t0

    def _event(self, t, what, **kw):
        e = {"t": round(t, 3), "event": what, **kw}
        self.events.append(e)
        if self.echo:
            self.echo(f"[robot t={e['t']:8.3f}] {what} " + " ".join(f"{k}={v}" for k, v in kw.items()))

    def _flag(self, t, kind, detail):
        f = {"t": round(t, 3), "flag": kind, "detail": detail}
        self.flags.append(f)
        if self.echo:
            self.echo(f"[audit t={f['t']:8.3f}] {kind}: {detail}")

    def advance(self, now: float | None = None) -> None:
        """Run the simulated loop() up to `now` (default: the clock)."""
        with self.lock:
            now = self.t() if now is None else now
            for _ in range(10000):                      # bounded
                if self.wd_armed and self.current_command not in F.MOVE_COMMANDS:
                    self.wd_armed = False                   # shouldStop(): the watched move already ended
                a = self.activity
                if a is None:
                    cmd = self.current_command
                    if cmd in F.MOVE_COMMANDS or cmd in F.POSE_COST:
                        start = max(self.free_since, self.cmd_set_at)
                        if start > now:
                            return
                        if cmd in F.MOVE_COMMANDS and self._watchdog_due(start):
                            self._watchdog_stop(start, cmd)     # loop()'s check, before the move starts
                            continue
                        self._start(cmd, start)
                        continue
                    return
                if a["kind"] == "move":
                    if a["end"] is None:
                        due = self._watchdog_deadline()
                        if due is not None and due <= now:  # pressingCheck() notices between frames
                            self._watchdog_stop(due, a["name"])
                            continue
                        if not a["runaway"] and now - a["start"] > RUNAWAY_S:
                            a["runaway"] = True
                            self._flag(a["start"] + RUNAWAY_S, "RUNAWAY",
                                       f"{a['name']} has run {RUNAWAY_S:g} s with no stop")
                        return
                if a["end"] > now:
                    return
                self._finish(a)

    # -------------------------------------------------------------- safety patch: motion watchdog
    def _watchdog_deadline(self) -> float | None:
        timeout_ms = self.settings.get("motionTimeout", 0)
        if not (self.safety_patch and self.wd_armed and timeout_ms > 0):
            return None
        return self.wd_last + timeout_ms / 1000.0

    def _watchdog_due(self, t: float) -> bool:
        due = self._watchdog_deadline()
        return due is not None and due <= t

    def _watchdog_stop(self, t: float, name: str):
        """checkMotionWatchdog(): currentCommand = "" at time t; a running move stands up at its next frame."""
        self.wd_armed = False
        self._event(t, "watchdog_stop", action=name, detail=f"no /api/command for {self.settings['motionTimeout']} ms")
        self._flag(t, "WATCHDOG_STOP", f"{name}: no /api/command for {self.settings['motionTimeout']} ms, "
                                       "the robot stopped itself (the controller went quiet)")
        self._set_command("", t)

    def _start(self, name, start):
        s = self.settings
        if name in F.MOVE_COMMANDS:
            self.activity = {"kind": "move", "name": name, "start": start, "end": None, "runaway": False}
            self.current_face = "walk"
            self.posture = f"{name} (repeats until stopped)"
            self._event(start, "start", action=name, kind="move",
                        cycle_s=round(F.gait_cycle_seconds(name, s), 3))
        else:
            dur = F.pose_seconds(name, s)
            self.activity = {"kind": "pose", "name": name, "start": start, "end": start + dur}
            self.current_face = F.POSE_FACE.get(name, self.current_face)
            self.posture = f"doing {name}"
            self._event(start, "start", action=name, kind="pose", duration_s=round(dur, 3))

    def _finish(self, a):
        end = a["end"]
        if a["kind"] == "move":
            run = max(0.0, a["changed_at"] - a["start"] - F.move_start_seconds(a["name"], self.settings))
            cycles = run / F.gait_cycle_seconds(a["name"], self.settings)
            self._event(end, "end", action=a["name"], kind="move", gait_cycles=round(cycles, 2),
                        moved_s=round(a["changed_at"] - a["start"], 3), then="stand")
            self.posture, self.current_face = "standing", "idle"
        else:
            name = a["name"]
            if self.current_command == name:              # each pose clears its own name (movement-sequences.h)
                self.current_command = ""
            self.posture = ("standing" if name in F.POSES_ENDING_STANDING else
                            "resting (all servos 90)" if name == "rest" else "lying (played dead)")
            self.current_face = {"rest": "rest", "dead": "dead"}.get(name, "idle")
            self._event(end, "end", action=name, kind="pose")
        self.activity, self.free_since = None, end

    def _set_command(self, cmd: str, now: float):
        """currentCommand = cmd, as the HTTP handlers do; a running move notices at its next frame."""
        a = self.activity
        if a is not None and a["kind"] == "move" and a["end"] is None and cmd != a["name"]:
            a["changed_at"] = now
            a["end"] = now + STOP_NOTICE_S + F.stand_seconds(self.settings)
        if a is not None and a["kind"] == "pose" and cmd != a["name"]:
            self._event(now, "note", action=a["name"],
                        detail=f"command {cmd or 'stop'!r} arrived during a pose; the firmware finishes the pose first")
        self.current_command, self.cmd_set_at = cmd, now

    def _set_face(self, name: str):
        """setFace(), :903-934: case-insensitive lookup; unknown or bitmap-less -> 'default'."""
        known = {f.lower(): f for f in F.FIRMWARE_FACES}
        if name.lower() not in known or known[name.lower()] in F.FACES_WITHOUT_BITMAP:
            self.current_face = "default"
        else:
            self.current_face = name

    # -------------------------------------------------------------- HTTP-level behaviour
    def handle(self, method: str, raw_path: str, body: str = "") -> tuple:
        """One request -> (status, content_type, response_text). Same routes and answers as the firmware."""
        with self.lock:
            now = self.t()
            self.advance(now)
            url = urlsplit(raw_path)
            path, args = url.path, parse_qs(url.query, keep_blank_values=True)
            status, ctype, text = self._route(method, path, args, body, now)
            self.advance(now)
            rec = {"t": round(now, 6), "method": method, "path": raw_path, "body": body, "status": status, "response": text}
            self.requests.append(rec)
            if self.echo:
                self.echo(f"[http  t={now:8.3f}] {method} {raw_path} {body} -> {status} {text[:80]}")
            self._audit(method, path, body, status, now)
            return status, ctype, text

    def _route(self, method, path, args, body, now):
        J = "application/json"
        if path == "/":
            return 200, "text/html", "<html><body>Sesame MOCK (no robot). Captive portal not reproduced.</body></html>"
        if path == "/api/command":
            if method != "POST":
                return 405, J, '{"error":"Method not allowed"}'
            p = parse_api_command(body)
            if p["status"] != 200:
                return p["status"], J, json.dumps(p["body"], separators=(",", ":"))
            if self.safety_patch and p["command"] is not None and p["command"] not in F.FIRMWARE_COMMANDS:
                return 400, J, ('{"error":"' + F.PATCH_UNKNOWN_COMMAND_ERROR + '","command":"'
                                + json_escape(p["command"]) + '"}')
            if p["face"]:
                self._set_face(p["face"])
            if self.safety_patch:
                self.wd_last = now                          # any accepted command feeds the watchdog
            if p["command"] is None:
                return 200, J, '{"status":"ok","message":"Face updated"}'
            if p["command"] == "stop":
                self._set_command("", now)
                return 200, J, '{"status":"ok","message":"Command stopped"}'
            self._set_command(p["command"], now)
            if self.safety_patch and p["command"] in F.MOVE_COMMANDS:
                self.wd_armed = True
            return 200, J, '{"status":"ok","message":"Command executed"}'
        if path == "/api/status":
            d = {"currentCommand": self.current_command, "currentFace": self.current_face,
                 "networkConnected": False, "apIP": "192.168.4.1"}
            return 200, J, json.dumps(d, separators=(",", ":"))
        if path == "/getSettings":
            s = self.settings
            keys = ("frameDelay", "walkCycles", "motorCurrentDelay", "faceFps") + \
                (("motionTimeout",) if self.safety_patch else ())
            return 200, J, json.dumps({k: s[k] for k in keys}, separators=(",", ":"))
        if path == "/setSettings":
            for k in ("frameDelay", "walkCycles", "motorCurrentDelay"):
                if k in args:
                    self.settings[k] = arduino_to_int(args[k][0])
            if "faceFps" in args:
                self.settings["faceFps"] = max(1, arduino_to_int(args["faceFps"][0]))
            if self.safety_patch and "motionTimeout" in args:
                lo, hi = F.PATCH_MOTION_TIMEOUT_RANGE
                self.settings["motionTimeout"] = min(hi, max(lo, arduino_to_int(args["motionTimeout"][0])))
            return 200, "text/plain", "OK"
        if path == "/cmd":                                   # legacy web endpoint, :230-268
            if "pose" in args:
                self._set_command(args["pose"][0], now)
                return 200, "text/plain", "OK"
            if "go" in args:
                self._set_command(args["go"][0], now)
                self.wd_armed = False                       # patch: the portal's hold-to-move is not watched
                return 200, "text/plain", "OK"
            if "stop" in args:
                self._set_command("", now)
                return 200, "text/plain", "OK"
            if "motor" in args and "value" in args:
                m, ang = arduino_to_int(args["motor"][0]), arduino_to_int(args["value"][0])
                ok = 0 <= ang <= 180 and (1 <= m <= 8 or args["motor"][0] in SERVO_NAMES)
                if ok:
                    self._event(now, "servo", motor=args["motor"][0], angle=ang)
                    return 200, "text/plain", "OK"
                return 400, "text/plain", "Invalid motor or angle"
            return 400, "text/plain", "Bad Args"
        if path == "/api/wifi/scan":
            return 200, J, "[]"
        if path == "/api/wifi/connect":
            if method != "POST":
                return 405, J, '{"success":false,"error":"Method not allowed"}'
            return 400, J, '{"success":false,"error":"SSID required"}'   # the mock never joins a network
        if path == "/api/wifi/status":
            return 200, J, '{"connected":false,"connecting":false}'
        if path.startswith("/api/"):
            return 404, J, '{"error":"Not found"}'
        return 200, "text/html", "<html><body>Sesame MOCK (captive-portal redirect)</body></html>"

    def _audit(self, method, path, body, status, now):
        if path != "/api/command" or method != "POST":
            return
        try:
            payload = json.loads(body)
        except ValueError:
            return
        cmd = payload.get("command") if isinstance(payload, dict) else None
        if isinstance(cmd, str) and cmd not in F.FIRMWARE_COMMANDS:
            if self.safety_patch:
                self._flag(now, "REJECTED_BY_FIRMWARE", f"command {cmd!r}: refused with 400 (unknown command)")
            else:
                self._flag(now, "IGNORED_BY_FIRMWARE", f"command {cmd!r}: accepted with 200 but loop() has no such action")

    # -------------------------------------------------------------- summaries
    def sequence(self) -> list:
        """What reached /api/command successfully, in order: ('command', x) / ('face', x)."""
        out = []
        for r in self.requests:
            if r["path"] == "/api/command" and r["status"] == 200:
                try:
                    p = json.loads(r["body"])
                except ValueError:
                    continue
                for k in ("face", "command"):
                    if k in p:
                        out.append((r["t"], k, p[k]))
        return out

    def snapshot(self) -> dict:
        with self.lock:
            self.advance()
            return {"t": round(self.t(), 3), "currentCommand": self.current_command, "currentFace": self.current_face,
                    "posture": self.posture, "busy": self.activity is not None,
                    "activity": None if self.activity is None else self.activity["name"]}

    def log(self) -> dict:
        with self.lock:
            return {"requests": list(self.requests), "events": list(self.events), "flags": list(self.flags),
                    "final": self.snapshot()}


class _Handler(BaseHTTPRequestHandler):
    server_version = "SesameMock/0.2"
    mock: MockSesame = None

    def _do(self, method):
        n = int(self.headers.get("Content-Length") or 0)
        body = self.rfile.read(n).decode("utf-8", "replace") if n > 0 else ""
        status, ctype, text = self.mock.handle(method, self.path, body)
        data = text.encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        self._do("GET")

    def do_POST(self):
        self._do("POST")

    def do_PUT(self):
        self._do("PUT")

    def do_DELETE(self):
        self._do("DELETE")

    def log_message(self, *args):          # the mock keeps its own log
        pass


def serve(mock: MockSesame, port: int = 0, host: str = "127.0.0.1"):
    """Start the mock on localhost in a daemon thread -> (server, base_url). port=0 picks a free port."""
    if host not in ("127.0.0.1", "localhost", "::1"):
        raise ValueError("the mock only listens on localhost")
    handler = type("Handler", (_Handler,), {"mock": mock})
    httpd = HTTPServer((host, port), handler)
    th = threading.Thread(target=httpd.serve_forever, kwargs={"poll_interval": 0.05}, name="sesame-mock", daemon=True)
    th.start()
    return httpd, f"http://{host}:{httpd.server_address[1]}"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--log", default=None, help="write the full request/event/audit log here (JSON) on exit")
    ap.add_argument("--quiet", action="store_true")
    ap.add_argument("--safety-patch", action="store_true",
                    help="imitate the firmware with BotInfo's safety patch (motion timeout, 400 for unknown commands)")
    a = ap.parse_args(argv)
    mock = MockSesame(echo=None if a.quiet else (lambda s: print(s, flush=True)), safety_patch=a.safety_patch)
    httpd, url = serve(mock, a.port)
    kind = "WITH the safety patch (motion timeout %d ms)" % mock.settings["motionTimeout"] if a.safety_patch \
        else "stock d106cbc8 (no motion timeout)"
    print(f"Sesame firmware MOCK, {kind}, listening on {url} (localhost only; Ctrl-C to quit)", flush=True)
    done = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: done.set())
    try:
        while not done.wait(0.5):
            pass
    except KeyboardInterrupt:
        pass
    finally:
        httpd.shutdown()
        if a.log:
            Path(a.log).parent.mkdir(parents=True, exist_ok=True)
            Path(a.log).write_text(json.dumps(mock.log(), indent=1))
            print(f"log written to {a.log}")
        flags = mock.flags
        print(f"audit flags: {len(flags)}" + "".join(f"\n  {f['flag']}: {f['detail']}" for f in flags))
    return 0


if __name__ == "__main__":
    sys.exit(main())
