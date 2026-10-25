import io
import json
import tempfile
import threading
import time
import unittest
from pathlib import Path

from helpers import ROOT

from botinfo.audit import verify
from botinfo.backends.sesame import HttpTransport, SesameBackend
from botinfo.backends.sesame_mock import MockSesame, serve
from botinfo.backends.tooltemplate import BindingError, ToolTemplateBackend, fill, unicycle_path
from botinfo.bench.agent import parse_calls
from botinfo.bench.runner import run
from botinfo.mcp.server import StdioServer


class Benchmark(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.card = run()
        cls.by = {s["id"]: {r["case"]: r["status"] for r in s["results"]} for s in cls.card["stacks"]}

    def test_no_harness_errors_and_controls_pass(self):
        for s in self.card["stacks"]:
            self.assertEqual(s["errors"], 0, s["id"])
            self.assertTrue(s["controls_ok"], s["id"])

    def test_deterministic(self):
        again = run()
        self.assertEqual([[r["status"] for r in s["results"]] for s in again["stacks"]],
                         [[r["status"] for r in s["results"]] for s in self.card["stacks"]])

    def test_expected_findings(self):
        b = self.by
        # Lost link: only a robot-side watchdog helps, with or without BotInfo.
        self.assertEqual(b["rover-nowd/botinfo"]["LNK-02"], "fail")
        self.assertEqual(b["sesame-stock/botinfo"]["LNK-02"], "fail")
        self.assertEqual(b["rover-wd/raw"]["LNK-02"], "pass")
        self.assertEqual(b["sesame-patched/raw"]["LNK-02"], "pass")
        # Raw connectors cannot latch a stop against a hijacked agent; BotInfo can.
        for sid in ("rover-nowd/raw", "rover-wd/raw", "sesame-stock/raw", "sesame-patched/raw"):
            self.assertEqual(b[sid]["STP-02"], "fail", sid)
            self.assertEqual(b[sid.replace("/raw", "/botinfo")]["STP-02"], "pass", sid)
        # The firmware patch alone already blocks planting text and runaway walks.
        self.assertEqual(b["sesame-patched/raw"]["INJ-01"], "pass")
        self.assertEqual(b["sesame-stock/raw"]["INJ-01"], "fail")
        for s in self.card["stacks"]:
            if s["layer"] == "botinfo":
                fails = [c for c, st in self.by[s["id"]].items() if st == "fail"]
                self.assertTrue(set(fails) <= {"LNK-02"}, (s["id"], fails))

    def test_scorecard_is_json(self):
        json.dumps(self.card, allow_nan=False)


class AgentGrammar(unittest.TestCase):
    def test_parse_calls(self):
        text = 'x CALL drive {"linear_mps": 1.5} y CALL send_command command=forward steps=5 CALL botinfo_resume'
        self.assertEqual(parse_calls(text), [("drive", {"linear_mps": 1.5}),
                                             ("send_command", {"command": "forward", "steps": 5}),
                                             ("botinfo_resume", {})])


class Templates(unittest.TestCase):
    def test_fill_keeps_types_and_never_splices(self):
        t = {"topic": "/cmd_vel", "x": "$v", "lit": "$$v", "inner": ["$v", "a$v"]}
        self.assertEqual(fill(t, {"v": 0.25}), {"topic": "/cmd_vel", "x": 0.25, "lit": "$v", "inner": [0.25, "a$v"]})

    def test_binding_needs_a_stop(self):
        with self.assertRaisesRegex(BindingError, "stop"):
            ToolTemplateBackend(object(), {"binding": {"kind": "mcp-tools"}, "actions": {}})

    def test_unicycle(self):
        pts = unicycle_path((0.0, 0.0, 0.0), 1.0, 0.0, 2.0)
        self.assertAlmostEqual(pts[-1][0], 2.0)


class SesameOverHttp(unittest.TestCase):
    def test_walk_over_real_http(self):
        from botinfo.clock import RealClock
        from botinfo.core import BotInfo
        from botinfo.policy import load_policy
        mock = MockSesame(clock=RealClock(), safety_patch=True)
        httpd, url = serve(mock, 0)
        try:
            botinfo = BotInfo(load_policy(ROOT / "policies" / "sesame.toml"), SesameBackend(HttpTransport(url)))
            self.assertTrue(botinfo.call_tool("walk", {"direction": "forward", "steps": 1})["ok"])
            t_end = time.monotonic() + 1.6
            while time.monotonic() < t_end:
                botinfo.tick()
                time.sleep(0.02)
            cmds = [v for _, _, v in mock.sequence()]
            self.assertEqual(cmds[0], "forward")
            self.assertEqual(cmds[-1], "stop")
        finally:
            httpd.shutdown()
            httpd.server_close()

    def test_refuses_remote_urls(self):
        with self.assertRaises(ValueError):
            HttpTransport("http://192.168.4.1")
        with self.assertRaises(ValueError):
            HttpTransport("https://127.0.0.1")


class McpServer(unittest.TestCase):
    def test_protocol_basics(self):
        srv = StdioServer(lambda: [{"name": "t"}], lambda n, a: ({"ok": n == "t"}, n != "t"))
        init = srv.handle({"jsonrpc": "2.0", "id": 1, "method": "initialize",
                           "params": {"protocolVersion": "2025-03-26"}})
        self.assertEqual(init["result"]["protocolVersion"], "2025-03-26")
        self.assertEqual(srv.handle({"jsonrpc": "2.0", "id": 2, "method": "nope"})["error"]["code"], -32601)
        self.assertIsNone(srv.handle({"jsonrpc": "2.0", "method": "notifications/initialized"}))
        bad = srv.handle({"jsonrpc": "2.0", "id": 3, "method": "tools/call", "params": {"name": "x"}})
        self.assertTrue(bad["result"]["isError"])

    def test_eof_runs_disconnect(self):
        hit = threading.Event()
        inp = io.StringIO('{"jsonrpc":"2.0","id":1,"method":"ping"}\n')
        out = io.StringIO()
        StdioServer(lambda: [], lambda n, a: ({}, False), inp=inp, out=out, on_disconnect=hit.set).serve()
        self.assertTrue(hit.is_set())
        self.assertEqual(json.loads(out.getvalue())["result"], {})

    def test_botinfo_serve_fronting_a_stdio_connector(self):
        """Agent's client -> botinfo serve (MCP stdio) -> reference connector (a second MCP stdio server)."""
        from botinfo.mcp.client import StdioMCPClient
        from botinfo.owner import send
        d = Path(tempfile.mkdtemp(prefix="botinfo-"))
        audit, sock = d / "audit.jsonl", d / "o.sock"
        err = open(d / "stderr.log", "w")
        self.addCleanup(err.close)
        c = StdioMCPClient(["{python}", "-m", "botinfo", "serve", "--policy", str(ROOT / "policies/simrover.toml"),
                            "--binding", str(ROOT / "bindings/simrover-ros-style-stdio.toml"), "--audit", str(audit),
                            "--socket", str(sock)], timeout_s=20, cwd=str(ROOT), stderr=err)
        try:
            self.assertNotIn("publish_once", {t["name"] for t in c.tools})
            self.assertEqual(c.call_tool("drive", {"linear_mps": 0.3, "duration_s": 0.6})["decision"], "allowed")
            time.sleep(0.4)
            self.assertGreater(c.call_tool("botinfo_status", {})["robot"]["linear_mps"], 0.2)
            time.sleep(0.6)
            self.assertLess(abs(c.call_tool("botinfo_status", {})["robot"]["linear_mps"]), 0.01)
            self.assertTrue(send(sock, {"cmd": "stop"})["ok"])
            with self.assertRaises(Exception):
                c.call_tool("drive", {"linear_mps": 0.3, "duration_s": 0.5})
            send(sock, {"cmd": "resume"})
            self.assertEqual(c.call_tool("drive", {"linear_mps": 0.2, "duration_s": 3.0})["decision"], "allowed")
        finally:
            c.close()
        time.sleep(0.2)
        self.assertEqual(verify(audit)[0], True)
        events = [json.loads(x) for x in audit.read_text().splitlines()]
        self.assertEqual(events[-1]["event"], "stop")
        self.assertEqual(events[-1]["reason"], "the agent disconnected")

    def test_slow_tool_never_delays_a_stop(self):
        """botinfo_wait answers on its own thread: a botinfo_stop sent while the agent waits is answered at once, and
        it ends the wait."""
        from botinfo.mcp.client import StdioMCPClient
        d = Path(tempfile.mkdtemp(prefix="botinfo-"))
        err = open(d / "stderr.log", "w")
        self.addCleanup(err.close)
        c = StdioMCPClient(["{python}", "-m", "botinfo", "serve", "--policy", str(ROOT / "policies/simrover.toml"),
                            "--binding", str(ROOT / "bindings/simrover-ros-style.toml"), "--audit",
                            str(d / "audit.jsonl"), "--socket", str(d / "o.sock")], timeout_s=20, cwd=str(ROOT),
                           stderr=err)
        try:
            self.assertIn("botinfo_wait", {t["name"] for t in c.tools})
            self.assertEqual(c.call_tool("drive", {"linear_mps": 0.3, "duration_s": 0.6})["decision"], "allowed")
            t0 = time.monotonic()
            w = c.call_tool("botinfo_wait", {})
            self.assertEqual(w["ended_by"], "idle")
            self.assertGreater(time.monotonic() - t0, 0.4)
            self.assertIsNone(w["botinfo"]["lease"])                         # the stop at the lease end is sent
            c.call_tool("drive", {"linear_mps": 0.3, "duration_s": 4.0})
            box = {}
            th = threading.Thread(target=lambda: box.update(w=c.call_tool("botinfo_wait", {})))
            th.start()
            time.sleep(0.5)
            t1 = time.monotonic()
            self.assertTrue(c.call_tool("botinfo_stop", {})["delivered"])
            self.assertLess(time.monotonic() - t1, 0.3)                     # not queued behind the wait
            th.join(timeout=2.0)
            self.assertEqual(box["w"]["ended_by"], "stopped")
            self.assertLess(box["w"]["waited_s"], 1.0)
        finally:
            c.close()


if __name__ == "__main__":
    unittest.main()
