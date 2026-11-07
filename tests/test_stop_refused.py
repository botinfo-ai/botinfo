import unittest

from helpers import RoverRig

from botinfo.backends.base import refusal


class RefusingConnector:
    """The reference connector, except that it answers the stop (a zero Twist on /cmd_vel) with a refusal returned
    as data instead of an error, and does not pass it on, so the rover keeps moving."""

    def __init__(self, conn, reply, refuse_first_only=False):
        self.conn, self.reply, self.first_only, self.stops = conn, reply, refuse_first_only, 0

    def call_tool(self, name, args, on_sent=None):
        msg = args.get("msg") or {}
        if (args.get("topic") == "/cmd_vel" and (msg.get("linear") or {}).get("x") == 0.0
                and (msg.get("angular") or {}).get("z") == 0.0):
            self.stops += 1
            if not self.first_only or self.stops == 1:
                return dict(self.reply)
        return self.conn.call_tool(name, args)

    def __getattr__(self, name):
        return getattr(self.conn, name)


def refusing(reply=None, **kw):
    r = RoverRig()
    r.backend.caller = RefusingConnector(r.conn, reply or {"status": "error", "message": "robot busy"}, **kw)
    return r


class RefusalShapes(unittest.TestCase):
    def test_replies_that_say_the_call_failed(self):
        for reply in (False, {"ok": False}, {"success": False}, {"successful": False}, {"delivered": False},
                      {"isError": True}, {"error": "estop pressed"}, {"status": "error", "message": "busy"},
                      {"status": "Failed"}):
            self.assertTrue(refusal(reply), reply)
        self.assertIn("busy", refusal({"status": "error", "message": "busy"}))

    def test_replies_that_do_not(self):
        for reply in (None, True, {}, {"success": True}, {"ok": True, "status": "stopped"}, {"text": "Stopped"},
                      {"result": "done"}, {"error": ""}, {"status": "ok"}, "stopped"):
            self.assertIsNone(refusal(reply), reply)


class RefusedStop(unittest.TestCase):
    def test_a_refused_stop_is_logged_not_delivered(self):
        r = refusing()
        r.call("drive", {"linear_mps": 0.3, "duration_s": 4.0})
        r.run(1.0)
        res = r.botinfo.owner_stop()
        self.assertFalse(res["delivered"])
        self.assertFalse(res["ok"])
        self.assertIn("stop refused", res["error"])
        self.assertIn("robot busy", res["error"])
        self.assertTrue(res["latched"])
        ev = r.botinfo.audit.events("stop")[-1]
        self.assertFalse(ev["delivered"])
        self.assertIn("robot busy", ev["error"])
        self.assertEqual(r.call("drive", {"linear_mps": 0.2, "duration_s": 1.0})["rule"], "stopped")

    def test_a_refused_stop_at_lease_end_reaches_the_latch(self):
        r = refusing({"ok": False})
        r.call("drive", {"linear_mps": 0.3, "duration_s": 1.0})
        r.run(1.5)
        end = r.botinfo.audit.events("lease_end")[-1]
        self.assertFalse(end["stop_sent"])
        self.assertIn("ok is false", end["error"])
        err = r.botinfo.audit.events("backend_error")[-1]
        self.assertEqual(err["during"], "stop at lease end")
        stop = r.botinfo.audit.events("stop")[-1]
        self.assertEqual((stop["by"], stop["delivered"], stop["latched"]), ("botinfo", False, True))
        self.assertTrue(r.botinfo.latched)
        self.assertEqual(r.call("drive", {"linear_mps": 0.2, "duration_s": 1.0})["rule"], "stopped")

    def test_a_refused_replace_stop_does_not_start_the_new_motion(self):
        r = refusing()
        r.call("drive", {"linear_mps": 0.3, "duration_s": 4.0})
        r.run(0.5)
        res = r.call("drive", {"linear_mps": 0.2, "duration_s": 1.0})
        self.assertEqual((res["decision"], res["rule"]), ("failed", "backend_error"))
        self.assertIn("stop refused", res["reason"])
        self.assertTrue(r.botinfo.latched)

    def test_one_refusal_among_repeats_counts(self):
        r = refusing(refuse_first_only=True)          # the binding sends its stop twice; only the first is refused
        r.call("drive", {"linear_mps": 0.3, "duration_s": 4.0})
        r.run(1.0)
        res = r.botinfo.owner_stop()
        self.assertFalse(res["delivered"])

    def test_a_stop_the_connector_carried_out_is_still_delivered(self):
        r = RoverRig()
        r.call("drive", {"linear_mps": 0.3, "duration_s": 4.0})
        r.run(1.0)
        res = r.botinfo.owner_stop()
        self.assertTrue(res["delivered"])
        self.assertNotIn("error", res)
        self.assertIsNone(r.botinfo.audit.events("stop")[-1]["error"])


if __name__ == "__main__":
    unittest.main()
