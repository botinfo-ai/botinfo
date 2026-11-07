import unittest

from helpers import ROOT, RoverRig, SesameRig

from botinfo.backends.tooltemplate import BindingError, ToolTemplateBackend, load_binding

TWIST = "geometry_msgs/msg/Twist"


def receipt(r, sid):
    return next(e for e in r.botinfo.audit.events("stop_receipt") if e["stop_id"] == sid)


class StopReceipts(unittest.TestCase):
    def test_receipt_says_when_the_rover_came_to_rest(self):
        r = RoverRig()
        r.call("drive", {"linear_mps": 0.3, "duration_s": 4.0})
        r.run(1.0)
        res = r.call("botinfo_stop")
        self.assertEqual(r.botinfo.owner_status()["last_stop"], {"id": res["stop_id"], "by": "agent", "rest": "watching"})
        r.run(1.0)
        rec = receipt(r, res["stop_id"])
        self.assertEqual((rec["by"], rec["outcome"]), ("agent", "at_rest"))
        self.assertLessEqual(rec["first_read"]["after_stop_s"], 0.05)       # read at the first tick after the stop
        self.assertGreater(rec["first_read"]["linear_mps"], 0.1)            # still rolling when first read
        self.assertGreater(rec["peak_after_stop"]["linear_mps"], 0.1)
        self.assertGreater(rec["at_rest_after_s"], 0.1)                     # 0.3 m/s at 2 m/s^2 takes 0.15 s
        self.assertLess(rec["at_rest_after_s"], 0.5)
        self.assertGreater(rec["moved_m_after_first_read"], 0.0)
        self.assertEqual(r.call("botinfo_status")["botinfo"]["last_stop"]["rest"], "at_rest")

    def test_receipt_says_when_the_rover_did_not_come_to_rest(self):
        r = RoverRig()
        r.call("drive", {"linear_mps": 0.3, "duration_s": 4.0})
        r.run(1.0)
        res = r.call("botinfo_stop")
        # another client of the same connector keeps driving: BotInfo cannot stop what it does not send
        r.conn.call_tool("publish_for_durations", {"topic": "/cmd_vel", "msg_type": TWIST, "rate_hz": 10,
                                                   "messages": [{"linear": {"x": 0.2}, "angular": {"z": 0.0}}],
                                                   "durations": [5.0]})
        r.run(4.0)
        rec = receipt(r, res["stop_id"])
        self.assertEqual(rec["outcome"], "not_at_rest")
        self.assertGreaterEqual(rec["watched_s"], 3.0)
        self.assertGreater(rec["last_read"]["linear_mps"], 0.15)
        self.assertNotIn("at_rest_after_s", rec)
        self.assertEqual(r.botinfo.owner_status()["last_stop"]["rest"], "not_at_rest")

    def test_receipt_gives_up_when_the_status_cannot_be_read(self):
        r = RoverRig()
        r.call("drive", {"linear_mps": 0.3, "duration_s": 4.0})
        r.run(1.0)
        r.link.up = False
        res = r.botinfo.owner_stop()
        self.assertFalse(res["delivered"])
        r.run(1.0)
        rec = receipt(r, res["stop_id"])
        self.assertEqual((rec["outcome"], rec["reads"], rec["read_errors"]), ("unreadable", 3, 3))
        self.assertIsNone(rec["first_read"])
        self.assertIn("link down", rec["last_error"])

    def test_lease_end_receipt_closes_when_the_next_motion_starts(self):
        r = RoverRig()
        r.call("drive", {"linear_mps": 0.3, "duration_s": 1.0})
        r.run(1.05)
        end = r.botinfo.audit.events("lease_end")[-1]
        self.assertEqual(r.botinfo.owner_status()["last_stop"], {"id": end["stop_id"], "by": "lease_end",
                                                                 "rest": "watching"})
        self.assertTrue(r.call("drive", {"linear_mps": 0.0, "angular_rps": 0.5, "duration_s": 1.0})["ok"])
        rec = receipt(r, end["stop_id"])
        self.assertEqual(rec["outcome"], "superseded")
        self.assertIn("new motion", rec["note"])

    def test_no_receipt_without_a_settle_table(self):
        r = SesameRig()
        res = r.botinfo.owner_stop()
        r.run(0.5)
        self.assertEqual(r.botinfo.audit.events("stop_receipt"), [])
        self.assertEqual(r.botinfo.owner_status()["last_stop"], {"id": res["stop_id"], "by": "owner",
                                                                 "rest": "not_measured"})

    def test_settle_table_is_checked(self):
        b = load_binding(ROOT / "bindings" / "simrover-ros-style.toml")
        for bad in ({"fields": ["nope"]}, {"fields": ["linear_mps", "heading_rad"]}, {"fields": []},
                    {"fields": ["linear_mps"], "below": 0}, {"fields": ["linear_mps"], "timeout_s": "3"},
                    {"fields": ["linear_mps"], "hold_s": True}, {"fields": ["linear_mps"], "extra": 1}):
            b["settle"] = bad
            with self.assertRaises(BindingError, msg=str(bad)):
                ToolTemplateBackend(object(), b)
        b["settle"] = {"fields": ["linear_mps"]}
        spec = ToolTemplateBackend(object(), b).settle_spec()
        self.assertEqual((spec["below"], spec["hold_s"], spec["timeout_s"], spec["every_s"]), (0.01, 0.2, 3.0, 0.1))
        self.assertEqual(spec["pose"], {"x": "x", "y": "y", "heading": "heading_rad"})


if __name__ == "__main__":
    unittest.main()
