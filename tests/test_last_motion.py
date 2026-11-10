import threading
import time
import unittest

from botinfo.backends.base import BackendError
from helpers import RoverRig
from test_stop_refused import refusing


class LastMotion(unittest.TestCase):
    def test_stop_says_how_long_the_cut_motion_ran(self):
        r = RoverRig()
        lid = r.call("drive", {"linear_mps": 0.3, "duration_s": 4.0})["lease_id"]
        r.run(1.2)
        res = r.botinfo.owner_stop()
        m = res["stopped_motion"]
        self.assertEqual((m["lease_id"], m["action"], m["ended"], m["stop_id"]),
                         (lid, "drive", "stop by owner", res["stop_id"]))
        self.assertAlmostEqual(m["of_s"], 4.0, places=3)
        self.assertAlmostEqual(m["ran_s"], 1.2, delta=0.03)
        self.assertEqual(r.call("botinfo_status")["botinfo"]["last_motion"], m)
        ev = r.botinfo.audit.events("stop")[-1]
        self.assertEqual((ev["ran_s"], ev["of_s"]), (m["ran_s"], m["of_s"]))

    def test_stop_with_nothing_running_names_no_motion(self):
        r = RoverRig()
        res = r.call("botinfo_stop")
        self.assertNotIn("stopped_motion", res)
        self.assertIsNone(r.botinfo.owner_status()["last_motion"])
        self.assertNotIn("ran_s", r.botinfo.audit.events("stop")[-1])

    def test_lease_end_records_the_full_run(self):
        r = RoverRig()
        r.call("drive", {"linear_mps": 0.3, "duration_s": 1.0})
        r.run(1.1)
        end = r.botinfo.audit.events("lease_end")[-1]
        lm = r.botinfo.owner_status()["last_motion"]
        self.assertEqual((lm["lease_id"], lm["ended"], lm["stop_id"], lm["ran_s"], lm["of_s"]),
                         (end["lease_id"], "lease_end", end["stop_id"], end["ran_s"], end["of_s"]))
        self.assertAlmostEqual(lm["of_s"], 1.0, places=3)

    def test_wait_returns_the_run_of_the_motion_that_ended(self):
        r = RoverRig()
        r.botinfo.sleep = r.run
        lid = r.call("drive", {"linear_mps": 0.3, "duration_s": 1.0})["lease_id"]
        w = r.call("botinfo_wait")
        lm = w["botinfo"]["last_motion"]
        self.assertEqual((w["ended_by"], w["moving"], lm["lease_id"], lm["ended"]), ("idle", False, lid, "lease_end"))
        self.assertAlmostEqual(lm["ran_s"], 1.0, delta=0.03)

    def test_replace_records_the_replaced_motion(self):
        r = RoverRig()
        lid = r.call("drive", {"linear_mps": 0.3, "duration_s": 3.0})["lease_id"]
        r.run(0.5)
        res = r.call("drive", {"linear_mps": 0.2, "duration_s": 1.0})
        self.assertTrue(res["ok"])
        lm = r.botinfo.owner_status()["last_motion"]
        self.assertEqual((lm["lease_id"], lm["ended"], lm["ran_s"]), (lid, "replaced by drive", res["replaced"]["ran_s"]))

    def test_a_replace_whose_new_motion_failed_to_start_says_it_may_have_reached_the_robot(self):
        r = RoverRig()
        lid = r.call("drive", {"linear_mps": 0.3, "duration_s": 3.0})["lease_id"]
        r.run(0.5)

        def refuse_start(action, params):
            raise BackendError("start refused")

        r.botinfo.backend.start_motion = refuse_start
        self.assertEqual(r.call("drive", {"linear_mps": 0.2, "duration_s": 1.0})["rule"], "backend_error")
        lm = r.botinfo.owner_status()["last_motion"]
        self.assertEqual((lm["lease_id"], lm["ended"]),
                         (lid, "stopped for drive; starting drive failed (the connector's error is in the audit log), "
                               "and the command may have reached the robot"))
        failed = [e for e in r.botinfo.audit.events("decision") if e.get("decision") == "failed"]
        self.assertEqual(failed[-1]["error"], "start refused")
        self.assertNotIn("start refused", str(r.call("botinfo_status")["botinfo"]["last_motion"]))

    def test_a_replace_cut_by_a_stop_as_it_started_says_so(self):
        r = RoverRig()
        lid = r.call("drive", {"linear_mps": 0.3, "duration_s": 3.0})["lease_id"]
        r.run(0.5)
        start = r.botinfo.backend.start_motion

        def start_then_owner_stop(action, params):
            start(action, params)
            r.botinfo.owner_stop()

        r.botinfo.backend.start_motion = start_then_owner_stop
        self.assertEqual(r.call("drive", {"linear_mps": 0.2, "duration_s": 1.0})["rule"], "stopped")
        lm = r.botinfo.owner_status()["last_motion"]
        self.assertEqual((lm["lease_id"], lm["ended"]), (lid, "replaced by drive, which a stop cut as it started"))
        self.assertIsNone(r.botinfo.owner_status()["lease"])

    def test_a_replace_cut_by_a_refused_stop_as_it_started_says_the_stop_was_not_delivered(self):
        r = RoverRig()
        lid = r.call("drive", {"linear_mps": 0.3, "duration_s": 3.0})["lease_id"]
        r.run(0.5)
        start = r.botinfo.backend.start_motion

        def refuse_stop():
            raise BackendError("stop refused")

        def start_then_refused_owner_stop(action, params):
            start(action, params)
            r.botinfo.backend.stop = refuse_stop
            self.assertFalse(r.botinfo.owner_stop()["delivered"])

        r.botinfo.backend.start_motion = start_then_refused_owner_stop
        self.assertEqual(r.call("drive", {"linear_mps": 0.2, "duration_s": 1.0})["rule"], "stopped")
        lm = r.botinfo.owner_status()["last_motion"]
        self.assertEqual((lm["lease_id"], lm["ended"]),
                         (lid, "replaced by drive; the stop that arrived as drive started was not delivered, so drive "
                               "may still be running"))

    def test_a_stop_sent_after_the_starting_motion_rewrites_the_record_once_it_is_sent(self):
        r = RoverRig()
        lid = r.call("drive", {"linear_mps": 0.3, "duration_s": 3.0})["lease_id"]
        r.run(0.5)
        start, gen = r.botinfo.backend.start_motion, r.botinfo.stop_gen
        stopper = threading.Thread(target=r.botinfo.owner_stop)

        def start_while_the_owner_stops(action, params):
            start(action, params)
            stopper.start()                    # on another thread the owner's stop waits for this call to finish
            deadline = time.monotonic() + 5
            while r.botinfo.stop_gen == gen and time.monotonic() < deadline:
                time.sleep(0.001)

        r.botinfo.backend.start_motion = start_while_the_owner_stops
        self.assertEqual(r.call("drive", {"linear_mps": 0.2, "duration_s": 1.0})["rule"], "stopped")
        stopper.join(5)
        self.assertFalse(stopper.is_alive())
        lm = r.botinfo.owner_status()["last_motion"]
        self.assertEqual((lm["lease_id"], lm["ended"]), (lid, "replaced by drive, which a stop cut as it started"))
        self.assertTrue(r.botinfo.audit.events("stop")[-1]["delivered"])

    def test_a_refused_stop_after_a_delivered_one_does_not_undo_it(self):
        r = RoverRig()
        lid = r.call("drive", {"linear_mps": 0.3, "duration_s": 3.0})["lease_id"]
        r.run(0.5)
        start = r.botinfo.backend.start_motion

        def refuse_stop():
            raise BackendError("stop refused")

        def start_then_two_owner_stops(action, params):
            start(action, params)
            self.assertTrue(r.botinfo.owner_stop()["delivered"])
            r.botinfo.backend.stop = refuse_stop
            self.assertFalse(r.botinfo.owner_stop()["delivered"])

        r.botinfo.backend.start_motion = start_then_two_owner_stops
        self.assertEqual(r.call("drive", {"linear_mps": 0.2, "duration_s": 1.0})["rule"], "stopped")
        lm = r.botinfo.owner_status()["last_motion"]
        self.assertEqual((lm["lease_id"], lm["ended"]), (lid, "replaced by drive, which a stop cut as it started"))

    def test_a_stop_that_raises_after_the_starting_motion_leaves_no_being_sent_text(self):
        r = RoverRig()
        lid = r.call("drive", {"linear_mps": 0.3, "duration_s": 3.0})["lease_id"]
        r.run(0.5)
        start, gen = r.botinfo.backend.start_motion, r.botinfo.stop_gen
        raised = []

        def broken_stop():
            raise RuntimeError("connector crashed")

        def owner_stop():
            try:
                r.botinfo.owner_stop()
            except RuntimeError as e:
                raised.append(e)

        stopper = threading.Thread(target=owner_stop)

        def start_while_the_owner_stops(action, params):
            start(action, params)
            r.botinfo.backend.stop = broken_stop
            stopper.start()                    # on another thread the owner's stop waits for this call to finish
            deadline = time.monotonic() + 5
            while r.botinfo.stop_gen == gen and time.monotonic() < deadline:
                time.sleep(0.001)

        r.botinfo.backend.start_motion = start_while_the_owner_stops
        self.assertEqual(r.call("drive", {"linear_mps": 0.2, "duration_s": 1.0})["rule"], "stopped")
        stopper.join(5)
        self.assertFalse(stopper.is_alive())
        self.assertEqual(len(raised), 1)
        lm = r.botinfo.owner_status()["last_motion"]
        self.assertEqual((lm["lease_id"], lm["ended"]),
                         (lid, "replaced by drive; the stop that arrived as drive started was not delivered, so drive "
                               "may still be running"))

    def test_a_stop_that_was_refused_says_so_in_last_motion(self):
        r = refusing()
        r.call("drive", {"linear_mps": 0.3, "duration_s": 4.0})
        r.run(1.0)
        res = r.botinfo.owner_stop()
        self.assertFalse(res["delivered"])
        self.assertEqual(res["stopped_motion"]["ended"], "stop by owner, stop not delivered")
        self.assertEqual(r.botinfo.owner_status()["last_motion"], res["stopped_motion"])

    def test_a_refused_replace_stop_says_so_in_last_motion(self):
        r = refusing()
        lid = r.call("drive", {"linear_mps": 0.3, "duration_s": 4.0})["lease_id"]
        r.run(0.5)
        self.assertEqual(r.call("drive", {"linear_mps": 0.2, "duration_s": 1.0})["rule"], "backend_error")
        lm = r.botinfo.owner_status()["last_motion"]
        self.assertEqual((lm["lease_id"], lm["ended"]), (lid, "stop for drive not delivered; drive was not sent"))


if __name__ == "__main__":
    unittest.main()
