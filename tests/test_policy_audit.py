import json
import math
import tempfile
import unittest
from pathlib import Path

from helpers import ROOT, tiny_policy

from botinfo.audit import AuditLog, jsonable, verify
from botinfo.clock import VirtualClock
from botinfo.policy import (PolicyError, check_params, load_policy, needs_confirmation, policy_from_dict,
                          split_meta)


class PolicyLoading(unittest.TestCase):
    def test_shipped_policies_load(self):
        for f in ("simrover.toml", "sesame.toml"):
            p = load_policy(ROOT / "policies" / f)
            self.assertTrue(p.actions)

    def base(self):
        return {"policy": {"name": "t", "robot": "r"},
                "actions": {"a": {"kind": "instant", "params": {"x": {"type": "number", "min": 0, "max": 1}}}}}

    def test_unknown_key_is_an_error(self):
        d = self.base()
        d["policy"]["comand_ttl_s"] = 5          # typo must not silently drop a limit
        with self.assertRaisesRegex(PolicyError, "unknown key"):
            policy_from_dict(d)
        d = self.base()
        d["actions"]["a"]["params"]["x"]["maximum"] = 2
        with self.assertRaisesRegex(PolicyError, "unknown key"):
            policy_from_dict(d)

    def test_numbers_need_both_bounds(self):
        d = self.base()
        del d["actions"]["a"]["params"]["x"]["max"]
        with self.assertRaisesRegex(PolicyError, "both min and max"):
            policy_from_dict(d)

    def test_bad_values_rejected(self):
        for mutate, msg in [
            (lambda d: d["actions"]["a"].update(kind="teleport"), "kind"),
            (lambda d: d["actions"]["a"]["params"]["x"].update(min=2), "above max"),
            (lambda d: d["actions"]["a"]["params"]["x"].update(default=5), "default"),
            (lambda d: d["policy"].update(on_busy="maybe"), "on_busy"),
            (lambda d: d.update(actions={"botinfo_x": {"kind": "instant"}}), "bad action name"),
            (lambda d: d["actions"]["a"]["params"].update(request_id={"type": "boolean"}), "bad parameter name"),
            (lambda d: d["actions"]["a"].update(kind="motion", uninterruptible=True), "interruptible"),
        ]:
            d = self.base()
            mutate(d)
            with self.assertRaisesRegex(PolicyError, msg):
                policy_from_dict(d)

    def test_duration_cannot_exceed_lease(self):
        with self.assertRaisesRegex(PolicyError, "max_lease_s"):
            tiny_policy(lease={"max_lease_s": 2.0})

    def test_workspace_margin_must_leave_room(self):
        with self.assertRaisesRegex(PolicyError, "no room"):
            tiny_policy(workspace={"x": [0, 1], "y": [0, 1], "margin_m": 0.5})


class ParamChecks(unittest.TestCase):
    def setUp(self):
        self.p = tiny_policy()
        self.drive = self.p.actions["drive"]

    def check(self, args):
        return check_params(self.drive, args)

    def test_good_call_and_default(self):
        v, why = self.check({"linear_mps": 0.3, "duration_s": 2})
        self.assertIsNone(why)
        self.assertEqual(v, {"linear_mps": 0.3, "angular_rps": 0.0, "duration_s": 2.0})
        self.assertIsInstance(v["duration_s"], float)

    def test_refused_never_clamped(self):
        for bad in [{"linear_mps": 0.6, "duration_s": 1}, {"linear_mps": float("nan"), "duration_s": 1},
                    {"linear_mps": float("inf"), "duration_s": 1}, {"linear_mps": True, "duration_s": 1},
                    {"linear_mps": "0.3", "duration_s": 1}, {"linear_mps": [0.3], "duration_s": 1},
                    {"linear_mps": 0.3}, {"linear_mps": 0.3, "duration_s": 1, "turbo": True},
                    {"linear_mps": 0.3, "duration_s": 0}]:
            v, why = self.check(bad)
            self.assertIsNone(v, bad)
            self.assertTrue(why)

    def test_integer_and_enum(self):
        p = load_policy(ROOT / "policies" / "sesame.toml")
        walk = p.actions["walk"]
        self.assertEqual(check_params(walk, {"direction": "forward", "steps": 3.0})[0]["steps"], 3)
        for bad in ({"direction": "forward", "steps": 3.5}, {"direction": "up", "steps": 3},
                    {"direction": "forward", "steps": 11}, {"direction": "forward", "steps": False}):
            self.assertIsNone(check_params(walk, bad)[0], bad)

    def test_confirmation_rules(self):
        p = load_policy(ROOT / "policies" / "simrover.toml")
        d = p.actions["drive"]
        self.assertIsNone(needs_confirmation(d, {"linear_mps": 0.3}))
        self.assertIn("above 0.35", needs_confirmation(d, {"linear_mps": 0.45}))
        s = load_policy(ROOT / "policies" / "sesame.toml")
        self.assertIn("cannot interrupt", needs_confirmation(s.actions["emote"], {"name": "wave"}))

    def test_meta(self):
        args, meta, why = split_meta({"x": 1, "request_id": "a-1", "issued_at": 5})
        self.assertEqual((args, meta, why), ({"x": 1}, {"request_id": "a-1", "issued_at": 5.0}, None))
        for bad in ({"request_id": "has space"}, {"request_id": ""}, {"issued_at": float("nan")},
                    {"issued_at": "now"}, {"request_id": "x" * 200}):
            self.assertTrue(split_meta(bad)[2], bad)


class Audit(unittest.TestCase):
    def test_chain_verifies_and_detects_edits(self):
        d = Path(tempfile.mkdtemp())
        f = d / "a.jsonl"
        log = AuditLog(f, clock=VirtualClock())
        for i in range(5):
            log.write("request", tool="drive", args={"linear_mps": i, "bad": float("nan")})
        self.assertEqual(verify(f), (True, 5, None))
        lines = f.read_text().splitlines()
        # edit one value
        rec = json.loads(lines[2])
        rec["tool"] = "stop"
        tampered = lines[:2] + [json.dumps(rec, sort_keys=True, separators=(",", ":"))] + lines[3:]
        f.write_text("\n".join(tampered) + "\n")
        self.assertFalse(verify(f)[0])
        # delete one line
        f.write_text("\n".join(lines[:2] + lines[3:]) + "\n")
        self.assertFalse(verify(f)[0])
        # reopening a good log continues the chain
        f.write_text("\n".join(lines) + "\n")
        log2 = AuditLog(f, clock=VirtualClock())
        log2.write("stop", by="owner")
        self.assertEqual(verify(f), (True, 6, None))

    def test_jsonable_bounds_hostile_input(self):
        v = jsonable({"a": float("nan"), "b": "x" * 5000, "c": list(range(500)), "d": 2**80})
        self.assertEqual(v["a"], "nan")
        self.assertIn("cut", v["b"])
        self.assertEqual(len(v["c"]), 101)
        self.assertEqual(v["d"], str(2**80))
        json.dumps(v, allow_nan=False)
        self.assertTrue(math.isfinite(jsonable(1.5)))


if __name__ == "__main__":
    unittest.main()
