import copy
from dataclasses import replace
import importlib.util
import math
import os
from pathlib import Path
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("bridge_mapping", ROOT / "mapping.py")
mapping = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = mapping
spec.loader.exec_module(mapping)


def snapshot(sequence=1, source_ns=100):
    return {"source_ns": source_ns, "domain": "device_session_a", "sequence": sequence,
            "reset_counter": 0, "epoch_id": 0, "map_id": 7, "state": "TRACKING",
            "valid": True, "reasons": 0, "covariance_availability": "UNAVAILABLE",
            "T_world_camera": {"parent": "slam_world", "child": "camera_left_optical",
                "matrix": [0, -1, 0, 1.25, 1, 0, 0, -2.5, 0, 0, 1, 3.75, 0, 0, 0, 1]}}


def clock(s, **changes):
    e = mapping.ClockEvidence(s["source_ns"], s["domain"], "adapter_boot_a",
        "ros_live", 2, 10, 1700000000123456789, 20, 3)
    return replace(e, **changes)


def gate(**changes):
    config = dict(session="adapter_boot_a", source_domain="device_session_a",
        target_domain="ros_live", generation=2, max_age_ns=100, max_uncertainty_ns=10)
    config.update(changes)
    return mapping.Gate(**config)


class MappingTests(unittest.TestCase):
    def test_nonidentity_source_pose_and_exact_stamp(self):
        s = snapshot()
        before = copy.deepcopy(s)
        fields, reason = gate().advance(s, clock(s))
        self.assertEqual(reason, "pose_published")
        self.assertEqual(fields["header"], {"frame_id": "slam_world",
            "stamp": {"sec": 1700000000, "nanosec": 123456789}})
        self.assertEqual(fields["pose"]["position"], {"x": 1.25, "y": -2.5, "z": 3.75})
        self.assertEqual(fields["pose"]["orientation"]["x"], 0)
        self.assertEqual(fields["pose"]["orientation"]["y"], 0)
        self.assertAlmostEqual(fields["pose"]["orientation"]["z"], math.sqrt(0.5))
        self.assertAlmostEqual(fields["pose"]["orientation"]["w"], math.sqrt(0.5))
        self.assertEqual(s, before)
        self.assertNotIn("covariance", fields["pose"])

    def test_identity_and_half_turn_quaternions(self):
        cases = [([1, 1, 1], [0, 0, 0, 1]), ([1, -1, -1], [1, 0, 0, 0]),
                 ([-1, 1, -1], [0, 1, 0, 0]), ([-1, -1, 1], [0, 0, 1, 0])]
        for diagonal, expected in cases:
            with self.subTest(diagonal=diagonal):
                s = snapshot()
                m = s["T_world_camera"]["matrix"]
                m[:] = [0]*16
                m[0], m[5], m[10], m[15] = *diagonal, 1
                fields = mapping.pose_fields(s["T_world_camera"], 0)
                self.assertEqual(list(fields["pose"]["orientation"].values()), expected)

    def test_stamp_limits_and_no_float_conversion(self):
        t = snapshot()["T_world_camera"]
        limit = 2147483647999999999
        self.assertEqual(mapping.pose_fields(t, limit)["header"]["stamp"],
            {"sec": 2147483647, "nanosec": 999999999})
        for ns in (-1, limit+1, 1.0):
            with self.assertRaises(ValueError):
                mapping.pose_fields(t, ns)

    def test_frame_and_se3_rejection(self):
        for field, value in (("parent", "odom"), ("child", "base_link")):
            s = snapshot()
            s["T_world_camera"][field] = value
            self.assertEqual(gate().advance(s, clock(s))[1], "bad_frame")
        for index, value in ((0, float("nan")), (0, 2), (15, 0), (10, -1)):
            s = snapshot()
            s["T_world_camera"]["matrix"][index] = value
            self.assertEqual(gate().advance(s, clock(s))[1], "bad_pose")

    def test_tracking_loss_fault_and_reason_flags(self):
        for state in ("STARTING", "INITIALIZING", "LOST", "FAULT"):
            s = snapshot()
            s["state"] = state
            self.assertEqual(gate().advance(s, clock(s))[1], "invalid_snapshot")
        for key, value in (("valid", False), ("reasons", 4), ("reasons", 8)):
            s = snapshot()
            s[key] = value
            self.assertEqual(gate().advance(s, clock(s))[1], "invalid_snapshot")

    def test_frozen_heartbeat_becomes_stale_without_new_pose(self):
        s = snapshot()
        g = gate()
        self.assertIsNotNone(g.advance(s, clock(s))[0])
        self.assertEqual(g.advance(s, clock(s, age_upper_ns=30)), (None, "duplicate"))
        self.assertEqual(g.advance(s, clock(s, age_upper_ns=101)), (None, "stale_source"))
        self.assertEqual(s["source_ns"], 100)
        self.assertEqual(s["sequence"], 1)

    def test_invalid_same_sequence_cannot_republish(self):
        s = snapshot()
        g = gate()
        g.advance(s, clock(s))
        bad = copy.deepcopy(s)
        bad.update(valid=False, reasons=16, state="FAULT", T_world_camera=None)
        self.assertEqual(g.advance(bad, clock(bad))[1], "invalid_snapshot")
        self.assertEqual(g.advance(s, clock(s))[1], "duplicate")

    def test_reset_without_sequence_advance_and_recovery(self):
        s = snapshot()
        g = gate()
        g.advance(s, clock(s))
        reset = copy.deepcopy(s)
        reset.update(reset_counter=1, epoch_id=1, map_id=None, valid=False, reasons=4)
        self.assertEqual(g.advance(reset, clock(reset))[1], "reset_boundary")
        fresh = snapshot(2, 200)
        fresh.update(reset_counter=1, epoch_id=1, map_id=None)
        self.assertEqual(g.advance(fresh, clock(fresh))[1], "pose_published")

    def test_map_change_suppresses_boundary_and_old_epoch(self):
        g = gate()
        s = snapshot()
        g.advance(s, clock(s))
        s = snapshot(2, 200)
        s.update(map_id=8, reset_counter=1, epoch_id=1)
        self.assertEqual(g.advance(s, clock(s))[1], "reset_boundary")
        self.assertEqual(g.advance(s, clock(s))[1], "duplicate")
        s.update(sequence=3, source_ns=300)
        self.assertEqual(g.advance(s, clock(s))[1], "pose_published")
        old = snapshot(4, 400)
        self.assertEqual(g.advance(old, clock(old))[1], "regression_latched")
        self.assertEqual(g.advance(s, clock(s))[1], "regression_latched")

    def test_sequence_and_source_regressions_latch(self):
        for next_s in (snapshot(0, 200), snapshot(2, 100), snapshot(2, 99)):
            g = gate()
            s = snapshot()
            g.advance(s, clock(s))
            self.assertEqual(g.advance(next_s, clock(next_s))[1], "regression_latched")

    def test_missing_clock_and_mismatched_identity(self):
        s = snapshot()
        self.assertEqual(gate().advance(s, None)[1], "clock_unavailable")
        for key, value in (("source_ns", 101), ("source_domain", "pi_steady"),
                ("session", "old_boot"), ("target_domain", "ros_replay"), ("generation", 3)):
            self.assertEqual(gate().advance(s, clock(s, **{key: value}))[1],
                "clock_identity_mismatch")

    def test_foreign_domain_never_changes_source_watermark(self):
        s = snapshot()
        g = gate()
        g.advance(s, clock(s))
        wrong = snapshot(2, 0)
        wrong["domain"] = "unrelated_clock"
        self.assertEqual(g.advance(wrong, clock(wrong))[1], "bad_source")
        good = snapshot(2, 200)
        self.assertEqual(g.advance(good, clock(good))[1], "pose_published")

    def test_missing_source_and_pose(self):
        s = snapshot()
        s["source_ns"] = None
        self.assertEqual(gate().advance(s, None)[1], "bad_source")
        s = snapshot()
        s["T_world_camera"] = None
        self.assertEqual(gate().advance(s, clock(s))[1], "bad_pose")

    def test_age_and_mapping_uncertainty_bounds(self):
        s = snapshot()
        for changes, reason in (({"age_upper_ns": -1}, "clock_unavailable"),
                ({"uncertainty_ns": -1}, "clock_unavailable"),
                ({"uncertainty_ns": 11}, "clock_uncertain"),
                ({"age_upper_ns": 101}, "stale_source")):
            self.assertEqual(gate().advance(s, clock(s, **changes))[1], reason)
        self.assertEqual(gate().advance(s, clock(s, age_upper_ns=100,
            uncertainty_ns=10))[1], "pose_published")

    def test_available_covariance_remains_out_of_pose_message(self):
        s = snapshot()
        s.update(covariance_availability="AVAILABLE", covariance=[1.0]*36)
        fields, reason = gate().advance(s, clock(s))
        self.assertEqual(reason, "pose_published")
        self.assertNotIn("covariance", fields["pose"])
        for cov in (None, [1.0]*35, [float("nan")]*36):
            s["covariance"] = cov
            self.assertEqual(gate().advance(s, clock(s))[1], "bad_covariance_metadata")
        s.update(covariance_availability="UNAVAILABLE", covariance=[0.0]*36)
        self.assertEqual(gate().advance(s, clock(s))[1], "bad_covariance_metadata")

    def test_unconfigured_session_or_budgets(self):
        s = snapshot()
        for changes in ({"session": ""}, {"max_age_ns": -1}, {"max_uncertainty_ns": -1}):
            self.assertEqual(gate(**changes).advance(s, clock(s))[1], "unconfigured")

    def test_numeric_inputs_are_typed_and_bounded_before_mutation(self):
        bad = (True, False, 1.0, float('nan'), float('inf'), -1, 2**64)
        s = snapshot()
        for field in ('source_ns', 'generation', 'revision', 'ros_ns',
                      'age_upper_ns', 'uncertainty_ns'):
            for value in bad:
                with self.subTest(evidence=field, value=value):
                    g = gate()
                    self.assertIsNone(g.advance(s, clock(s, **{field: value}))[0])
                    self.assertEqual(g.advance(s, clock(s))[1], 'pose_published')
        for field in ('source_ns', 'sequence', 'reset_counter', 'epoch_id', 'map_id', 'reasons'):
            for value in bad:
                with self.subTest(snapshot=field, value=value):
                    g = gate()
                    invalid = {**s, field: value}
                    self.assertIsNone(g.advance(invalid, clock(s))[0])
                    self.assertEqual(g.advance(s, clock(s))[1], 'pose_published')
        for field in ('generation', 'max_age_ns', 'max_uncertainty_ns'):
            for value in bad:
                with self.subTest(config=field, value=value):
                    self.assertIsNone(gate(**{field: value}).advance(s, clock(s))[0])

    def test_native_integer_range_edges(self):
        s = snapshot()
        for field in ('source_ns', 'ros_ns', 'age_upper_ns', 'uncertainty_ns'):
            with self.subTest(field=field):
                self.assertIsNone(gate().advance(s, clock(s, **{field: 2**63}))[0])
        for field in ('max_age_ns', 'max_uncertainty_ns'):
            self.assertEqual(gate(**{field: 2**63}).advance(s, clock(s))[1], 'unconfigured')
        g = gate(generation=2**64-1, max_age_ns=2**63-1, max_uncertainty_ns=2**63-1)
        s.update(source_ns=2**63-1, sequence=2**64-1, reset_counter=2**64-1,
                 epoch_id=2**64-1, map_id=2**64-1)
        self.assertEqual(g.advance(s, clock(s, generation=2**64-1, revision=2**64-1,
            age_upper_ns=2**63-1, uncertainty_ns=2**63-1,
            ros_ns=mapping.ROS_MAX))[1], 'pose_published')

    def test_replay_requires_distinct_binding(self):
        s = snapshot()
        g = gate(target_domain="ros_replay", generation=7)
        fields, reason = g.advance(s, clock(s, target_domain="ros_replay", generation=7,
            ros_ns=1000000001))
        self.assertEqual(reason, "pose_published")
        self.assertEqual(fields["header"]["stamp"], {"sec": 1, "nanosec": 1})
        self.assertEqual(g.advance(s, clock(s, target_domain="ros_replay", generation=7,
            age_upper_ns=101))[1], "stale_source")

    @unittest.skipUnless(os.environ.get("WBR_CPP_PROBE"), "optional ROS-free C++ parity probe")
    def test_cpp_field_mapping_parity(self):
        transforms = [snapshot()["T_world_camera"]]
        for diagonal in ([1, 1, 1], [1, -1, -1], [-1, 1, -1], [-1, -1, 1]):
            t = copy.deepcopy(transforms[0])
            t["matrix"] = [0]*16
            t["matrix"][0], t["matrix"][5], t["matrix"][10], t["matrix"][15] = *diagonal, 1
            transforms.append(t)
        for t in transforms:
            ns = 1700000000123456789
            output = subprocess.run([os.environ["WBR_CPP_PROBE"]], input=
                " ".join(map(str, [ns]+t["matrix"])), text=True, capture_output=True, check=True)
            values = output.stdout.split()
            fields = mapping.pose_fields(t, ns)
            self.assertEqual(list(map(int, values[:2])), [1700000000, 123456789])
            expected = list(fields["pose"]["position"].values()) + list(fields["pose"]["orientation"].values())
            for actual, wanted in zip(map(float, values[2:]), expected):
                self.assertAlmostEqual(actual, wanted, places=12)


if __name__ == "__main__":
    unittest.main()
