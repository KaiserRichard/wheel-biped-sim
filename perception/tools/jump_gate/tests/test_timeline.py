import csv
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from tools.jump_gate.demo import build_xml, overlay
from tools.jump_gate.gate import JumpGate, Phase, Tick
from tools.jump_gate.timeline import build_timeline, export_csv, synthetic_pose
from tools.obstacle_decision.decide import Advisory
from tools.scene_twin.replay import Packet, Replay


class TimelineTests(unittest.TestCase):
    def fixture(self):
        # Exact integer nanoseconds with both JUMP and STOP; no external archive.
        origin = 1792900867686080566
        packets = [Packet(k, 1 if k < 15 else 2, .13, 1.0, origin + k * 100_000_000)
                   for k in range(40)]
        return build_timeline(Replay(packets, None, motion="static"), jump_at=1.0)

    def test_mapping_reuses_pose_timeline_and_inhibits_all_injected_phases(self):
        timeline = self.fixture()
        self.assertEqual(synthetic_pose(.5, 1).phase, "STAND")
        expected = {
            "CROUCH": (0, False, False), "PUSH": (1, False, False),
            "FLIGHT": (2, True, True), "LAND": (2, False, True),
            "SETTLE": (3, False, False), "STAND": (3, False, False),
        }
        self.assertEqual({r.pose.phase for r in timeline.rows}, set(expected))
        for row in timeline.rows:
            self.assertEqual((row.inputs.jump_status, row.inputs.airborne_l, row.inputs.airborne_r),
                             expected[row.pose.phase])
            if row.output.phase != Phase.STAND:
                self.assertEqual(row.output.advisory, Advisory.UNCERTAIN)
                self.assertEqual(row.output.confidence, 0)
        self.assertEqual(timeline.at(.9).output.advisory, Advisory.JUMP_CANDIDATE)
        self.assertEqual(timeline.at(3).output.advisory, Advisory.STOP_CANDIDATE)

    def test_csv_roundtrip_reproduces_every_gate_tick(self):
        timeline = self.fixture()
        with tempfile.TemporaryDirectory() as tmp:
            path = export_csv(timeline, Path(tmp) / "timeline.csv")
            with path.open(newline="") as stream:
                rows = list(csv.DictReader(stream))
        self.assertEqual(len(rows), len(timeline.rows))
        gate = JumpGate()
        for saved, original in zip(rows, timeline.rows):
            self.assertEqual(int(saved["frame_stamp_ns"]), original.frame_stamp_ns)
            self.assertEqual(saved["phase_source"], "SYNTHETIC")
            tick = Tick(float(saved["t_s"]), int(saved["jump_status_synthetic"]),
                        bool(int(saved["airborne_l_synthetic"])), bool(int(saved["airborne_r_synthetic"])),
                        float(saved["telemetry_age_s_synthetic"]), float(saved["pitch_rate_deg_s_synthetic"]),
                        float(saved["roll_rate_deg_s_synthetic"]), bool(int(saved["plane_valid_assumed"])),
                        Advisory[saved["raw_advisory"]], float(saved["raw_confidence_assumed"]),
                        float(saved["plane_stamp_s"]))
            out = gate.update(tick)
            self.assertEqual(out.advisory.name, saved["gated_advisory"])
            self.assertEqual(out.phase.value, saved["gate_phase"])
            self.assertEqual(out.confidence, float(saved["gated_confidence"]))
            self.assertEqual(out.reason, saved["reason"])

    def test_saved_lab_day_csv_reproduces_gate_and_contains_no_promotions(self):
        path = Path(__file__).resolve().parents[3] / "docs/jump_gate/timeline.csv"
        if not path.is_file():
            self.skipTest("recorded timeline is available at the archived Phase 1 revision")
        gate, phases, raw_names, count = JumpGate(), set(), set(), 0
        with path.open(newline="") as stream:
            for row in csv.DictReader(stream):
                count += 1
                phases.add(row["gate_phase"])
                raw_names.add(row["raw_advisory"])
                self.assertEqual(row["phase_source"], "SYNTHETIC")
                result = gate.update(Tick(
                    float(row["t_s"]), int(row["jump_status_synthetic"]),
                    bool(int(row["airborne_l_synthetic"])), bool(int(row["airborne_r_synthetic"])),
                    float(row["telemetry_age_s_synthetic"]), float(row["pitch_rate_deg_s_synthetic"]),
                    float(row["roll_rate_deg_s_synthetic"]), bool(int(row["plane_valid_assumed"])),
                    Advisory[row["raw_advisory"]], float(row["raw_confidence_assumed"]),
                    float(row["plane_stamp_s"])))
                self.assertEqual((result.advisory.name, result.phase.value, result.confidence, result.reason),
                                 (row["gated_advisory"], row["gate_phase"],
                                  float(row["gated_confidence"]), row["reason"]))
                if result.phase != Phase.STAND:
                    self.assertEqual(result.advisory, Advisory.UNCERTAIN)
                    self.assertEqual(result.confidence, 0)
        self.assertGreater(count, 0)
        self.assertEqual(phases, {"STAND", "CROUCH", "FLIGHT", "LAND", "SETTLE"})
        self.assertEqual(raw_names, {"NONE", "STOP_CANDIDATE", "UNCERTAIN"})

    def test_held_packet_retains_stamp_and_does_not_count_as_fresh(self):
        timeline = self.fixture()
        first = timeline.at(.01)
        self.assertEqual(first.frame_stamp_ns, timeline.rows[0].frame_stamp_ns)
        self.assertEqual(first.inputs.plane_stamp_s, 0)
        self.assertEqual(timeline.at(.19).output.phase, Phase.SETTLE)
        self.assertEqual(timeline.at(.2).output.phase, Phase.STAND)

    def test_model_contains_pose_fivebar_and_gated_obstacle_without_mujoco(self):
        timeline = self.fixture()
        root = ET.fromstring(build_xml(timeline.replay))
        self.assertIsNotNone(root.find("worldbody/body[@name='left_upper_a']"))
        self.assertIsNotNone(root.find("worldbody/body[@name='right_lower_b']"))
        box = root.find("worldbody/geom[@name='obstacle']")
        self.assertIsNotNone(box)
        self.assertAlmostEqual(float(box.get("size").split()[2]) * 2, .13)
        text = overlay(timeline.at(1.6))
        for label in ("SYNTHETIC", "raw=", "gated=", "phase=", "reason=", "STM32"):
            self.assertIn(label, text)

    def test_invalid_jump_time_is_rejected(self):
        replay = self.fixture().replay
        for jump_at in (-1, float("nan"), replay.duration, replay.duration + 1):
            with self.assertRaises(ValueError):
                build_timeline(replay, jump_at)


if __name__ == "__main__":
    unittest.main()
