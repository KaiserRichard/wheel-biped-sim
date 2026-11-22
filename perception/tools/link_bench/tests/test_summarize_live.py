import csv
import json
from pathlib import Path
import tempfile
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from tools.link_bench.pi.summarize_live import summarize_run


class SummarizeLiveTest(unittest.TestCase):
    def test_synthetic_packets_and_plot(self):
        with tempfile.TemporaryDirectory() as temp:
            out = Path(temp) / "synthetic-run"
            out.mkdir()
            (out / "summary.json").write_text(json.dumps({
                "mode": "ros", "frames": 4, "dropped_frames": 2,
                "rejected_echoes": 1, "stale_events": 1,
            }))
            rows = [
                {"seq": 10, "advisory": 0, "height_m": .0, "distance_m": 0,
                 "frame_receipt_pi_ns": 1_000_000_000, "detector_ns": 10_000_000,
                 "before_write_ns": 1_001_000_000, "rtt_ns": 2_000_000,
                 "echo_result": 0, "stale": 0},
                {"seq": 11, "advisory": 1, "height_m": .1, "distance_m": 2,
                 "frame_receipt_pi_ns": 2_000_000_000, "detector_ns": 20_000_000,
                 "before_write_ns": 2_003_000_000, "rtt_ns": 4_000_000,
                 "echo_result": 0, "stale": 1},
                {"seq": 12, "advisory": 2, "height_m": .2, "distance_m": 1,
                 "frame_receipt_pi_ns": 3_000_000_000, "detector_ns": 30_000_000,
                 "before_write_ns": 3_005_000_000, "rtt_ns": 6_000_000,
                 "echo_result": 2, "stale": 1},
                {"seq": 13, "advisory": "UNCERTAIN", "height_m": .3, "distance_m": 3,
                 "frame_receipt_pi_ns": 4_000_000_000, "detector_ns": 40_000_000,
                 "before_write_ns": 4_007_000_000, "rtt_ns": 8_000_000,
                 "echo_result": 0, "stale": 0},
            ]
            with (out / "packets.csv").open("w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=rows[0].keys())
                writer.writeheader()
                writer.writerows(rows)

            result = summarize_run(out)

            self.assertEqual(result["frames"], 4)
            self.assertEqual(result["dropped_frames"], 2)
            self.assertEqual(result["advisory_histogram"],
                             {"JUMP": 1, "NONE": 1, "STOP": 1, "UNCERTAIN": 1})
            self.assertEqual(result["first_confirmed_seq"], 11)
            self.assertEqual(result["first_confirmed_distance_m"], 2)
            self.assertAlmostEqual(result["confirmed_median_height_m"], .15)
            self.assertAlmostEqual(result["confirmed_median_distance_m"], 1.5)
            self.assertAlmostEqual(result["detector_p50_ms"], 25)
            self.assertAlmostEqual(result["detector_p95_ms"], 38.5)
            self.assertAlmostEqual(result["receipt_to_send_p50_ms"], 4)
            self.assertAlmostEqual(result["receipt_to_send_p95_ms"], 6.7)
            self.assertAlmostEqual(result["rtt_p50_ms"], 5)
            self.assertAlmostEqual(result["rtt_p99_ms"], 7.94)
            self.assertEqual(result["echo_rejects"], 1)
            self.assertEqual(result["stale_events"], 1)
            self.assertTrue(Path(result["plot"]).is_file())
            self.assertGreater(Path(result["plot"]).stat().st_size, 0)


if __name__ == "__main__":
    unittest.main()
