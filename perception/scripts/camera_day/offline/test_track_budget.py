import csv
import tempfile
import unittest
from pathlib import Path

import track_budget


class TrackBudgetTests(unittest.TestCase):
    def test_strict_66_7_ms_boundary(self):
        self.assertFalse(track_budget.over_budget(66_700_000, track_budget.NOMINAL_BUDGET_NS))
        self.assertTrue(track_budget.over_budget(66_700_001, track_budget.NOMINAL_BUDGET_NS))

    def test_exact_15_hz_rational_boundary(self):
        # Integer nanoseconds cannot represent 1/15 s exactly. Compare 15*d > 1e9.
        self.assertFalse(track_budget.over_exact_15hz_budget(66_666_666))
        self.assertTrue(track_budget.over_exact_15hz_budget(66_666_667))

    def test_summarize_distinguishes_nominal_and_exact_15_hz_budgets(self):
        summary = track_budget.summarize(
            [{"duration_ns": 66_700_000, "pose_published": True}], "boundary"
        )
        nominal, exact_15hz = summary["thresholds"]
        self.assertEqual(nominal["overrun_count"], 0)
        self.assertEqual(exact_15hz["overrun_count"], 1)

    def test_rejects_nonserial_track_interval(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "frames.csv"
            with path.open("w", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(track_budget.REQUIRED_COLUMNS)
                writer.writerow((1, 2, "true", 1, 0, 100, 200, 0))
                writer.writerow((2, 2, "true", 2, 0, 190, 300, 0))
            with self.assertRaisesRegex(ValueError, "positive and serial"):
                track_budget.read_frames(path)

    def test_histogram_accounts_for_all_calls(self):
        records = [
            {"duration_ns": 1, "pose_published": True},
            {"duration_ns": 66_700_000, "pose_published": False},
            {"duration_ns": 250_000_000, "pose_published": True},
        ]
        counts = track_budget._histogram_counts(records)
        self.assertEqual(sum(counts), len(records))
        self.assertEqual(counts[-1], 1)


if __name__ == "__main__":
    unittest.main()
