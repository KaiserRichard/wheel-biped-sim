import io
from pathlib import Path
import tempfile
import unittest

from tools.bench_twin_lab.checklist import (
    SCENARIOS,
    missing_artifacts,
    print_checklist,
    required_files,
    validate_output,
)


class ChecklistTests(unittest.TestCase):
    def test_prints_all_scenarios_and_simulation_names(self):
        output = io.StringIO()
        print_checklist(output)
        text = output.getvalue()
        for scenario_id, scenario in SCENARIOS.items():
            self.assertIn(scenario_id, text)
            self.assertIn(scenario.simulation, text)

    def test_fake_output_folder_dry_run_passes(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for relative in required_files():
                path = root / relative
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text("dry-run evidence\n", encoding="utf-8")

            report = io.StringIO()
            self.assertTrue(validate_output(root, report))
            self.assertEqual(missing_artifacts(root), [])
            self.assertIn("PASS:", report.getvalue())

    def test_reports_missing_or_empty_scenario_evidence(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for relative in required_files():
                path = root / relative
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text("dry-run evidence\n", encoding="utf-8")

            missing_path = root / "scenarios/link_loss/live/packets.csv"
            missing_path.unlink()
            empty_path = root / "scenarios/imu_sway/imu_view_run.csv"
            empty_path.write_text("", encoding="utf-8")

            missing = missing_artifacts(root)
            self.assertIn("scenarios/link_loss/live/packets.csv", missing)
            self.assertIn("scenarios/imu_sway/imu_view_run.csv", missing)
            report = io.StringIO()
            self.assertFalse(validate_output(root, report))
            self.assertIn("FAIL:", report.getvalue())


if __name__ == "__main__":
    unittest.main()
