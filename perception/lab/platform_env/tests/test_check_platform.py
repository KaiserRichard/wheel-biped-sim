import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch


HERE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("check_platform", HERE / "check_platform.py")
check_platform = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(check_platform)
FIXTURE = json.loads((HERE / "tests/fixtures/snapshot-2026-10-13.json").read_text())
MANIFEST = json.loads((HERE / "manifest.yaml").read_text())


class PlatformAnalysisTests(unittest.TestCase):
    def test_real_snapshot_fixture_passes_pinned_fields(self):
        with tempfile.TemporaryDirectory() as temp:
            result = check_platform.evaluate(FIXTURE, MANIFEST, Path(temp))
        self.assertEqual(result["status"], "PASS")
        self.assertTrue(any(c["status"] == "INFO" for c in result["criteria"]))

    def test_target_board_model_and_ram_are_recorded_without_failing_standin_pins(self):
        changed = json.loads(json.dumps(FIXTURE))
        changed["facts"]["pi_model"] = "Raspberry Pi 5 Model B Rev 1.1"
        changed["facts"]["board_ram_bytes"] = 8589934592
        with tempfile.TemporaryDirectory() as temp:
            result = check_platform.evaluate(changed, MANIFEST, Path(temp))
        statuses = {c["id"]: c["status"] for c in result["criteria"]}
        self.assertEqual(statuses["pi_model"], "INFO")
        self.assertEqual(statuses["board_ram_bytes"], "INFO")
        self.assertEqual(result["status"], "PASS")

    def test_required_mismatch_fails_field(self):
        changed = json.loads(json.dumps(FIXTURE))
        changed["facts"]["kernel_release"] = "6.8.0-unexpected"
        with tempfile.TemporaryDirectory() as temp:
            result = check_platform.evaluate(changed, MANIFEST, Path(temp))
        criterion = next(c for c in result["criteria"] if c["id"] == "kernel_release")
        self.assertEqual(criterion["status"], "FAIL")
        self.assertEqual(result["status"], "FAIL")

    def test_missing_required_fact_is_blocked(self):
        changed = json.loads(json.dumps(FIXTURE))
        changed["facts"].pop("ros_distro")
        with tempfile.TemporaryDirectory() as temp:
            result = check_platform.evaluate(changed, MANIFEST, Path(temp))
        criterion = next(c for c in result["criteria"] if c["id"] == "ros_distro")
        self.assertEqual(criterion["status"], "BLOCKED")
        self.assertEqual(result["status"], "BLOCKED")

    def test_minimum_version_accepts_newer_and_present_policy_rejects_wrong_serial(self):
        changed = json.loads(json.dumps(FIXTURE))
        changed["facts"]["python_version"] = "3.13.0"
        changed["facts"]["d435i_udev_rule_present"] = "99004"
        with tempfile.TemporaryDirectory() as temp:
            result = check_platform.evaluate(changed, MANIFEST, Path(temp))
        statuses = {c["id"]: c["status"] for c in result["criteria"]}
        self.assertEqual(statuses["python_version"], "PASS")
        self.assertEqual(statuses["d435i_udev_rule_present"], "FAIL")
        self.assertEqual(result["status"], "FAIL")

    def test_missing_command_is_saved_and_blocks(self):
        with tempfile.TemporaryDirectory() as temp:
            raw = Path(temp)
            with patch.object(check_platform.subprocess, "run", side_effect=FileNotFoundError("missing tool")):
                entry = check_platform.run_command("absent", ["tool-that-is-not-installed"], raw)
            self.assertIsNone(entry["returncode"])
            self.assertIn("missing tool", entry["error"])
            self.assertEqual((raw / "absent.stdout.txt").read_text(), "")
        changed = json.loads(json.dumps(FIXTURE))
        changed["facts"]["ros_distro"] = None
        with tempfile.TemporaryDirectory() as temp:
            result = check_platform.evaluate(changed, MANIFEST, Path(temp))
        self.assertEqual(result["status"], "BLOCKED")

    def test_analyze_only_reuses_saved_fixture_without_collection(self):
        with tempfile.TemporaryDirectory() as temp:
            outdir = Path(temp)
            (outdir / "collection.json").write_text(json.dumps(FIXTURE))
            with patch.object(check_platform, "collect", side_effect=AssertionError("must not collect")):
                code = check_platform.main(["--analyze-only", str(outdir), "--manifest", str(HERE / "manifest.yaml")])
            self.assertEqual(code, 0)
            self.assertTrue((outdir / "result.json").is_file())
            self.assertTrue((outdir / "snapshot.json").is_file())


if __name__ == "__main__":
    unittest.main()
