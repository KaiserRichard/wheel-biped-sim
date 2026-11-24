import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "lab" / "run_lab_session.py"
STUB = '''#!/usr/bin/env python3
import json, os, sys
from pathlib import Path
args = sys.argv[1:]
name = Path(sys.argv[0]).name
outarg = {"check_power_usb.py": "--out", "check.py": "--outdir",
          "record_check.py": "--output-dir", "replay_check.py": "--output-dir",
          "check_calib_take.py": "--outdir"}[name]
out = Path(args[args.index(outarg) + 1])
out.mkdir(parents=True, exist_ok=True)
key = {"check_power_usb.py": "power_usb", "check.py": "d435i_raw",
       "record_check.py": "rosbag_record", "replay_check.py": "rosbag_replay",
       "check_calib_take.py": "calib_record"}[name]
status = json.loads(os.environ.get("STUB_STATUSES", "{}")).get(key, "PASS")
evidence = "missing.txt" if os.environ.get("STUB_MISSING_EVIDENCE") == key else "stub.txt"
criteria = [{"id": key + ":stub", "description": "stub criterion", "measured": 1,
             "unit": "count", "threshold": ">= 1", "status": status,
             "evidence": evidence, "threshold_source": "test fixture"}]
(out / "stub.txt").write_text("stub evidence\\n")
(out / "result.json").write_text(json.dumps({"check": key, "tool_version": "stub",
    "status": status, "criteria": criteria, "raw_files": [], "notes": []}))
print(key)
sys.exit({"PASS": 0, "FAIL": 1, "BLOCKED": 2}[status])
'''


class LabSessionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.base = Path(self.temp.name)
        self.checks = self.base / "checks"
        scripts = {
            "power_usb/check_power_usb.py": "check_power_usb.py",
            "d435i_raw/check.py": "check.py",
            "rosbag_qual/record_check.py": "record_check.py",
            "rosbag_qual/replay_check.py": "replay_check.py",
            "calib_record/check_calib_take.py": "check_calib_take.py",
        }
        for relative, filename in scripts.items():
            path = self.checks / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(STUB)
            path.chmod(0o755)
        self.take = self.base / "take"
        self.take.mkdir()

    def tearDown(self):
        self.temp.cleanup()

    def run_session(self, name, statuses=None, extra=(), calib_take=None, missing_evidence=None):
        out = self.base / name
        env = dict(os.environ)
        env["HOME"] = str(self.base)
        env["STUB_STATUSES"] = json.dumps(statuses or {})
        if missing_evidence:
            env["STUB_MISSING_EVIDENCE"] = missing_evidence
        proc = subprocess.run([sys.executable, str(SCRIPT), "--session-dir", str(out),
                               "--calib-take", str(calib_take or self.take), "--checks-dir", str(self.checks),
                               "--yes", *extra], cwd=ROOT, env=env, text=True,
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
        return proc, out

    def test_pass_chain(self):
        proc, out = self.run_session("pass")
        self.assertEqual(proc.returncode, 0, proc.stderr + proc.stdout)
        data = json.loads((out / "SESSION.json").read_text())
        self.assertEqual(data["status"], "PASS")
        self.assertEqual([step["id"] for step in data["steps"]],
                         ["power_usb", "d435i_raw", "rosbag_record", "rosbag_replay", "calib_record"])
        self.assertEqual(len(data["criteria"]), 5)
        self.assertTrue((out / "SESSION.md").is_file())
        self.assertEqual((out / "01_power_usb/orchestrator.stdout").read_text().strip(), "power_usb")

    def test_tilde_calibration_path_is_expanded_for_child_command(self):
        proc, out = self.run_session("tilde-take", calib_take="~/take")
        self.assertEqual(proc.returncode, 0, proc.stderr + proc.stdout)
        data = json.loads((out / "SESSION.json").read_text())
        command = data["steps"][4]["command"]
        take_arg = command[command.index("--take-dir") + 1]
        self.assertEqual(take_arg, str(self.take.resolve()))

    def test_missing_criterion_evidence_is_error_and_session_is_written(self):
        proc, out = self.run_session("missing-evidence", missing_evidence="power_usb")
        self.assertEqual(proc.returncode, 3, proc.stderr + proc.stdout)
        data = json.loads((out / "SESSION.json").read_text())
        self.assertEqual(data["status"], "ERROR")
        self.assertEqual(data["steps"][0]["status"], "ERROR")
        self.assertIn("evidence file is missing", " ".join(data["steps"][0]["notes"]))

    def test_early_fail_stops_dependents(self):
        proc, out = self.run_session("early-fail", {"power_usb": "FAIL"})
        self.assertEqual(proc.returncode, 1)
        data = json.loads((out / "SESSION.json").read_text())
        self.assertEqual(data["status"], "FAIL")
        self.assertEqual(data["steps"][0]["status"], "FAIL")
        self.assertTrue(all(step.get("skipped") for step in data["steps"][1:]))
        self.assertFalse((out / "02_d435i_raw").exists())

    def test_blocked_raw_propagates_to_ros_but_checks_saved_calibration(self):
        proc, out = self.run_session("blocked", {"d435i_raw": "BLOCKED"})
        self.assertEqual(proc.returncode, 2)
        data = json.loads((out / "SESSION.json").read_text())
        self.assertEqual(data["steps"][2]["status"], "BLOCKED")
        self.assertTrue(data["steps"][2]["skipped"])
        self.assertTrue(data["steps"][3]["skipped"])
        self.assertEqual(data["steps"][4]["status"], "PASS")

    def test_continue_on_fail_runs_whole_chain(self):
        proc, out = self.run_session("continue", {"power_usb": "FAIL"}, ("--continue-on-fail",))
        self.assertEqual(proc.returncode, 1)
        data = json.loads((out / "SESSION.json").read_text())
        self.assertEqual([step["status"] for step in data["steps"]],
                         ["FAIL", "PASS", "PASS", "PASS", "PASS"])

    def test_plan_prints_commands_without_running_or_creating_session(self):
        out = self.base / "planned"
        proc = subprocess.run([sys.executable, str(SCRIPT), "--session-dir", str(out),
                               "--calib-take", str(self.take), "--checks-dir", str(self.checks), "--plan"],
                              cwd=ROOT, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertIn("power_usb:", proc.stdout)
        self.assertIn("rosbag_replay:", proc.stdout)
        self.assertIn("CONFIRM:", proc.stdout)
        self.assertFalse(out.exists())


if __name__ == "__main__":
    unittest.main()
