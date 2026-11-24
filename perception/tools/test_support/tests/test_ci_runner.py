"""Exercise the real shell runner in temporary repositories, without hardware."""

import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest


REPO = Path(__file__).resolve().parents[3]
RECEIPT = Path(__file__).parent / "fixtures/pr8-summary.txt"
PASS_TEST = "import os, unittest\nclass TestFixture(unittest.TestCase):\n    def test_ok(self):\n        self.assertEqual(os.environ.get('PYTHONDONTWRITEBYTECODE'), '1')\n"


class RunnerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="ci review with spaces ")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.rows = [line.split()[0] for line in RECEIPT.read_text().splitlines()]
        self.python_dirs = [row.removeprefix("python:") for row in self.rows
                            if row.startswith("python:")]
        self.make_dirs = [row.split(":")[1] for row in self.rows if row.startswith("make:")]
        self.write("tools/run_all_tests.sh", (REPO / "tools/run_all_tests.sh").read_text())
        self.write("tools/test_support/run_unittest.py", (REPO / "tools/test_support/run_unittest.py").read_text())
        for directory in self.python_dirs + ["tools/test_support/tests"]:
            self.write(directory + "/test_sample.py", PASS_TEST)
        for directory in self.make_dirs:
            self.write(directory + "/Makefile", ".PHONY: test\ntest\x3a\n\t@echo HOST_TARGET_EXECUTED\n")

    def write(self, relative, content):
        self.assertFalse(Path(relative).is_absolute())
        self.assertNotIn("..", Path(relative).parts)
        path = self.root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)
        return path

    def run_runner(self):
        env = os.environ.copy()
        env["WBR_WITH_ORBSLAM3"] = "1"  # The CI entrypoint must select the fake engine.
        result = subprocess.run(["bash", str(self.root / "tools/run_all_tests.sh")],
                                cwd=self.root / "tools/test_support", env=env, text=True,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=60)
        rows = {}
        for line in result.stdout.splitlines():
            match = re.match(r"^((?:make:|python:).*?)\s+(PASS|FAIL|BLOCKED|SKIPPED)\s+", line)
            if match:
                rows[match[1]] = match[2]
        return result, rows

    def test_saved_real_receipt_inventory_and_future_locations(self):
        # The frozen inventory comes from the engineer's actual Mac test receipt.
        future = ["lab/platform/tests", "tools/eval_pipeline/ate/tests",
                  "tools/eval_pipeline/tests", "tools/perception_health/tests",
                  "tools/n4b_offline/tests", "lab/path with spaces/tests"]
        for directory in future:
            self.write(directory + "/test_sample.py", PASS_TEST)
        result, rows = self.run_runner()
        self.assertEqual(result.returncode, 0, result.stdout)
        expected = set(self.rows) | {"python:tools/test_support/tests"} | {"python:" + d for d in future}
        self.assertEqual(set(rows), expected)
        self.assertEqual(set(rows.values()), {"PASS"})
        self.assertFalse(list(self.root.rglob("*.pyc")))

    def test_missing_baseline_python_suite_is_blocked(self):
        shutil.rmtree(self.root / "tools/clockmap/tests")
        result, rows = self.run_runner()
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertEqual(rows["python:tools/clockmap/tests"], "BLOCKED")
        self.assertEqual(rows["python:calibration/lab/tests"], "PASS")

    def test_missing_makefile_is_blocked(self):
        (self.root / "tools/link_proto/Makefile").unlink()
        result, rows = self.run_runner()
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertEqual(rows["make:tools/link_proto:test"], "BLOCKED")

    def test_all_python_directories_missing_still_prints_summary(self):
        for directory in self.python_dirs + ["tools/test_support/tests"]:
            shutil.rmtree(self.root / directory)
        result, rows = self.run_runner()
        self.assertEqual(result.returncode, 1, result.stdout)
        self.assertIn("SUITE", result.stdout)
        self.assertEqual(rows["make:tools/link_proto:test"], "PASS")
        for directory in self.python_dirs + ["tools/test_support/tests"]:
            self.assertEqual(rows["python:" + directory], "BLOCKED")

    def test_empty_baseline_suite_is_blocked(self):
        (self.root / "tools/clockmap/tests/test_sample.py").unlink()
        result, rows = self.run_runner()
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertEqual(rows["python:tools/clockmap/tests"], "BLOCKED")

    def test_new_empty_suite_is_blocked(self):
        (self.root / "tools/n4b_offline/tests").mkdir(parents=True)
        result, rows = self.run_runner()
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertEqual(rows["python:tools/n4b_offline/tests"], "BLOCKED")

    def test_partially_undiscovered_nested_suite_is_blocked(self):
        self.write("tools/clockmap/tests/unpackaged/test_hidden.py", PASS_TEST)
        result, rows = self.run_runner()
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertEqual(rows["python:tools/clockmap/tests"], "BLOCKED")
        self.assertIn("no tests discovered from unpackaged/test_hidden.py", result.stdout)

    def test_nested_package_is_discovered(self):
        self.write("tools/clockmap/tests/packaged/__init__.py", "")
        self.write("tools/clockmap/tests/packaged/test_nested.py", PASS_TEST)
        result, rows = self.run_runner()
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertEqual(rows["python:tools/clockmap/tests"], "PASS")
        self.assertIn("DISCOVERED: 2 test files, 2 tests", result.stdout)

    def test_failure_does_not_hide_remaining_suites(self):
        self.write("tools/clockmap/tests/test_sample.py", PASS_TEST.replace("'1')", "'wrong')"))
        result, rows = self.run_runner()
        self.assertEqual(result.returncode, 1, result.stdout)
        self.assertEqual(rows["python:tools/clockmap/tests"], "FAIL")
        self.assertEqual(rows["python:calibration/lab/tests"], "PASS")

    def test_import_error_is_failure(self):
        self.write("tools/clockmap/tests/test_sample.py", "import nonexistent_ci_fixture_module\n")
        result, rows = self.run_runner()
        self.assertEqual(result.returncode, 1, result.stdout)
        self.assertEqual(rows["python:tools/clockmap/tests"], "FAIL")
        self.assertIn("ModuleNotFoundError", result.stdout)

    def test_comments_cannot_skip_reviewed_host_targets(self):
        for directory in self.make_dirs:
            self.write(directory + "/Makefile", "# ROS ORB_SLAM3 arm-none-eabi hardware\n"
                       ".PHONY: test\ntest\x3a\n\t@echo HOST_TARGET_EXECUTED\n")
        result, rows = self.run_runner()
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertEqual(result.stdout.count("HOST_TARGET_EXECUTED"), len(self.make_dirs))
        for directory in self.make_dirs:
            self.assertEqual(rows[f"make:{directory}:test"], "PASS")

    def test_make_failure_is_reported(self):
        self.write("tools/link_proto/Makefile", ".PHONY: test\ntest\x3a\n\t@exit 1\n")
        result, rows = self.run_runner()
        self.assertEqual(result.returncode, 1, result.stdout)
        self.assertEqual(rows["make:tools/link_proto:test"], "FAIL")
        self.assertEqual(rows["python:calibration/lab/tests"], "PASS")

    def test_optional_orb_environment_cannot_enable_real_engine(self):
        self.write("tools/vio_adapter/Makefile", "ifeq ($(WBR_WITH_ORBSLAM3),1)\n"
                   "$(error CI must use fake engine)\nendif\n.PHONY: test\ntest\x3a\n\t@true\n")
        result, rows = self.run_runner()
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertEqual(rows["make:tools/vio_adapter:test"], "PASS")

    def test_all_skipped_is_blocked_and_reason_is_visible(self):
        self.write("tools/clockmap/tests/test_sample.py", "import unittest\n"
                   "@unittest.skip('fixture dependency unavailable')\n"
                   "class TestFixture(unittest.TestCase):\n    def test_optional(self): pass\n")
        result, rows = self.run_runner()
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertEqual(rows["python:tools/clockmap/tests"], "BLOCKED")
        self.assertIn("skipped 'fixture dependency unavailable'", result.stdout)

    def test_individual_skip_with_executed_tests_is_visible(self):
        self.write("tools/clockmap/tests/test_optional.py", "import unittest\n"
                   "class TestOptional(unittest.TestCase):\n"
                   "    @unittest.skip('fixture dependency unavailable')\n"
                   "    def test_optional(self): pass\n")
        result, rows = self.run_runner()
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertEqual(rows["python:tools/clockmap/tests"], "PASS")
        self.assertIn("skipped 'fixture dependency unavailable'", result.stdout)


if __name__ == "__main__":
    unittest.main()
