import importlib.util
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

HERE = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("accuracy_pipeline", HERE / "evaluate.py")
pipeline = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = pipeline
spec.loader.exec_module(pipeline)

EVIDENCE = Path("./Desktop/Research/wbr-perception-phase1-runtime/experiments/2026-10-13-n4a-euroc-mh01")

class StatusTests(unittest.TestCase):
    def test_pass(self):
        self.assertEqual(pipeline.overall_status([{"status":"PASS"}], [{"status":"PASS"}]), "PASS")

    def test_fail(self):
        self.assertEqual(pipeline.overall_status([{"status":"FAIL"}], [{"status":"PASS"}]), "FAIL")

    def test_blocked(self):
        self.assertEqual(pipeline.overall_status([{"status":"BLOCKED"}], [{"status":"PASS"}]), "BLOCKED")
        self.assertEqual(pipeline.overall_status([], [{"status":"BLOCKED"}]), "BLOCKED")

    def test_aggregate_population_stats(self):
        self.assertEqual(pipeline.aggregate([1.0, 3.0]),
                         {"n":2, "mean":2.0, "std_population":1.0, "min":1.0, "max":3.0})

    def test_detects_euroc_nanoseconds_and_seconds(self):
        config = __import__("json").loads((HERE / "thresholds.json").read_text())
        with tempfile.TemporaryDirectory() as tmp:
            ns = Path(tmp) / "f.txt"; ns.write_text("1403636579963555584 0 0 0 0 0 0 1\n")
            sec = Path(tmp) / "CameraTrajectory.txt"; sec.write_text("1403636579.963556 0 0 0 0 0 0 1\n")
            self.assertEqual(pipeline.detect_timestamp_format(ns, config), ("orbslam3", "ns"))
            self.assertEqual(pipeline.detect_timestamp_format(sec, config), ("tum", "s"))

    def test_exit_code_status_blocks_missing_and_fails_nonzero(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            run = root / "run01"
            run.mkdir()
            self.assertEqual(pipeline.run_exit_code(run), (None, None))
            self.assertEqual(pipeline.overall_status([], [{"status":"BLOCKED"}]), "BLOCKED")
            (run / "exit_code.txt").write_text("7\n")
            self.assertEqual(pipeline.run_exit_code(run), (7, str(run / "exit_code.txt")))
            self.assertEqual(pipeline.overall_status([], [{"status":"FAIL"}]), "FAIL")

    def test_exit_code_can_be_read_from_legacy_evidence_log(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            run = root / "run2"
            run.mkdir()
            (root / "runs.log").write_text("run1 end 2026-10-13T07:56:10Z rc=0\nrun2 end 2026-10-13T08:00:52Z rc=4 throttled=0x50000\n")
            self.assertEqual(pipeline.run_exit_code(run), (4, str(root / "runs.log")))

    def test_pose_file_rejects_multiple_supported_outputs(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = Path(tmp)
            (run / "f_run.txt").write_text("")
            (run / "CameraTrajectory.txt").write_text("")
            self.assertIsNone(pipeline.pose_file(run))

    def test_pose_fraction_rejects_more_rows_than_expected_frames(self):
        self.assertEqual(pipeline.pose_fraction_status(
            [{"diagnostics":{"pose_fraction":1.01}}], 0.95, 1.0), "FAIL")
        self.assertEqual(pipeline.pose_fraction_status(
            [{"diagnostics":{"pose_fraction":None}}], 0.95, 1.0), "BLOCKED")
        self.assertEqual(pipeline.pose_fraction_status(
            [{"diagnostics":{"pose_fraction":0.99}}], 0.95, 1.0), "PASS")

class RealEvidenceTests(unittest.TestCase):
    @unittest.skipUnless(EVIDENCE.is_dir(), "read-only N4A evidence is not available")
    def test_saved_mh01_runs_reproduce_report(self):
        out = Path(tempfile.mkdtemp(prefix="accuracy-mh01-")) / "validation"
        command = [sys.executable, str(HERE / "evaluate.py"), "--sequence", "MH01",
                   "--runs", *[str(EVIDENCE / f"run{i}") for i in (1,2,3)],
                   "--ground-truth", str(EVIDENCE / "eval/gt_data.csv"), "--expected-frames", "3682",
                   "--out", str(out)]
        proc = subprocess.run(command, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        self.assertEqual(proc.returncode, 0, proc.stdout)
        result = __import__("json").loads((out / "summary.json").read_text())
        values = [r["ate_rpe_1m"]["se3"]["ate_translation_m"]["rmse"] for r in result["runs"]]
        for got, expected in zip(values, (0.041461, 0.036844, 0.041604)):
            self.assertAlmostEqual(got, expected, places=5)
        self.assertEqual(result["status"], "PASS")

if __name__ == "__main__":
    unittest.main()
