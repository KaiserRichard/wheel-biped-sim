"""Synthetic offline evidence tests; no camera, ROS, Pi, training or native runner."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "replay_metrics.py"
spec = importlib.util.spec_from_file_location("replay_metrics", SCRIPT)
metrics = importlib.util.module_from_spec(spec)
spec.loader.exec_module(metrics)


class MetricsTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix="synthetic-replay-metrics-")
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)

    def write(self, name, content):
        path = self.root / name
        path.write_text(content)
        return path

    def tum(self, positions=((0, 0, 0), (4, 0, 0), (1, 0, 0))):
        return self.write("poses.tum", "# SYNTHETIC\n" + "".join(
            f"{i} {x} {y} {z} 0 0 0 1\n" for i, (x, y, z) in enumerate(positions)))

    def frames(self, states, extra_header="", extras=None):
        extras = extras or [""] * len(states)
        return self.write("frames.csv", "source_ns,tracking_state" + extra_header + "\n" + "".join(
            f"{1700000000000000000 + i},{state}{extra}\n"
            for i, (state, extra) in enumerate(zip(states, extras))))

    def test_numeric_path_endpoint_and_percent(self):
        result = metrics.analyze("synthetic", self.tum(), coverage="complete", same_start_end="yes")
        self.assertEqual(result["errors"], [])
        self.assertEqual(result["geometry"]["path_length_m"], 7)
        self.assertEqual(result["geometry"]["endpoint_drift_m"], 1)
        self.assertAlmostEqual(result["geometry"]["endpoint_drift_percent_path"], 100 / 7)
        self.assertEqual(len(result["inputs"]["trajectory"]["sha256"]), 64)

    def test_three_dimensional_distance(self):
        poses = metrics.read_tum(self.tum(((0, 0, 0), (0, 3, 4), (0, 0, 0))))
        result = metrics.geometry(poses, 1e-6)
        self.assertEqual(result["path_length_m"], 10)
        self.assertEqual(result["endpoint_drift_m"], 0)

    def test_empty_single_static_and_short_paths_explicitly_unavailable(self):
        for positions, status in (((), "unavailable"), (((1, 2, 3),), "unavailable"),
                                  (((0, 0, 0), (0, 0, 0)), "degenerate"),
                                  (((0, 0, 0), (1e-8, 0, 0)), "degenerate")):
            with self.subTest(positions=positions):
                result = metrics.geometry(metrics.read_tum(self.tum(positions)), 1e-6)
                self.assertEqual(result["status"], status)
                self.assertIsNone(result["endpoint_drift_percent_path"])

    def test_finite_shape_quaternion_and_monotonic_validation(self):
        valid = "0 0 0 0 0 0 0 1\n"
        for text in ("0 0 0\n", "nan 0 0 0 0 0 0 1\n", "0 inf 0 0 0 0 0 1\n",
                     "0 0 0 0 nan 0 0 1\n", "0 0 0 0 0 0 0 0\n",
                     valid + valid, valid + "-1 0 0 0 0 0 0 1\n"):
            with self.subTest(text=text), self.assertRaises(ValueError):
                metrics.read_tum(self.write("invalid.tum", text))

    def test_tum_preserves_epoch_precision(self):
        path = self.write("precise.tum", "1700000000.000000001 0 0 0 0 0 0 1\n"
                          "1700000000.000000002 1 0 0 0 0 0 1\n")
        self.assertEqual(metrics.geometry(metrics.read_tum(path), 1e-6)["path_length_m"], 1)

    def test_partial_unknown_and_physical_endpoint_warnings(self):
        for coverage in ("partial", "unknown"):
            result = metrics.analyze("synthetic", self.tum(), coverage=coverage)
            self.assertTrue(any("observed pose span" in w for w in result["warnings"]))
            self.assertTrue(any("Equal physical" in w for w in result["warnings"]))
            self.assertEqual(result["geometry"]["endpoint_drift_m"], 1)
        self.assertIsNone(result["tracking"]["loss_episode_count"])
        self.assertIsNone(result["tracking"]["processedFPS"])

    def test_loss_episodes_follow_success_not_every_lost_frame(self):
        path = self.frames([1, 3, 4, 2, 3, 4, 4, 5, 3, 2, 1, 4])
        result = metrics.tracking(metrics.read_frames(path), "complete")
        self.assertEqual(result["loss_episode_count"], 2)
        self.assertEqual(result["tracked_frames"], 3)
        self.assertEqual(result["nontracking_frames"], 9)
        self.assertIsNone(result["processedFPS"])
        self.assertIsNone(result["trackedFPS"])

    def test_named_states_and_missing_published_outputs(self):
        path = self.frames(["NOT_INITIALIZED", "OK", "RECENTLY_LOST", "LOST", "OK_KLT"],
                           ",pose_published", [",false", ",true", ",false", ",false", ",false"])
        result = metrics.analyze("synthetic", self.tum(), frames=path)
        self.assertEqual(result["tracking"]["loss_episode_count"], 1)
        self.assertEqual(result["tracking"]["frames_without_pose"], 4)
        self.assertEqual(result["tracking"]["successful_frames_without_pose"], 1)
        self.assertTrue(any("Pose count differs" in w for w in result["warnings"]))
        self.assertTrue(any("captured boundary" in w for w in result["warnings"]))

    def test_steady_span_fps_uses_integer_difference_and_not_source_duration(self):
        base = 9000000000000000000
        path = self.frames([2, 3, 5], ",processing_start_steady_ns,processing_end_steady_ns",
                           [f",{base},{base + 100000000}",
                            f",{base + 200000000},{base + 300000000}",
                            f",{base + 400000000},{base + 500000000}"])
        result = metrics.tracking(metrics.read_frames(path))
        self.assertEqual(result["measured_steady_span_s"], 0.5)
        self.assertEqual(result["processedFPS"], 6)
        self.assertEqual(result["trackedFPS"], 4)
        self.assertEqual(result["source_span_s"], 2e-9)

    def test_counter_gaps_expected_tail_and_drop_counter_are_separate(self):
        path = self.frames([2, 4, 2], ",processed,application_dropped",
                           [",2,3", ",4,5", ",5,5"])
        rows = metrics.read_frames(path)
        self.assertEqual(metrics.tracking(rows)["missing_status_rows"], 1)
        self.assertEqual(metrics.tracking(rows, "complete")["missing_status_rows"], 2)
        result = metrics.tracking(rows, "partial", expected_processed=7)
        self.assertEqual(result["missing_status_rows"], 4)
        self.assertEqual(result["application_dropped_last"], 5)
        self.assertIn("incomplete", result["loss_count_scope"])
        with self.assertRaises(ValueError):
            metrics.tracking(rows, expected_processed=4)

    def test_invalid_csv_states_stamps_timing_counters_and_flags(self):
        for text in ("source_ns\n1\n", "source_ns,tracking_state\n1,99\n",
                     "source_ns,tracking_state\n1,2\n1,4\n",
                     "source_ns,tracking_state\n2,2\n1,4\n",
                     "source_ns,tracking_state\nnan,2\n",
                     "source_ns,tracking_state,pose_published\n1,4,true\n",
                     "source_ns,tracking_state,pose_published\n1,2,maybe\n",
                     "source_ns,tracking_state,processing_start_steady_ns\n1,2,10\n",
                     "source_ns,tracking_state,processing_start_steady_ns,processing_end_steady_ns\n1,2,10,10\n",
                     "source_ns,tracking_state,processing_start_steady_ns,processing_end_steady_ns\n1,2,10,20\n2,2,19,30\n",
                     "source_ns,tracking_state,processed\n1,2,1\n2,2,1\n",
                     "source_ns,tracking_state,application_dropped\n1,2,2\n2,2,1\n",
                     "source_ns,tracking_state,tracking_state\n1,2,2\n",
                     "source_ns,tracking_state\n1\n"):
            with self.subTest(text=text), self.assertRaises(ValueError):
                metrics.read_frames(self.write("invalid.csv", text))

    def test_missing_empty_or_partial_log_never_claims_no_closure(self):
        self.assertEqual(metrics.loop_events()["loop_closure_detected"], "unknown")
        log = self.write("orb.log", "PR: Loop detected with Reffine Sim3\nLoop detected!\nLoop closed.\n")
        for coverage in ("partial", "unknown"):
            self.assertEqual(metrics.loop_events(log, coverage)["loop_closure_detected"], "unknown")
        result = metrics.loop_events(log, "complete")
        self.assertEqual(result["loop_closure_detected"], "no")
        self.assertEqual(result["counts"]["detection"], 0)

    def test_detection_rejection_merge_and_gba_are_distinct(self):
        log = self.write("orb.log", "PR: Loop detected with Reffine Sim3\n"
                         "[d435_node-1] *Loop detected\nBAD LOOP!!!\n*Merge detected\nMerge finished!\n"
                         "Global Bundle Adjustment finished\nMap updated!\n")
        result = metrics.loop_events(log, "partial")
        self.assertEqual(result["loop_closure_detected"], "yes")
        self.assertEqual(result["correction_completed"], "unknown")
        self.assertEqual(result["counts"], dict(detection=1, rejected=1, merge_detection=1,
                                              merge_finished=1, gba_finished=1, map_updated=1))
        self.assertEqual(result["events"][0]["line"], 2)

    def test_merge_alone_or_quoted_positive_does_not_count_as_loop(self):
        log = self.write("orb.log", '*Merge detected\nMerge finished!\nexample: *Loop detected\n"*Loop detected"\n')
        self.assertEqual(metrics.loop_events(log, "complete")["loop_closure_detected"], "no")

    def test_missing_trajectory_and_log_are_visible_errors(self):
        result = metrics.analyze("synthetic", self.root / "missing.tum", orb_log=self.root / "missing.log")
        self.assertEqual(len(result["errors"]), 2)
        self.assertEqual(result["geometry"]["status"], "invalid")
        self.assertEqual(result["loop"]["loop_closure_detected"], "unknown")

    def cli(self, *extra):
        path = self.tum()
        return subprocess.run([sys.executable, str(SCRIPT), "--replay", "a", str(path),
                               "--replay", "b", str(path), "--replay", "c", str(path), *extra],
                              capture_output=True, text=True, timeout=10)

    def test_three_replay_cli_writes_reports_outside_git_without_rewriting_inputs(self):
        output = self.root / "report"
        result = self.cli("--output-dir", str(output), "--coverage", "b", "partial")
        self.assertEqual(result.returncode, 0, result.stderr)
        document = json.loads((output / "replay_metrics.json").read_text())
        self.assertEqual([r["label"] for r in document["replays"]], ["a", "b", "c"])
        self.assertEqual(document["replays"][1]["coverage"], "partial")
        self.assertIn("| b | partial |", (output / "replay_metrics.md").read_text())
        original = (output / "replay_metrics.json").read_bytes()
        result = self.cli("--output-dir", str(output))
        self.assertEqual(result.returncode, 2)
        self.assertEqual((output / "replay_metrics.json").read_bytes(), original)
        self.assertTrue((self.root / "poses.tum").read_text().startswith("# SYNTHETIC"))

    def test_cli_rejects_inside_git_outputs_and_unknown_labels(self):
        output = SCRIPT.parent / "should-not-be-created-by-metrics-test"
        result = self.cli("--output-dir", str(output))
        self.assertEqual(result.returncode, 2)
        self.assertIn("outside git", result.stderr)
        self.assertFalse(output.exists())
        result = self.cli("--output-dir", str(self.root / "report"), "--coverage", "missing", "complete")
        self.assertEqual(result.returncode, 2)
        self.assertIn("unknown/duplicate", result.stderr)

    def test_cli_bad_replay_preserves_other_results_and_exits_two(self):
        good = self.tum()
        bad = self.write("bad.tum", "0 nan 0 0 0 0 0 1\n")
        output = self.root / "report"
        result = subprocess.run([sys.executable, str(SCRIPT), "--replay", "good", str(good),
                                 "--replay", "bad", str(bad), "--replay", "missing", str(self.root / "missing"),
                                 "--output-dir", str(output)], capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 2, result.stderr)
        rows = json.loads((output / "replay_metrics.json").read_text())["replays"]
        self.assertEqual(rows[0]["geometry"]["path_length_m"], 7)
        self.assertTrue(rows[1]["errors"])
        self.assertTrue(rows[2]["errors"])
        self.assertTrue(all(line.startswith("|") for line in result.stdout.splitlines()[:5]))


if __name__ == "__main__":
    unittest.main()
