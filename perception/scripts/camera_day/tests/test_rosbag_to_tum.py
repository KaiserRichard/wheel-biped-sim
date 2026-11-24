"""Pure helper and file-contract tests; importing the exporter never needs ROS."""
import csv
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / "rosbag_to_tum.py"
spec = importlib.util.spec_from_file_location("rosbag_to_tum", SCRIPT)
exporter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(exporter)
BASE = 1761000000123456789


def header(ns=BASE):
    sec, nanosec = divmod(ns, 10**9)
    return NS(stamp=NS(sec=sec, nanosec=nanosec), frame_id="slam_world")


def pose(ns=BASE, values=(0.10000000000000002, -0.0, 1e-20, 0.0, 0.0, 0.0, 1.0)):
    return NS(header=header(ns), pose=NS(position=NS(**dict(zip("xyz", values[:3]))),
                                       orientation=NS(**dict(zip("xyzw", values[3:])))))


def status(ns=BASE, **changes):
    row = dict(source_ns=ns, second_ns=ns, tracking_state=2, pose_published=True,
               processed=1, application_dropped=0)
    row.update(changes)
    return row


def records(poses, rows):
    # All statuses before all poses: DDS topics may be interleaved differently.
    return ([(exporter.STATUS_TOPIC, NS(data=json.dumps(r))) for r in rows] +
            [(exporter.POSE_TOPIC, p) for p in poses])


class ParseTests(unittest.TestCase):
    def test_nanosecond_epoch_and_binary64_values_round_trip(self):
        first = exporter.pose_row(pose())
        second = exporter.pose_row(pose(BASE + 1), first)
        self.assertEqual(exporter.tum_stamp(first["source_ns"]), "1761000000.123456789")
        self.assertEqual(exporter.tum_stamp(second["source_ns"]), "1761000000.123456790")
        self.assertEqual([float(repr(v)) for v in first["values"]], list(first["values"]))
        self.assertEqual(repr(first["values"][1]), "-0.0")
        rollover = 1761000000999999999
        self.assertEqual(exporter.tum_stamp(exporter.pose_row(pose(rollover))["source_ns"]),
                         "1761000000.999999999")
        self.assertEqual(exporter.tum_stamp(exporter.pose_row(pose(rollover + 1))["source_ns"]),
                         "1761000001.000000000")

    def test_pose_backstep_repeat_frame_change_and_bad_header_rejected(self):
        previous = exporter.pose_row(pose())
        for ns in (BASE, BASE - 1):
            with self.subTest(ns=ns), self.assertRaisesRegex(ValueError, "strictly increase"):
                exporter.pose_row(pose(ns), previous)
        changed = pose(BASE + 1)
        changed.header.frame_id = "other_world"
        with self.assertRaisesRegex(ValueError, "frame_id changed"):
            exporter.pose_row(changed, previous)
        for sec, nsec in ((-1, 0), (0, 10**9), (0, -1), (0, True)):
            bad = pose()
            bad.header.stamp.sec, bad.header.stamp.nanosec = sec, nsec
            with self.subTest(sec=sec, nsec=nsec), self.assertRaises(ValueError):
                exporter.pose_row(bad)
        empty = pose()
        empty.header.frame_id = ""
        with self.assertRaisesRegex(ValueError, "empty"):
            exporter.pose_row(empty)

    def test_nonfinite_pose_and_zero_quaternion_rejected_without_normalizing(self):
        for value in (float("nan"), float("inf"), -float("inf")):
            for index in range(7):
                values = [0., 0., 0., 0., 0., 0., 1.]
                values[index] = value
                with self.subTest(value=value, index=index), self.assertRaisesRegex(ValueError, "nonfinite"):
                    exporter.pose_row(pose(values=values))
        with self.assertRaisesRegex(ValueError, "zero quaternion"):
            exporter.pose_row(pose(values=[0.] * 7))
        self.assertEqual(exporter.pose_row(pose(values=[0.] * 6 + [2.]))["values"][-1], 2.)

    def test_status_exact_types_schema_and_state_validation(self):
        for state in (-1, 0, 1, 3, 4):
            self.assertEqual(exporter.status_row(json.dumps(status(
                tracking_state=state, pose_published=False)))["tracking_state"], state)
        for changes in ({"tracking_state": 6}, {"tracking_state": "OK"},
                        {"tracking_state": True}, {"tracking_state": 2.0},
                        {"tracking_state": 4}, {"pose_published": "true"},
                        {"pose_published": 1}, {"processed": 0}, {"processed": True},
                        {"processed": 1.0}, {"application_dropped": -1},
                        {"source_ns": 1.0}, {"source_ns": -1},
                        {"second_ns": None}, {"processed": 2**64}, {"extra": 0}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                exporter.status_row(json.dumps(status(**changes)))
        for key in exporter.FIELDS:
            row = status()
            del row[key]
            with self.subTest(missing=key), self.assertRaisesRegex(ValueError, "fields"):
                exporter.status_row(json.dumps(row))
        for text in ("[]", "null", "{", json.dumps(status())[:-1] + ',"source_ns":0}',
                     json.dumps(status()).replace('"processed": 1', '"processed": NaN'),
                     json.dumps(status()).replace('"processed": 1', '"processed": 1e999')):
            with self.subTest(text=text), self.assertRaises(ValueError):
                exporter.status_row(text)

    def test_source_second_counter_backstep_and_reset_rejected(self):
        previous = status(processed=10, application_dropped=3)
        valid = status(BASE + 1, processed=12, application_dropped=3)
        self.assertEqual(exporter.status_row(json.dumps(valid), previous), valid)
        for changes in ({"source_ns": BASE}, {"source_ns": BASE - 1},
                        {"second_ns": BASE}, {"processed": 10}, {"processed": 1},
                        {"application_dropped": 2}):
            row = dict(valid, **changes)
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                exporter.status_row(json.dumps(row), previous)

    def test_missing_pose_unclaimed_pose_false_flag_and_source_mismatch_fail(self):
        cases = [records([], [status()]), records([pose()], []),
                 records([pose()], [status(pose_published=False)]),
                 records([pose(BASE + 1)], [status()])]
        for case in cases:
            with self.subTest(case=case), self.assertRaisesRegex(ValueError, "contradiction"):
                exporter.collect(case)
        poses, rows = exporter.collect(records([pose()], [status()]))
        self.assertEqual(poses[0]["source_ns"], rows[0]["source_ns"])
        self.assertEqual(exporter.collect(records([], [status(pose_published=False)]))[0], [])

    def test_counter_gap_prefix_and_unknown_tail_coverage(self):
        rows = [status(processed=3), status(BASE + 2, processed=6, application_dropped=5)]
        coverage = exporter.frame_coverage(rows)
        self.assertEqual(coverage["known_missing_status_prefix"], 2)
        self.assertEqual(coverage["known_missing_status_count"], 4)
        self.assertEqual(coverage["internal_processed_gaps"][0]["first_missing_processed"], 4)
        self.assertEqual(coverage["coverage"], "partial")
        self.assertEqual(coverage["application_dropped_last"], 5)
        self.assertEqual(coverage["unobserved_tail"], "unknown")
        self.assertEqual(exporter.frame_coverage([status()])["coverage"], "unknown")
        self.assertIsNone(exporter.frame_coverage([])["known_missing_status_count"])

    def test_input_exact_source_matching_and_gap_runs_without_clock_subtraction(self):
        images = [(exporter.IMAGE_TOPIC, NS(header=header(BASE + i))) for i in range(6)]
        rows = [status(BASE + i, processed=j + 1) for j, i in enumerate((1, 2, 4))]
        result = exporter.input_coverage(images, rows, exporter.IMAGE_TOPIC)
        self.assertEqual(result["input_images"]["count"], 6)
        self.assertEqual(result["matched"]["first_source_ns"], BASE + 1)
        self.assertEqual(result["matched"]["last_source_ns"], BASE + 4)
        self.assertEqual([(g["first_input_index"], g["last_input_index"], g["count"])
                          for g in result["unmatched_input_runs"]], [(0, 0, 1), (3, 3, 1), (5, 5, 1)])
        with self.assertRaisesRegex(ValueError, "absent"):
            exporter.input_coverage(images, [status(BASE + 10)], exporter.IMAGE_TOPIC)
        for bad in ([images[0], images[0]], [images[1], images[0]]):
            with self.assertRaisesRegex(ValueError, "strictly increase"):
                exporter.input_coverage(bad, [], exporter.IMAGE_TOPIC)
        complete = exporter.input_coverage(images[:1], [status()], exporter.IMAGE_TOPIC)
        self.assertEqual(complete["coverage"], "unknown")


class ExportTests(unittest.TestCase):
    def setUp(self):
        # Evidence is outside git; these are raw placeholder files for injected pure readers.
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.bag = self.root / "source-bag"
        self.bag.mkdir()
        (self.bag / "placeholder.db3").write_bytes(b"test-only, not a ROS bag")
        self.output = self.root / "export"

    def run_export(self, poses=None, rows=None, **kwargs):
        return exporter.export(self.bag, self.output,
                               bag_reader=lambda *_: records(poses or [], rows or []), **kwargs)

    def test_export_exact_tum_csv_hashes_no_fps_and_exclusive_creation(self):
        values = pose().pose
        report = self.run_export([pose(), pose(BASE + 1)],
                                 [status(), status(BASE + 1, processed=2)])
        manifest = json.loads((self.output / "export.json").read_text())
        self.assertEqual(report, manifest)
        tum = (self.output / "CameraTrajectory.txt").read_text().splitlines()
        self.assertEqual(tum[1].split()[0], "1761000000.123456789")
        self.assertEqual(tum[2].split()[0], "1761000000.123456790")
        self.assertEqual(float(tum[1].split()[1]), values.position.x)
        self.assertEqual(tum[1].split()[2], "-0.0")
        with (self.output / "frames.csv").open(newline="") as stream:
            reader = csv.DictReader(stream)
            self.assertEqual(reader.fieldnames, list(exporter.FIELDS))
            self.assertEqual(list(reader)[1]["source_ns"], str(BASE + 1))
        self.assertIsNone(manifest["timing"]["processedFPS"])
        self.assertIsNone(manifest["timing"]["trackedFPS"])
        self.assertNotIn("processing_start_steady_ns", (self.output / "frames.csv").read_text())
        self.assertEqual(manifest["frame_coverage"]["coverage"], "unknown")
        self.assertEqual(manifest["statuses"]["published_pose_count"], 2)
        for evidence in manifest["outputs"].values():
            self.assertEqual(evidence["sha256"], hashlib.sha256(Path(evidence["path"]).read_bytes()).hexdigest())
        before = (self.output / "export.json").read_bytes()
        with self.assertRaisesRegex(ValueError, "already exists"):
            self.run_export()
        self.assertEqual((self.output / "export.json").read_bytes(), before)

    def test_absence_is_explicit_no_fake_rows_or_poses(self):
        report = self.run_export()
        self.assertEqual(report["poses"]["count"], 0)
        self.assertIsNone(report["poses"]["first_source_ns"])
        self.assertIn("absent", report["statuses"]["availability"])
        self.assertEqual(len((self.output / "CameraTrajectory.txt").read_text().splitlines()), 1)
        self.assertEqual(len((self.output / "frames.csv").read_text().splitlines()), 1)

    def test_contradiction_fails_before_output_creation(self):
        with self.assertRaisesRegex(ValueError, "contradiction"):
            self.run_export([], [status()])
        self.assertFalse(self.output.exists())

    def test_export_with_original_input_and_hashes(self):
        original = self.root / "original.db3"
        original.write_bytes(b"test-only original")
        def reader(path, topics):
            if Path(path) == original:
                return [(exporter.IMAGE_TOPIC, NS(header=header(BASE + i))) for i in range(3)]
            return records([pose(BASE + 1)], [status(BASE + 1)])
        report = exporter.export(self.bag, self.output, original, bag_reader=reader)
        self.assertEqual(report["original_input_comparison"]["input_images_without_observed_status"], 2)
        self.assertEqual(report["inputs"]["input_bag"]["files"][0]["sha256"],
                         hashlib.sha256(original.read_bytes()).hexdigest())

    def test_refuse_writes_inside_git_or_read_only_bags(self):
        checkout = self.root / "test-checkout"
        checkout.mkdir()
        (checkout / ".git").write_text("gitdir: test-only-worktree-marker\n")
        with self.assertRaisesRegex(ValueError, "outside git"):
            exporter.export(self.bag, checkout / "must-not-create")
        with self.assertRaisesRegex(ValueError, "read-only bags"):
            exporter.export(self.bag, self.bag / "must-not-create")
        with self.assertRaisesRegex(ValueError, "must differ"):
            self.run_export(pose_topic=exporter.STATUS_TOPIC)

    def test_cli_help_works_without_ros(self):
        result = subprocess.run([sys.executable, str(SCRIPT), "--help"],
                                capture_output=True, text=True, timeout=5)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("NEW_OUTPUT_DIR", result.stdout)


if __name__ == "__main__":
    unittest.main()
