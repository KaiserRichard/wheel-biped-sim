import csv
import json
from pathlib import Path
import sqlite3
import struct
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[3]
FIXTURES = Path(__file__).with_name("fixtures")
sys.path.insert(0, str(ROOT))

from tools.stamp_drift.analyze import Row, analyze_path, analyze_rows  # noqa: E402


class StampDriftTests(unittest.TestCase):
    def test_known_offset_drift_jitter_and_outliers(self):
        source_base = 1_700_000_000_000_000_000
        receive_base = 9_000_000_000
        rate = 1.0 + 35.0e-6
        rows = []
        for i in range(120):
            stamp = source_base + i * 10_000_000
            jitter = ((i * 7919) % 7 - 3) * 1_000
            outlier = 8_000_000 if i in (17, 83) else 0
            receive = receive_base + round(rate * (stamp - source_base)) + jitter + outlier
            rows.append(Row("/camera/imu", stamp, receive, i + 2))
        report = analyze_rows(rows)
        stream = report["streams"][0]
        self.assertEqual(stream["fit_status"], "DIAGNOSTIC")
        self.assertEqual(report["diagnostic_status"], "PASS")
        self.assertAlmostEqual(stream["drift_ppm"], 35.0, delta=1.0)
        segment = stream["segments"][0]
        self.assertAlmostEqual(segment["offset_at_reference_ns"], receive_base - source_base, delta=5000)
        self.assertEqual(segment["outlier_count"], 2)
        self.assertLess(segment["residual_abs_ns"]["p95"], 5_000)
        self.assertEqual(segment["lower_envelope_proxy_ns"], min(r.receive_ns - r.header_stamp_ns for r in rows))

    def test_forward_residual_step_is_reported_and_fits_are_split(self):
        source_base = 1_700_000_000_000_000_000
        rows = []
        for i in range(80):
            stamp = source_base + i * 10_000_000 + (20_000_000 if i >= 40 else 0)
            receive = 4_000_000_000 + i * 10_000_000
            rows.append(Row("/camera/infra1/image", stamp, receive, i + 2))
        stream = analyze_rows(rows)["streams"][0]
        self.assertEqual(stream["diagnostic_status"], "FAIL")
        self.assertGreaterEqual(len(stream["step_discontinuities"]), 1)
        self.assertGreaterEqual(stream["segment_count"], 2)
        self.assertTrue(any(event["type"] == "residual_step" for event in stream["step_discontinuities"]))
        for segment in stream["segments"]:
            self.assertAlmostEqual(segment["drift_ppm"], 0.0, delta=0.1)

    def test_header_reversal_splits_and_is_not_stitched(self):
        source_base = 1_700_000_000_000_000_000
        rows = []
        for i in range(20):
            source_index = i if i < 10 else i - 15
            rows.append(Row("/camera/imu", source_base + source_index * 10_000_000,
                            5_000_000_000 + i * 10_000_000, i + 2))
        stream = analyze_rows(rows)["streams"][0]
        self.assertEqual(stream["diagnostic_status"], "FAIL")
        self.assertEqual(len(stream["reversals"]), 1)
        self.assertEqual(stream["reversals"][0]["types"], ["header_stamp_nonincreasing"])
        self.assertEqual(stream["segment_count"], 2)

    def test_inter_topic_and_ir_imu_comparison(self):
        rows = []
        for i in range(20):
            stamp = 1_700_000_000_000_000_000 + i * 20_000_000
            rows.append(Row("/camera/infra1/image", stamp, 3_000_000_000 + i * 20_000_010, i + 2))
            rows.append(Row("/camera/gyro/imu", stamp, 4_000_000_000 + i * 19_999_980, i + 100))
        report = analyze_rows(rows)
        self.assertEqual(len(report["inter_topic_drift_comparison"]), 1)
        self.assertEqual(len(report["ir_vs_imu_drift_comparison"]), 1)
        self.assertAlmostEqual(report["ir_vs_imu_drift_comparison"][0]["drift_difference_ppm_b_minus_a"], 1.5, delta=0.2)

    def test_short_fixture_is_blocked_without_a_fit(self):
        report = analyze_rows([Row("/camera/imu", 1_000_000_000, 2_000_000_000, 1),
                              Row("/camera/imu", 1_010_000_000, 2_010_000_000, 2)])
        self.assertEqual(report["diagnostic_status"], "BLOCKED")
        self.assertEqual(report["streams"][0]["diagnostic_status"], "BLOCKED")

    def test_real_saved_evidence_fixture_is_read_and_labeled(self):
        metadata = json.loads((FIXTURES / "real_saved_evidence.json").read_text())
        self.assertEqual(metadata["evidence_label"], "MEASURED")
        report = analyze_path(FIXTURES / "real_saved_evidence.csv", header_clock="D435I_GLOBAL_TIME")
        self.assertEqual(report["input"]["row_count"], 12)
        self.assertEqual(report["clock_domains"]["header_stamp"], "D435I_GLOBAL_TIME")
        self.assertIsNone(report["clock_domains"]["global_time_enabled"])
        self.assertEqual(report["streams"][0]["sample_count"], 12)

    def test_synthetic_db3_cdr_header_reader(self):
        def cdr_header(stamp_ns):
            sec, nanosec = divmod(stamp_ns, 1_000_000_000)
            frame = b"frame\0"
            return b"\x00\x01\x00\x00" + struct.pack("<iI", sec, nanosec) + struct.pack("<I", len(frame)) + frame

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bag_0.db3"
            connection = sqlite3.connect(path)
            connection.executescript(
                "CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT, type TEXT, serialization_format TEXT);"
                "CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER, timestamp INTEGER, data BLOB);"
            )
            connection.execute("INSERT INTO topics VALUES (1, '/camera/imu', 'sensor_msgs/msg/Imu', 'cdr')")
            for i in range(5):
                stamp = 1_700_000_000_000_000_000 + i * 10_000_000
                connection.execute("INSERT INTO messages VALUES (?, 1, ?, ?)",
                                   (i + 1, 8_000_000_000 + i * 10_000_001, cdr_header(stamp)))
            connection.commit()
            connection.close()
            report = analyze_path(path)
        self.assertEqual(report["input"]["kind"], "rosbag2_sqlite3")
        self.assertEqual(report["streams"][0]["sample_count"], 5)

    def test_csv_and_cli_errors_are_nonzero(self):
        with tempfile.TemporaryDirectory() as directory:
            bad = Path(directory) / "bad.csv"
            bad.write_text("topic,header_stamp_ns\n/a,1\n")
            result = subprocess.run([sys.executable, str(ROOT / "tools/stamp_drift/analyze.py"), str(bad)],
                                    cwd=ROOT, capture_output=True, text=True)
            self.assertEqual(result.returncode, 2)
            self.assertIn("missing columns", (bad.with_suffix(".stamp-drift") / "diagnostic.txt").read_text())


if __name__ == "__main__":
    unittest.main()
