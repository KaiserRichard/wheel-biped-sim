"""Recorder-shaped CSV fixtures, generated entirely from known synthetic clocks."""

import contextlib
import csv
from decimal import Decimal
import hashlib
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from clockmap.analyze import analyze_timestamps, main


HEADER = ["stream", "index", "frame_number", "sdk_ms", "domain",
          "host_steady_ns", "host_utc_ns", "x", "y", "z", "exposure_us"]
UTC_BASE = 1_750_000_000_000_000_003
STEADY_BASE = 5_000_000_007


def row(i, *, stream="Gyro", index=0, delay_ns=2_000_000,
        domain="Global Time", rate=Decimal("1.00005")):
    sdk_ns = UTC_BASE + i * 1_000_000_000
    return [stream, index, i + 1, str(Decimal(sdk_ns) / 1_000_000), domain,
            STEADY_BASE + int(rate * (sdk_ns - UTC_BASE)) + delay_ns,
            sdk_ns + delay_ns, "", "", "", ""]


class AnalyzerTests(unittest.TestCase):
    def setUp(self):
        # Tests create fixtures only under the assigned directory.
        self.directory = tempfile.TemporaryDirectory(dir=Path(__file__).parent)
        self.path = Path(self.directory.name) / "timestamps.csv"

    def tearDown(self):
        self.directory.cleanup()

    def write(self, rows, header=HEADER):
        with self.path.open("w", newline="") as output:
            writer = csv.writer(output)
            writer.writerow(header)
            writer.writerows(rows)

    def test_fit_and_absolute_latency_separated(self):
        self.write([row(i) for i in range(10)])
        before = self.path.read_bytes()
        report = analyze_timestamps(self.path)
        stream = report["streams"][0]
        fit = stream["segments"][0]["sdk_to_host_steady_fit"]
        self.assertAlmostEqual(fit["drift_ppm"], 50, places=5)
        self.assertEqual(fit["source_reference_ns"], UTC_BASE)
        self.assertEqual(fit["offset_at_reference_ns"], STEADY_BASE + 2_000_000 - UTC_BASE)
        self.assertLess(fit["absolute_residual_ns"]["max"], 0.01)
        self.assertEqual(stream["arrival_latency_proxy_ns"]["p50"], 2_000_000)
        self.assertEqual(report["percentile_method"], "numpy linear interpolation")
        self.assertEqual(stream["arrival_latency_proxy_ns"]["percentile_method"],
                         "numpy linear interpolation")
        self.assertEqual(fit["residual_ns"]["percentile_method"], "numpy linear interpolation")
        self.assertIn("USB/driver delay", report["arrival_latency_label"])
        self.assertIn("not exposure-to-arrival", report["arrival_latency_label"])
        self.assertEqual(report["input"]["sha256"], hashlib.sha256(before).hexdigest())
        self.assertEqual(self.path.read_bytes(), before)

    def test_per_stream_counts_indices_and_latency_percentiles(self):
        rows = []
        for i in range(10):
            rows.extend([row(i, stream="Infrared", index=1, delay_ns=i * 1_000_000),
                         row(i, stream="Infrared", index=2, delay_ns=3_000_000)])
        self.write(rows)
        streams = analyze_timestamps(self.path)["streams"]
        self.assertEqual([stream["index"] for stream in streams], [1, 2])
        stats = streams[0]["arrival_latency_proxy_ns"]
        self.assertEqual(stats["count"], 10)
        self.assertEqual(stats["p50"], 4_500_000)
        self.assertEqual(stats["p99"], 8_910_000)
        self.assertEqual(streams[1]["arrival_latency_proxy_ns"]["max"], 3_000_000)

    def test_every_percentile_object_is_labeled(self):
        self.write([row(i, delay_ns=i * 1_000_000) for i in range(6)])
        report = analyze_timestamps(self.path)
        distributions = []

        def visit(value):
            if isinstance(value, dict):
                if "p50" in value:
                    distributions.append(value)
                    self.assertEqual(value["percentile_method"], "numpy linear interpolation")
                for child in value.values():
                    visit(child)
            elif isinstance(value, list):
                for child in value:
                    visit(child)

        visit(report)
        self.assertEqual(len(distributions), 4)  # Stream, segment, signed/absolute residuals.

    def test_domain_transitions_never_stitched(self):
        rows = [row(i) for i in range(3)]
        rows += [row(3, domain="Hardware Clock")]
        rows += [row(i, domain="RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME") for i in range(4, 7)]
        self.write(rows)
        stream = analyze_timestamps(self.path)["streams"][0]
        self.assertEqual(stream["skipped_non_global_time_count"], 1)
        self.assertEqual([s["sample_count"] for s in stream["segments"]], [3, 3])
        self.assertEqual(stream["segments"][1]["start_reason"], "domain transition")

    def test_hardware_only_unavailable(self):
        self.write([row(i, domain="Hardware Clock") for i in range(5)])
        stream = analyze_timestamps(self.path)["streams"][0]
        self.assertEqual(stream["segments"], [])
        self.assertIsNone(stream["arrival_latency_proxy_ns"])

    def test_utc_step_splits_fit(self):
        rows = [row(i) for i in range(6)]
        for data in rows[3:]:
            data[6] += 100_000_000
        self.write(rows)
        segments = analyze_timestamps(self.path)["streams"][0]["segments"]
        self.assertEqual([s["sample_count"] for s in segments], [3, 3])
        self.assertIn("UTC step", segments[1]["start_reason"])

    def test_sdk_and_frame_reversals_split_fit(self):
        self.write([row(i) for i in range(3)] + [row(i) for i in range(3)])
        segments = analyze_timestamps(self.path)["streams"][0]["segments"]
        self.assertEqual([s["sample_count"] for s in segments], [3, 3])
        self.assertEqual(segments[1]["start_reason"], "nonincreasing SDK epoch")

    def test_negative_latency_retained_as_diagnostic(self):
        self.write([row(i, delay_ns=-100) for i in range(3)])
        segment = analyze_timestamps(self.path)["streams"][0]["segments"][0]
        self.assertEqual(segment["negative_arrival_latency_count"], 3)
        self.assertEqual(segment["arrival_latency_proxy_ns"]["min"], -100)

    def test_decimal_ms_sub_nanosecond_rounding(self):
        rows = [row(i) for i in range(3)]
        # Conversion must round decimal ns half-even without UTC float loss.
        rows[0][3] = str(Decimal(rows[0][3]) + Decimal("0.0000005"))
        self.write(rows)
        fit = analyze_timestamps(self.path)["streams"][0]["segments"][0]["sdk_to_host_steady_fit"]
        self.assertEqual(fit["source_reference_ns"], UTC_BASE + 1)

    def test_invalid_headers_fields_and_nonfinite_sdk_refused(self):
        for value in ("NaN", "Infinity", "nonsense"):
            with self.subTest(value=value):
                data = row(0)
                data[3] = value
                self.write([data])
                with self.assertRaisesRegex(ValueError, "CSV line 2"):
                    analyze_timestamps(self.path)
        self.write([], header=["sdk_ms"])
        with self.assertRaisesRegex(ValueError, "missing CSV columns"):
            analyze_timestamps(self.path)
        self.write([], header=HEADER + ["sdk_ms"])
        with self.assertRaisesRegex(ValueError, "duplicate"):
            analyze_timestamps(self.path)
        self.write([row(0) + ["unexpected"]])
        with self.assertRaisesRegex(ValueError, "field count"):
            analyze_timestamps(self.path)

    def test_empty_and_short_csv(self):
        self.write([])
        with self.assertRaisesRegex(ValueError, "no timestamp rows"):
            analyze_timestamps(self.path)
        self.write([row(0), row(1)])
        segment = analyze_timestamps(self.path)["streams"][0]["segments"][0]
        self.assertIn("UNAVAILABLE", segment["fit_status"])

    def test_cli_both_module_and_script_and_input_errors(self):
        self.write([row(i) for i in range(3)])
        root = Path(__file__).resolve().parents[3]
        for command in ([sys.executable, "-m", "tools.clockmap.analyze", str(self.path)],
                        [sys.executable, "tools/clockmap/analyze.py", str(self.path)]):
            result = subprocess.run(command, cwd=root, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('"drift_ppm"', result.stdout)
            self.assertEqual(json.loads(result.stdout)["percentile_method"],
                             "numpy linear interpolation")
        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr):
            code = main([str(self.path.parent / "missing.csv")])
        self.assertEqual(code, 2)
        self.assertIn("clockmap:", stderr.getvalue())


if __name__ == "__main__":
    unittest.main()
