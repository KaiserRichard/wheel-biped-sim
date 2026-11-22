"""SYNTHETIC ONLY: tests of tooling, never physical sensor qualification."""
import csv
import json
import importlib.util
import os
import shutil
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import allan_analyze as allan
import power_monitor


class AllanTests(unittest.TestCase):
    def setUp(self):
        # Confine all temporary test evidence to the authorized subtree.
        self.temp = tempfile.TemporaryDirectory(prefix="synthetic-", dir=ROOT / "tests")
        self.addCleanup(self.temp.cleanup)
        self.work = Path(self.temp.name)

    def test_recorder_skips_interval_across_timestamp_domain_change(self):
        compiler = os.environ.get("CXX") or shutil.which("c++") or shutil.which("clang++")
        if not compiler:
            self.skipTest("no C++17 compiler available for recorder Stats regression")
        executable = self.work / "test_stats"
        source = ROOT / "tests/test_stats.cpp"
        command = [compiler, "-std=c++17", "-Wall", "-Wextra", "-pedantic",
                   "-pthread", "-I", str(ROOT / "tests/stub"), str(source), "-o", str(executable)]
        built = subprocess.run(command, capture_output=True, text=True)
        self.assertEqual(built.returncode, 0, built.stdout + built.stderr)
        ran = subprocess.run([str(executable)], capture_output=True, text=True)
        self.assertEqual(ran.returncode, 0, ran.stdout + ran.stderr)

    def write_stream(self, path, sensor, domain="Hardware Clock", seconds=100):
        rate = allan.RATES[sensor]
        rng = np.random.default_rng(120 if sensor == "gyro" else 121)
        data = rng.normal(size=(int(seconds*rate), 3)) * 0.02 * np.sqrt(rate)
        with path.open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(allan.HEADER)
            for i, xyz in enumerate(data):
                writer.writerow([format(123456.78901234567 + i*1000/rate, ".17g"), domain, i+1,
                                 9007199254740993+i, *(format(v, ".17g") for v in xyz)])
        return data

    def test_three_hour_native_200hz_noise_and_walk(self):
        start = time.monotonic()
        rate, hours, n_density, k_walk = 200.0, 3.0, 0.03, 0.01
        count = int(hours*3600*rate)
        dt = 1/rate
        rng = np.random.default_rng(20261014)
        # Discrete rate measurement white sigma = N/sqrt(dt).
        # Brownian rate/bias increments sigma = K*sqrt(dt), not K/sqrt(dt).
        values = rng.normal(0, n_density/np.sqrt(dt), (count, 3))
        values += np.cumsum(rng.normal(0, k_walk*np.sqrt(dt), (count, 3)), axis=0)
        stamps = np.arange(count, dtype=float)*1000/rate
        report = allan.analyze_samples(stamps, values, {"Hardware Clock"}, "gyro")
        receipts = {}
        for axis in "xyz":
            n, k = report["axes"][axis]["N"], report["axes"][axis]["K"]
            self.assertTrue(n["observed"], n)
            self.assertTrue(k["observed"], k)
            self.assertLess(abs(n["value"]/n_density - 1), 0.15)
            self.assertLess(abs(k["value"]/k_walk - 1), 0.30)
            receipts[axis] = {"N": n["value"], "K": k["value"], "N_slope": n["measured_slope"],
                              "K_slope": k["measured_slope"], "N_r2": n["r_squared"], "K_r2": k["r_squared"]}
        self.assertTrue(all(c >= 3 for c in report["independent_clusters"]))
        self.assertEqual(report["dropped_samples"], 0)
        print("\nSYNTHETIC 3 h, 200 Hz, no decimation:", json.dumps(receipts), flush=True)
        print(f"SYNTHETIC recovery elapsed_s={time.monotonic()-start:.3f}", flush=True)

    def test_gap_splitting(self):
        stamps = np.arange(1000)*5.0
        stamps[300:] += 100
        values = np.random.default_rng(7).normal(size=(1000, 3))
        selected, report = allan.clean_segment(stamps, values, {"Global Time"}, 200)
        self.assertEqual(report["selected_indices"], [300, 1000])
        self.assertEqual(report["segment_lengths"], [300, 700])
        self.assertEqual(report["dropped_samples"], 300)
        self.assertEqual(report["gap_count"], 1)
        np.testing.assert_array_equal(selected, values[300:])
        self.assertAlmostEqual(report["max_gap_s"], 0.105)

    def test_timestamp_validation(self):
        stamps = np.arange(1000)*5.0
        values = np.ones((1000, 3))
        with self.assertRaisesRegex(ValueError, "SYSTEM_TIME"):
            allan.clean_segment(stamps, values, {"System Time"}, 200)
        _, report = allan.clean_segment(stamps, values, {"System Time"}, 200, True)
        self.assertTrue(report["diagnostic"])
        with self.assertRaisesRegex(ValueError, "multiple"):
            allan.clean_segment(stamps, values, {"Hardware Clock", "Global Time"}, 200)
        for offset in (0, -1):
            bad = stamps.copy(); bad[10] = bad[9]+offset
            with self.assertRaisesRegex(ValueError, "duplicate/backward"):
                allan.clean_segment(bad, values, {"Hardware Clock"}, 200)
        with self.assertRaisesRegex(ValueError, "median dt"):
            allan.clean_segment(stamps*2, values, {"Hardware Clock"}, 200)
        bad = values.copy(); bad[10, 1] = np.nan
        with self.assertRaisesRegex(ValueError, "nonfinite"):
            allan.clean_segment(stamps, bad, {"Hardware Clock"}, 200)

    def test_csv_precision_and_cli_outputs_unknown_walk(self):
        run = self.work / "run"; run.mkdir()
        expected = self.write_stream(run / "gyro.csv", "gyro")
        self.write_stream(run / "accel.csv", "accel")
        stamps, values, domains, frames, hosts = allan.load_csv(run / "gyro.csv")
        np.testing.assert_array_equal(values, expected)
        self.assertEqual(stamps[0], 123456.78901234567)
        self.assertEqual(domains, {"Hardware Clock"})
        self.assertEqual(int(hosts[0]), 9007199254740993)
        self.assertEqual(int(frames[-1]), len(expected))
        out = self.work / "out"
        proc = subprocess.run([sys.executable, str(ROOT / "allan_analyze.py"), str(run), str(out), "--no-plots"],
                              capture_output=True, text=True)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        result = json.loads((out / "allan.json").read_text())
        self.assertIsNone(result["conservative_max_all_axes"]["gyroscope_random_walk"])
        self.assertIn("null # UNKNOWN", (out / "imu_allan_raw.yaml").read_text())
        self.assertIn("rostopic: /imu0", (out / "imu_allan_x10.yaml").read_text())
        self.assertIn("update_rate: 200.0", (out / "imu_allan_raw.yaml").read_text())
        self.assertIn("0.001176798", (out / "summary.md").read_text())
        with self.assertRaisesRegex(ValueError, "new or empty"):
            allan.analyze_run(run, out, plots=False)

    def test_system_time_cli_diagnostic(self):
        run = self.work / "run"; run.mkdir()
        self.write_stream(run / "gyro.csv", "gyro", "System Time", 10)
        self.write_stream(run / "accel.csv", "accel", "System Time", 10)
        out = self.work / "out"
        proc = subprocess.run([sys.executable, str(ROOT / "allan_analyze.py"), str(run), str(out), "--no-plots"],
                              capture_output=True, text=True)
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("SYSTEM_TIME", proc.stderr)
        self.assertFalse(out.exists())
        result = allan.analyze_run(run, out, allow_system_time=True, plots=False)
        self.assertEqual(result["evidence_label"], "DIAGNOSTIC")
        self.assertIn("DIAGNOSTIC", (out / "imu_allan_raw.yaml").read_text())

    def test_overlapping_matches_brute_force(self):
        values = np.random.default_rng(9).normal(size=(120, 3))
        tau, deviation, terms, clusters = allan.overlapping_allan(values, 200)
        for i, m in enumerate(np.rint(tau*200).astype(int)):
            differences = [values[j+m:j+2*m].mean(axis=0) - values[j:j+m].mean(axis=0)
                           for j in range(len(values)-2*m+1)]
            expected = np.sqrt(np.mean(np.array(differences)**2, axis=0)/2)
            np.testing.assert_allclose(deviation[i], expected, rtol=1e-12)
            self.assertEqual(terms[i], len(differences))
            self.assertEqual(clusters[i], len(values)//m)

    def test_yaml_maxima_and_inflation(self):
        reports = {s: {"axes": {a: {"N": {"value": i+1.0}, "K": {"value": (i+1)*0.001}}
                               for i, a in enumerate("xyz")}} for s in ("gyro", "accel")}
        values = allan.conservative_values(reports)
        self.assertEqual(values["gyroscope_noise_density"], 3)
        path = self.work / "x10.yaml"
        allan.write_yaml(path, values, 10, False)
        self.assertIn("gyroscope_noise_density: 30", path.read_text())
        reports["gyro"]["axes"]["y"]["K"]["value"] = None
        self.assertIsNone(allan.conservative_values(reports)["gyroscope_random_walk"])

    def test_optional_plotting_and_power_diagnostic(self):
        run = self.work / "run"; run.mkdir()
        self.write_stream(run / "gyro.csv", "gyro", seconds=10)
        self.write_stream(run / "accel.csv", "accel", seconds=10)
        (run / "power_summary.json").write_text(json.dumps({"monitoring_complete": True, "undervoltage_occurred": True}))
        available = importlib.util.find_spec("matplotlib") is not None
        with patch.dict(sys.modules, {"matplotlib": None}):
            report = allan.analyze_run(run, self.work / "no-matplotlib", plots=True)
        self.assertIn("unavailable", report["plots"])
        self.assertEqual(report["evidence_label"], "DIAGNOSTIC")
        if available:
            out = self.work / "plots"
            report = allan.analyze_run(run, out, plots=True)
            self.assertEqual(report["plots"], "generated")
            for sensor in ("gyro", "accel"):
                self.assertGreater((out / f"allan_{sensor}.png").stat().st_size, 1000)

    def test_power_bits_without_hardware(self):
        self.assertEqual(power_monitor.throttled_mask("throttled=0x50000"), 0x50000)
        self.assertIsNone(power_monitor.throttled_mask("failed"))
        self.assertTrue(power_monitor.summarize([0, 1, 0])["undervoltage_occurred"])
        self.assertTrue(power_monitor.summarize([0, 0x10000])["sticky_undervoltage_changed"])
        self.assertFalse(power_monitor.summarize([0x10000, 0x10000])["undervoltage_occurred"])
        self.assertFalse(power_monitor.summarize([0, None])["monitoring_complete"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
