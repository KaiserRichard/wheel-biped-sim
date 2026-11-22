import csv
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from tools.runtime_observer import aggregate as agg
from tools.runtime_observer import patch_timing as patch
from tools.runtime_observer import resource_sampler as resource
from tools.runtime_observer.stats import distribution
from tools.clockmap import ClockMap, Exchange, Stamp


def event(ns, kind, frame=1, source=None, session="run", **kwargs):
    return agg.Event(ns, session, frame, kind, source, **kwargs)


def clockmap():
    mapping = ClockMap("device", "host_steady", "device-boot", "run")
    # Synthetic +1000 ns offset with symmetric 100 ns one-way delay.
    for t in (0, 1_000_000, 2_000_000):
        mapping.add_exchange(Exchange(Stamp(t, "device", "device-boot"),
                                      Stamp(t + 1100, "host_steady", "run"),
                                      Stamp(t + 1100, "host_steady", "run"),
                                      Stamp(t + 200, "device", "device-boot")))
    return mapping


class AggregateTests(unittest.TestCase):
    def test_quantiles_nearest_rank(self):
        self.assertEqual(distribution(range(1, 101)),
                         dict(status="AVAILABLE", count=100, p50=50, p95=95, p99=99, max=100))
        self.assertEqual(distribution([]), dict(status="UNAVAILABLE", count=0))

    def test_known_stages_age_drops_backlog_throughput(self):
        rows = [event(100, "frame_received", source=(50, "host_steady", "run")),
                event(120, "queued"), event(220, "track_start"), event(250, "track_end"),
                event(260, "snapshot_published"), event(300, "frame_received", frame=2),
                event(350, "dropped", frame=2, reason="overload"),
                event(400, "dropped", frame=3, reason="overload"),
                event(450, "dropped", frame=4, reason="bad_input"),
                event(1_000_000_000, "queue_depth", depth=1),
                event(2_000_000_000, "queue_depth", depth=3),
                event(3_000_000_000, "queue_depth", depth=5)]
        report = agg.summarize(rows, losses=dict(overwritten=0, contention=0, invalid=0, clock_errors=0))
        self.assertEqual(report["trace_completeness"], "COMPLETE")
        result = report["sessions"]["run"]
        expected = dict(receive_to_queue_ns=20, queue_wait_ns=100, track_duration_ns=30,
                        publish_delay_ns=10, receive_to_publish_ns=160)
        for key, value in expected.items():
            self.assertEqual(result["stage_durations_ns"][key]["p99"], value)
            self.assertEqual(result["stage_durations_ns"][key]["missing_pairs"], 3)
        self.assertEqual(result["source_ages_ns"]["snapshot_published"]["upper_bound"]["max"], 210)
        self.assertEqual(result["host_ages_since_received_ns"]["track_end"]["p95"], 150)
        self.assertEqual(result["drops_by_reason"], dict(overload=2, bad_input=1))
        self.assertEqual(result["queue_depth"]["slope_depth_per_s"], 2)
        self.assertEqual(result["queue_depth"]["time_weighted_mean"], 2)
        self.assertEqual(result["queue_depth"]["sample_mean"], 3)
        self.assertAlmostEqual(result["throughput_per_s"]["frame_received"], 2e9 / (3e9 - 100))

    def test_cross_clock_age_requires_explicit_map(self):
        rows = [event(1_003_000, "frame_received", source=(1_000_000, "device", "device-boot")),
                event(1_004_000, "snapshot_published")]
        unavailable = agg.summarize(rows)["sessions"]["run"]["source_ages_ns"]["snapshot_published"]
        self.assertEqual(unavailable["point"]["status"], "UNAVAILABLE")
        self.assertEqual(unavailable["unavailable_reasons"], {"no clock map": 1})
        ages = agg.summarize(rows, clockmap())["sessions"]["run"]["source_ages_ns"]["snapshot_published"]
        self.assertEqual(ages["point"]["p50"], 3000)
        self.assertGreater(ages["upper_bound"]["p50"], 3000)
        self.assertEqual(ages["upper_bound"]["p50"], 3000 + ages["uncertainty"]["p50"])

    def test_invalid_maps_sessions_future_and_unknown_domains(self):
        for source, mapping, reason in [
            ((100, "host_steady", "old-run"), None, "host session mismatch"),
            ((101, "host_steady", "run"), None, "source epoch in future"),
            ((100, "device", "device-boot"), ClockMap("device", "host_steady", "device-boot", "run"), "insufficient exchanges"),
            ((100, "unrelated", "device-boot"), clockmap(), "unrelated")]:
            result = agg.summarize([event(100, "frame_received", source=source)], mapping)
            ages = result["sessions"]["run"]["source_ages_ns"]["frame_received"]
            self.assertEqual(ages["upper_bound"]["status"], "UNAVAILABLE")
            self.assertTrue(any(reason in key for key in ages["unavailable_reasons"]))

    def test_sessions_are_never_combined(self):
        rows = [event(1000, "frame_received"), event(2000, "track_end"),
                event(1, "frame_received", session="new"), event(2, "track_end", session="new")]
        reports = agg.summarize(rows)["sessions"]
        self.assertEqual(set(reports), {"run", "new"})
        self.assertEqual(reports["new"]["observed_span_ns"], 1)

    def test_large_epoch_ns_precision_and_single_sample(self):
        epoch = 2**62
        result = agg.summarize([event(epoch, "track_start"), event(epoch + 1, "track_end")])
        self.assertEqual(result["sessions"]["run"]["stage_durations_ns"]["track_duration_ns"]["max"], 1)
        result = agg.summarize([event(epoch, "queue_depth", depth=2)])["sessions"]["run"]
        self.assertEqual(result["queue_depth"]["slope_depth_per_s"], "UNAVAILABLE")
        self.assertEqual(result["throughput_per_s"]["track_end"], "UNAVAILABLE")

    def test_malformed_and_inconsistent_streams(self):
        streams = [[event(2, "track_start"), event(1, "track_end")],
                   [event(1, "track_end"), event(2, "track_start")],
                   [event(1, "queued"), event(2, "queued")],
                   [event(1, "snapshot_published"), event(2, "dropped", reason="late")],
                   [event(1, "queued", source=(0, "device", "boot")),
                    event(2, "track_start", source=(1, "device", "boot"))]]
        for rows in streams:
            with self.assertRaises(ValueError):
                agg.summarize(rows)
        for kwargs in (dict(ns=-1, kind="queued"), dict(ns=1, kind="queue_depth", depth=-1),
                       dict(ns=1, kind="dropped", reason="bad,reason"),
                       dict(ns=1, kind="queued", source=(1.5, "device", "boot"))):
            with self.assertRaises(ValueError):
                event(**kwargs)
        for text in ("", "wrong,header\n", ",".join(agg.FIELDS) + "\n1,host_utc,run,1,queued,,,,,\n",
                     "# observer_loss overwritten=0 contention=0 invalid=0 clock_errors=0 extra\n" + ",".join(agg.FIELDS),
                     ",".join(agg.FIELDS) + "\n1,host_steady,run,1,queued,2,device,,,\n"):
            with self.assertRaises(ValueError):
                agg.read_csv(text)

    def test_incomplete_trace_and_empty_ring(self):
        report = agg.summarize([event(1, "track_end")], losses={"overwritten": 20})
        self.assertEqual(report["trace_completeness"], "INCOMPLETE")
        self.assertEqual(report["sessions"]["run"]["stage_durations_ns"]["track_duration_ns"]["status"], "UNAVAILABLE")
        events, losses = agg.read_csv("# observer_loss overwritten=0 contention=0 invalid=0 clock_errors=0\n" + ",".join(agg.FIELDS) + "\n")
        self.assertEqual(agg.summarize(events, losses=losses)["sessions"], {})

    def test_cpp_csv_to_python_and_cli(self):
        executable = Path(__file__).parent / "test_recorder"
        if not executable.exists():
            self.skipTest("run make test to include C++ round-trip")
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as tmp:
            path = Path(tmp) / "events.csv"
            subprocess.run([str(executable), "--dump", str(path)], check=True)
            before = path.read_bytes()
            rows, losses = agg.read_csv(before.decode())
            self.assertEqual(len(rows), 7)
            self.assertEqual(losses, dict(overwritten=0, contention=0, invalid=0, clock_errors=0))
            result = subprocess.run([sys.executable, str(ROOT / "tools/runtime_observer/aggregate.py"), str(path)],
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(json.loads(result.stdout)["sessions"]["test-run"]["drops_by_reason"], {"backlog": 1})
            self.assertEqual(before, path.read_bytes())

    def test_clockmap_json_cli_loader(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as tmp:
            path = Path(tmp) / "clockmap.json"
            path.write_text(json.dumps(dict(domain_a="device", domain_b="host_steady", session_a="device-boot",
                                            session_b="run", exchanges_ns=[[t, t+1100, t+1100, t+200]
                                                                          for t in (0, 1_000_000, 2_000_000)])))
            self.assertTrue(agg.load_clockmap(path).valid)
            data = json.loads(path.read_text())
            data["exchanges_ns"][0][0] = 0.0
            path.write_text(json.dumps(data))
            with self.assertRaises(ValueError):
                agg.load_clockmap(path)


class ResourceTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(dir=Path(__file__).parent)
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.proc, self.sys = self.root / "proc", self.root / "sys"

    def write(self, root, name, text):
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)

    def process_stat(self, utime=20, stime=10, start=123):
        fields = ["0"] * 50
        fields[0] = "S"
        fields[11], fields[12], fields[19] = map(str, (utime, stime, start))
        self.write(self.proc, "42/stat", "42 (name with ) parentheses) " + " ".join(fields))

    def sampler(self, **kwargs):
        return resource.Sampler(42, "run", proc_root=self.proc, sys_root=self.sys,
                                system="Linux", clock_ticks=100, **kwargs)

    def test_fake_proc_sysfs_cpu_memory_frequency_throttle(self):
        self.write(self.proc, "stat", "cpu 100 0 100 800 0 0 0 0 9999 9999\ncpu0 1\ncpu1 1\n")
        self.write(self.proc, "42/status", "VmRSS\x3a\t2048 kB\nThreads\x3a\t7\n")
        self.process_stat()
        self.write(self.sys, "class/thermal/thermal_zone0/temp", "47500\n")
        self.write(self.sys, "class/thermal/thermal_zone0/type", "cpu-thermal\n")
        self.write(self.sys, "devices/system/cpu/cpufreq/policy0/scaling_cur_freq", "2400000\n")
        times = iter((1_000_000_000, 1_000_000_001, 2_000_000_000, 2_000_000_001))
        sampler = self.sampler(clock=lambda: next(times), throttle_reader=lambda: "throttled=0x50005")
        first = sampler.sample()
        self.assertEqual(first["cpu_busy_pct_board"], "UNAVAILABLE")
        self.assertEqual(first["process_cpu_pct_one_core"], "UNAVAILABLE")
        self.assertEqual(first["rss_bytes"], 2097152)
        self.assertEqual(first["threads"], 7)
        self.assertEqual(json.loads(first["thermal_c_json"])["thermal_zone0"]["c"], 47.5)
        self.assertEqual(json.loads(first["cpufreq_khz_json"]), {"policy0": 2400000})
        self.assertEqual(first["throttled_now"], 1)
        self.assertEqual(first["undervoltage_occurred"], 1)
        self.assertEqual(first["frequency_capped_now"], 0)
        self.write(self.proc, "stat", "cpu 150 0 150 900 0 0 0 0 123456 123456\ncpu0 1\ncpu1 1\n")
        self.process_stat(120, 60)
        second = sampler.sample()
        self.assertEqual(second["cpu_busy_pct_board"], 50)
        self.assertEqual(second["process_cpu_pct_one_core"], 150)
        self.assertEqual(second["sample_duration_ns"], 1)

    def test_missing_malformed_and_removed_files(self):
        self.write(self.proc, "stat", "cpu broken\n")
        self.write(self.proc, "42/status", "VmRSS: broken\nThreads: -1\n")
        self.write(self.proc, "42/stat", "not a proc stat")
        self.write(self.sys, "class/thermal/thermal_zone0/temp", "bad")
        self.write(self.sys, "class/thermal/thermal_zone1/temp", "-1000")
        self.write(self.sys, "devices/system/cpu/cpufreq/policy0/scaling_cur_freq", "-1")
        row = self.sampler(throttle_reader=lambda: "throttled=oops").sample()
        for key in ("cpu_busy_pct_board", "process_cpu_pct_one_core", "rss_bytes", "threads", "throttled_raw"):
            self.assertEqual(row[key], "UNAVAILABLE")
        self.assertEqual(json.loads(row["thermal_c_json"])["thermal_zone0"]["c"], "UNAVAILABLE")
        self.assertEqual(json.loads(row["thermal_c_json"])["thermal_zone1"]["c"], -1)
        self.assertEqual(json.loads(row["cpufreq_khz_json"])["policy0"], "UNAVAILABLE")

    def test_counter_reversal_pid_reuse_and_missing_baselines(self):
        times = iter((1, 1, 1_000_000_001, 1_000_000_001, 2_000_000_001, 2_000_000_001))
        sampler = self.sampler(clock=lambda: next(times), throttle_reader=lambda: None)
        self.write(self.proc, "stat", "cpu 100 0 100 800 0 0 0 0\ncpu0 1\n")
        self.process_stat()
        sampler.sample()
        self.write(self.proc, "stat", "cpu 0 0 0 1 0 0 0 0\ncpu0 1\n")
        self.process_stat(1, 0, start=999)
        row = sampler.sample()
        self.assertEqual(row["cpu_busy_pct_board"], "UNAVAILABLE")
        self.assertEqual(row["process_cpu_pct_one_core"], "UNAVAILABLE")
        (self.proc / "42/stat").unlink()
        row = sampler.sample()
        self.assertEqual(row["process_start_ticks"], "UNAVAILABLE")

    def test_macos_ignores_linux_readers(self):
        def forbidden():
            raise AssertionError("macOS must not run vcgencmd")
        row = resource.Sampler(42, "run", system="Darwin", proc_root=self.proc, sys_root=self.sys,
                               throttle_reader=forbidden).sample()
        for field in resource.FIELDS[5:]:
            self.assertEqual(row[field], "UNAVAILABLE")
        self.assertEqual(row["host_domain"], "host_steady")

    def test_one_hz_schedule_csv_and_missed_slots(self):
        times = iter((0, 100_000_000, 3_200_000_000))
        waits = []
        stream = io.StringIO()
        resource.write_samples(self.sampler(throttle_reader=lambda: None), stream, 3,
                               clock=lambda: next(times), sleep=waits.append)
        self.assertEqual(waits, [.9, .8])
        rows = list(csv.DictReader(io.StringIO(stream.getvalue())))
        self.assertEqual(len(rows), 3)
        self.assertEqual(rows[0]["throttled_raw"], "UNAVAILABLE")

    def test_command_absent_timeout_and_bad_output(self):
        from unittest.mock import patch as mock
        with mock("shutil.which", return_value=None):
            self.assertIsNone(resource.get_throttled())
        with mock("shutil.which", return_value="/fake/vcgencmd"), mock("subprocess.run", side_effect=subprocess.TimeoutExpired("vcgencmd", .2)):
            self.assertIsNone(resource.get_throttled())
        for value in ("throttled=garbage", "throttled=0x100000000", ""):
            self.assertEqual(resource.throttling(value)["throttled_raw"], "UNAVAILABLE")

    def test_sampler_cli_new_file_only(self):
        path = self.root / "resources.csv"
        cmd = [sys.executable, str(ROOT / "tools/runtime_observer/resource_sampler.py"), str(path),
               "--pid", str(__import__("os").getpid()), "--session", "cli-run", "--samples", "1"]
        result = subprocess.run(cmd, capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        original = path.read_bytes()
        result = subprocess.run(cmd, capture_output=True, text=True)
        self.assertEqual(result.returncode, 2)
        self.assertEqual(original, path.read_bytes())


class PatchTests(unittest.TestCase):
    CSV = "image_s,state,imu_initialized,imu_samples,track_ms\n1.000000001,2,0,6,10.5\n1.100000001,2,1,7,20.25\n"
    STDOUT = "other ORB output\nreceived=4 processed=2 prefix_images=1 prefix_gyro=4 uncovered_tail_images=1 pending_images=0\n"

    def test_exact_patch_formats(self):
        rows = patch.read_tracking(self.CSV)
        result = patch.summarize(rows, self.STDOUT, "global_time", "device-boot")
        self.assertEqual(rows[0]["image_ns"], 1_000_000_001)
        self.assertEqual(result["track_duration_ns"]["p50"], 10_500_000)
        self.assertEqual(result["track_duration_ns"]["p99"], 20_250_000)
        self.assertEqual(result["source_interval_processed_hz"], 10)
        self.assertEqual(result["initialized_rows"], 1)
        self.assertEqual(result["count_summary"]["prefix_gyro"], 4)
        for key in ("host_throughput_per_s", "queue_wait_ns", "source_ages_ns", "drops_by_reason"):
            self.assertEqual(result[key], "UNAVAILABLE")
        self.assertEqual(patch.summarize(rows)["source_interval_processed_hz"], "UNAVAILABLE")

    def test_negative_nonfinite_malformed_repeated_and_mismatched(self):
        for old, new in (("10.5", "-1"), ("10.5", "NaN"), ("10.5", "inf"),
                         ("2,0,6", "2,4,6"), ("1.100000001", "1.000000001"), ("20.25", "bad")):
            with self.assertRaises(ValueError):
                patch.read_tracking(self.CSV.replace(old, new))
        for stdout in ("received=oops", self.STDOUT.replace("processed=2", "processed=3"), self.STDOUT * 2):
            with self.assertRaises(ValueError):
                patch.summarize(patch.read_tracking(self.CSV), stdout)
        with self.assertRaises(ValueError):
            patch.summarize([], source_domain="device")

    def test_patch_cli_and_input_immutability(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as tmp:
            tracking, stdout = Path(tmp) / "tracking.csv", Path(tmp) / "stdout.log"
            tracking.write_text(self.CSV)
            stdout.write_text(self.STDOUT)
            result = subprocess.run([sys.executable, str(ROOT / "tools/runtime_observer/patch_timing.py"), str(tracking),
                                     "--stdout", str(stdout)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(json.loads(result.stdout)["track_duration_ns"]["count"], 2)
            self.assertEqual(tracking.read_text(), self.CSV)
            self.assertEqual(stdout.read_text(), self.STDOUT)


if __name__ == "__main__":
    unittest.main()
