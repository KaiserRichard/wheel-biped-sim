import csv
import json
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest.mock import patch

from tools.eval_pipeline.profile import profile

HERE = Path(__file__).resolve().parents[1]


def fixture_dir(source=None, *, throttle="0", temp=55):
    root = Path(tempfile.mkdtemp())
    if source is None:
        source = HERE / "tests/fixtures/n4a_real_throttle_samples.csv"
    if source != "synthetic":
        (root / "samples.csv").write_bytes(source.read_bytes())
        return root
    fields = ["monotonic_ns", "clock_domain", "pid", "cpu_count", "per_core_cpu_pct_json",
              "target_cpu_pct_one_core", "thread_cpu_pct_json", "process_start_ticks", "rss_bytes",
              "pss_bytes", "cpu_freq_khz_json", "thermal_c_json", "throttled_raw", "throttle_sampled", *profile.THROTTLE_NAMES,
              "fan_state_json", "sample_duration_ns"]
    row = {key: "UNKNOWN" for key in fields}
    row.update(monotonic_ns="1", clock_domain="CLOCK_MONOTONIC", pid="1", throttle_sampled="1", thermal_c_json=json.dumps(
        {"thermal_zone0": {"type": "cpu-thermal", "c": temp}}), throttled_raw="0x0", sample_duration_ns="1")
    for key in profile.THROTTLE_NAMES:
        row[key] = throttle
    with (root / "samples.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader(); writer.writerow(row)
    return root


class ProfileTests(unittest.TestCase):
    def test_validity_pass(self):
        root = fixture_dir("synthetic", throttle="0", temp=55)
        self.assertEqual(profile.validity(type("Args", (), {"analyze_only": root, "out": root})()), 0)
        self.assertEqual(json.loads((root / "result.json").read_text())["status"], "PASS")

    def test_temperature_boundary_79_9_passes_and_80_0_fails(self):
        for temperature, expected in ((79.9, 0), (80.0, 1)):
            with self.subTest(temperature=temperature):
                root = fixture_dir("synthetic", throttle="0", temp=temperature)
                self.assertEqual(profile.validity(type("Args", (), {"analyze_only": root, "out": root})()), expected)
                result = json.loads((root / "result.json").read_text())
                criterion = next(item for item in result["criteria"] if item["id"] == "max_temperature")
                self.assertEqual(criterion["status"], "PASS" if expected == 0 else "FAIL")

    def test_validity_ignores_rows_without_throttle_poll(self):
        root = fixture_dir("synthetic", throttle="0", temp=55)
        with (root / "samples.csv").open(newline="") as stream:
            rows = list(csv.DictReader(stream))
        rows[0]["throttle_sampled"] = "0"
        rows[0]["throttled_raw"] = "UNKNOWN"
        rows[0].update({key: "UNKNOWN" for key in profile.THROTTLE_NAMES})
        rows.append(dict(rows[0], monotonic_ns="2", throttle_sampled="1", throttled_raw="0x0",
                         **{key: "0" for key in profile.THROTTLE_NAMES}))
        with (root / "samples.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=rows[0].keys()); writer.writeheader(); writer.writerows(rows)
        self.assertEqual(profile.validity(type("Args", (), {"analyze_only": root, "out": root})()), 0)

    def test_validity_blocks_without_any_throttle_poll(self):
        root = fixture_dir("synthetic", throttle="0", temp=55)
        with (root / "samples.csv").open(newline="") as stream:
            rows = list(csv.DictReader(stream))
        rows[0]["throttle_sampled"] = "0"
        with (root / "samples.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=rows[0].keys()); writer.writeheader(); writer.writerows(rows)
        self.assertEqual(profile.validity(type("Args", (), {"analyze_only": root, "out": root})()), 2)

    def test_validity_blocks_on_missing_samples_file(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td) / "run"
            self.assertEqual(profile.validity(type("Args", (), {"analyze_only": root, "out": root})()), 2)
            result = json.loads((root / "result.json").read_text())
            self.assertEqual(result["status"], "BLOCKED")
            self.assertEqual(result["raw_files"], [])

    def test_validity_blocks_when_a_temperature_sample_is_missing(self):
        root = fixture_dir("synthetic", throttle="0", temp=55)
        with (root / "samples.csv").open(newline="") as stream:
            rows = list(csv.DictReader(stream))
        missing = dict(rows[0], monotonic_ns="2", thermal_c_json="{}")
        rows.append(missing)
        with (root / "samples.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=rows[0].keys()); writer.writeheader(); writer.writerows(rows)
        self.assertEqual(profile.validity(type("Args", (), {"analyze_only": root, "out": root})()), 2)

    def test_nonfinite_temperature_must_block(self):
        # Regression: a NaN reading must not hide behind other finite readings.
        root = fixture_dir("synthetic", throttle="0", temp=55)
        self.addCleanup(shutil.rmtree, root)
        with (root / "samples.csv").open(newline="") as stream:
            rows = list(csv.DictReader(stream))
        rows.append(dict(rows[0], monotonic_ns="2", thermal_c_json=json.dumps(
            {"thermal_zone0": {"type": "cpu-thermal", "c": float("nan")}})))
        with (root / "samples.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
            writer.writeheader()
            writer.writerows(rows)
        self.assertEqual(profile.validity(type("Args", (), {"analyze_only": root, "out": root})()), 2)

    def test_validity_fail_on_real_saved_n4a_throttle_fixture(self):
        root = fixture_dir()
        self.assertEqual(profile.validity(type("Args", (), {"analyze_only": root, "out": root})()), 1)
        result = json.loads((root / "result.json").read_text())
        self.assertEqual(result["status"], "FAIL")
        self.assertEqual(result["criteria"][0]["measured"], ["throttled_now", "throttled_occurred",
                                                                "undervoltage_now", "undervoltage_occurred"])

    def test_validity_blocked_on_unknown_sensor_data(self):
        root = fixture_dir("synthetic", throttle="UNKNOWN", temp="bad")
        with (root / "samples.csv").open(newline="") as stream:
            row = next(csv.DictReader(stream))
        # Missing thermal values and VC flags are explicitly UNKNOWN.
        row["thermal_c_json"] = "{}"
        row.update({k: "UNKNOWN" for k in profile.THROTTLE_NAMES})
        with (root / "samples.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=row.keys()); writer.writeheader(); writer.writerow(row)
        self.assertEqual(profile.validity(type("Args", (), {"analyze_only": root, "out": root})()), 2)

    def test_tracking_latency_uses_existing_parser_and_budget(self):
        with tempfile.TemporaryDirectory() as td:
            td = Path(td); tracking = td / "tracking.csv"
            tracking.write_text("image_s,state,imu_initialized,imu_samples,track_ms\n1,2,1,4,10\n2,2,1,4,40\n")
            profile.main(["latency", "--tracking", str(tracking), "--out", str(td)])
            data = json.loads((td / "latency.json").read_text())
            self.assertEqual(data["over_camera_budget_count"], 1)
            self.assertEqual(data["tracking_latency_ns"]["p50"], 10_000_000)
            self.assertEqual(data["end_to_end_latency_ns"], "UNKNOWN")

    def test_event_latency_one_host_clock(self):
        with tempfile.TemporaryDirectory() as td:
            td = Path(td); events = td / "events.csv"
            events.write_text("host_ns,host_domain,host_session,frame_id,event,source_ns,source_domain,source_session,reason,depth\n"
                "0,host_steady,run,1,frame_received,,,,,\n"
                "1000000,host_steady,run,1,queued,,,,,\n"
                "2000000,host_steady,run,1,track_start,,,,,\n"
                "12000000,host_steady,run,1,track_end,,,,,\n"
                "14000000,host_steady,run,1,snapshot_published,,,,,\n")
            profile.main(["latency", "--events", str(events), "--out", str(td)])
            data = json.loads((td / "latency.json").read_text())["sessions"]["run"]
            self.assertEqual(data["tracking_latency_ns"]["p50"], 10_000_000)
            self.assertEqual(data["end_to_end_latency_ns"]["p50"], 14_000_000)
            self.assertEqual(data["over_camera_budget_count"], 0)

    def test_clockmap_must_target_observer_session(self):
        with tempfile.TemporaryDirectory() as td:
            td = Path(td); events = td / "events.csv"
            events.write_text("host_ns,host_domain,host_session,frame_id,event,source_ns,source_domain,source_session,reason,depth\n"
                "0,host_steady,run,1,frame_received,0,camera,device,,\n"
                "2000000,host_steady,run,1,track_start,,,,,\n"
                "12000000,host_steady,run,1,track_end,,,,,\n"
                "14000000,host_steady,run,1,snapshot_published,,,,,\n")
            class WrongTargetMap:
                def convert(self, stamp):
                    from tools.clockmap import MappedStamp, Stamp
                    return MappedStamp(Stamp(stamp.value_ns, "other_clock", "run"), 0, 1)
            with patch.object(profile.aggregate, "load_clockmap", return_value=WrongTargetMap()):
                result = profile.main(["latency", "--events", str(events), "--clockmap", "map.json", "--out", str(td)])
            self.assertEqual(result, 3)

    def test_fake_proc_sys_snapshot(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td); proc = root / "proc"; sysroot = root / "sys"
            (proc / "10").mkdir(parents=True); (proc / "stat").write_text(
                "cpu 100 0 50 800 10 0 0 0 0 0\ncpu0 50 0 25 400 5 0 0 0 0 0\n")
            # Linux stat fields after comm begin at state; utime, stime and
            # starttime occupy offsets 11, 12 and 19 in the shared sampler.
            process_fields = ["S"] + ["0"] * 19
            process_fields[11], process_fields[12], process_fields[19] = "20", "5", "123"
            (proc / "10/stat").write_text("10 (fake process) " + " ".join(process_fields))
            (proc / "10/status").write_text("VmRSS\x3a\t42 kB\nThreads\x3a\t2\n")
            (proc / "10/smaps_rollup").write_text("Pss\x3a\t30 kB\n")
            zone = sysroot / "class/thermal/thermal_zone0"; zone.mkdir(parents=True)
            (zone / "temp").write_text("61000\n"); (zone / "type").write_text("cpu-thermal\n")
            with patch.object(profile.resource_sampler, "get_throttled", return_value="throttled=0x0"):
                row, _, process = profile.sample_once(10, {}, None, 1, proc_root=proc, sys_root=sysroot)
            self.assertEqual(row["rss_bytes"], 42 * 1024)
            self.assertEqual(row["pss_bytes"], 30 * 1024)
            self.assertEqual(row["thermal_c_json"], '{"thermal_zone0": {"c": 61.0, "type": "cpu-thermal"}}')
            self.assertEqual(process, (25, 123))

    def test_missing_proc_sys_fields_are_unknown(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            with patch.object(profile.resource_sampler, "get_throttled", return_value=None):
                row, _, _ = profile.sample_once(10, {}, None, 1, proc_root=root / "proc",
                                                 sys_root=root / "sys")
            self.assertEqual(row["per_core_cpu_pct_json"], '"UNKNOWN"')
            self.assertEqual(row["cpu_freq_khz_json"], '"UNKNOWN"')
            self.assertEqual(row["thermal_c_json"], '"UNKNOWN"')
            self.assertEqual(row["rss_bytes"], "UNKNOWN")


if __name__ == "__main__": unittest.main()
