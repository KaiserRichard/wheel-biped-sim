"""Hardware-free fixtures. All generated evidence is explicitly synthetic."""
import contextlib
import copy
import csv
import hashlib
import io
import json
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest import mock

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
import capture
import check
from analyze import analyze, clock_stats
from common import FIELDS, STREAMS, load_config, value, write_json


class QualificationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(dir=HERE / "tests", prefix=".tmp-")
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)
        self.raw = self.base / "raw"
        self.raw.mkdir()
        self.cfg = load_config(HERE / "thresholds.yaml")
        # Synthetic one-second fixtures keep the production 60-second limit intact.
        for group, key in (("capture", "duration_s"), ("limits", "qualification_duration_s")):
            self.cfg[group][key] = {"value": 1, "source": "PROPOSED synthetic test fixture duration only"}
        self.meta = {
            "schema_version": 1, "complete": True, "outcome": "PASS", "errors": [],
            "queue_overflow": 0, "callback_errors": [], "static_confirmed": True,
            "identity": {"serial": "99005", "firmware": "LOCAL_FIRMWARE_VERSION", "usb_type": "3.2"},
            "emitter_readback": [0], "window_start_host_clock_monotonic_ns": 10**16,
            "window_end_host_clock_monotonic_ns": 10**16 + 10**9,
            "evidence_label": "ASSUMED SYNTHETIC FIXTURE; not a hardware gate PASS",
            "profiles": [
                {"stream": s, "fps": 30, "width": 848, "height": 480, "format": "y8"}
                for s in ("ir_left", "ir_right")
            ] + [{"stream": "gyro", "fps": 200, "format": "motion_xyz32f"},
                 {"stream": "accel", "fps": 100, "format": "motion_xyz32f"}],
            "received_counts": {"ir_left": 30, "ir_right": 30, "gyro": 200, "accel": 100},
        }
        self.rows = []
        for stream, rate in self.meta["received_counts"].items():
            for i in range(rate):
                row = dict.fromkeys(FIELDS, "")
                stamp = 1800000000000 + i * 1000 / rate
                row.update(stream=stream, frame_number=i, sdk_timestamp_ms=stamp, sdk_timestamp_domain="global_time",
                           hardware_timestamp_ms=i * 1000 / rate, hardware_timestamp_source="SYNTHETIC hardware counter ms",
                           sdk_global_system_timestamp_ms=stamp,
                           host_clock_monotonic_ns=10**16 + round((i + .5) * 10**9 / rate),
                           host_clock_realtime_ns=1800000000000000000 + round((i + .5) * 10**9 / rate))
                if stream in ("gyro", "accel"):
                    row.update(x=0, y=0, z=9.80665 if stream == "accel" else .001)
                elif i in (0, 10, 20):
                    row["image_file"] = f"{stream}-{i}.pgm"
                    (self.raw / row["image_file"]).write_bytes(b"P5\n848 480\n255\n" + bytes([128]) * (848 * 480))
                self.rows.append(row)

    def save(self):
        write_json(self.raw / "capture.json", self.meta)
        write_json(self.raw / "capture_config.yaml", self.cfg)
        with (self.raw / "samples.csv").open("w", newline="") as handle:
            writer = csv.DictWriter(handle, FIELDS)
            writer.writeheader()
            writer.writerows(self.rows)
        (self.raw / "capture.stdout").write_bytes(b"raw stdout\n\x00unaltered\n")
        (self.raw / "capture.stderr").write_bytes(b"raw stderr\n")

    def run_analysis(self):
        self.save()
        return analyze(self.raw, self.cfg)

    def criterion(self, result, identifier):
        return next(c for c in result["criteria"] if c["id"] == identifier)

    def test_pass_contract_and_no_unmapped_latency(self):
        result = self.run_analysis()
        self.assertEqual(result["status"], "PASS", result["criteria"])
        for key in ("check", "tool_version", "started_utc", "ended_utc", "host", "criteria", "raw_files", "notes"):
            self.assertIn(key, result)
        for item in result["criteria"]:
            for key in ("description", "measured", "unit", "threshold", "status", "evidence", "threshold_source"):
                self.assertIn(key, item)
            self.assertTrue((self.raw / item["evidence"]).is_file())
            self.assertIsInstance(item["threshold"], str)
            self.assertIsInstance(item["threshold_source"], str)
        for item in result["raw_files"]:
            self.assertEqual(item["sha256"], hashlib.sha256((self.raw / item["path"]).read_bytes()).hexdigest())
        metrics = json.loads((self.raw / "metrics.json").read_text())
        self.assertIsNone(metrics["latency"]["host_receipt_latency_s"])
        self.assertEqual(metrics["latency"]["status"], "UNKNOWN")
        self.assertIn("population_std_interval", metrics["streams"]["gyro"]["receipt_jitter_s"])

    def test_wrong_serial_firmware_usb_fail(self):
        self.meta["identity"] = {"serial": "99003", "firmware": "bad", "usb_type": "2.1"}
        result = self.run_analysis()
        for key in ("serial", "firmware", "usb_type"):
            self.assertEqual(self.criterion(result, f"identity.{key}")["status"], "FAIL")

    def test_frame_gap_and_shortfall_fail(self):
        self.rows = [r for r in self.rows if not (r["stream"] == "gyro" and 20 <= r["frame_number"] < 50)]
        result = self.run_analysis()
        for key in ("gyro.drops", "gyro.rate", "gyro.receipt_gap", "sample_accounting"):
            self.assertEqual(self.criterion(result, key)["status"], "FAIL")

    def test_boundary_stall_cannot_hide_behind_full_rate(self):
        for row in self.rows:
            if row["stream"] == "gyro":
                row["host_clock_monotonic_ns"] = 10**16 + round((row["frame_number"] + .5) * 700000000 / 200)
        result = self.run_analysis()
        self.assertEqual(self.criterion(result, "gyro.rate")["status"], "PASS")
        self.assertEqual(self.criterion(result, "gyro.receipt_gap")["status"], "FAIL")

    def test_duplicate_reversed_and_nonincreasing_frames(self):
        gyro = [r for r in self.rows if r["stream"] == "gyro"]
        gyro[1]["sdk_timestamp_ms"] = gyro[0]["sdk_timestamp_ms"]
        gyro[1]["sdk_global_system_timestamp_ms"] = gyro[0]["sdk_timestamp_ms"]
        gyro[2]["hardware_timestamp_ms"] = -1
        gyro[2]["frame_number"] = gyro[1]["frame_number"]
        result = self.run_analysis()
        self.assertEqual(self.criterion(result, "gyro.timestamps")["status"], "FAIL")
        self.assertEqual(self.criterion(result, "gyro.drops")["status"], "FAIL")

    def test_domain_transition_has_no_cross_clock_delta(self):
        gyro = [r for r in self.rows if r["stream"] == "gyro"]
        gyro[1].update(sdk_timestamp_domain="hardware_clock", sdk_timestamp_ms=5,
                       hardware_timestamp_ms=5, sdk_global_system_timestamp_ms="",
                       hardware_timestamp_source="SDK hardware ms")
        result = self.run_analysis()
        criterion = self.criterion(result, "gyro.timestamps")
        self.assertEqual(criterion["status"], "FAIL")
        self.assertEqual(criterion["measured"]["clocks"]["sdk/hardware_clock"]["reversals"], 0)
        self.assertEqual(criterion["measured"]["clocks"]["sdk/global_time"]["reversals"], 0)

    def test_unknown_domain_and_clock_field_inconsistency(self):
        self.rows[1]["sdk_timestamp_domain"] = "unknown"
        self.rows[2]["sdk_global_system_timestamp_ms"] = 1
        result = self.run_analysis()
        self.assertEqual(self.criterion(result, "ir_left.timestamps")["status"], "FAIL")

    def test_host_realtime_reversal_is_separate_from_monotonic(self):
        self.rows[1]["host_clock_realtime_ns"] = self.rows[0]["host_clock_realtime_ns"] - 1
        result = self.run_analysis()
        clocks = self.criterion(result, "ir_left.timestamps")["measured"]["clocks"]
        self.assertEqual(clocks["host/CLOCK_REALTIME_ns"]["reversals"], 1)
        self.assertEqual(clocks["host/CLOCK_MONOTONIC_ns"]["reversals"], 0)
        self.assertEqual(self.criterion(result, "ir_left.timestamps")["status"], "FAIL")

    def test_stereo_tolerance_and_missing_pair(self):
        right = [r for r in self.rows if r["stream"] == "ir_right"]
        right[5]["sdk_timestamp_ms"] += .1
        right[5]["sdk_global_system_timestamp_ms"] += .1
        self.rows.remove(right[6])
        result = self.run_analysis()
        self.assertEqual(self.criterion(result, "stereo")["status"], "FAIL")
        self.assertEqual(self.criterion(result, "stereo")["measured"]["unpaired"], 1)

    def test_stereo_domain_mismatch_never_subtracts_clocks(self):
        for row in self.rows:
            if row["stream"] == "ir_right":
                row["sdk_timestamp_domain"] = "system_time"
        result = self.run_analysis()
        pairing = self.criterion(result, "stereo")
        self.assertEqual(pairing["status"], "FAIL")
        self.assertIsNone(pairing["measured"]["max_sdk_delta_ms"])

    def test_static_confirmation_required_and_fail_beats_blocked(self):
        self.meta["static_confirmed"] = False
        result = self.run_analysis()
        self.assertEqual(result["status"], "BLOCKED")
        self.assertEqual(self.criterion(result, "gyro.static")["status"], "BLOCKED")
        self.meta["identity"]["serial"] = "wrong"
        self.assertEqual(self.run_analysis()["status"], "FAIL")

    def test_static_plausibility_fail_and_noise_not_calibration(self):
        for row in self.rows:
            if row["stream"] == "gyro":
                row["x"] = .2 + (.1 if row["frame_number"] % 2 else -.1)
            if row["stream"] == "accel":
                row["z"] = 8 + (1 if row["frame_number"] % 2 else -1)
        result = self.run_analysis()
        for stream in ("gyro", "accel"):
            self.assertEqual(self.criterion(result, f"{stream}.static")["status"], "FAIL")

    def test_zero_and_saturated_images_fail(self):
        for name, pixel in (("ir_left-0.pgm", 0), ("ir_right-0.pgm", 255)):
            (self.raw / name).write_bytes(b"P5\n848 480\n255\n" + bytes([pixel]) * (848 * 480))
        result = self.run_analysis()
        for stream in ("ir_left", "ir_right"):
            self.assertEqual(self.criterion(result, f"{stream}.ir_content")["status"], "FAIL")

    def test_missing_ir_samples_blocked_and_bad_pgm_fail(self):
        for row in self.rows:
            if row["stream"] == "ir_left":
                row["image_file"] = ""
        result = self.run_analysis()
        self.assertEqual(self.criterion(result, "ir_left.ir_content")["status"], "BLOCKED")
        (self.raw / "ir_right-0.pgm").write_bytes(b"broken")
        self.assertEqual(self.criterion(self.run_analysis(), "ir_right.ir_content")["status"], "FAIL")

    def test_nan_truncated_csv_and_unsafe_image_fail(self):
        self.rows[0]["sdk_timestamp_ms"] = "nan"
        result = self.run_analysis()
        self.assertEqual(self.criterion(result, "csv")["status"], "FAIL")
        self.rows[0]["sdk_timestamp_ms"] = 0
        self.rows[0]["image_file"] = "../escape.pgm"
        self.assertEqual(self.criterion(self.run_analysis(), "csv")["status"], "FAIL")
        self.save()
        with (self.raw / "samples.csv").open("a") as f:
            f.write("gyro,1,2\n")
        self.assertEqual(self.criterion(analyze(self.raw, self.cfg), "csv")["status"], "FAIL")

    def test_missing_raw_blocked_and_malformed_metadata_fail(self):
        result = analyze(self.raw, self.cfg)
        self.assertEqual(result["status"], "BLOCKED")
        (self.raw / "capture.json").write_text('{"profiles": [7]}')
        self.assertEqual(analyze(self.raw, self.cfg)["status"], "FAIL")

    def test_short_duration_wrong_profiles_emitter_and_overflow_fail(self):
        self.cfg["limits"]["qualification_duration_s"]["value"] = 60
        self.meta["profiles"][0]["width"] = 640
        self.meta["emitter_readback"] = [1]
        self.meta["queue_overflow"] = 1
        result = self.run_analysis()
        for key in ("duration", "profiles", "emitter", "capture"):
            self.assertEqual(self.criterion(result, key)["status"], "FAIL")

    def test_replay_cli_immutable_inputs_raw_logs_and_exit_codes(self):
        self.save()
        before = {p.name: p.read_bytes() for p in self.raw.iterdir()}
        out = self.base / "out"
        cli = [sys.executable, str(HERE / "check.py"), "--analyze-only", str(self.raw), "--outdir", str(out)]
        proc = subprocess.run(cli, capture_output=True, text=True)
        self.assertEqual(proc.returncode, 0, proc.stderr + proc.stdout)
        self.assertEqual(before, {p.name: p.read_bytes() for p in self.raw.iterdir()})
        self.assertEqual((out / "capture.stdout").read_bytes(), before["capture.stdout"])
        self.assertEqual(subprocess.run(cli, capture_output=True).returncode, 3)
        cli[-1] = str(self.base / "refused-static")
        self.assertEqual(subprocess.run(cli + ["--static"], capture_output=True).returncode, 3)
        self.meta["static_confirmed"] = False
        self.save()
        cli[-1] = str(self.base / "blocked")
        self.assertEqual(subprocess.run(cli, capture_output=True).returncode, 2)
        self.meta["identity"]["serial"] = "wrong"
        self.save()
        cli[-1] = str(self.base / "failed")
        self.assertEqual(subprocess.run(cli, capture_output=True).returncode, 1)

    def test_config_requires_sources_and_finite_numeric_values(self):
        config = self.base / "bad.yaml"
        self.cfg["limits"]["gyro_mean_norm_rad_s"]["source"] = ""
        write_json(config, self.cfg)
        with self.assertRaises(ValueError):
            load_config(config)

        self.cfg["limits"]["gyro_mean_norm_rad_s"]["source"] = "PROPOSED test"
        self.cfg["capture"]["poll_s"]["value"] = 0
        write_json(config, self.cfg)
        with self.assertRaises(ValueError):
            load_config(config)

    def test_cli_usage_errors_use_contract_exit_three(self):
        for arguments in ([], ["--unknown-option"], ["--capture", "--outdir", "somewhere", "--seconds", "not-a-number"]):
            proc = subprocess.run([sys.executable, str(HERE / "check.py"), *arguments], capture_output=True, text=True)
            self.assertEqual(proc.returncode, 3, proc.stderr)

    def test_real_old_unit_fixture_cli_and_provenance(self):
        fixture = HERE / "tests/fixtures/old-unit-smoke"
        provenance = json.loads((fixture / "PROVENANCE.json").read_text())
        self.assertEqual(provenance["sdk_serial"], "99003")
        for f in provenance["files"]:
            self.assertEqual(hashlib.sha256((fixture / f["file"]).read_bytes()).hexdigest(), f["sha256"])
        out = self.base / "real-old"
        proc = subprocess.run([sys.executable, str(HERE / "check.py"), "--analyze-only", str(fixture), "--outdir", str(out)], capture_output=True, text=True)
        self.assertEqual(proc.returncode, 1, proc.stdout + proc.stderr)
        result = json.loads((out / "result.json").read_text())
        self.assertEqual(self.criterion(result, "identity.serial")["status"], "FAIL")
        self.assertEqual(self.criterion(result, "timestamps")["status"], "BLOCKED")
        metrics = json.loads((out / "metrics.json").read_text())
        self.assertGreater(metrics["vendor_relative_timer_diagnostics"]["Gyro"]["count"], 3000)
        self.assertIsNone(metrics["host_clock_monotonic_ns"])
        self.assertIsNone(metrics["latency"]["host_receipt_latency_s"])

    def test_parent_timeout_preserves_partial_metadata_and_logs(self):
        write_json(self.raw / "capture_config.yaml", self.cfg)
        write_json(self.raw / "capture.json", {"complete": False, "marker": "partial raw metadata"})
        child = mock.Mock()
        child.wait.side_effect = [subprocess.TimeoutExpired("sdk", 1), -9]
        with mock.patch.object(check.subprocess, "Popen", return_value=child):
            check.capture(self.raw, self.cfg, 1, True)
        child.kill.assert_called_once()
        self.assertEqual(json.loads((self.raw / "capture.partial.json").read_text())["marker"], "partial raw metadata")
        self.assertEqual(analyze(self.raw, self.cfg)["status"], "FAIL")

    def test_sdk_missing_is_blocked_without_hardware(self):
        with mock.patch.dict(sys.modules, {"pyrealsense2": None}), contextlib.redirect_stdout(io.StringIO()):
            code = capture.run(self.raw, self.cfg, 1, True)
        self.assertEqual(code, 2)
        self.assertEqual(analyze(self.raw, self.cfg)["status"], "BLOCKED")


class FakeProfile:
    def __init__(self, stream, fps, index=0):
        self.kind, self.rate, self.index = stream, fps, index
    def stream_type(self): return self.kind
    def stream_index(self): return self.index
    def fps(self): return self.rate
    def format(self): return "y8" if self.kind == "infrared" else "motion_xyz32f"
    def as_video_stream_profile(self): return self
    def width(self): return 848
    def height(self): return 480


class FakeFrame:
    def __init__(self, profile, number): self.profile, self.number = profile, number
    def get_profile(self): return self.profile
    def get_frame_timestamp_domain(self): return "timestamp_domain.hardware_clock"
    def get_timestamp(self): return self.number * 1000 / self.profile.fps()
    def get_frame_number(self): return self.number
    def as_motion_frame(self): return self
    def get_motion_data(self):
        return type("Vector", (), {"x": 0, "y": 0, "z": 9.80665 if self.profile.kind == "accel" else .001})()
    def as_video_frame(self): return self
    def get_width(self): return 848
    def get_height(self): return 480
    def get_stride_in_bytes(self): return 848
    def get_data(self): return bytes([128]) * (848 * 480)


class FakeSensor:
    def __init__(self, accel=100):
        self.profiles = [FakeProfile("infrared", 30, 1), FakeProfile("infrared", 30, 2), FakeProfile("gyro", 200), FakeProfile("accel", accel)]
        self.emitter = 1
        self.opened = False
        self.stopped = threading.Event()
    def get_stream_profiles(self): return self.profiles
    def supports(self, key): return True
    def get_option(self, key): return self.emitter if key == "emitter" else 1
    def set_option(self, key, value): self.emitter = value
    def open(self, profiles):
        if self.emitter != 0: raise RuntimeError("Emitter must be off before opening")
        self.opened = True
    def start(self, callback):
        def send():
            start = time.monotonic()
            numbers = [0] * len(self.profiles)
            while not self.stopped.is_set():
                elapsed = time.monotonic() - start
                for i, p in enumerate(self.profiles):
                    if elapsed >= (numbers[i] + .5) / p.fps():
                        callback(FakeFrame(p, numbers[i]))
                        numbers[i] += 1
                self.stopped.wait(.0005)
        self.thread = threading.Thread(target=send)
        self.thread.start()
    def stop(self):
        self.stopped.set()
        self.thread.join()
    def close(self): self.opened = False


class FakeSDK:
    stream = type("Streams", (), {"infrared": "infrared", "gyro": "gyro", "accel": "accel"})
    camera_info = type("Info", (), {"serial_number": "serial", "firmware_version": "firmware", "usb_type_descriptor": "usb"})
    option = type("Option", (), {"global_time_enabled": "global", "emitter_enabled": "emitter"})
    log_severity = type("Log", (), {"info": "info"})
    def __init__(self, accel=100, absent=False):
        self.sensor, self.absent = FakeSensor(accel), absent
    def log_to_file(self, severity, path): Path(path).write_text("SYNTHETIC SDK log\n")
    def context(self, settings):
        if json.loads(settings).get("dds", {}).get("enabled") is not False:
            raise ValueError("DDS discovery must be disabled")
        return self
    def query_devices(self): return [] if self.absent else [self]
    def supports(self, key): return True
    def get_info(self, key): return {"serial": "99005", "firmware": "LOCAL_FIRMWARE_VERSION", "usb": "3.2"}[key]
    def query_sensors(self): return [self.sensor]


class CaptureTests(unittest.TestCase):
    def test_sdk_global_timestamp_does_not_become_fake_hardware_time(self):
        class Frame:
            def get_profile(self): return FakeProfile("gyro", 200)
            def get_frame_timestamp_domain(self): return "timestamp_domain.global_time"
            def get_timestamp(self): return 1800000000000
            def supports_frame_metadata(self, key): return False
            def get_frame_number(self): return 1
            def as_motion_frame(self): return self
            def get_motion_data(self):
                return type("V", (), {"x": 0, "y": 0, "z": 0})()
        rs = FakeSDK()
        rs.frame_metadata_value = type("Metadata", (), {"frame_timestamp": "frame_timestamp"})
        row = capture.sample_row(rs, Frame(), 10**16, 1800000000000000000)
        self.assertEqual(row["hardware_timestamp_ms"], "")
        self.assertIn("UNKNOWN", row["hardware_timestamp_source"])
        self.assertEqual(row["sdk_global_system_timestamp_ms"], row["sdk_timestamp_ms"])
        self.assertEqual(row["host_clock_monotonic_ns"], 10**16)

    def test_queue_overflow_is_a_failure(self):
        cfg = load_config(HERE / "thresholds.yaml")
        class FullQueue(capture.queue.Queue):
            def put_nowait(self, item): raise capture.queue.Full
        with tempfile.TemporaryDirectory(dir=HERE / "tests", prefix=".tmp-") as tmp:
            rs = FakeSDK()
            with mock.patch.object(capture.queue, "Queue", FullQueue), contextlib.redirect_stdout(io.StringIO()):
                code = capture.run(Path(tmp), cfg, .1, True, rs=rs)
            self.assertEqual(code, 1)
            meta = json.loads((Path(tmp) / "capture.json").read_text())
            self.assertGreater(meta["queue_overflow"], 0)
            self.assertFalse(meta["complete"])
            self.assertEqual(rs.sensor.emitter, 1)

    def test_fake_sdk_complete_capture_no_imu_decimation_and_emitter_restore(self):
        cfg = load_config(HERE / "thresholds.yaml")
        cfg["limits"]["qualification_duration_s"]["value"] = 1
        with tempfile.TemporaryDirectory(dir=HERE / "tests", prefix=".tmp-") as tmp:
            out = Path(tmp)
            rs = FakeSDK(accel=200)
            with contextlib.redirect_stdout(io.StringIO()):
                code = capture.run(out, cfg, 1, True, rs=rs)
            self.assertEqual(code, 0)
            self.assertEqual(rs.sensor.emitter, 1)
            self.assertFalse(rs.sensor.opened)
            result = analyze(out, cfg)
            self.assertEqual(result["status"], "PASS", result["criteria"])
            metadata = json.loads((out / "capture.json").read_text())
            self.assertGreater(metadata["received_counts"]["gyro"], 180)
            self.assertGreater(metadata["received_counts"]["accel"], 180)
            self.assertEqual(next(p for p in metadata["profiles"] if p["stream"] == "accel")["fps"], 200)

    def test_absent_device_and_unsupported_profile_are_blocked(self):
        cfg = load_config(HERE / "thresholds.yaml")
        for rs in (FakeSDK(absent=True), FakeSDK(accel=400)):
            with self.subTest(absent=rs.absent), tempfile.TemporaryDirectory(dir=HERE / "tests", prefix=".tmp-") as tmp:
                with contextlib.redirect_stdout(io.StringIO()):
                    code = capture.run(Path(tmp), cfg, 1, True, rs=rs)
                self.assertEqual(code, 2)
                self.assertFalse(rs.sensor.opened)
                self.assertEqual(rs.sensor.emitter, 1)

    def test_shutdown_error_and_callback_error_are_failures(self):
        cfg = load_config(HERE / "thresholds.yaml")
        for break_callback in (False, True):
            with self.subTest(callback=break_callback), tempfile.TemporaryDirectory(dir=HERE / "tests", prefix=".tmp-") as tmp:
                rs = FakeSDK()
                if break_callback:
                    patch = mock.patch.object(capture, "sample_row", side_effect=ValueError("bad callback"))
                else:
                    original_close = rs.sensor.close
                    def bad_close():
                        original_close()
                        raise RuntimeError("shutdown failure")
                    patch = mock.patch.object(rs.sensor, "close", side_effect=bad_close)
                with patch, contextlib.redirect_stdout(io.StringIO()):
                    code = capture.run(Path(tmp), cfg, .1, True, rs=rs)
                self.assertEqual(code, 1)
                self.assertEqual(rs.sensor.emitter, 1)
                self.assertFalse(json.loads((Path(tmp) / "capture.json").read_text())["complete"])


if __name__ == "__main__":
    unittest.main()
