"""Portable fixture/collector tests; no Pi, network, camera or serial access."""
from contextlib import redirect_stdout, redirect_stderr
import copy
import importlib.util
import io
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

TOOL = Path(__file__).resolve().parents[1] / "check_power_usb.py"
spec = importlib.util.spec_from_file_location("power_usb", TOOL)
p = importlib.util.module_from_spec(spec)
spec.loader.exec_module(p)


def fixture(out):
    """ASSUMED synthetic complete idle run. Numbers are test vectors, not policy."""
    (out / "raw").mkdir()
    shutil.copyfile(p.DEFAULT_CONFIG, out / "thresholds.yaml")
    counter = 0

    def entry(value, command=False):
        nonlocal counter
        counter += 1
        name = f"raw/f{counter}"
        (out / name).write_bytes(value if isinstance(value, bytes) else value.encode())
        return {"stdout" if command else "path": name, "error": None,
                **({"returncode": 0} if command else {})}

    def sample(phase, ns):
        return {"phase": phase, "clock_monotonic_ns": ns,
            "clock_realtime_utc": "2026-10-18T00:00:00+00:00",
            "throttled": entry("throttled=0x0\n", True),
            "temperature": entry("temp=50.5'C\n", True),
            "volts": entry("volt=0.8000V\n", True),
            "pmic": entry("3V7_WL_SW_A current(0)=0.001A\nEXT5V_V voltage(24)=5.100V\n", True),
            "thermal": [{"temperature": entry("51000\n"), "type": entry("cpu-thermal\n")} ]}

    def supply():
        return {"config": entry("usb_max_current_enable=1\n", True),
                "max_current": entry((5000).to_bytes(4, "big"))}

    def usb():
        d = {"sysfs_path": "/sys/bus/usb/devices/5-1", "port_path": "5-1"}
        for key, value in {"idVendor": "8086", "idProduct": "0b3a", "speed": "5000", "busnum": "5",
                           "devnum": "2", "devpath": "1", "serial": "SYNTHETIC"}.items():
            d[key] = entry(value + "\n")
        return {"devices": [d], "tree": entry("Bus 05: Video, 5000M\n", True)}

    old = "[    0.000000] Linux version fixture\n[    1.000000] usb 5-1: USB disconnect, device number 2\n"
    cap = {"schema_version": 1, "tool_version": p.VERSION, "evidence_label": "ASSUMED synthetic fixture",
        "started_utc": "2026-10-18T00:00:00+00:00", "ended_utc": "2026-10-18T00:00:02+00:00",
        "host": {"system": "fixture", "machine": "fixture"}, "complete": True,
        "requested_window_s": 1, "requested_sample_interval_s": 1,
        "window_start": {"clock_monotonic_ns": 500_000_000},
        "window_end": {"clock_monotonic_ns": 1_500_000_000},
        "samples": [sample("before", 0), sample("during", 1_000_000_000), sample("after", 2_000_000_000)],
        "supply_before": supply(), "supply_after": supply(), "usb_before": usb(), "usb_after": usb(),
        "kernel_before": {"dmesg": entry(old, True), "boot_id": entry("boot-fixture")},
        "kernel_after": {"dmesg": entry(old + "[    2.000000] benign message\n", True), "boot_id": entry("boot-fixture")},
        "load": {"mode": "idle", "command": None, "returncode": None}}
    refresh(out, cap)
    return cap


def refresh(out, cap):
    cap["raw_files"] = [{"path": str(f.relative_to(out)), "sha256": p.sha(f)}
                        for f in sorted((out / "raw").iterdir())]
    p.write_json(out / "capture.json", cap)


class AnalyzeTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.out = Path(self.temp.name)
        self.cap = fixture(self.out)
        self.cfg = p.load_config(p.DEFAULT_CONFIG)
        # Actual Pi 5 output includes descriptive text between EXT5V_V and '='.

    def tearDown(self):
        self.temp.cleanup()

    def replace(self, entry, value, command=True):
        (self.out / entry["stdout" if command else "path"]).write_bytes(
            value if isinstance(value, bytes) else value.encode())

    def run_analysis(self, expected, criterion=None, state=None, rehash=True):
        if rehash:
            refresh(self.out, self.cap)
        with redirect_stdout(io.StringIO()):
            code = p.analyze(self.out, self.cap, self.cfg)
        result = json.loads((self.out / "result.json").read_text())
        self.assertEqual(result["status"], expected)
        self.assertEqual(code, p.EXIT[expected])
        if criterion:
            self.assertEqual(next(c for c in result["criteria"] if c["id"] == criterion)["status"], state or expected)
        for c in result["criteria"]:
            self.assertTrue((self.out / c["evidence"]).is_file())
            self.assertTrue(c["threshold_source"])
        for raw in result["raw_files"]:
            self.assertEqual(p.sha(self.out / raw["path"]), raw["sha256"])
        return result

    def test_pass_and_contract(self):
        result = self.run_analysis("PASS")
        self.assertTrue(any("Idle check" in n for n in result["notes"]))
        self.assertEqual(json.loads((self.out / "kernel-matches.json").read_text()), [])

    def test_all_throttle_bits_and_existing_history(self):
        for sample in self.cap["samples"]:
            self.replace(sample["throttled"], "throttled=0xf000e")
        self.run_analysis("PASS", "under_voltage")
        decoded = json.loads((self.out / "analysis.json").read_text())["decoded_samples"][0]["throttled"]
        self.assertEqual(decoded["undervoltage_now"], 0)
        for key in p.throttling("throttled=0xf000f"):
            if key != "throttled_raw" and key != "undervoltage_now":
                self.assertEqual(decoded[key], 1)

    def test_current_under_voltage_fails_even_missing_telemetry(self):
        self.replace(self.cap["samples"][1]["throttled"], "throttled=0x50001")
        self.cap["samples"][2]["temperature"]["error"] = "missing tool"
        self.run_analysis("FAIL", "under_voltage")

    def test_new_historical_under_voltage_fails(self):
        self.replace(self.cap["samples"][1]["throttled"], "throttled=0x50000")
        self.run_analysis("FAIL", "under_voltage")

    def test_disappearing_history_blocks(self):
        self.replace(self.cap["samples"][0]["throttled"], "throttled=0x50000")
        self.run_analysis("BLOCKED", "under_voltage")

    def test_missing_or_malformed_throttle(self):
        for value in ("", "throttled=garbage", "vcgencmd unavailable"):
            with self.subTest(value=value):
                self.replace(self.cap["samples"][1]["throttled"], value)
                self.run_analysis("BLOCKED", "under_voltage")

    def test_low_ext5v_fails(self):
        self.replace(self.cap["samples"][1]["pmic"], "EXT5V_V voltage(24)=4.60V\n")
        self.run_analysis("FAIL", "ext5v")

    def test_core_voltage_never_substitutes_for_ext5v(self):
        self.replace(self.cap["samples"][1]["pmic"], "VDD_CORE_V=5.10V\n")
        self.run_analysis("BLOCKED", "ext5v")

    def test_ext5v_threshold_boundary_passes(self):
        self.replace(self.cap["samples"][1]["pmic"], "EXT5V_V=4.750V\n")
        self.run_analysis("PASS", "ext5v")

    def test_pmic_volt_channel_format(self):
        self.replace(self.cap["samples"][1]["pmic"], "EXT5V_V volt(24)=5.10000000V\n")
        self.run_analysis("PASS", "ext5v")

    def test_temperature_hot_sysfs(self):
        self.replace(self.cap["samples"][1]["thermal"][0]["temperature"], "81000", False)
        self.run_analysis("FAIL", "temperature")

    def test_temperature_hot_vcgencmd(self):
        self.replace(self.cap["samples"][1]["temperature"], "temp=80.1'C")
        self.run_analysis("FAIL", "temperature")

    def set_all_temperatures(self, degrees_c):
        for sample in self.cap["samples"]:
            self.replace(sample["temperature"], f"temp={degrees_c:.1f}'C")
            self.replace(sample["thermal"][0]["temperature"], f"{degrees_c * 1000:.0f}\n", False)

    def test_temperature_79_9_passes(self):
        self.set_all_temperatures(79.9)
        self.run_analysis("PASS", "temperature")

    def test_temperature_80_0_fails_inclusively(self):
        self.set_all_temperatures(80.0)
        self.run_analysis("FAIL", "temperature")

    def test_temperature_boundary_fails_for_either_source_alone(self):
        for source in ("vcgencmd", "sysfs"):
            with self.subTest(source=source):
                self.set_all_temperatures(79.9)
                sample = self.cap["samples"][1]
                if source == "vcgencmd":
                    self.replace(sample["temperature"], "temp=80.0'C")
                else:
                    self.replace(sample["thermal"][0]["temperature"], "80000\n", False)
                self.run_analysis("FAIL", "temperature")

    def test_temperature_missing_one_source_blocks(self):
        self.cap["samples"][1]["thermal"] = []
        self.run_analysis("BLOCKED", "temperature")

    def test_psu_3a_fails(self):
        self.replace(self.cap["supply_before"]["max_current"], (3000).to_bytes(4, "big"), False)
        self.run_analysis("FAIL", "supply")

    def test_psu_big_endian_no_ascii_or_usb_override_inference(self):
        self.replace(self.cap["supply_before"]["max_current"], "5000\n", False)
        self.run_analysis("BLOCKED", "supply")

    def test_no_negotiation_evidence_blocks_even_override_enabled(self):
        self.cap["supply_before"]["max_current"]["error"] = "not found"
        self.run_analysis("BLOCKED", "supply")

    def test_malformed_successful_usb_current_output_blocks(self):
        self.replace(self.cap["supply_before"]["config"], "usb_max_current_enable=2")
        self.run_analysis("BLOCKED", "supply")

    def test_low_usb_current_setting_fails(self):
        self.replace(self.cap["supply_after"]["config"], "usb_max_current_enable=0")
        self.run_analysis("FAIL", "supply")

    def test_optional_5a_policy(self):
        self.cfg["require_5a"]["value"] = False
        self.replace(self.cap["supply_before"]["max_current"], (3000).to_bytes(4, "big"), False)
        self.run_analysis("PASS", "supply")

    def test_5a_dt_sufficient_if_get_config_unsupported(self):
        self.cap["supply_before"]["config"]["returncode"] = 1
        self.run_analysis("PASS", "supply")

    def test_usb2_fails(self):
        self.replace(self.cap["usb_after"]["devices"][0]["speed"], "480", False)
        self.run_analysis("FAIL", "usb_link")

    def test_non_d435i_super_speed_device_cannot_pass(self):
        self.replace(self.cap["usb_after"]["devices"][0]["idProduct"], "9999", False)
        self.run_analysis("BLOCKED", "usb_link")

    def test_missing_camera_blocks(self):
        self.cap["usb_after"]["devices"] = []
        self.run_analysis("BLOCKED", "usb_link")

    def test_usb_reenumeration_blocks_without_kernel_proof(self):
        self.replace(self.cap["usb_after"]["devices"][0]["devnum"], "3", False)
        self.run_analysis("BLOCKED", "usb_link")

    def test_multiple_devices_one_usb2_fails(self):
        second = copy.deepcopy(self.cap["usb_after"]["devices"][0])
        second["speed"] = dict(second["speed"], path="raw/second-speed")
        (self.out / "raw/second-speed").write_text("480")
        self.cap["usb_after"]["devices"].append(second)
        self.run_analysis("FAIL", "usb_link")

    def test_multiple_matching_cameras_block_as_ambiguous(self):
        second = copy.deepcopy(self.cap["usb_after"]["devices"][0])
        second["port_path"] = "5-2"
        second["sysfs_path"] = "/sys/bus/usb/devices/5-2"
        self.cap["usb_after"]["devices"].append(second)
        self.run_analysis("BLOCKED", "usb_link")

    def test_missing_lsusb_blocks(self):
        self.cap["usb_before"]["tree"]["error"] = "command not found"
        self.run_analysis("BLOCKED", "usb_topology")

    def test_kernel_patterns_each_fail_and_save_matches(self):
        old = p.Evidence(self.out).text(self.cap["kernel_before"]["dmesg"], True)
        messages = ["usb 5-1: reset SuperSpeed USB device number 2 using xhci-hcd",
                    "usb 5-1: USB disconnect, device number 2", "uvcvideo: Failed to submit URB",
                    "xhci-hcd: host controller not responding", "hwmon: Under-voltage detected!",
                    "printk: 9 messages suppressed"]
        for message in messages:
            with self.subTest(message=message):
                self.replace(self.cap["kernel_after"]["dmesg"], old + "\n[    2.000000] " + message)
                self.run_analysis("FAIL", "kernel_log")
                matches = json.loads((self.out / "kernel-matches.json").read_text())
                self.assertEqual(matches[0]["text"], "[    2.000000] " + message)

    def test_kernel_permission_denied_blocks(self):
        self.cap["kernel_before"]["dmesg"]["returncode"] = 1
        self.run_analysis("BLOCKED", "kernel_log")

    def test_kernel_rollover_blocks(self):
        self.replace(self.cap["kernel_after"]["dmesg"], "[    9.000000] ring overwritten")
        self.run_analysis("BLOCKED", "kernel_log")

    def test_journal_fallback_and_no_utc_subtraction(self):
        self.cap["kernel_before"]["dmesg"]["error"] = "permission denied"
        self.cap["kernel_before"]["journal"] = self.cap["kernel_after"]["dmesg"]
        self.cap["kernel_after"]["journal"] = self.cap["kernel_after"]["dmesg"]
        self.run_analysis("PASS", "kernel_log")

    def test_journal_no_entries_cannot_pass(self):
        self.replace(self.cap["kernel_before"]["dmesg"], "-- No entries --")
        self.replace(self.cap["kernel_after"]["dmesg"], "-- No entries --")
        self.run_analysis("BLOCKED", "kernel_log")

    def test_changed_boot_blocks(self):
        self.replace(self.cap["kernel_after"]["boot_id"], "different-boot", False)
        self.run_analysis("BLOCKED", "kernel_log")

    def test_sample_coverage_blocks(self):
        self.cap["samples"][1]["clock_monotonic_ns"] = 10_000_000_000
        self.run_analysis("BLOCKED", "window_coverage")

    def test_non_utc_sample_clock_blocks(self):
        self.cap["samples"][1]["clock_realtime_utc"] = "not-a-utc-timestamp"
        self.run_analysis("BLOCKED", "window_coverage")

    def test_incomplete_capture_blocks(self):
        self.cap["complete"] = False
        self.run_analysis("BLOCKED", "window_coverage")

    def test_integrity_tamper_blocks(self):
        self.replace(self.cap["samples"][0]["volts"], "changed")
        self.run_analysis("BLOCKED", "raw_integrity", rehash=False)

    def test_missing_raw_blocks(self):
        (self.out / self.cap["samples"][0]["pmic"]["stdout"]).unlink()
        self.run_analysis("BLOCKED", "raw_integrity", rehash=False)

    def load(self, **kwargs):
        self.cap["load"] = {"mode": "load", "command": "synthetic", "returncode": 0,
            "started": {"clock_monotonic_ns": 500_000_000}, **kwargs}

    def test_load_clean_full_window_passes(self):
        self.load()
        self.run_analysis("PASS", "load_window")

    def test_load_natural_nonzero_fails(self):
        self.load(returncode=7)
        self.run_analysis("FAIL", "load_window")

    def test_load_clean_early_exit_fails_coverage(self):
        self.load(exited={"clock_monotonic_ns": 600_000_000})
        self.run_analysis("FAIL", "load_window")

    def test_load_planned_termination_records_exit(self):
        self.load(returncode=-15, stopped_by_checker=True)
        self.run_analysis("PASS", "load_window")

    def test_load_failure_racing_checker_stop_still_fails(self):
        self.load(returncode=7, stopped_by_checker=True)
        self.run_analysis("FAIL", "load_window")

    def test_load_launch_error_blocks(self):
        self.load(error="launch failed")
        self.run_analysis("BLOCKED", "load_window")

    def test_replay_immutable_and_cli_status(self):
        before = {str(f): p.sha(f) for f in self.out.rglob("*") if f.is_file()}
        with tempfile.TemporaryDirectory() as target:
            with redirect_stdout(io.StringIO()):
                with patch.object(p, "Collector", side_effect=AssertionError("no live collector in replay")):
                    code = p.main(["--analyze-only", str(self.out), "--out", str(Path(target) / "new")])
            self.assertEqual(code, 0)
        self.assertEqual(before, {str(f): p.sha(f) for f in self.out.rglob("*") if f.is_file()})

    def test_usage_errors_return_three_without_output(self):
        for extra in (["--duration", "nan"], ["--duration", "0"], ["--interval", "-1"], ["--load-cmd", ""],
                      ["--analyze-only", str(self.out), "--load-cmd", "echo no"], ["--unknown"]):
            with self.subTest(extra=extra), redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                self.assertEqual(p.main(["--out", str(self.out / "new"), *extra]), 3)
                self.assertFalse((self.out / "new").exists())

    def test_config_rejects_unjustified_nonfinite_thresholds_and_bad_patterns(self):
        for mutate in (lambda c: c["max_temperature_c"].pop("source"),
                       lambda c: c["sample_interval_s"].update(value=float("nan")),
                       lambda c: c["kernel_patterns"]["patterns"].update(bad="[")):
            bad = copy.deepcopy(self.cfg)
            mutate(bad)
            config = self.out / "bad.yaml"
            config.write_text(json.dumps(bad))
            with redirect_stderr(io.StringIO()):
                self.assertEqual(p.main(["--out", str(self.out / "new"), "--thresholds", str(config)]), 3)

    def test_existing_output_refused(self):
        with redirect_stderr(io.StringIO()):
            self.assertEqual(p.main(["--out", str(self.out)]), 3)


class CaptureTests(unittest.TestCase):
    def test_pmic_capture_selects_ext5v_channel(self):
        with tempfile.TemporaryDirectory() as tmp:
            c = p.Collector(Path(tmp), p.load_config(p.DEFAULT_CONFIG))
            commands = []

            def fake_command(argv, label):
                commands.append(argv)
                return {"argv": argv, "returncode": 0, "error": None,
                        "stdout": c.artifact(label + ".stdout", b""),
                        "stderr": c.artifact(label + ".stderr", b"")}

            with patch.object(c, "command", side_effect=fake_command):
                c.sample("before")
            self.assertIn(["vcgencmd", "pmic_read_adc", "EXT5V_V"], commands)

    def test_raw_stdout_stderr_verbatim_and_file_bytes(self):
        with tempfile.TemporaryDirectory() as tmp:
            c = p.Collector(Path(tmp), p.load_config(p.DEFAULT_CONFIG))
            result = c.command([sys.executable, "-c", "import os; os.write(1,b'\\xff\\r\\n'); os.write(2,b'err\\x00')"], "test")
            self.assertEqual((Path(tmp) / result["stdout"]).read_bytes(), b"\xff\r\n")
            self.assertEqual((Path(tmp) / result["stderr"]).read_bytes(), b"err\x00")
            self.assertEqual(result["returncode"], 0)
            self.assertIn("clock_monotonic_ns", result["started"])
            self.assertIn("clock_realtime_utc", result["ended"])

    def test_missing_command_and_timeout_record_blocking_errors(self):
        with tempfile.TemporaryDirectory() as tmp:
            c = p.Collector(Path(tmp), p.load_config(p.DEFAULT_CONFIG))
            with patch.object(p.subprocess, "run", side_effect=FileNotFoundError("missing")):
                self.assertIn("missing", c.command(["missing"], "missing")["error"])
            with patch.object(p.subprocess, "run", side_effect=subprocess.TimeoutExpired("x", 2, b"partial", b"err")):
                result = c.command(["x"], "timeout")
                self.assertEqual(result["error"], "timeout")
                self.assertEqual((Path(tmp) / result["stdout"]).read_bytes(), b"partial")

    def test_sysfs_enumeration_and_binary_supply(self):
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp)
            sysroot, procroot = base / "sys", base / "proc"
            d = sysroot / "bus/usb/devices/5-1"
            d.mkdir(parents=True)
            for field, value in {"idVendor": "8086", "idProduct": "0b3a", "speed": "5000", "serial": "fixture"}.items():
                (d / field).write_text(value)
            dt = procroot / "device-tree/chosen/power/max_current"
            dt.parent.mkdir(parents=True)
            dt.write_bytes((5000).to_bytes(4, "big"))
            c = p.Collector(base, p.load_config(p.DEFAULT_CONFIG), sysroot, procroot)
            with patch.object(c, "command", return_value={"error": "fake command"}):
                found, supply = c.usb(), c.supply()
            self.assertEqual(found["devices"][0]["port_path"], "5-1")
            self.assertEqual(p.Evidence(base).text(found["devices"][0]["speed"]), "5000")
            self.assertEqual(p.Evidence(base).bytes(supply["max_current"]), (5000).to_bytes(4, "big"))

    def collect_fake(self, out, cmd=None):
        class FakeCollector(p.Collector):
            # Complete synthetic capture, commands are not executed.
            def kernel(self):
                return {}
            def supply(self):
                return {}
            def usb(self):
                return {}
            def sample(self, phase):
                return {"phase": phase, **p.stamp()}
        cfg = p.load_config(p.DEFAULT_CONFIG)
        return p.collect(out, cfg, .10, .03, cmd, collector_cls=FakeCollector)

    def test_idle_capture_has_before_during_after(self):
        with tempfile.TemporaryDirectory() as tmp:
            cap = self.collect_fake(Path(tmp))
            self.assertTrue(cap["complete"])
            self.assertEqual(cap["load"]["mode"], "idle")
            self.assertEqual(cap["samples"][0]["phase"], "before")
            self.assertEqual(cap["samples"][-1]["phase"], "after")
            self.assertIn("during", [s["phase"] for s in cap["samples"]])

    def test_bounded_load_process_and_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            cap = self.collect_fake(Path(tmp), "printf load; exec sleep 30")
            self.assertTrue(cap["load"]["stopped_by_checker"])
            self.assertIsNotNone(cap["load"]["returncode"])
            self.assertEqual((Path(tmp) / cap["load"]["stdout"]).read_bytes(), b"load")

    def test_nonzero_load_exit_saved(self):
        with tempfile.TemporaryDirectory() as tmp:
            cap = self.collect_fake(Path(tmp), "printf problem >&2; exit 7")
            self.assertEqual(cap["load"]["returncode"], 7)
            self.assertFalse(cap["load"]["stopped_by_checker"])
            self.assertIn("exited", cap["load"])
            self.assertEqual((Path(tmp) / cap["load"]["stderr"]).read_bytes(), b"problem")


class RealEvidenceTests(unittest.TestCase):
    def test_real_snapshot_is_partial_blocked_not_new_fault(self):
        source = TOOL.parent / "tests/fixtures/real-partial"
        with tempfile.TemporaryDirectory() as tmp, redirect_stdout(io.StringIO()):
            target = Path(tmp) / "analysis"
            self.assertEqual(p.main(["--analyze-only", str(source), "--out", str(target)]), 2)
            result = json.loads((target / "result.json").read_text())
            details = json.loads((target / "analysis.json").read_text())
            self.assertEqual(details["decoded_samples"][0]["throttled"]["throttled_raw"], "0x50000")
            self.assertEqual(details["latched_under_voltage_at_baseline"], 1)
            self.assertEqual(details["decoded_samples"][0]["vcgencmd_temperature_c"], 50.5)
            self.assertEqual(result["status"], "BLOCKED")


if __name__ == "__main__":
    unittest.main()
