#!/usr/bin/env python3
"""Read-only Pi power/USB diagnostics and portable replay. Python 3.11+, stdlib."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import platform
import re
import shutil
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from tools.runtime_observer.resource_sampler import host_now_ns, throttling, UNAVAILABLE

VERSION = "1.0.0"
DEFAULT_CONFIG = Path(__file__).with_name("thresholds.yaml")
EXIT = {"PASS": 0, "FAIL": 1, "BLOCKED": 2}


def utc():
    return datetime.now(timezone.utc).isoformat()


def stamp():
    return {"clock_monotonic_ns": host_now_ns(), "clock_realtime_utc": utc()}


def write_json(path, obj):
    path.write_text(json.dumps(obj, indent=2, allow_nan=False) + "\n")


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_config(path):
    # JSON is a YAML 1.2 subset. Avoid an installation dependency on the Pi.
    cfg = json.loads(path.read_text())
    for key in ("window_s", "sample_interval_s", "max_sample_gap_s", "command_timeout_s",
                "load_stop_grace_s", "min_load_coverage_fraction", "min_psu_current_ma",
                "min_ext5v_v", "max_temperature_c", "usb_min_speed_mbps"):
        entry = cfg[key]
        value = entry["value"]
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"{key}: expected finite positive number")
        if not (entry.get("source") or entry.get("rationale")):
            raise ValueError(f"{key}: missing threshold source/rationale")
    if cfg["min_load_coverage_fraction"]["value"] > 1:
        raise ValueError("min_load_coverage_fraction must be <=1")
    if type(cfg["require_5a"]["value"]) is not bool:
        raise ValueError("require_5a must be boolean")
    if cfg["allowed_fault_count"]["value"] != 0:
        raise ValueError("phase-1 requires allowed_fault_count=0")
    for key in ("require_5a", "allowed_fault_count"):
        if not (cfg[key].get("source") or cfg[key].get("rationale")):
            raise ValueError(f"{key}: missing threshold source/rationale")
    for key in ("usb_identity", "throttle_bits", "kernel_patterns"):
        if not cfg[key].get("source"):
            raise ValueError(f"{key}: missing source")
    for key in ("vid", "pid"):
        if not re.fullmatch(r"[0-9a-fA-F]{4}", cfg["usb_identity"][key]):
            raise ValueError("USB IDs must be four hex digits")
    patterns = cfg["kernel_patterns"]["patterns"]
    if not patterns:
        raise ValueError("kernel_patterns must not be empty")
    for pattern in patterns.values():
        re.compile(pattern, re.I)
    return cfg


class Collector:
    def __init__(self, out, cfg, sys_root=Path("/sys"), proc_root=Path("/proc")):
        self.out, self.cfg, self.sys, self.proc = out, cfg, sys_root, proc_root
        (out / "raw").mkdir()
        self.index = 0

    def artifact(self, label, payload):
        self.index += 1
        path = f"raw/{self.index:05d}-{label}"
        (self.out / path).write_bytes(payload)
        return path

    def command(self, argv, label):
        info = {"argv": argv, "started": stamp(), "returncode": None, "error": None}
        try:
            result = subprocess.run(argv, capture_output=True,
                                    timeout=self.cfg["command_timeout_s"]["value"], check=False)
            stdout, stderr = result.stdout, result.stderr
            info["returncode"] = result.returncode
        except subprocess.TimeoutExpired as exc:
            stdout, stderr = exc.stdout or b"", exc.stderr or b""
            info["error"] = "timeout"
        except OSError as exc:
            stdout, stderr = b"", b""
            info["error"] = str(exc)
        info.update(ended=stamp(), stdout=self.artifact(label + ".stdout", stdout),
                    stderr=self.artifact(label + ".stderr", stderr))
        return info

    def file(self, path, label):
        info = {"source_path": str(path), "started": stamp(), "error": None}
        try:
            raw = path.read_bytes()
        except OSError as exc:
            raw = b""
            info["error"] = str(exc)
        info.update(path=self.artifact(label, raw), ended=stamp())
        return info

    def sample(self, phase):
        sample = {"phase": phase, **stamp()}
        for key, cmd in (("throttled", ["get_throttled"]), ("temperature", ["measure_temp"]),
                         ("volts", ["measure_volts"]),
                         ("pmic", ["pmic_read_adc", "EXT5V_V"])):
            sample[key] = self.command(["vcgencmd", *cmd], key)
        sample["thermal"] = []
        for zone in sorted((self.sys / "class/thermal").glob("thermal_zone*")):
            sample["thermal"].append({"zone": zone.name,
                "type": self.file(zone / "type", "thermal-type"),
                "temperature": self.file(zone / "temp", "thermal-temp")})
        sample["ended"] = stamp()
        return sample

    def usb(self):
        result = {"stamp": stamp(), "devices": [], "error": None}
        root = self.sys / "bus/usb/devices"
        try:
            entries = sorted(root.iterdir())
        except OSError as exc:
            entries = []
            result["error"] = str(exc)
        for path in entries:
            if ":" in path.name or not (path / "idVendor").exists():
                continue
            fields = {key: self.file(path / key, "usb-" + key)
                      for key in ("idVendor", "idProduct", "speed", "busnum", "devnum", "devpath", "serial")}
            result["devices"].append({"sysfs_path": str(path), "port_path": path.name, **fields})
        result["tree"] = self.command(["lsusb", "-t"], "lsusb-tree")
        return result

    def supply(self):
        return {"config": self.command(["vcgencmd", "get_config", "usb_max_current_enable"], "usb-current"),
                "max_current": self.file(self.proc / "device-tree/chosen/power/max_current", "max-current.bin")}

    def kernel(self):
        return {"dmesg": self.command(["dmesg", "--raw"], "dmesg"),
                "journal": self.command(["journalctl", "-k", "-b", "--no-pager", "-o", "short-monotonic"], "journal"),
                "boot_id": self.file(self.proc / "sys/kernel/random/boot_id", "boot-id")}


def stop_load(process, grace):
    # Clean descendants even if the shell already exited. Only our new session is signalled.
    sent = process.poll() is None
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        code = process.wait(timeout=grace)
    except subprocess.TimeoutExpired:
        code = None
    # The leader can exit while children ignore TERM; always clean the owned group.
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    return process.wait() if code is None else code, sent


def collect(out, cfg, duration, interval, load_cmd, *, collector_cls=Collector):
    c = collector_cls(out, cfg)
    c.artifact("thresholds-at-capture.yaml", (json.dumps(cfg, indent=2) + "\n").encode())
    capture = {"schema_version": 1, "tool_version": VERSION, "started_utc": utc(),
               "host": {"hostname": platform.node(), "system": platform.system(),
                        "machine": platform.machine(), "python": platform.python_version()},
               "evidence_label": "MEASURED", "requested_window_s": duration,
               "requested_sample_interval_s": interval, "complete": False,
               "load": {"command": load_cmd, "mode": "load" if load_cmd else "idle",
                        "returncode": None, "error": None}, "samples": []}
    process = None
    handles = []
    try:
        capture["kernel_before"] = c.kernel()
        capture["supply_before"] = c.supply()
        capture["usb_before"] = c.usb()
        capture["samples"].append(c.sample("before"))
        capture["window_start"] = stamp()
        load = capture["load"]
        if load_cmd:
            for channel in ("stdout", "stderr"):
                path = f"raw/load.{channel}"
                load[channel] = path
                handles.append((out / path).open("wb"))
            try:
                process = subprocess.Popen(load_cmd, shell=True, executable="/bin/bash",
                    stdin=subprocess.DEVNULL, stdout=handles[0], stderr=handles[1], start_new_session=True)
                load["started"] = stamp()
            except OSError as exc:
                load["error"] = str(exc)
        deadline = capture["window_start"]["clock_monotonic_ns"] + int(duration * 1e9)
        next_ns = host_now_ns()
        while host_now_ns() < deadline:
            if process is not None and process.poll() is not None and "exited" not in load:
                load["exited"] = stamp()
            now = host_now_ns()
            if now >= next_ns:
                capture["samples"].append(c.sample("during"))
                next_ns += int(interval * 1e9)
                if next_ns < host_now_ns():
                    next_ns = host_now_ns() + int(interval * 1e9)
            else:
                time.sleep(min((next_ns - now) / 1e9, max(0, (deadline - now) / 1e9)))
        capture["window_end"] = stamp()
        capture["complete"] = True
    except (KeyboardInterrupt, OSError) as exc:
        capture["collection_error"] = type(exc).__name__ + ": " + str(exc)
        capture["window_end"] = stamp()
    finally:
        if process is not None:
            if process.poll() is not None and "exited" not in capture["load"]:
                capture["load"]["exited"] = stamp()
            code, sent = stop_load(process, cfg["load_stop_grace_s"]["value"])
            capture["load"].update(returncode=code, stopped_by_checker=sent, ended=stamp())
        for handle in handles:
            handle.close()
    capture["samples"].append(c.sample("after"))
    capture["usb_after"] = c.usb()
    capture["supply_after"] = c.supply()
    capture["kernel_after"] = c.kernel()
    capture["ended_utc"] = utc()
    capture["raw_files"] = [{"path": str(p.relative_to(out)), "sha256": sha(p)}
                            for p in sorted((out / "raw").iterdir())]
    write_json(out / "capture.json", capture)
    return capture


class Evidence:
    def __init__(self, out):
        self.out = out

    def bytes(self, entry, key="path"):
        if not entry or entry.get("error") or ("returncode" in entry and entry["returncode"] != 0):
            return None
        path = entry.get(key)
        if not isinstance(path, str) or not path.startswith("raw/") or ".." in Path(path).parts:
            return None
        try:
            return (self.out / path).read_bytes()
        except OSError:
            return None

    def text(self, entry, command=False):
        raw = self.bytes(entry, "stdout" if command else "path")
        return raw.decode("utf-8", errors="replace").strip() if raw is not None else None


def number(text, pattern):
    match = re.fullmatch(pattern, text or "")
    if not match:
        return None
    value = float(match[1])
    return value if math.isfinite(value) else None


def is_utc_timestamp(value):
    if not isinstance(value, str) or not value:
        return False
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError:
        return False
    return parsed.tzinfo is not None and parsed.utcoffset() == timezone.utc.utcoffset(parsed)


def analyze(out, capture, cfg):
    e = Evidence(out)
    criteria, details = [], {"decoded_samples": [], "usb": [], "supply": []}

    def add(cid, description, measured, unit, key, ok=None, fail=False, extra=""):
        entry = cfg[key]
        status = "FAIL" if fail else "PASS" if ok else "BLOCKED"
        criteria.append({"id": cid, "description": description, "measured": measured,
            "unit": unit, "threshold": f"{key}={entry.get('value', entry.get('patterns', 'documented policy'))} {extra}".strip(),
            "status": status, "evidence": "analysis.json",
            "threshold_source": entry.get("source", entry.get("rationale"))})

    integrity = []
    for item in capture.get("raw_files", []):
        raw = e.bytes({"path": item["path"]})
        if raw is None or hashlib.sha256(raw).hexdigest() != item["sha256"]:
            integrity.append(item["path"])
    details["integrity_errors"] = integrity
    add("raw_integrity", "Saved raw evidence matches capture SHA-256 manifest", integrity,
        "files", "allowed_fault_count", ok=not integrity and bool(capture.get("raw_files")))

    samples = capture.get("samples", [])
    flags, temperatures, voltage, temp_complete = [], [], [], []
    for sample in samples:
        decoded = throttling(e.text(sample.get("throttled"), True))
        flags.append(decoded)
        temp = number(e.text(sample.get("temperature"), True), r"temp=(-?[0-9]+(?:\.[0-9]+)?)'C")
        zone_values = []
        for zone in sample.get("thermal", []):
            val = number(e.text(zone.get("temperature")), r"(-?[0-9]+(?:\.[0-9]+)?)")
            zone_values.append(val / 1000 if val is not None else None)
        temp_complete.append(temp is not None and bool(zone_values) and all(v is not None for v in zone_values))
        temperatures.extend(v for v in [temp, *zone_values] if v is not None)
        adc_text = e.text(sample.get("pmic"), True)
        volts = []
        for line in (adc_text or "").splitlines():
            match = re.search(r"\bEXT5V_V\s*(?:(?:volt|voltage)\(\d+\)\s*)?(?:=|:)\s*([0-9]+(?:\.[0-9]+)?)\s*V\b", line)
            if match:
                volts.append(float(match[1]))
        voltage.append(min(volts) if volts else None)
        details["decoded_samples"].append({"phase": sample.get("phase"),
            "clock_monotonic_ns": sample.get("clock_monotonic_ns"),
            "clock_realtime_utc": sample.get("clock_realtime_utc"), "throttled": decoded,
            "vcgencmd_temperature_c": temp, "thermal_zone_temperature_c": zone_values,
            "ext5v_v": voltage[-1]})

    complete_flags = bool(flags) and all(f["throttled_raw"] != UNAVAILABLE for f in flags)
    baseline = flags[0]["undervoltage_occurred"] if flags else UNAVAILABLE
    now_faults = sum(f["undervoltage_now"] == 1 for f in flags)
    newly = any(a["undervoltage_occurred"] == 0 and b["undervoltage_occurred"] == 1
                for a, b in zip(flags, flags[1:]))
    history_cleared = any(a["undervoltage_occurred"] == 1 and b["undervoltage_occurred"] == 0
                          for a, b in zip(flags, flags[1:]))
    details["latched_under_voltage_at_baseline"] = baseline
    add("under_voltage", "No under-voltage now or newly latched since baseline",
        {"current_fault_samples": now_faults, "newly_occurred": newly, "baseline_latched": baseline},
        "samples/bits", "allowed_fault_count", ok=complete_flags and not history_cleared,
        fail=bool(now_faults or newly),
        extra=cfg["throttle_bits"]["source"])
    available_volts = [v for v in voltage if v is not None]
    add("ext5v", "Sampled PMIC EXT5V_V minimum (measure_volts is a core rail)",
        min(available_volts) if available_volts else None, "V", "min_ext5v_v",
        ok=bool(voltage) and all(v is not None for v in voltage),
        fail=any(v < cfg["min_ext5v_v"]["value"] for v in available_volts))
    add("temperature", f"Maximum across vcgencmd and all sysfs thermal zones; FAIL at or above the configured {cfg['max_temperature_c']['value']} degC throttling boundary",
        max(temperatures) if temperatures else None, "degC", "max_temperature_c",
        ok=bool(samples) and all(temp_complete),
        fail=any(v >= cfg["max_temperature_c"]["value"] for v in temperatures))

    times = [s.get("clock_monotonic_ns") for s in samples]
    valid_times = bool(times) and all(type(t) is int for t in times)
    gaps = [(b-a)/1e9 for a, b in zip(times, times[1:])] if valid_times else []
    phases = [s.get("phase") for s in samples]
    ws, we = capture.get("window_start", {}), capture.get("window_end", {})
    start, end = ws.get("clock_monotonic_ns"), we.get("clock_monotonic_ns")
    elapsed = (end-start)/1e9 if type(start) is int and type(end) is int else None
    coverage = (capture.get("complete") is True and valid_times and bool(gaps) and
                phases[0] == "before" and phases[-1] == "after" and "during" in phases and
                all(g > 0 for g in gaps) and max(gaps) <= cfg["max_sample_gap_s"]["value"] and
                all(is_utc_timestamp(s.get("clock_realtime_utc")) for s in samples) and elapsed is not None and
                elapsed >= capture["requested_window_s"] and times[0] <= start <= end <= times[-1])
    add("window_coverage", "Complete before/during/after observations in host CLOCK_MONOTONIC",
        {"elapsed_s": elapsed, "max_gap_s": max(gaps) if gaps else None}, "s",
        "max_sample_gap_s", ok=coverage)

    for label in ("supply_before", "supply_after"):
        supplied = capture.get(label, {})
        raw = e.bytes(supplied.get("max_current"))
        ma = int.from_bytes(raw, "big") if raw is not None and len(raw) == 4 else None
        text = e.text(supplied.get("config"), True)
        match = re.fullmatch(r"usb_max_current_enable=([01])", text or "")
        enabled = int(match[1]) if match else None
        config_entry = supplied.get("config") or {}
        config_malformed = (not config_entry.get("error") and
                            config_entry.get("returncode") == 0 and enabled is None)
        details["supply"].append({"phase": label, "max_current_ma": ma,
                                   "usb_max_current_enable": enabled,
                                   "config_malformed": config_malformed})
    require = cfg["require_5a"]["value"]
    supply_fail = require and any((s["max_current_ma"] is not None and s["max_current_ma"] < cfg["min_psu_current_ma"]["value"])
                                  or s["usb_max_current_enable"] == 0 for s in details["supply"])
    supply_known = all(s["max_current_ma"] is not None for s in details["supply"])
    supply_malformed = any(s["config_malformed"] for s in details["supply"])
    add("supply", "Declared PSU current; USB-current override cannot prove negotiation", details["supply"],
        "mA", "min_psu_current_ma", ok=not require or (supply_known and not supply_malformed), fail=supply_fail,
        extra=cfg["require_5a"].get("source", cfg["require_5a"].get("rationale", "")))

    usb_known, usb_fail, trees_known = True, False, True
    identities = []
    for label in ("usb_before", "usb_after"):
        snapshot = capture.get(label, {})
        matched, unknown = [], bool(snapshot.get("error"))
        for device in snapshot.get("devices", []):
            vid, pid = (e.text(device.get(k)) for k in ("idVendor", "idProduct"))
            if vid is None or pid is None:
                unknown = True
                continue
            if vid.lower() != cfg["usb_identity"]["vid"].lower() or pid.lower() != cfg["usb_identity"]["pid"].lower():
                continue
            speed = number(e.text(device.get("speed")), r"([0-9]+(?:\.[0-9]+)?)")
            matched.append({"sysfs_path": device["sysfs_path"], "port_path": device["port_path"],
                "speed_mbps": speed, **{k: e.text(device.get(k)) for k in ("busnum", "devnum", "devpath", "serial")}})
            if speed is None:
                unknown = True
            elif speed < cfg["usb_min_speed_mbps"]["value"]:
                usb_fail = True
        if len(matched) != 1:
            unknown = True
        trees_known &= bool(e.text(snapshot.get("tree"), True))
        usb_known &= bool(matched) and not unknown
        identities.append([(d["port_path"], d["serial"], d["devnum"]) for d in matched])
        details["usb"].append({"phase": label, "d435i_devices": matched, "incomplete": unknown})
    # A disappeared camera is unavailable evidence; kernel disconnect remains a FAIL.
    add("usb_link", "All detected 8086:0b3a devices at SuperSpeed; stable enumeration", details["usb"],
        "Mbit/s", "usb_min_speed_mbps", ok=usb_known and identities[0] == identities[1], fail=usb_fail)
    add("usb_topology", "lsusb -t recorded before and after", trees_known, "available",
        "usb_min_speed_mbps", ok=trees_known)

    # Reuse the existing classifier with an isolated module's patterns set by this task's config.
    spec = importlib.util.spec_from_file_location("power_usb_kernel_classifier",
        ROOT / "lab/sensor_validation/kernel_analysis.py")
    classifier = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(classifier)
    classifier.PATTERNS = {key: re.compile(value, re.I) for key, value in cfg["kernel_patterns"]["patterns"].items()}
    kernel_window, kernel_ok, kernel_source = "", False, None
    before, after = capture.get("kernel_before", {}), capture.get("kernel_after", {})
    boot_before, boot_after = e.text(before.get("boot_id")), e.text(after.get("boot_id"))
    for source in ("dmesg", "journal"):
        a, b = e.text(before.get(source), True), e.text(after.get(source), True)
        if a is None or b is None or not a or not b:
            continue
        timestamp = r"^(?:<\d+>)?\[\s*\d+\.\d+\]"
        if not re.search(timestamp, a, re.M) or not re.search(timestamp, b, re.M):
            continue
        stderr = e.bytes(after.get(source), "stderr") or b""
        if re.search(rb"not seeing messages|permission denied|no journal files|failed to", stderr, re.I):
            continue
        aa, bb = a.splitlines(), b.splitlines()
        # No UTC/kernel-clock arithmetic. Require exact retained prefix, else BLOCKED.
        if bb[:len(aa)] == aa:
            kernel_window = "\n".join(bb[len(aa):])
            kernel_ok = boot_before is not None and boot_before == boot_after
            kernel_source = source
            break
    matches = classifier.classify(kernel_window)["faults"]
    (out / "kernel-window.log").write_text(kernel_window + ("\n" if kernel_window else ""))
    write_json(out / "kernel-matches.json", matches)
    details["kernel"] = {"source": kernel_source, "complete": kernel_ok, "matches": matches,
                         "clock_policy": "Exact ordered snapshot-prefix difference; no cross-clock subtraction."}
    add("kernel_log", "No configured USB/UVC/xHCI/power faults in retained kernel window", len(matches),
        "matches", "kernel_patterns", ok=kernel_ok, fail=bool(matches))

    load = capture.get("load", {})
    mode = load.get("mode")
    load_ok, load_fail, fraction = mode == "idle", False, None
    if mode == "load" and not load.get("error") and load.get("started") and elapsed and elapsed > 0:
        ls = load["started"]["clock_monotonic_ns"]
        le = load.get("exited", we).get("clock_monotonic_ns")
        if type(le) is int:
            fraction = max(0, min(le, end) - max(ls, start)) / (end-start)
            natural_ok = load.get("returncode") == 0
            timed_stop = (load.get("stopped_by_checker") is True and
                          load.get("returncode") in (-signal.SIGTERM, 128 + signal.SIGTERM,
                                                     -signal.SIGKILL, 128 + signal.SIGKILL, 0))
            load_fail = (fraction < cfg["min_load_coverage_fraction"]["value"] or
                         (not timed_stop and load.get("returncode") not in (None, 0)))
            load_ok = (natural_ok or timed_stop) and not load_fail
    add("load_window", "Idle explicitly recorded, or load command covered window with acceptable exit",
        {"mode": mode, "returncode": load.get("returncode"), "coverage_fraction": fraction,
         "stopped_by_checker": load.get("stopped_by_checker", False)}, "fraction",
        "min_load_coverage_fraction", ok=load_ok, fail=load_fail)

    write_json(out / "analysis.json", details)
    status = "FAIL" if any(c["status"] == "FAIL" for c in criteria) else (
        "BLOCKED" if any(c["status"] == "BLOCKED" for c in criteria) else "PASS")
    notes = ["CALCULATED: status from saved observations and the effective thresholds.yaml.",
             "Idle check: no load command ran." if mode == "idle" else "Load command requested; see launch/exit evidence. Process lifetime does not prove streaming or useful CPU load.",
             "This software check is not a perception, calibration, electrical-safety or hardware gate PASS.",
             "Raw observations retain their original evidence labels, host and clock stamps; ADC calibration and unsampled transients UNKNOWN.",
             f"Capture evidence label: {capture.get('evidence_label', 'UNKNOWN')}.",
             "Other throttle bits are decoded and recorded, not acceptance criteria for this task."]
    if baseline == 1:
        notes.append("Under-voltage was already latched before the window; another historical-only event cannot be distinguished. Current bits, ADC and kernel logs remain active checks.")
    result = {"check": "power_usb", "tool_version": VERSION, "status": status,
        "started_utc": capture.get("started_utc"), "ended_utc": capture.get("ended_utc"),
        "host": capture.get("host", {}), "criteria": criteria, "notes": notes,
        "analysis_started_utc": utc(),
        "raw_files": [{"path": str(p.relative_to(out)), "sha256": sha(p)}
                      for p in sorted(out.rglob("*")) if p.is_file() and
                      (p.is_relative_to(out / "raw") or p.name in ("capture.json", "thresholds.yaml"))]}
    write_json(out / "result.json", result)
    print(f"power_usb {status} ({mode}; capture {capture.get('evidence_label', 'UNKNOWN')})")
    print(f"{'criterion':<22} status")
    for criterion in criteria:
        print(f"{criterion['id']:<22} {criterion['status']}")
    return EXIT[status]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True, help="New output directory; never overwritten")
    parser.add_argument("--analyze-only", type=Path, help="Replay saved capture without commands/hardware")
    parser.add_argument("--thresholds", type=Path, help="JSON-form YAML 1.2 config; replay defaults to saved config")
    parser.add_argument("--duration", type=float, help="Observation window seconds (default from config)")
    parser.add_argument("--interval", type=float, help="Sample interval seconds (default from config)")
    parser.add_argument("--load-cmd", help="Trusted bash command; terminated at window end")
    try:
        args = parser.parse_args(argv)
        source = args.analyze_only.resolve() if args.analyze_only else None
        if source and any(value is not None for value in (args.load_cmd, args.duration, args.interval)):
            raise ValueError("Replay cannot take --load-cmd, --duration or --interval")
        config_path = args.thresholds or (source / "thresholds.yaml" if source else DEFAULT_CONFIG)
        cfg = load_config(config_path)
        duration = args.duration if args.duration is not None else cfg["window_s"]["value"]
        interval = args.interval if args.interval is not None else cfg["sample_interval_s"]["value"]
        if not all(math.isfinite(v) and v > 0 for v in (duration, interval)):
            raise ValueError("duration and interval must be finite and positive")
        if args.load_cmd is not None and not args.load_cmd.strip():
            raise ValueError("load command must not be empty")
        if not source:
            for key, override, option in (("window_s", args.duration, "--duration"),
                                          ("sample_interval_s", args.interval, "--interval")):
                if override is not None:
                    cfg[key] = {"value": override,
                                "rationale": f"PROPOSED operator {option} override for this capture."}
        out = args.out.resolve()
        if out.exists() or (source and (out.is_relative_to(source) or source.is_relative_to(out))):
            raise ValueError("output must be new and must not overlap replay input")
        if config_path.resolve().is_relative_to(out):
            raise ValueError("output must not contain input config")
        if source:
            capture = json.loads((source / "capture.json").read_text())
            if capture.get("schema_version") != 1:
                raise ValueError("unsupported capture schema")
            if any(p.is_symlink() for p in source.rglob("*")):
                raise ValueError("replay input must not contain symlinks")
        out.mkdir(parents=True)
        write_json(out / "thresholds.yaml", cfg)
        if source:
            shutil.copyfile(source / "capture.json", out / "capture.json")
            if (source / "raw").is_dir():
                shutil.copytree(source / "raw", out / "raw")
        else:
            capture = collect(out, cfg, duration, interval, args.load_cmd)
        return analyze(out, capture, cfg)
    except SystemExit as exc:
        return 0 if exc.code == 0 else 3
    except (OSError, ValueError, KeyError, TypeError, AttributeError, re.error) as exc:
        print(f"power_usb: usage/internal error: {exc}", file=sys.stderr)
        return 3


if __name__ == "__main__":
    raise SystemExit(main())
