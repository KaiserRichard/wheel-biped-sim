"""Historical vendor CSV diagnostics; never invent missing clock metadata."""
import hashlib
import json
import re

from common import VERSION, host, legacy_timing, utc, value, write_json


def analyze_legacy(directory, cfg):
    started = utc()
    criteria = []
    metrics = {"evidence_label": "CALCULATED diagnostics from historical vendor CSV",
               "format": "rs-data-collect: vendor relative arrival timer is not proven CLOCK_MONOTONIC",
               "sdk_timestamp_domain": "UNKNOWN: not serialized; column title HW Timestamp is not domain evidence",
               "hardware_timestamp_ms": None, "host_clock_monotonic_ns": None,
               "latency": {"status": "UNKNOWN", "host_receipt_latency_s": None,
                           "reason": "No qualified clock mapping exists in these files; no cross-clock subtraction performed."}}

    def add(identifier, description, measured=None, status="BLOCKED", specs=()):
        criteria.append({"id": identifier, "description": description, "measured": measured, "unit": "",
                         "threshold": json.dumps({f"{g}.{k}": cfg[g][k]["value"] for g, k in specs}, sort_keys=True) if specs else "Required evidence available",
                         "threshold_source": "; ".join(f"{g}.{k}: {cfg[g][k]['source']}" for g, k in specs) or "SOURCE-SUPPORTED: phase-2 explicit clock/evidence contract",
                         "status": status, "evidence": "metrics.json"})
    try:
        command = json.loads((directory / "command.json").read_text())
        camera = command.get("camera", {})
        metrics["command"] = command
    except FileNotFoundError:
        camera = {}
    except (ValueError, OSError, AttributeError) as error:
        camera = {}
        add("legacy.command", "Historical command metadata malformed", str(error), "FAIL")
    for field, raw_key, key in (("serial", "device_serial", "serial"), ("firmware", "firmware", "firmware"),
                                ("usb_type", "sdk_usb_type_descriptor", "usb_type_regex")):
        actual = camera.get(raw_key)
        expected = value(cfg, "identity", key)
        passed = bool(re.fullmatch(expected, str(actual))) if field == "usb_type" else actual == expected
        add(f"identity.{field}", f"Historical SDK {field}; old unit does not transfer", actual,
            "BLOCKED" if actual is None else "PASS" if passed else "FAIL", [("identity", key)])
    try:
        capture = legacy_timing().read_capture(directory / "frames.csv")
        metrics["opened_profiles"] = capture["profiles"]
        metrics["vendor_relative_timer_diagnostics"] = {
            stream: legacy_timing().summarize(rows) for stream, rows in capture["streams"].items()}
        add("legacy.csv", "Historical raw CSV parsed using existing sdk_analysis.read_capture", metrics["vendor_relative_timer_diagnostics"], "PASS")
        # This parser deliberately supports the old three-stream profile only.
        add("profiles", "Historical 640x480 IR1-only/15 Hz profile differs from required stereo profile",
            capture["profiles"], "FAIL", [("profile", k) for k in ("width", "height", "ir_hz", "gyro_hz", "accel_hz_preference")])
    except (ValueError, OSError, KeyError) as error:
        add("legacy.csv", "Historical CSV invalid or unsupported", str(error), "FAIL")
    for identifier, description in (
        ("capture", "No finalized current-check capture metadata"),
        ("rates", "Host CLOCK_MONOTONIC full-window rates unavailable; vendor relative diagnostics only"),
        ("receipt_gaps", "Host CLOCK_MONOTONIC inter-arrival jitter unavailable; vendor timer is not assumed monotonic"),
        ("timestamps", "SDK timestamp domain and separate hardware/global/system fields not serialized"),
        ("stereo", "Right IR absent; historical CSV cannot qualify stereo pairing"),
        ("emitter", "Emitter OFF readback unavailable in these raw files"),
        ("static", "Current static confirmation and full-precision camera IMU evidence unavailable"),
        ("ir_content", "Current-profile raw Y8 samples unavailable"),
    ):
        add(identifier, description)
    add("latency_policy", "No receipt latency computed across unmapped clocks", metrics["latency"], "PASS")
    statuses = {c["status"] for c in criteria}
    result = {"check": "d435i_raw", "tool_version": VERSION,
              "status": "FAIL" if "FAIL" in statuses else "BLOCKED" if "BLOCKED" in statuses else "PASS",
              "started_utc": started, "ended_utc": utc(), "host": host(), "criteria": criteria,
              "raw_files": [{"path": p.name, "sha256": hashlib.sha256(p.read_bytes()).hexdigest()}
                            for p in sorted(directory.iterdir()) if p.is_file() and p.name not in ("result.json", "metrics.json")],
              "notes": ["MEASURED historical evidence: OLD SDK camera 99003. It cannot qualify current unit 99005.",
                        "CALCULATED vendor-timer diagnostics only; clock domains are UNKNOWN. A fixture run is not a gate PASS."]}
    write_json(directory / "metrics.json", metrics)
    write_json(directory / "result.json", result)
    return result
