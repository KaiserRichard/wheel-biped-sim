"""Shared schema/configuration. thresholds.yaml is JSON, a YAML 1.2 subset."""
import importlib.util
import json
import math
import platform
import re
import sys
from datetime import datetime, timezone
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
VERSION = "1.0.0"
STREAMS = ("ir_left", "ir_right", "gyro", "accel")
FIELDS = (
    "stream", "frame_number", "sdk_timestamp_ms", "sdk_timestamp_domain",
    "hardware_timestamp_ms", "hardware_timestamp_source", "sdk_global_system_timestamp_ms",
    "host_clock_monotonic_ns", "host_clock_realtime_ns", "x", "y", "z", "image_file",
)
DOMAINS = {"hardware_clock", "global_time", "system_time"}


def utc():
    return datetime.now(timezone.utc).isoformat()


def host():
    return {"hostname": platform.node(), "system": platform.system(),
            "release": platform.release(), "machine": platform.machine(),
            "python": sys.version.split()[0]}


def write_json(path, obj):
    Path(path).write_text(json.dumps(obj, indent=2, allow_nan=False) + "\n")


def load_config(path):
    cfg = json.loads(Path(path).read_text())
    reference = json.loads((HERE / "thresholds.yaml").read_text())
    for group in ("identity", "profile", "capture", "limits"):
        for key, spec in reference[group].items():
            item = cfg[group][key]
            if not isinstance(item.get("source"), str) or not item["source"].strip():
                raise ValueError(f"Missing threshold source: {group}.{key}")
            value = item["value"]
            if isinstance(spec["value"], (int, float)):
                if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
                    raise ValueError(f"Invalid numeric threshold: {group}.{key}")
            elif not isinstance(value, type(spec["value"])):
                raise ValueError(f"Invalid threshold type: {group}.{key}")
    for group, keys in {
        "capture": ("queue_capacity", "ir_samples_per_stream"),
        "profile": ("width", "height", "ir_hz", "gyro_hz"),
        "limits": ("min_samples",),
    }.items():
        for key in keys:
            v = cfg[group][key]["value"]
            if type(v) is not int or v <= 0:
                raise ValueError(f"Expected positive integer {group}.{key}")
    for key in ("min_duration_s", "max_duration_s", "duration_s", "worker_grace_s", "poll_s"):
        if cfg["capture"][key]["value"] <= 0:
            raise ValueError(f"Expected positive capture.{key}")
    rates = cfg["profile"]["accel_hz_preference"]["value"]
    if not rates or any(type(x) is not int or x not in (100, 200) for x in rates):
        raise ValueError("Accel preferences must select 100 and/or 200 Hz")
    if not cfg["capture"]["min_duration_s"]["value"] <= cfg["capture"]["duration_s"]["value"] <= cfg["capture"]["max_duration_s"]["value"]:
        raise ValueError("Default duration outside capture bounds")
    if cfg["profile"]["emitter"]["value"] != 0:
        raise ValueError("This check requires emitter OFF")
    if cfg["limits"]["min_samples"]["value"] < 2:
        raise ValueError("At least two samples required")
    re.compile(cfg["identity"]["usb_type_regex"]["value"])
    return cfg


def value(cfg, group, key):
    return cfg[group][key]["value"]


def legacy_timing():
    """Reuse existing summarize(), with no old profile or acceptance thresholds."""
    path = ROOT / "lab/sensor_validation/sdk_analysis.py"
    spec = importlib.util.spec_from_file_location("d435i_existing_sdk_analysis", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module
