"""Shared stdlib-only plumbing for the rosbag qualification CLIs."""
from __future__ import annotations

from datetime import datetime, timezone
import argparse
import importlib.util
import math
import sys
import hashlib
import json
import os
from pathlib import Path
import platform
import shlex
import socket
import subprocess
from typing import Any


HERE = Path(__file__).resolve().parent
DEFAULT_CONFIG = HERE / "config.json"

# Reuse the ROS-independent N3 manifest validator; do not modify historical tools.
_n3_path = HERE.parents[1] / "lab/sensor_validation/n3_metrics.py"
_n3_spec = importlib.util.spec_from_file_location("rosbag_qual_n3_metrics", _n3_path)
_n3 = importlib.util.module_from_spec(_n3_spec)
_n3_spec.loader.exec_module(_n3)


class UsageParser(argparse.ArgumentParser):
    def error(self, message):
        self.print_usage(sys.stderr)
        self.exit(3, f"{self.prog}: error: {message}\n")



def utc() -> str:
    return datetime.now(timezone.utc).isoformat()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_config(path: Path | None = None) -> dict[str, Any]:
    source = Path(path or DEFAULT_CONFIG)
    config = json.loads(source.read_text())
    if config.get("schema_version") != 1 or not config.get("topics"):
        raise ValueError("config schema_version 1 and non-empty topics are required")
    _n3.read_topics(source)
    for topic in config["topics"]:
        if topic.get("required", True) and "type" not in topic:
            raise ValueError(f"required topic has no type: {topic.get('name')}")
    # Schema/wire invariants (counts >= 0, rates > 0) are not acceptance limits.
    for topic in config["topics"]:
        if "rate_min_hz" in topic:
            for key in ("rate_min_hz", "rate_max_hz", "nominal_hz", "max_gap_s", "drop_gap_factor"):
                value = topic[key]
                if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                    raise ValueError(f"{topic['name']}: {key} must be finite and positive")
            if topic["rate_min_hz"] > topic["rate_max_hz"]:
                raise ValueError("inverted rate band")
    for section in ("storage", "record", "replay", "validation"):
        for key, value in config.get(section, {}).items():
            if isinstance(value, (int, float)) and (not math.isfinite(value) or value < 0):
                raise ValueError(f"{section}.{key} must be finite and nonnegative")
    for key in ("max_write_rate_bps", "min_duration_s"):
        if config.get("storage", {}).get(key, 0) <= 0:
            raise ValueError(f"storage.{key} must be positive")
    for key, value in config.get("record", {}).items():
        if key.endswith("_s") and value <= 0:
            raise ValueError(f"record.{key} must be positive")
    for key in ("min_count_fraction", "rate_relative_tolerance"):
        if not 0 <= config["replay"][key] <= 1:
            raise ValueError(f"replay.{key} must be in [0, 1]")
    for key in ("playback_rate", "startup_delay_s", "spin_timeout_s", "drain_duration_s", "subscriber_depth", "static_subscriber_depth"):
        if key in config["replay"] and config["replay"][key] <= 0:
            raise ValueError(f"replay.{key} must be positive")
    for key in ("min_elapsed_fraction", "min_recorded_duration_fraction"):
        if key in config.get("record", {}) and not 0 < config["record"][key] <= 1:
            raise ValueError(f"record.{key} must be in (0, 1]")
    if config.get("pairing") and not 0 <= config["pairing"]["min_fraction"] <= 1:
        raise ValueError("pairing.min_fraction must be in [0, 1]")
    return config


def host_info() -> dict[str, str]:
    return {"hostname": socket.gethostname(), "platform": platform.platform(),
            "python": platform.python_version(), "architecture": platform.machine()}


def command_text(argv: list[str]) -> str:
    return shlex.join(str(item) for item in argv)


def run_command(argv: list[str], *, timeout: float, env: dict[str, str] | None = None,
                cwd: Path | None = None) -> dict[str, Any]:
    try:
        completed = subprocess.run(argv, text=True, capture_output=True, timeout=timeout,
                                   env=env, cwd=cwd, check=False)
        return {"argv": [str(item) for item in argv], "returncode": completed.returncode,
                "stdout": completed.stdout, "stderr": completed.stderr, "timed_out": False}
    except FileNotFoundError as error:
        return {"argv": [str(item) for item in argv], "returncode": None, "stdout": "",
                "stderr": str(error), "timed_out": False, "missing_command": True}
    except subprocess.TimeoutExpired as error:
        stdout = error.stdout or ""
        stderr = error.stderr or ""
        if isinstance(stdout, bytes):
            stdout = stdout.decode(errors="replace")
        if isinstance(stderr, bytes):
            stderr = stderr.decode(errors="replace")
        return {"argv": [str(item) for item in argv], "returncode": None, "stdout": stdout,
                "stderr": stderr, "timed_out": True}


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n")


def file_record(path: Path, root: Path | None = None) -> dict[str, Any]:
    record: dict[str, Any] = {"path": str(path), "sha256": sha256(path)}
    if root is not None:
        try:
            record["path"] = str(path.resolve().relative_to(root.resolve()))
        except ValueError:
            pass
    return record


def criterion(ident: str, description: str, measured: Any, unit: str,
              threshold: str, status: str, evidence: str, threshold_source: str) -> dict[str, Any]:
    return {"id": ident, "description": description, "measured": measured, "unit": unit,
            "threshold": threshold, "status": status, "evidence": evidence,
            "threshold_source": threshold_source}


def overall_status(criteria: list[dict[str, Any]]) -> str:
    if any(item["status"] == "FAIL" for item in criteria):
        return "FAIL"
    if any(item["status"] == "BLOCKED" for item in criteria):
        return "BLOCKED"
    return "PASS" if criteria else "BLOCKED"


def base_result(check: str, started: str) -> dict[str, Any]:
    return {"check": check, "tool_version": "rosbag_qual/2", "status": "BLOCKED",
            "started_utc": started, "ended_utc": utc(), "host": host_info(),
            "criteria": [], "raw_files": [], "notes": []}


def finalize_result(result: dict[str, Any], output_dir: Path, criteria: list[dict[str, Any]],
                    raw_files: list[dict[str, Any]], notes: list[str] | None = None, *, error_code: int | None = None) -> int:
    result["criteria"] = criteria
    result["raw_files"] = raw_files
    result["notes"] = notes or []
    result["status"] = overall_status(criteria)
    result["ended_utc"] = utc()
    write_json(output_dir / "result.json", result)
    for item in criteria:
        measured = "null" if item["measured"] is None else str(item["measured"])
        print(f"{item['status']:8} {item['id']}: {measured} {item['unit']}".rstrip())
    print(f"Overall: {result['status']}")
    return error_code if error_code is not None else {"PASS": 0, "FAIL": 1, "BLOCKED": 2}[result["status"]]

