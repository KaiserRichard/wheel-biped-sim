#!/usr/bin/env python3
"""Analyze D435i ROS header.stamp against rosbag receive timestamps.

The two timestamps are deliberately kept as separate clock-domain values.  The
fit is a diagnostic affine relation, not a latency measurement or an online
clock map.  CSV input is ``topic,header_stamp_ns,receive_ns``.  SQLite input
is a rosbag2 sqlite3 database and uses ``messages.timestamp`` as receive time.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
import io
import itertools
import json
import math
import shutil
from pathlib import Path
import sqlite3
import struct
import sys
from typing import Iterable, Sequence
from urllib.parse import quote

try:
    import numpy as np
except ModuleNotFoundError:
    np = None

# Reuse the repository's centered affine fit.  This keeps large nanosecond
# epochs out of an absolute float and gives this tool the same fit convention
# as tools/clockmap.  The function is intentionally private in that module;
# this analyzer records the dependency in its report and README.
REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
if np is not None:
    from tools.clockmap.clockmap import _fit_affine  # noqa: E402

# bag_core's aggregate reader does not expose timestamp pairs. Reuse its
# type-aware Header decoder and the result-contract helpers, keeping this
# task's SQLite iteration local.
sys.path.insert(0, str(REPO_ROOT / "lab/rosbag_qual"))
from bag_core import extract_headers  # noqa: E402
from qual_common import (UsageParser, sha256, utc, criterion, base_result,
                         finalize_result, file_record)  # noqa: E402

HEADER_TYPES = {"sensor_msgs/msg/Image", "sensor_msgs/msg/Imu",
                "sensor_msgs/msg/CameraInfo", "realsense2_camera_msgs/msg/IMUInfo"}


VERSION = "0.1.1"
CONFIG_PATH = Path(__file__).with_name("config.json")
CSV_COLUMNS = {"topic", "header_stamp_ns", "receive_ns"}
LIMITS = (
    "The fit observes receive = offset + (1 + drift) * header_stamp plus "
    "transport delay and jitter. It cannot separate absolute transport latency "
    "from clock offset without a two-way exchange or an independently known "
    "clock relationship. A lower receive-minus-header envelope is therefore "
    "only an offset-plus-minimum-transport proxy. It is not exposure latency."
)


@dataclass(frozen=True)
class Row:
    topic: str
    header_stamp_ns: int
    receive_ns: int
    source_row: int


def _load_config(path: Path = CONFIG_PATH) -> dict:
    data = json.loads(path.read_text())
    if not isinstance(data, dict):
        raise ValueError("config must be an object")
    thresholds = data.get("thresholds")
    if not isinstance(thresholds, dict) or not isinstance(data.get("percentiles"), list):
        raise ValueError("config must contain thresholds and percentiles")
    values = {}
    for name, entry in thresholds.items():
        if not isinstance(entry, dict) or "value" not in entry or "source" not in entry or "rationale" not in entry:
            raise ValueError(f"config threshold {name!r} needs value, source and rationale")
        if any(not isinstance(entry[key], str) or not entry[key].strip() for key in ("source", "rationale")):
            raise ValueError(f"config threshold {name!r} needs nonempty source and rationale")
        values[name] = entry["value"]
    required = {"minimum_fit_samples", "outlier_mad_scale", "minimum_mad_ns",
                "fit_max_iterations", "step_window_samples", "step_threshold_ns",
                "maximum_step_segments", "max_discontinuities"}
    if required - values.keys():
        raise ValueError("config is missing thresholds: " + ", ".join(sorted(required - values.keys())))
    if any(type(values[name]) is not int or values[name] <= 0 for name in (
            "minimum_fit_samples", "minimum_mad_ns", "fit_max_iterations",
            "step_window_samples", "step_threshold_ns", "maximum_step_segments")):
        raise ValueError("integer thresholds must be positive integers")
    if type(values["outlier_mad_scale"]) not in (int, float) or not math.isfinite(values["outlier_mad_scale"]) or values["outlier_mad_scale"] <= 0:
        raise ValueError("outlier_mad_scale must be positive")
    if values["minimum_fit_samples"] < 3:
        raise ValueError("minimum_fit_samples must be at least three")
    if type(values.get("max_discontinuities")) is not int or values["max_discontinuities"] != 0:
        raise ValueError("max_discontinuities must be zero for these diagnostics")
    percentiles = data["percentiles"]
    if not percentiles or any(type(p) not in (int, float) or not 0 <= p <= 100 for p in percentiles):
        raise ValueError("percentiles must be numbers in [0, 100]")
    if not {0, 50, 95, 99, 100}.issubset(percentiles):
        raise ValueError("percentiles must include 0, 50, 95, 99, 100")
    return data


def _thresholds(config: dict) -> dict:
    return {name: entry["value"] for name, entry in config["thresholds"].items()}


def _stats(values: Sequence[float | int], percentiles: Sequence[float | int]) -> dict | None:
    if not values:
        return None
    array = np.asarray(values, dtype=float)
    result = {"count": len(values)}
    names = {0: "min", 50: "p50", 95: "p95", 99: "p99", 100: "max"}
    for p in percentiles:
        name = names.get(p, f"p{p:g}")
        result[name] = float(np.percentile(array, p))
    return result


def _parse_csv(raw: bytes) -> list[Row]:
    try:
        text = raw.decode("utf-8-sig")
    except UnicodeDecodeError as exc:
        raise ValueError("CSV is not UTF-8") from exc
    reader = csv.DictReader(io.StringIO(text))
    if reader.fieldnames is None:
        raise ValueError("CSV is missing a header")
    if len(set(reader.fieldnames)) != len(reader.fieldnames):
        raise ValueError("CSV has duplicate columns")
    missing = CSV_COLUMNS - set(reader.fieldnames)
    if missing:
        raise ValueError("CSV is missing columns: " + ", ".join(sorted(missing)))
    rows = []
    for fields in reader:
        line = reader.line_num
        try:
            if None in fields or any(fields.get(key) in (None, "") for key in CSV_COLUMNS):
                raise ValueError("empty field or incorrect field count")
            topic = fields["topic"].strip()
            if not topic:
                raise ValueError("empty topic")
            rows.append(Row(topic, int(fields["header_stamp_ns"]), int(fields["receive_ns"]), line))
        except (TypeError, ValueError) as exc:
            raise ValueError(f"CSV line {line}: {exc}") from exc
    if not rows:
        raise ValueError("CSV has no timestamp rows")
    return rows


class _CDRHeader:
    """Minimal CDR reader for a ROS message whose first member is std_msgs/Header."""

    def __init__(self, blob: bytes):
        if len(blob) < 12:
            raise ValueError("serialized message is too short for a ROS Header")
        if blob[:2] == b"\x00\x01":
            self.endian = "<"
        elif blob[:2] == b"\x00\x00":
            self.endian = ">"
        else:
            raise ValueError("unsupported CDR encapsulation; expected CDR little/big endian")

    def stamp_ns(self, blob: bytes) -> int:
        sec, nanosec = struct.unpack_from(self.endian + "iI", blob, 4)
        if nanosec >= 1_000_000_000:
            raise ValueError("ROS Header nanosec is outside [0, 1e9)")
        # Validate frame_id as well; a stamp prefix alone is not a Header.
        if len(blob) < 16:
            raise ValueError("truncated ROS Header frame_id")
        length = struct.unpack_from(self.endian + "I", blob, 12)[0]
        if length < 1 or 16 + length > len(blob) or blob[15 + length] != 0:
            raise ValueError("invalid ROS Header frame_id")
        blob[16:15 + length].decode("utf-8")
        return sec * 1_000_000_000 + nanosec


def _parse_db3(path: Path) -> tuple[list[Row], dict]:
    quoted = quote(str(path.resolve()), safe="/")
    rows = []
    connection = sqlite3.connect(f"file:{quoted}?mode=ro", uri=True)
    try:
        topics = {int(i): (str(name), str(kind), str(serialization))
                  for i, name, kind, serialization in connection.execute(
                      "SELECT id, name, type, serialization_format FROM topics ORDER BY id")}
        supported = {i: value for i, value in topics.items() if value[1] in HEADER_TYPES}
        for message_id, topic_id, receive_ns, blob in connection.execute(
                "SELECT id, topic_id, timestamp, data FROM messages ORDER BY id"):
            if topic_id not in topics:
                raise ValueError(f"message {message_id} references unknown topic {topic_id}")
            if topic_id not in supported:
                continue  # TF sequences and unstamped context are not sensor Headers.
            name, kind, serialization = supported[topic_id]
            try:
                if serialization != "cdr":
                    raise ValueError(f"unsupported serialization {serialization!r}")
                payload = bytes(blob)
                if payload[:2] == b"\x00\x00":
                    stamp = _CDRHeader(payload).stamp_ns(payload)
                else:
                    stamp = extract_headers(payload, kind, "")[0][0]
                rows.append(Row(name, stamp, int(receive_ns), int(message_id)))
            except (TypeError, ValueError, struct.error, UnicodeError) as exc:
                raise ValueError(f"DB3 message {message_id} on {name!r}: {exc}") from exc
    finally:
        connection.close()
    return rows, {
        "sensor_topics": [value[0] for value in supported.values()],
        "skipped_topics": [{"topic": name, "type": kind,
                            "reason": "not a supported first-Header sensor type"}
                           for i, (name, kind, _) in topics.items() if i not in supported],
    }


def _fit_for_indices(rows: Sequence[Row], indices: Sequence[int]):
    source = [rows[i].header_stamp_ns for i in indices]
    target = [rows[i].receive_ns for i in indices]
    return _fit_affine(source, target)


def _step_candidate(rows: Sequence[Row], first: int, last: int, thresholds: dict):
    window = thresholds["step_window_samples"]
    if last - first < max(thresholds["minimum_fit_samples"], 2 * window):
        return None
    local = list(range(first, last))
    try:
        fit = _fit_for_indices(rows, local)
    except ValueError:
        return None
    residuals = np.asarray(fit.residuals_ns, dtype=float)
    best = None
    for boundary in range(window, len(local) - window + 1):
        left = float(np.median(residuals[boundary - window:boundary]))
        right = float(np.median(residuals[boundary:boundary + window]))
        jump = right - left
        candidate = (abs(jump), boundary, jump)
        if best is None or candidate[0] > best[0]:
            best = candidate
    if best is None or best[0] < thresholds["step_threshold_ns"]:
        return None
    _, boundary, jump = best
    return first + boundary, jump


def _split_monotonic(rows: Sequence[Row], first: int, last: int, thresholds: dict):
    pieces = [(first, last, "start")]
    events = []
    while len(pieces) < thresholds["maximum_step_segments"]:
        changed = False
        next_pieces = []
        for position, (piece_first, piece_last, reason) in enumerate(pieces):
            if len(next_pieces) + len(pieces) - position >= thresholds["maximum_step_segments"]:
                next_pieces.append((piece_first, piece_last, reason))
                continue
            candidate = _step_candidate(rows, piece_first, piece_last, thresholds)
            if candidate is None:
                next_pieces.append((piece_first, piece_last, reason))
                continue
            boundary, jump = candidate
            next_pieces.extend(((piece_first, boundary, reason), (boundary, piece_last, "residual_step")))
            events.append({
                "type": "residual_step",
                "boundary_index": boundary,
                "boundary_row": rows[boundary].source_row,
                "residual_jump_ns": float(jump),
                "classification": "possible clock/transport discontinuity; inspect raw timestamps",
            })
            changed = True
        pieces = next_pieces
        if not changed:
            break
    if len(pieces) >= thresholds["maximum_step_segments"]:
        for first, last, _ in pieces:
            candidate = _step_candidate(rows, first, last, thresholds)
            if candidate is not None:
                boundary, jump = candidate
                events.append({"type": "residual_step", "boundary_index": boundary,
                               "boundary_row": rows[boundary].source_row,
                               "residual_jump_ns": float(jump), "segment_cap_reached": True,
                               "classification": "possible clock/transport discontinuity; split cap reached"})
    return pieces, events


def _robust_fit(rows: Sequence[Row], first: int, last: int, thresholds: dict, percentiles):
    indices = list(range(first, last))
    if len(indices) < thresholds["minimum_fit_samples"]:
        return {"fit_status": "UNAVAILABLE: fewer than minimum_fit_samples", "sample_count": len(indices)}
    fit_indices = indices
    for _ in range(thresholds["fit_max_iterations"]):
        fit = _fit_for_indices(rows, fit_indices)
        residuals = np.asarray(fit.residuals_ns, dtype=float)
        centre = float(np.median(residuals))
        mad = float(np.median(np.abs(residuals - centre)))
        scale = max(mad, thresholds["minimum_mad_ns"])
        keep = [index for index, residual in zip(fit_indices, residuals)
                if abs(float(residual) - centre) <= thresholds["outlier_mad_scale"] * scale]
        if len(keep) < thresholds["minimum_fit_samples"] or keep == fit_indices:
            break
        fit_indices = keep
    fit = _fit_for_indices(rows, fit_indices)
    source_ref = fit.source_ref_ns
    target_ref = fit.target_ref_ns
    predicted = np.asarray([
        fit.slope * (rows[i].header_stamp_ns - source_ref) + fit.intercept_ns
        for i in indices
    ], dtype=float)
    actual = np.asarray([rows[i].receive_ns - target_ref for i in indices], dtype=float)
    signed = actual - predicted
    absolute = np.abs(signed)
    offset_proxy = np.asarray([
        rows[i].receive_ns - rows[i].header_stamp_ns for i in indices
    ], dtype=float)
    return {
        "fit_status": "DIAGNOSTIC",
        "sample_count": len(indices),
        "inlier_count": len(fit_indices),
        "source_reference_ns": source_ref,
        "receive_reference_ns": target_ref,
        "slope": float(fit.slope),
        "drift_ppm": float((fit.slope - 1.0) * 1_000_000.0),
        "offset_at_reference_ns": target_ref + int(round(fit.intercept_ns)) - source_ref,
        "residual_signed_ns": _stats(signed.tolist(), percentiles),
        "residual_abs_ns": _stats(absolute.tolist(), percentiles),
        "receive_minus_header_proxy_ns": _stats(offset_proxy.tolist(), percentiles),
        "lower_envelope_proxy_ns": min(rows[i].receive_ns - rows[i].header_stamp_ns for i in indices),
        "outlier_count": len(indices) - len(fit_indices),
    }


def _topic_kind(topic: str) -> str:
    name = topic.lower()
    if "infrared" in name or "infra" in name or "/ir" in name or "infra_" in name:
        return "IR"
    if "imu" in name or "gyro" in name or "accel" in name:
        return "IMU"
    return "OTHER"


def analyze_rows(rows: Iterable[Row], *, config: dict | None = None,
                 header_clock: str = "UNKNOWN", input_description: dict | None = None,
                 global_time_enabled: bool | None = None) -> dict:
    if np is None:
        raise ValueError("NumPy is unavailable")
    config = _load_config() if config is None else config
    thresholds = _thresholds(config)
    rows = list(rows)
    grouped: dict[str, list[Row]] = {}
    for row in rows:
        if not row.topic:
            raise ValueError("empty topic")
        grouped.setdefault(row.topic, []).append(row)
    for topic in (input_description or {}).get("sensor_topics", []):
        grouped.setdefault(topic, [])
    stream_reports = []
    for topic, topic_rows in sorted(grouped.items()):
        reversals = []
        monotonic_pieces = []
        start = 0
        for index in range(1, len(topic_rows)):
            previous, current = topic_rows[index - 1], topic_rows[index]
            stamp_reversed = current.header_stamp_ns <= previous.header_stamp_ns
            receive_reversed = current.receive_ns < previous.receive_ns
            if stamp_reversed or receive_reversed:
                if start < index:
                    monotonic_pieces.append((start, index, "start"))
                reasons = []
                if stamp_reversed:
                    reasons.append("header_stamp_nonincreasing")
                if receive_reversed:
                    reasons.append("receive_time_reversal")
                reversals.append({
                    "index": index,
                    "row": current.source_row,
                    "types": reasons,
                    "classification": "possible global-time reset or recorder ordering failure",
                })
                start = index
        if start < len(topic_rows):
            monotonic_pieces.append((start, len(topic_rows), "start"))

        segments = []
        step_events = []
        for first, last, _ in monotonic_pieces:
            split_pieces, events = _split_monotonic(topic_rows, first, last, thresholds)
            segments.extend(split_pieces)
            step_events.extend(events)
        segment_reports = []
        drifts = []
        weights = []
        for first, last, reason in segments:
            try:
                result = _robust_fit(topic_rows, first, last, thresholds, config["percentiles"])
            except ValueError as exc:
                result = {"fit_status": f"UNAVAILABLE: {exc}", "sample_count": last - first}
            result.update({
                "start_index": first,
                "end_index": last - 1,
                "start_row": topic_rows[first].source_row,
                "end_row": topic_rows[last - 1].source_row,
                "start_reason": reason,
            })
            segment_reports.append(result)
            if result.get("fit_status") == "DIAGNOSTIC":
                drifts.append(result["drift_ppm"])
                weights.append(result["inlier_count"])
        representative = (float(np.average(np.asarray(drifts), weights=np.asarray(weights)))
                          if drifts else None)
        diagnostic_status = ("FAIL" if reversals or step_events else
                             "BLOCKED" if not drifts or any(s["fit_status"] != "DIAGNOSTIC" for s in segment_reports) else "PASS")
        stream_reports.append({
            "topic": topic,
            "kind": _topic_kind(topic),
            "sample_count": len(topic_rows),
            "segment_count": len(segment_reports),
            "fit_status": "DIAGNOSTIC" if drifts else "UNAVAILABLE",
            "diagnostic_status": diagnostic_status,
            "drift_ppm": representative,
            "drift_definition": "receive clock rate minus header clock rate, in ppm",
            "reversals": reversals,
            "step_discontinuities": step_events,
            "segments": segment_reports,
        })

    comparisons = []
    for left, right in itertools.combinations(stream_reports, 2):
        if left["drift_ppm"] is None or right["drift_ppm"] is None:
            continue
        comparisons.append({
            "topic_a": left["topic"],
            "topic_b": right["topic"],
            "drift_difference_ppm_b_minus_a": right["drift_ppm"] - left["drift_ppm"],
            "interpretation": "difference of diagnostic fits; shared device-clock expectation applies only if both streams use the same D435i clock",
        })
    ir_imu = [item for item in comparisons
              if {next(s["kind"] for s in stream_reports if s["topic"] == item["topic_a"]),
                  next(s["kind"] for s in stream_reports if s["topic"] == item["topic_b"])} == {"IR", "IMU"}]
    statuses = [stream["diagnostic_status"] for stream in stream_reports]
    overall_status = ("FAIL" if "FAIL" in statuses else
                      "BLOCKED" if not statuses or "BLOCKED" in statuses else "PASS")
    return {
        "tool": "stamp_drift",
        "version": VERSION,
        "evidence": "CALCULATED diagnostic affine fit; a passing fit is not a hardware or N3 gate PASS",
        "diagnostic_status": overall_status,
        "input": input_description or {"kind": "in-memory"},
        "clock_domains": {
            "header_stamp": header_clock,
            "global_time_enabled": global_time_enabled,
            "receive_timestamp": ("rosbag2 recorder clock; exact host clock UNKNOWN from DB3 schema"
                                  if (input_description or {}).get("kind") == "rosbag2_sqlite3" else
                                  "CSV receive_ns clock UNKNOWN; supplied by input producer"),
            "global_time_note": "Set header_clock to D435i global_time only when realsense2_camera global_time_enabled was recorded/read back; otherwise use D435i hardware/system-time wording appropriate to the run.",
        },
        "model": "receive = a + (1 + d) * header_stamp",
        "fit_method": "tools.clockmap._fit_affine centered least-squares with iterative MAD outlier rejection",
        "thresholds": config["thresholds"],
        "streams": stream_reports,
        "inter_topic_drift_comparison": comparisons,
        "ir_vs_imu_drift_comparison": ir_imu,
        "inference_limits": [LIMITS,
                             "A residual step can be caused by a clock reset, timestamp-domain transition, or changing transport delay; raw topic configuration and logs are needed to distinguish them.",
                             "A monotonic header clock does not prove exposure timing, camera-IMU physical timing, or absolute synchronization."]
    }


def analyze_path(path: str | Path, *, config_path: str | Path = CONFIG_PATH,
                 header_clock: str = "UNKNOWN", global_time_enabled: bool | None = None) -> dict:
    path = Path(path)
    config = _load_config(Path(config_path))
    if path.suffix.lower() == ".db3":
        rows, metadata = _parse_db3(path)
        kind = "rosbag2_sqlite3"
    else:
        rows = _parse_csv(path.read_bytes())
        metadata = {}
        kind = "csv"
    return analyze_rows(rows, config=config, header_clock=header_clock,
                        global_time_enabled=global_time_enabled,
                        input_description={"path": str(path), "kind": kind,
                                           "sha256": sha256(path),
                                           "row_count": len(rows), **metadata})


def render_markdown(report: dict) -> str:
    input_value = report["input"].get("path", report["input"]) if isinstance(report["input"], dict) else report["input"]
    lines = ["# D435i stamp drift report", "", f"- Tool: `{report['tool']} {report['version']}`",
             f"- Diagnostic status: **{report['diagnostic_status']}** (not a hardware gate verdict)",
             f"- Evidence label: **{report['evidence']}**", f"- Input: `{input_value}`", "",
             "## Clock domains", "",
             f"- `header.stamp`: **{report['clock_domains']['header_stamp']}**",
             f"- `global_time_enabled` declaration: **{report['clock_domains']['global_time_enabled']}**",
             f"- DB3 receive timestamp: **{report['clock_domains']['receive_timestamp']}**", "",
             "The fit is `receive = a + (1 + d) * header_stamp`. The header and receive values are in different clock domains.",
             "The reported receive-minus-header lower envelope is an offset-plus-minimum-transport proxy, not absolute latency.", "",
             "## Per-topic results", "", "| Topic | Samples | Segments | Drift (ppm) | Reversals | Steps |", "|---|---:|---:|---:|---:|---:|"]
    for stream in report["streams"]:
        drift = "UNKNOWN" if stream["drift_ppm"] is None else f"{stream['drift_ppm']:.6f}"
        lines.append(f"| `{stream['topic']}` | {stream['sample_count']} | {stream['segment_count']} | {drift} | {len(stream['reversals'])} | {len(stream['step_discontinuities'])} |")
        for segment in stream["segments"]:
            if segment["fit_status"] == "DIAGNOSTIC":
                lines.append(f"  - rows {segment['start_row']}–{segment['end_row']}: residual abs P50/P95/P99/max = {segment['residual_abs_ns']['p50']:.1f}/{segment['residual_abs_ns']['p95']:.1f}/{segment['residual_abs_ns']['p99']:.1f}/{segment['residual_abs_ns']['max']:.1f} ns; lower-envelope proxy = {segment['lower_envelope_proxy_ns']:.1f} ns")
            else:
                lines.append(f"  - rows {segment['start_row']}–{segment['end_row']}: {segment['fit_status']}")
    lines += ["", "## Inter-topic drift", ""]
    if report["inter_topic_drift_comparison"]:
        for item in report["inter_topic_drift_comparison"]:
            lines.append(f"- `{item['topic_b']}` minus `{item['topic_a']}`: {item['drift_difference_ppm_b_minus_a']:.6f} ppm")
    else:
        lines.append("No pair of topics had a usable fit.")
    lines += ["", "## What this can and cannot infer", ""]
    lines.extend(f"- {item}" for item in report["inference_limits"])
    return "\n".join(lines) + "\n"


def _find_analyze_only_artifact(directory: Path) -> Path:
    candidates = [directory / name for name in ("input.csv", "raw.csv", "input.db3") if (directory / name).is_file()]
    candidates += sorted(directory.glob("*.db3")) + sorted(directory.glob("*.csv"))
    unique = []
    for candidate in candidates:
        if candidate not in unique and candidate.name not in {"report.csv"}:
            unique.append(candidate)
    if len(unique) != 1:
        raise ValueError("--analyze-only directory must contain exactly one input .csv or .db3")
    return unique[0]


def _criteria(report: dict, config: dict) -> list[dict]:
    items = []
    minimum = config["thresholds"]["minimum_fit_samples"]
    maximum = config["thresholds"]["max_discontinuities"]
    for stream in report["streams"]:
        usable = bool(stream["segments"]) and all(
            segment["fit_status"] == "DIAGNOSTIC" for segment in stream["segments"])
        items.append(criterion(
            stream["topic"] + ":fit", "Every segment has a usable diagnostic affine fit",
            min((s["sample_count"] for s in stream["segments"]), default=0),
            "samples/segment", f">= {minimum['value']} and positive fit slope",
            "PASS" if usable else "BLOCKED", "report.json", minimum["source"] + ": " + minimum["rationale"]))
        events = len(stream["reversals"]) + len(stream["step_discontinuities"])
        items.append(criterion(
            stream["topic"] + ":continuity", "Detected reversals and residual steps",
            events, "events", f"<= {maximum['value']}",
            "FAIL" if events else "PASS", "report.json", maximum["source"] + ": " + maximum["rationale"]))
    if not items:
        items.append(criterion("input", "Supported timestamp pairs exist", None,
                               "pairs", "available", "BLOCKED", "report.json", "phase-19 input requirement"))
    return items


def main(argv: list[str] | None = None) -> int:
    parser = UsageParser(description=__doc__)
    parser.add_argument("input", nargs="?", type=Path, help="CSV or rosbag2 .db3")
    parser.add_argument("--analyze-only", type=Path, help="directory of saved raw input and configuration")
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--json-out", type=Path)
    parser.add_argument("--md-out", type=Path)
    parser.add_argument("--config", type=Path)
    parser.add_argument("--header-clock", choices=("UNKNOWN", "D435I_GLOBAL_TIME", "D435I_HARDWARE_TIME", "SYSTEM_TIME"))
    parser.add_argument("--global-time-enabled", choices=("true", "false", "unknown"),
                        help="independent saved option readback; header domain does not imply this value")
    args = parser.parse_args(argv)
    if bool(args.input) == bool(args.analyze_only):
        parser.error("provide exactly one input path or --analyze-only directory")
    output_dir = args.output_dir or (args.json_out.parent if args.json_out else
                                   args.analyze_only or args.input.with_suffix(".stamp-drift"))
    config_path = args.config or (args.analyze_only / "config.json" if args.analyze_only else CONFIG_PATH)
    config_error = None
    try:
        config = _load_config(config_path)
    except (OSError, ValueError, TypeError) as exc:
        if args.analyze_only and args.config is None:
            config, config_error = None, f"saved config unavailable: {exc}"
        else:
            print(f"stamp_drift configuration: {exc}", file=sys.stderr)
            return 3
    try:
        json_out = args.json_out or output_dir / "report.json"
        md_out = args.md_out or output_dir / "report.md"
        reserved = {output_dir / name for name in ("input.csv", "input.db3", "config.json", "run.json", "result.json", "diagnostic.txt")}
        if json_out.resolve() == md_out.resolve() or any(
                path.resolve() in {p.resolve() for p in reserved} for path in (json_out, md_out)):
            raise ValueError("report paths must be distinct from each other and saved evidence files")
        if any(args.input and path.resolve() == args.input.resolve() for path in (json_out, md_out)):
            raise ValueError("a report cannot overwrite the input")
        if any(path.parent.resolve() != output_dir.resolve() for path in (json_out, md_out)):
            raise ValueError("JSON and Markdown must be in --output-dir (or --json-out parent)")
        if any(path.is_dir() for path in (json_out, md_out)):
            raise ValueError("report destinations must be files")
        output_dir.mkdir(parents=True, exist_ok=True)
    except (OSError, ValueError, TypeError) as exc:
        print(f"stamp_drift configuration/usage: {exc}", file=sys.stderr)
        return 3

    result = base_result("stamp_drift", utc())
    result["tool_version"] = "stamp_drift/" + VERSION
    raw_files = []
    try:
        if config_error:
            raise ValueError(config_error)
        if np is None:
            raise ValueError("NumPy is unavailable; install the repository runtime dependency before analysis")
        previous = json.loads((args.analyze_only / "run.json").read_text()) if args.analyze_only else {}
        if not isinstance(previous, dict):
            raise ValueError("saved run.json must be an object")
        header_clock = args.header_clock or previous.get("header_clock", "UNKNOWN")
        global_time = (args.global_time_enabled if args.global_time_enabled is not None else
                       previous.get("global_time_enabled", "unknown"))
        if header_clock not in ("UNKNOWN", "D435I_GLOBAL_TIME", "D435I_HARDWARE_TIME", "SYSTEM_TIME") or global_time not in ("true", "false", "unknown"):
            raise ValueError("invalid saved clock declarations")
        run = {"header_clock": header_clock, "global_time_enabled": global_time}
        input_path = args.input or _find_analyze_only_artifact(args.analyze_only)
        saved_input = output_dir / ("input.db3" if input_path.suffix.lower() == ".db3" else "input.csv")
        if input_path.resolve() != saved_input.resolve():
            shutil.copyfile(input_path, saved_input)
        raw_files.append(file_record(saved_input, output_dir))
        (output_dir / "config.json").write_text(json.dumps(config, indent=2, allow_nan=False) + "\n")
        (output_dir / "run.json").write_text(json.dumps(run, indent=2) + "\n")
        raw_files.extend(file_record(output_dir / name, output_dir) for name in ("config.json", "run.json"))
        report = analyze_path(saved_input, config_path=output_dir / "config.json", header_clock=header_clock,
                              global_time_enabled={"true": True, "false": False, "unknown": None}[global_time])
        json_out.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
        md_out.write_text(render_markdown(report))
        criteria = _criteria(report, config)
        for item in criteria:
            item["evidence"] = json_out.name
        diagnostic = "Analysis completed; status is a diagnostic result, not an N3 gate verdict.\n"
    except (OSError, ValueError, UnicodeError, csv.Error, sqlite3.Error, KeyError, TypeError, OverflowError) as exc:
        diagnostic = f"stamp_drift: {exc}\n"
        # Remove earlier derived reports, so missing evidence cannot leave a stale PASS.
        for path in (json_out, md_out):
            path.unlink(missing_ok=True)
        criteria = [criterion("input", "Readable, valid saved timestamp evidence", None,
                               "pairs", "available", "BLOCKED", "diagnostic.txt", "phase-19 input requirement")]
    try:
        (output_dir / "diagnostic.txt").write_text(diagnostic)
        raw_files.append(file_record(output_dir / "diagnostic.txt", output_dir))
        return finalize_result(result, output_dir, criteria, raw_files,
                               ["CALCULATED diagnostics only; fixture PASS is not a hardware gate PASS.", LIMITS])
    except OSError as exc:
        print(f"stamp_drift output error: {exc}", file=sys.stderr)
        return 3


if __name__ == "__main__":
    raise SystemExit(main())
