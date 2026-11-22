"""Offline rosbag2 SQLite analysis with a small, explicit CDR reader.

The reader only extracts ROS header stamps and frame IDs. Payload semantics are
left to the ROS-side record/replay check; this keeps Mac analysis independent of
ROS packages while preserving the two clock domains required by the gate:
header.stamp and rosbag2 messages.timestamp.
"""
from __future__ import annotations

from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import struct
from typing import Any, Iterable

from qual_common import criterion


class CDR:
    def __init__(self, data: bytes):
        if len(data) < 4 or data[:2] != b"\x00\x01":
            raise ValueError("expected little-endian CDR encapsulation")
        self.data = memoryview(data)
        self.pos = 4

    def align(self, boundary: int) -> None:
        self.pos = 4 + ((self.pos - 4 + boundary - 1) & ~(boundary - 1))

    def unpack(self, fmt: str) -> Any:
        size = struct.calcsize("<" + fmt)
        self.align(min(size, 8))
        if self.pos + size > len(self.data):
            raise ValueError("truncated CDR field")
        value = struct.unpack_from("<" + fmt, self.data, self.pos)
        self.pos += size
        return value[0] if len(value) == 1 else value

    def string(self) -> str:
        length = self.unpack("I")
        if length < 1 or self.pos + length > len(self.data):
            raise ValueError("invalid CDR string")
        raw = bytes(self.data[self.pos:self.pos + length])
        self.pos += length
        if raw[-1:] != b"\0":
            raise ValueError("CDR string is not NUL terminated")
        return raw[:-1].decode("utf-8")


def header(cdr: CDR) -> tuple[int, str]:
    sec = cdr.unpack("i")
    nanosec = cdr.unpack("I")
    if not 0 <= nanosec < 1_000_000_000:
        raise ValueError("ROS Time.nanosec must be in [0, 1e9)")
    return int(sec) * 1_000_000_000 + int(nanosec), cdr.string()


def extract_header(data: bytes, msg_type: str, role: str) -> tuple[int | None, str | None]:
    """Extract a standard ROS header without importing ROS message classes."""
    values = extract_headers(data, msg_type, role)
    return values[0] if values else (None, None)


def extract_headers(data: bytes, msg_type: str, role: str) -> list[tuple[int, str]]:
    cdr = CDR(data)
    if msg_type in {"sensor_msgs/msg/Image", "sensor_msgs/msg/CameraInfo",
                    "sensor_msgs/msg/Imu", "realsense2_camera_msgs/msg/IMUInfo"}:
        return [header(cdr)]
    if msg_type == "tf2_msgs/msg/TFMessage":
        values = []
        for _ in range(cdr.unpack("I")):
            values.append(header(cdr))
            cdr.string()  # child_frame_id
            for _ in range(7):  # translation xyz, quaternion xyzw
                cdr.unpack("d")
        return values
    if role in {"extrinsics", "imu_info"}:
        return []
    raise ValueError(f"unsupported header extraction for {msg_type}")


def find_db3(path: Path) -> Path:
    path = path.resolve()
    if path.is_file():
        if path.suffix not in {".db3", ".sqlite3", ".sqlite"}:
            raise ValueError(f"expected a rosbag2 SQLite file: {path}")
        return path
    if not path.is_dir():
        raise FileNotFoundError(path)
    candidates = sorted(path.glob("*.db3")) + sorted(path.glob("*.sqlite3"))
    if len(candidates) != 1:
        raise ValueError(f"expected exactly one SQLite storage file in {path}, found {len(candidates)}")
    return candidates[0]


def storage_files(path: Path) -> list[Path]:
    path = Path(path).resolve()
    if path.is_file():
        return [path]
    if not path.is_dir():
        return []
    return sorted(item for item in path.iterdir() if item.is_file() and
                  (item.suffix in {".db3", ".sqlite3", ".sqlite", ".mcap"} or item.name == "metadata.yaml"))


def bag_bytes(path: Path) -> int:
    if path.is_file():
        return path.stat().st_size
    return sum(item.stat().st_size for item in storage_files(path))


def _rate(stamps: list[int]) -> float | None:
    if len(stamps) < 2 or stamps[-1] <= stamps[0]:
        return None
    return (len(stamps) - 1) / ((stamps[-1] - stamps[0]) / 1e9)


def _digest_stamps(stamps: Iterable[int]) -> str:
    digest = hashlib.sha256()
    for stamp in stamps:
        digest.update(struct.pack("<q", int(stamp)))
    return digest.hexdigest()


def _topic_report(spec: dict[str, Any], rows: Iterable[tuple[int, bytes]]) -> dict[str, Any]:
    header_stamps: list[int] = []
    frame_ids: list[str] = []
    parse_errors = 0
    receive_stamps: list[int] = []
    count = 0
    for receive_ns, payload in rows:
        count += 1
        receive_stamps.append(int(receive_ns))
        try:
            for stamp, frame_id in extract_headers(payload, spec["type"], spec.get("role", "")):
                header_stamps.append(stamp)
                frame_ids.append(frame_id)
        except (ValueError, struct.error, UnicodeError):
            parse_errors += 1
    header_reversals = sum(b < a for a, b in zip(header_stamps, header_stamps[1:]))
    header_repeats = sum(b == a for a, b in zip(header_stamps, header_stamps[1:]))
    receive_reversals = sum(b < a for a, b in zip(receive_stamps, receive_stamps[1:]))
    header_gaps = [b - a for a, b in zip(header_stamps, header_stamps[1:]) if b > a]
    receive_gaps = [b - a for a, b in zip(receive_stamps, receive_stamps[1:]) if b > a]
    nominal = spec.get("nominal_hz")
    factor = spec.get("drop_gap_factor")
    drop_limit = (float(factor) / float(nominal)) if nominal and factor else None
    drops = sum(gap / 1e9 > drop_limit for gap in header_gaps) if drop_limit else None
    receive_drops = sum(gap / 1e9 > drop_limit for gap in receive_gaps) if drop_limit else None
    return {
        "name": spec["name"], "type": spec["type"], "role": spec.get("role"),
        "count": count, "header_count": len(header_stamps),
        "first_header_stamp_ns": header_stamps[0] if header_stamps else None,
        "last_header_stamp_ns": header_stamps[-1] if header_stamps else None,
        "header_rate_hz": _rate(header_stamps),
        "bag_receive_rate_hz": _rate(receive_stamps),
        "first_bag_receive_stamp_ns": receive_stamps[0] if receive_stamps else None,
        "last_bag_receive_stamp_ns": receive_stamps[-1] if receive_stamps else None,
        "header_reversals": header_reversals, "header_repeated_stamps": header_repeats,
        "bag_receive_reversals": receive_reversals,
        "max_header_gap_s": max(header_gaps, default=0) / 1e9,
        "max_bag_receive_gap_s": max(receive_gaps, default=0) / 1e9,
        "drop_gap_limit_s": drop_limit, "header_drop_count": drops,
        "bag_receive_drop_count": receive_drops, "parse_errors": parse_errors,
        "frame_ids": sorted(set(frame_ids)), "header_stamp_sha256": _digest_stamps(header_stamps),
        "_header_stamps": header_stamps,
    }


def read_sqlite(path: Path, config: dict[str, Any]) -> dict[str, Any]:
    db3 = find_db3(path)
    conn = sqlite3.connect(db3.as_uri() + "?mode=ro", uri=True)
    try:
        topic_rows = conn.execute("SELECT id,name,type,serialization_format FROM topics ORDER BY id").fetchall()
        available = {row[1]: row for row in topic_rows}
        reports: dict[str, dict[str, Any]] = {}
        all_receive: list[int] = []
        metadata_message_count = 0
        for spec in config["topics"]:
            topic = available.get(spec["name"])
            if topic is None:
                reports[spec["name"]] = {"name": spec["name"], "type": spec["type"],
                                           "role": spec.get("role"), "count": 0, "header_count": 0,
                                           "missing": True, "header_rate_hz": None,
                                           "bag_receive_rate_hz": None, "header_stamp_sha256": _digest_stamps(())}
                continue
            if topic[2] != spec["type"]:
                raise ValueError(f"{spec['name']}: type {topic[2]} != configured {spec['type']}")
            if topic[3] != "cdr":
                raise ValueError(f"{spec['name']}: unsupported serialization {topic[3]}")
            # Stream large image payloads instead of retaining the entire bag in RAM.
            rows = conn.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY id", (topic[0],))
            reports[spec["name"]] = _topic_report(spec, rows)
            metadata_message_count += reports[spec["name"]]["count"]
            bounds = conn.execute("SELECT MIN(timestamp),MAX(timestamp) FROM messages WHERE topic_id=?", (topic[0],)).fetchone()
            all_receive.extend(value for value in bounds if value is not None)
        all_receive.sort()
        return {"database": str(db3), "storage_format": "sqlite3", "topic_count": len(topic_rows),
                "configured_message_count": metadata_message_count,
                "message_count": int(conn.execute("SELECT COUNT(*) FROM messages").fetchone()[0]),
                "first_bag_receive_stamp_ns": all_receive[0] if all_receive else None,
                "last_bag_receive_stamp_ns": all_receive[-1] if all_receive else None,
                "duration_s": ((all_receive[-1] - all_receive[0]) / 1e9 if len(all_receive) > 1 else None),
                "bag_bytes": bag_bytes(path if path.is_dir() else db3), "topics": reports,
                "available_topics": [{"name": row[1], "type": row[2], "serialization_format": row[3]}
                                     for row in topic_rows]}
    finally:
        conn.close()


def read_rosbag2(path: Path, config: dict[str, Any]) -> dict[str, Any]:
    """Read MCAP or another rosbag2 storage through rosbag2_py when present."""
    try:
        import rosbag2_py
    except ImportError as error:
        raise RuntimeError("no SQLite storage and rosbag2_py is unavailable for non-SQLite bags") from error
    uri = Path(path).resolve()
    storage_id = "mcap" if uri.suffix == ".mcap" or (uri.is_dir() and any(uri.glob("*.mcap"))) else config.get("storage", {}).get("storage_id", "mcap")
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(uri), storage_id=storage_id),
                rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
    available = {item.name: item.type for item in reader.get_all_topics_and_types()}
    rows_by_name: dict[str, list[tuple[int, bytes]]] = {spec["name"]: [] for spec in config["topics"]}
    total = 0
    while reader.has_next():
        name, payload, receive_ns = reader.read_next()
        total += 1
        if name in rows_by_name:
            if available.get(name) in {"sensor_msgs/msg/Image", "sensor_msgs/msg/Imu", "sensor_msgs/msg/CameraInfo"}:
                # Only header bytes are needed; do not retain MCAP image payloads.
                try:
                    cdr = CDR(payload)
                    header(cdr)
                    payload = payload[:cdr.pos]
                except (ValueError, struct.error, UnicodeError):
                    payload = b""  # retained as a decode error by _topic_report
            rows_by_name[name].append((int(receive_ns), bytes(payload)))
    reports = {}
    all_receive: list[int] = []
    for spec in config["topics"]:
        if spec["name"] not in available:
            reports[spec["name"]] = {"name": spec["name"], "type": spec["type"],
                                       "role": spec.get("role"), "count": 0, "header_count": 0,
                                       "missing": True, "header_rate_hz": None,
                                       "bag_receive_rate_hz": None, "header_stamp_sha256": _digest_stamps(())}
            continue
        if available[spec["name"]] != spec["type"]:
            raise ValueError(f"{spec['name']}: type {available[spec['name']]} != configured {spec['type']}")
        reports[spec["name"]] = _topic_report(spec, rows_by_name[spec["name"]])
        all_receive.extend(row[0] for row in rows_by_name[spec["name"]])
    all_receive.sort()
    return {"database": str(uri), "storage_format": storage_id, "topic_count": len(available),
            "configured_message_count": sum(len(value) for value in rows_by_name.values()),
            "message_count": total, "first_bag_receive_stamp_ns": all_receive[0] if all_receive else None,
            "last_bag_receive_stamp_ns": all_receive[-1] if all_receive else None,
            "duration_s": ((all_receive[-1] - all_receive[0]) / 1e9 if len(all_receive) > 1 else None),
            "bag_bytes": bag_bytes(uri), "topics": reports,
            "available_topics": [{"name": name, "type": msg_type} for name, msg_type in sorted(available.items())]}


def _topic_criteria(report: dict[str, Any], spec: dict[str, Any], evidence: str,
                    limits: dict[str, Any]) -> list[dict[str, Any]]:
    source = spec.get("threshold_source", "UNKNOWN: no threshold source provided")
    required = spec.get("required", True)
    if not report:
        return [criterion(f"topic:{spec['name']}:evidence", "Saved topic measurements exist",
                          None, "", "available", "BLOCKED", evidence, source)]
    if report.get("missing"):
        return [criterion(f"topic:{spec['name']}:present", "Configured topic is present in the bag",
                          0, "messages", ">= 1", "FAIL" if required else "PASS", evidence, source)]
    count = report.get("count")
    criteria = [criterion(f"topic:{spec['name']}:count", "Configured topic message count", count,
                          "messages", ">= 1" if required else "optional",
                          "BLOCKED" if count is None else ("PASS" if count or not required else "FAIL"), evidence, source)]
    if not required and count == 0:
        return criteria
    if "rate_min_hz" in spec:
        for metric, ident, description in (
            ("header_rate_hz", "header_rate", "Rate from ROS header.stamp (source mapping UNKNOWN)"),
            ("bag_receive_rate_hz", "bag_receive_rate", "Rate from rosbag2 receive timestamp (recorder clock; mapping UNKNOWN)")):
            rate = report.get(metric)
            status = "BLOCKED" if rate is None else ("PASS" if spec["rate_min_hz"] <= rate <= spec["rate_max_hz"] else "FAIL")
            criteria.append(criterion(f"topic:{spec['name']}:{ident}", description, rate, "Hz",
                                      f"{spec['rate_min_hz']}..{spec['rate_max_hz']}", status, evidence, source))
        max_gap = report.get("max_header_gap_s")
        criteria.append(criterion(f"topic:{spec['name']}:max_gap", "Maximum positive header-stamp gap", max_gap,
                                  "s", f"<= {spec['max_gap_s']}", "BLOCKED" if max_gap is None else (
                                      "PASS" if max_gap <= spec["max_gap_s"] else "FAIL"), evidence, source))
        for metric, ident, unit in (("header_drop_count", "drops", "gaps"),
                                    ("header_reversals", "stamp_order", "reversals"),
                                    ("header_repeated_stamps", "stamp_repeats", "repeats"),
                                    ("bag_receive_reversals", "bag_receive_order", "reversals")):
            measured, limit = report.get(metric), limits.get("max_" + metric)
            status = "BLOCKED" if measured is None or limit is None else ("PASS" if measured <= limit else "FAIL")
            criteria.append(criterion(f"topic:{spec['name']}:{ident}", metric, measured, unit,
                                      f"<= {limit}", status, evidence, limits.get("threshold_source", "UNKNOWN")))
    errors, limit = report.get("parse_errors"), limits.get("max_parse_errors")
    criteria.append(criterion(f"topic:{spec['name']}:cdr", "CDR headers decode without errors", errors, "errors",
                              f"<= {limit}", "BLOCKED" if errors is None or limit is None else (
                                  "PASS" if errors <= limit else "FAIL"), evidence, limits.get("threshold_source", "UNKNOWN")))
    return criteria


def evaluate_report(report: dict[str, Any], config: dict[str, Any], evidence: str = "analysis.json") -> list[dict[str, Any]]:
    criteria: list[dict[str, Any]] = []
    topics = report.get("topics", {})
    for spec in config["topics"]:
        criteria.extend(_topic_criteria(topics.get(spec["name"], {}), spec, evidence, config.get("validation", {})))
        if "rate_min_hz" in spec and not topics.get(spec["name"], {}).get("missing"):
            topic = topics.get(spec["name"], {})
            bounds = [report.get("first_bag_receive_stamp_ns"), report.get("last_bag_receive_stamp_ns"),
                      topic.get("first_bag_receive_stamp_ns"), topic.get("last_bag_receive_stamp_ns")]
            gap = None if any(value is None for value in bounds) else max(bounds[2] - bounds[0], bounds[1] - bounds[3]) / 1e9
            criteria.append(criterion(f"topic:{spec['name']}:receive_boundary_gap", "Largest start/end gap within bag receive-time window",
                                      gap, "s", f"<= {spec['max_gap_s']}", "BLOCKED" if gap is None else (
                                          "PASS" if gap <= spec["max_gap_s"] else "FAIL"), evidence, spec["threshold_source"]))

    pairing = config.get("pairing", {})
    if pairing:
        pairing_counts = report.get("pairing", {})
        matched = pairing_counts.get("matched")
        denominator = pairing_counts.get("denominator")
        fraction = (matched / denominator) if matched is not None and denominator else None
        status = "BLOCKED" if fraction is None else ("PASS" if fraction >= pairing["min_fraction"] else "FAIL")
        criteria.append(criterion("ir_pairing", "IR left/right exact header-stamp pairing fraction",
                                  fraction, "fraction", f">= {pairing['min_fraction']}", status,
                                  evidence, pairing.get("threshold_source", "UNKNOWN")))

    storage = config.get("storage", {})
    duration = report.get("duration_s")
    write_rate = (report["bag_bytes"] / duration) if duration and duration > 0 and report.get("bag_bytes") is not None else None
    duration_status = "BLOCKED" if duration is None else ("PASS" if duration >= storage.get("min_duration_s", 0) else "FAIL")
    criteria.append(criterion("disk:duration", "Bag has a positive measurable receive-time duration",
                              duration, "s", f">= {storage.get('min_duration_s', 0)}", duration_status,
                              evidence, storage.get("threshold_source", "UNKNOWN")))
    write_status = "BLOCKED" if write_rate is None else ("PASS" if write_rate <= storage["max_write_rate_bps"] else "FAIL")
    criteria.append(criterion("disk:write_rate", "Bag bytes divided by bag receive-time duration",
                              write_rate, "bytes/s", f"<= {storage['max_write_rate_bps']}", write_status,
                              evidence, storage.get("threshold_source", "UNKNOWN")))
    return criteria


def analyze_database(path: Path, config: dict[str, Any]) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(path)
    sqlite_available = False
    if path.is_file():
        sqlite_available = path.suffix in {".db3", ".sqlite3", ".sqlite"}
    elif path.is_dir():
        sqlite_available = len(list(path.glob("*.db3")) + list(path.glob("*.sqlite3"))) == 1
    report = read_sqlite(path, config) if sqlite_available else read_rosbag2(path, config)
    # Pairing is computed from the DB here and reduced to counts in the saved artifact.
    left_spec = next((s for s in config["topics"] if s.get("role") == config.get("pairing", {}).get("left_role")), None)
    right_spec = next((s for s in config["topics"] if s.get("role") == config.get("pairing", {}).get("right_role")), None)
    if left_spec and right_spec:
        left_stamps = report.get("topics", {}).get(left_spec["name"], {}).get("_header_stamps", [])
        right_stamps = report.get("topics", {}).get(right_spec["name"], {}).get("_header_stamps", [])
        if left_stamps or right_stamps:
            common = Counter(left_stamps) & Counter(right_stamps)
            matched = sum(common.values())
            report["pairing"] = {"left_count": len(left_stamps), "right_count": len(right_stamps),
                                  "matched": matched, "denominator": max(len(left_stamps), len(right_stamps)),
                                  "unmatched_left": len(left_stamps) - matched, "unmatched_right": len(right_stamps) - matched}
    for topic in report.get("topics", {}).values():
        topic.pop("_header_stamps", None)
    report["evidence_label"] = "MEASURED"
    report["clock_domains"] = {"header_stamp": "ROS header.stamp; device hardware/global mapping UNKNOWN unless established by run evidence",
                               "bag_receive_stamp": "rosbag2 messages.timestamp; recorder clock identity/mapping UNKNOWN; never subtracted from header.stamp"}
    report["criteria"] = evaluate_report(report, config)
    return report, report["criteria"]

