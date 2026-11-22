#!/usr/bin/env python3
"""Offline summary and depth sanity check for an RGB-D ROS 2 bag."""

from __future__ import annotations

import argparse
import json
import math
import struct
import sys
from collections import defaultdict
from pathlib import Path
from typing import Any, Iterable


RGBD_TOPICS = {
    "/camera/camera/color/image_raw": "image",
    "/camera/camera/color/camera_info": "camera_info",
    "/camera/camera/depth/image_rect_raw": "raw_depth",
    "/camera/camera/depth/camera_info": "camera_info",
    "/camera/camera/aligned_depth_to_color/image_raw": "aligned_depth",
    "/camera/camera/aligned_depth_to_color/camera_info": "camera_info",
}
DEPTH_KINDS = {"raw_depth", "aligned_depth"}


def _depth_counts(msg: Any) -> tuple[int, int]:
    """Return (valid, total) for supported ROS Image depth encodings."""
    encoding = str(msg.encoding).upper()
    layouts = {"16UC1": ("H", 2), "MONO16": ("H", 2), "32FC1": ("f", 4)}
    if encoding not in layouts:
        raise ValueError(f"unsupported depth encoding {msg.encoding!r}")
    code, pixel_bytes = layouts[encoding]
    width, height, step = int(msg.width), int(msg.height), int(msg.step)
    if width <= 0 or height <= 0 or step < width * pixel_bytes:
        raise ValueError("invalid depth image dimensions or row step")
    data = bytes(msg.data)
    if len(data) < step * height:
        raise ValueError("depth image data is shorter than height * step")
    endian = ">" if bool(msg.is_bigendian) else "<"
    valid = 0
    total = width * height
    for row in range(height):
        base = row * step
        for col in range(width):
            value = struct.unpack_from(endian + code, data, base + col * pixel_bytes)[0]
            if math.isfinite(value) and value > 0:
                valid += 1
    return valid, total


def summarize_records(records: Iterable[tuple[str, int, Any]]) -> dict[str, Any]:
    """Summarize (topic, bag timestamp ns, decoded message) records."""
    counts: dict[str, int] = defaultdict(int)
    stamps: dict[str, list[int]] = defaultdict(list)
    depth_valid: dict[str, int] = defaultdict(int)
    depth_total: dict[str, int] = defaultdict(int)
    errors: list[str] = []
    for topic, stamp, msg in records:
        counts[topic] += 1
        stamps[topic].append(int(stamp))
        kind = RGBD_TOPICS.get(topic)
        if kind in DEPTH_KINDS:
            try:
                valid, total = _depth_counts(msg)
            except (AttributeError, TypeError, ValueError, struct.error) as exc:
                errors.append(f"{topic}: {exc}")
            else:
                depth_valid[topic] += valid
                depth_total[topic] += total

    topics: dict[str, Any] = {}
    for topic, kind in RGBD_TOPICS.items():
        ts = stamps.get(topic, [])
        span_s = ((max(ts) - min(ts)) / 1e9) if len(ts) > 1 else 0.0
        topics[topic] = {
            "kind": kind,
            "count": counts.get(topic, 0),
            "span_s": round(span_s, 6),
            "rate_hz": round((len(ts) - 1) / span_s, 4) if span_s > 0 else None,
        }
        if kind in DEPTH_KINDS:
            total = depth_total[topic]
            topics[topic]["depth_valid_pixels"] = depth_valid[topic]
            topics[topic]["depth_total_pixels"] = total
            topics[topic]["depth_valid_fraction"] = round(depth_valid[topic] / total, 8) if total else None

    missing = [topic for topic in RGBD_TOPICS if counts.get(topic, 0) == 0]
    return {
        "format": "rgbd-capture-check-v1",
        "passed": not missing and not errors and all(
            depth_total[topic] > 0 and depth_valid[topic] > 0
            for topic, kind in RGBD_TOPICS.items()
            if kind in DEPTH_KINDS
        ),
        "missing_required_topics": missing,
        "decode_errors": errors,
        "topics": topics,
        "rate_note": "Rates use bag receive timestamps; they do not prove camera hardware continuity.",
        "depth_note": "Valid means finite and greater than zero in the encoded Image; it is not a distance-accuracy test.",
    }


def read_bag(path: Path) -> dict[str, Any]:
    """Read MCAP or rosbag2 SQLite via rosbags AnyReader."""
    try:
        from rosbags.highlevel import AnyReader
    except ImportError as exc:
        raise RuntimeError("rosbags is required; use the perception virtualenv") from exc
    with AnyReader([path]) as reader:
        selected = [connection for connection in reader.connections if connection.topic in RGBD_TOPICS]
        records = (
            (connection.topic, timestamp, reader.deserialize(raw, connection.msgtype))
            for connection, timestamp, raw in reader.messages(connections=selected)
        )
        return summarize_records(records)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="rosbag2 directory or MCAP file")
    parser.add_argument("--json", type=Path, help="also write summary to this new JSON path")
    args = parser.parse_args(argv)
    if not args.bag.exists():
        parser.error(f"bag path does not exist: {args.bag}")
    result = read_bag(args.bag)
    rendered = json.dumps(result, indent=2, sort_keys=True)
    print(rendered)
    if args.json:
        args.json.write_text(rendered + "\n", encoding="utf-8")
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
