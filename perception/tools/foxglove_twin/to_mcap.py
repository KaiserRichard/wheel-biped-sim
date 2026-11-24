"""Convert jump_gate/scene_twin rendered PNGs to a camera-clock JSON MCAP.

Run with: python -m tools.foxglove_twin.to_mcap FRAME_DIR [OUTPUT.mcap]
"""
from __future__ import annotations

import argparse
import base64
from bisect import bisect_right
import csv
import json
import math
from pathlib import Path

from mcap.writer import Writer


DEFAULT_OUTPUT = Path.home() / "wbr-evidence-archive/foxglove-twin-20261030-01/twin.mcap"
DEFAULT_FPS = 10.0  # Same presentation default as jump_gate.demo, not a sensor rate.
SCHEMA_NAMES = {
    "/twin/image": "foxglove.CompressedImage",
    "/twin/advisory": "wbr.twin.Advisory",
    "/twin/phase": "wbr.twin.Phase",
}


def _object_schema(title, properties):
    return {"$schema": "http://json-schema.org/draft-07/schema#", "title": title,
            "type": "object", "properties": properties, "required": list(properties),
            "additionalProperties": False}


SCHEMAS = {
    "/twin/image": _object_schema("foxglove.CompressedImage", {
        "timestamp": _object_schema("Time", {
            "sec": {"type": "integer", "minimum": 0},
            "nsec": {"type": "integer", "minimum": 0, "maximum": 999_999_999},
        }),
        "frame_id": {"type": "string"},
        "data": {"type": "string", "contentEncoding": "base64"},
        "format": {"type": "string"},
    }),
    "/twin/advisory": _object_schema("wbr.twin.Advisory", {
        "raw": {"type": ["string", "null"]},
        "gated": {"type": ["string", "null"]},
        "phase": {"type": "string"},
        "reason": {"type": ["string", "null"]},
        "confidence": {"type": ["number", "null"], "minimum": 0, "maximum": 1},
    }),
    # The single field matches the layout's /twin/phase.message expression.
    "/twin/phase": _object_schema("wbr.twin.Phase", {"message": {"type": "string"}}),
}


def _json(value):
    return json.dumps(value, separators=(",", ":"), allow_nan=False).encode("utf-8")


def _read_rows(path, required):
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        missing = set(required) - set(reader.fieldnames or ())
        if missing:
            raise ValueError(f"{path}: missing columns: {', '.join(sorted(missing))}")
        rows = list(reader)
    if not rows:
        raise ValueError(f"{path}: no rows")
    previous = -1
    for line, row in enumerate(rows, 2):
        try:
            stamp = int(row["frame_stamp_ns"])
            if not 0 < stamp < 2**64 or stamp < previous:
                raise ValueError("frame_stamp_ns must be positive uint64 and nondecreasing")
            previous = stamp
        except (ValueError, TypeError) as exc:
            raise ValueError(f"{path}:{line}: {exc}") from exc
    return rows


def _frames(frame_dir, fps):
    """Match the recorder's selection, without importing MuJoCo or replay code."""
    csvs = [frame_dir / name for name in ("timeline.csv", "frames.csv")
            if (frame_dir / name).is_file()]
    if len(csvs) != 1:
        raise ValueError("expected exactly one of timeline.csv or frames.csv")
    csv_path = csvs[0]
    images = sorted(frame_dir.glob("frame_*.png"))
    if not images:
        raise ValueError(f"{frame_dir}: no frame_*.png files")
    if csv_path.name == "frames.csv":
        rows = _read_rows(csv_path, ("png", "frame_stamp_ns"))
        names = [row["png"] for row in rows]
        if len(set(names)) != len(names) or set(names) != {p.name for p in images}:
            raise ValueError("frames.csv PNG names must match frame_*.png files exactly")
        pairs = [(frame_dir / row["png"], row) for row in rows]
    else:
        rows = _read_rows(csv_path, ("t_s", "frame_stamp_ns", "raw_advisory",
                                    "gated_advisory", "gate_phase", "reason", "gated_confidence"))
        times = [float(row["t_s"]) for row in rows]
        if (not all(math.isfinite(t) for t in times) or times[0] != 0
                or any(b <= a for a, b in zip(times, times[1:]))):
            raise ValueError("timeline.csv t_s must start at zero and increase strictly")
        # timeline.csv is one row per gate tick, not one row per PNG. This is
        # exactly demo.record's k/fps -> Timeline.at (bisect_right) selection.
        count = math.ceil(times[-1] * fps) + 1
        expected = [f"frame_{k:05d}.png" for k in range(count)]
        if [p.name for p in images] != expected:
            raise ValueError(f"expected {count} contiguous jump PNGs for --fps {fps:g}; "
                             "pass the same --fps used by the renderer")
        pairs = [(path, rows[bisect_right(times, min(k / fps, times[-1])) - 1])
                 for k, path in enumerate(images)]
    return csv_path, pairs


def _advisory(row, is_jump):
    if not is_jump:
        # scene_twin frames.csv contains no advisory/gate fields. Do not invent
        # confidence, gating, or a measured phase from a rendered image.
        return dict(raw=None, gated=None, phase="UNKNOWN", reason=None, confidence=None)
    confidence = float(row["gated_confidence"])
    if not math.isfinite(confidence) or not 0 <= confidence <= 1:
        raise ValueError("gated_confidence must be finite and between zero and one")
    return dict(raw=row["raw_advisory"], gated=row["gated_advisory"],
                phase=row["gate_phase"], reason=row["reason"], confidence=confidence)


def convert(frame_dir: str | Path, output: str | Path = DEFAULT_OUTPUT, *, fps=DEFAULT_FPS):
    """Write three messages per rendered frame; never overwrite an existing file."""
    if not math.isfinite(fps) or fps <= 0:
        raise ValueError("fps must be positive and finite")
    frame_dir = Path(frame_dir).expanduser().resolve()
    output = Path(output).expanduser().resolve()
    protected = Path.home() / "wbr-evidence-archive/lab-day-20261025"
    if output.is_relative_to(frame_dir) or output.is_relative_to(protected.resolve()):
        raise ValueError("output must be outside the render inputs and lab-day source archive")
    if output.exists():
        raise FileExistsError(f"refusing to overwrite: {output}")
    csv_path, pairs = _frames(frame_dir, fps)
    samples = [(path, int(row["frame_stamp_ns"]), _advisory(row, csv_path.name == "timeline.csv"))
               for path, row in pairs]
    output.parent.mkdir(parents=True, exist_ok=True)
    # Exclusive create also protects against another process creating the output.
    with output.open("xb") as stream:
        writer = Writer(stream)
        writer.start(profile="", library="wbr.foxglove_twin")
        channels = {}
        for topic, schema in SCHEMAS.items():
            schema_id = writer.register_schema(SCHEMA_NAMES[topic], "jsonschema", _json(schema))
            channels[topic] = writer.register_channel(topic, "json", schema_id)
        for sequence, (path, stamp, advisory) in enumerate(samples):
            sec, nsec = divmod(stamp, 1_000_000_000)
            image = {"timestamp": {"sec": sec, "nsec": nsec}, "frame_id": "twin",
                     "data": base64.b64encode(path.read_bytes()).decode("ascii"), "format": "png"}
            for topic, payload in (("/twin/image", image), ("/twin/advisory", advisory),
                                   ("/twin/phase", {"message": advisory["phase"]})):
                writer.add_message(channels[topic], log_time=stamp, publish_time=stamp,
                                   sequence=sequence, data=_json(payload))
        writer.finish()
    return {"output": str(output), "source_csv": str(csv_path), "frames": len(samples),
            "message_counts": {topic: len(samples) for topic in channels},
            "first_stamp_ns": samples[0][1], "last_stamp_ns": samples[-1][1],
            "size_bytes": output.stat().st_size}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("frame_dir", type=Path)
    parser.add_argument("output_mcap", nargs="?", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--fps", type=float, default=DEFAULT_FPS,
                        help="jump_gate render FPS (default: 10); unused for scene_twin frames.csv")
    args = parser.parse_args(argv)
    try:
        result = convert(args.frame_dir, args.output_mcap, fps=args.fps)
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
