#!/usr/bin/env python3
"""Export recorded d435_node online poses/status; ROS imports are CLI-read only."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import sys


FIELDS = ("source_ns", "tracking_state", "pose_published", "processed",
          "application_dropped", "second_ns")
STATES = {-1, 0, 1, 2, 3, 4, 5}
TRACKED = {2, 5}
POSE_TOPIC = "/d435_orb/pose"
STATUS_TOPIC = "/d435_orb/status"
IMAGE_TOPIC = "/camera/camera/infra1/image_rect_raw"
TYPES = {"pose": "geometry_msgs/msg/PoseStamped", "status": "std_msgs/msg/String",
         "image": "sensor_msgs/msg/Image"}


def integer(value, name, minimum=0, maximum=2**63 - 1):
    if type(value) is not int or not minimum <= value <= maximum:
        raise ValueError(f"{name} must be an integer in [{minimum}, {maximum}]")
    return value


def header_ns(header):
    sec = integer(header.stamp.sec, "header sec", maximum=2**31 - 1)
    nsec = integer(header.stamp.nanosec, "header nanosec", maximum=10**9 - 1)
    return sec * 10**9 + nsec


def tum_stamp(ns):
    # Never convert an epoch to float: one-nanosecond differences must survive.
    sec, nsec = divmod(ns, 10**9)
    return f"{sec}.{nsec:09d}"


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate status JSON key: {key}")
        result[key] = value
    return result


def invalid_constant(value):
    raise ValueError(f"nonfinite status JSON value: {value}")


def status_row(text, previous=None):
    row = json.loads(text, object_pairs_hook=unique_object, parse_constant=invalid_constant)
    if not isinstance(row, dict) or set(row) != set(FIELDS):
        raise ValueError("status must have exactly the d435_node fields: " + ",".join(FIELDS))
    for key in ("source_ns", "second_ns", "processed", "application_dropped"):
        integer(row[key], key, minimum=1 if key == "processed" else 0,
                maximum=2**64 - 1 if key in {"processed", "application_dropped"} else 2**63 - 1)
    if type(row["tracking_state"]) is not int or row["tracking_state"] not in STATES:
        raise ValueError("invalid tracking_state; expected ORB state -1 through 5")
    if type(row["pose_published"]) is not bool:
        raise ValueError("pose_published must be a JSON boolean")
    if row["pose_published"] and row["tracking_state"] not in TRACKED:
        raise ValueError("pose_published contradicts nontracking state")
    if previous:
        for key in ("source_ns", "second_ns", "processed"):
            if row[key] <= previous[key]:
                raise ValueError(f"{key} must strictly increase; no sorting/stitching/reset/dedup")
        if row["application_dropped"] < previous["application_dropped"]:
            raise ValueError("application_dropped counter regressed")
    return row


def pose_row(message, previous=None):
    ns = header_ns(message.header)
    if previous is not None and ns <= previous["source_ns"]:
        raise ValueError("pose header must strictly increase; no sorting/stitching/reset/dedup")
    frame = message.header.frame_id
    if not isinstance(frame, str) or not frame:
        raise ValueError("pose header frame_id is empty")
    if previous is not None and frame != previous["frame_id"]:
        raise ValueError("pose frame_id changed; split coordinate-frame sessions")
    p, q = message.pose.position, message.pose.orientation
    values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
    if not all(type(v) in (float, int) and math.isfinite(v) for v in values):
        raise ValueError("nonfinite or nonnumeric pose")
    if math.hypot(*values[3:]) == 0:
        raise ValueError("zero quaternion")
    return {"source_ns": ns, "frame_id": frame, "values": values}


def collect(records, pose_topic=POSE_TOPIC, status_topic=STATUS_TOPIC):
    """Keep each topic's reader order; cross-topic recording order is irrelevant."""
    poses, rows = [], []
    for topic, message in records:
        if topic == pose_topic:
            poses.append(pose_row(message, poses[-1] if poses else None))
        elif topic == status_topic:
            rows.append(status_row(message.data, rows[-1] if rows else None))
    # Check both directions, even if entire topics are absent. Never fabricate poses.
    published = [r["source_ns"] for r in rows if r["pose_published"]]
    observed = [p["source_ns"] for p in poses]
    if published != observed:
        raise ValueError("pose/status contradiction: published source_ns must exactly match pose headers; "
                         "missing pose or missing/false status (capture may be incomplete)")
    return poses, rows


def span(stamps):
    return {"count": len(stamps), "first_source_ns": stamps[0] if stamps else None,
            "last_source_ns": stamps[-1] if stamps else None}


def frame_coverage(rows):
    counters = [r["processed"] for r in rows]
    gaps = [{"first_missing_processed": a + 1, "last_missing_processed": b - 1,
             "missing_count": b - a - 1}
            for a, b in zip(counters, counters[1:]) if b > a + 1]
    prefix = counters[0] - 1 if counters else None
    return {"coverage": "partial" if prefix or gaps else "unknown",
            "reason": "reader EOF does not prove the recorder captured the start or tail",
            "first_processed": counters[0] if counters else None,
            "last_processed": counters[-1] if counters else None,
            "known_missing_status_prefix": prefix,
            "internal_processed_gaps": gaps,
            "known_missing_status_count": (prefix + sum(g["missing_count"] for g in gaps)
                                           if counters else None),
            "unobserved_tail": "unknown",
            "application_dropped_last": rows[-1]["application_dropped"] if rows else None}


def input_coverage(records, rows, image_topic):
    stamps = []
    for topic, message in records:
        if topic != image_topic:
            continue
        ns = header_ns(message.header)
        if stamps and ns <= stamps[-1]:
            raise ValueError("input image header must strictly increase; no sorting/reset/dedup")
        stamps.append(ns)
    sources = [r["source_ns"] for r in rows]
    input_set, source_set = set(stamps), set(sources)
    matched = [ns for ns in stamps if ns in source_set]
    # Run-length gaps in IMAGE INDEX order, never inferred from a nominal frame rate.
    gaps, start = [], None
    for index in range(len(stamps) + 1):
        missing = index < len(stamps) and stamps[index] not in source_set
        if missing and start is None:
            start = index
        if not missing and start is not None:
            gaps.append({"first_input_index": start, "last_input_index": index - 1,
                         "first_source_ns": stamps[start], "last_source_ns": stamps[index - 1],
                         "count": index - start})
            start = None
    unmatched_status = [ns for ns in sources if ns not in input_set]
    if unmatched_status:
        raise ValueError("status source_ns absent from supplied original input image headers")
    return {"image_topic": image_topic, "input_images": span(stamps),
            "processed_statuses": span(sources), "matched": span(matched),
            "input_images_without_observed_status": len(stamps) - len(matched),
            "unmatched_input_runs": gaps, "status_sources_without_input": 0,
            "coverage": "partial" if gaps else "unknown",
            "meaning": "header equality only; missing status may be DDS/pairing/processing/recording loss; "
                       "all matches do not prove complete runtime capture"}


def read_bag(path, topics):
    """Yield deserialized selected topics; ignore bag record clock timestamps."""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:
        raise RuntimeError("run with ROS-installed Python after sourcing ROS Jazzy setup.bash") from error
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=""),
                rosbag2_py.ConverterOptions("", ""))
    available = {t.name: t for t in reader.get_all_topics_and_types()}
    classes = {}
    for topic, expected in topics.items():
        if topic not in available:
            continue  # Export manifest explicitly reports absent topics/counts.
        info = available[topic]
        if info.type != expected or info.serialization_format != "cdr":
            raise ValueError(f"{topic}: expected {expected} with cdr, got "
                             f"{info.type} with {info.serialization_format}")
        classes[topic] = get_message(expected)
    if not classes:
        return
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(classes)))
    while reader.has_next():
        topic, data, _record_timestamp = reader.read_next()
        yield topic, deserialize_message(data, classes[topic])


def file_evidence(path):
    path = Path(path)
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return {"path": str(path.resolve()), "bytes": path.stat().st_size, "sha256": digest.hexdigest()}


def bag_evidence(path):
    path = Path(path).resolve(strict=True)
    files = [path] if path.is_file() else sorted(p for p in path.rglob("*") if p.is_file())
    if not files:
        raise ValueError(f"bag has no files: {path}")
    return {"path": str(path), "files": [file_evidence(p) for p in files]}


def outside_git(path):
    path = Path(path).resolve()
    if any((p / ".git").exists() for p in (path, *path.parents)):
        raise ValueError("output directory must be outside git checkouts")
    return path


def export(output_bag, output_dir, input_bag=None, image_topic=IMAGE_TOPIC,
           pose_topic=POSE_TOPIC, status_topic=STATUS_TOPIC, bag_reader=read_bag):
    if len({pose_topic, status_topic, image_topic}) != 3:
        raise ValueError("pose, status and image topics must differ")
    output_dir = outside_git(output_dir)
    if output_dir.exists():
        raise ValueError("output directory already exists; choose a new path")
    for bag in (output_bag, input_bag):
        if bag is not None:
            root = Path(bag).resolve(strict=True)
            if root.is_dir() and root in output_dir.parents:
                raise ValueError("output directory must be outside both read-only bags")
    # Hash finalized read-only evidence before reading; do not run against active bags.
    bags = {"output_bag": bag_evidence(output_bag)}
    poses, rows = collect(bag_reader(output_bag, {pose_topic: TYPES["pose"],
                                               status_topic: TYPES["status"]}),
                          pose_topic, status_topic)
    original = None
    if input_bag is not None:
        bags["input_bag"] = bag_evidence(input_bag)
        original = input_coverage(bag_reader(input_bag, {image_topic: TYPES["image"]}), rows, image_topic)
    manifest = {
        "schema_version": 1, "inputs": bags,
        "exporter": file_evidence(__file__),
        "topics": {"pose": pose_topic, "status": status_topic},
        "poses": dict(span([p["source_ns"] for p in poses]),
                      availability="observed" if poses else "absent; no tracked poses exported",
                      frame_id=poses[0]["frame_id"] if poses else None),
        "statuses": dict(span([r["source_ns"] for r in rows]),
                         availability="observed" if rows else "absent; tracking history unavailable",
                         published_pose_count=sum(r["pose_published"] for r in rows)),
        "frame_coverage": frame_coverage(rows), "original_input_comparison": original,
        "timing": {"processedFPS": None, "trackedFPS": None,
                   "reason": "no measured steady processing intervals in d435_node status; "
                             "bag record timestamps are not monotonic processing time"},
        "interpretation": "Recorded online camera-optical poses in the published map/world frame. "
                          "Loop correction, relocalization or resets may jump that frame. "
                          "Not a final-optimized atlas trajectory or ground truth.",
        "links": {"trajectory": "CameraTrajectory.txt", "frames": "frames.csv",
                  "contract": "scripts/camera_day/ROSBAG_EXPORT.md"},
    }
    # Validate everything before creating files; each file and directory is exclusive.
    output_dir.mkdir(parents=True, exist_ok=False)
    tum = output_dir / "CameraTrajectory.txt"
    with tum.open("x") as stream:
        stream.write("# online camera-optical pose: t x y z qx qy qz qw; not final atlas or ground truth\n")
        for pose in poses:
            stream.write(tum_stamp(pose["source_ns"]) + " " +
                         " ".join(repr(v) for v in pose["values"]) + "\n")
    frames = output_dir / "frames.csv"
    with frames.open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow(dict(row, pose_published=str(row["pose_published"]).lower()))
    manifest["outputs"] = {"trajectory": file_evidence(tum), "frames": file_evidence(frames)}
    with (output_dir / "export.json").open("x") as stream:
        json.dump(manifest, stream, indent=2, allow_nan=False)
        stream.write("\n")
    return manifest


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output_bag", metavar="OUTPUT_BAG")
    parser.add_argument("output_dir", metavar="NEW_OUTPUT_DIR")
    parser.add_argument("--input-bag", help="original finalized sensor bag, read-only")
    parser.add_argument("--image-topic", default=IMAGE_TOPIC)
    parser.add_argument("--pose-topic", default=POSE_TOPIC)
    parser.add_argument("--status-topic", default=STATUS_TOPIC)
    args = parser.parse_args(argv)
    try:
        result = export(**vars(args))
    except (OSError, ValueError, RuntimeError) as error:
        parser.exit(2, f"rosbag_to_tum: {error}\n")
    print(json.dumps({"poses": result["poses"], "statuses": result["statuses"],
                      "export": str(Path(args.output_dir).resolve() / "export.json")}, allow_nan=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
