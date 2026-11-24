#!/usr/bin/env python3
"""Compare stereo headers and CameraInfo; nominal timing is an estimate, not GT."""
import argparse
import json
import math
from pathlib import Path


PREFIX = "/camera/camera/"
TOPICS = {f"{side}_{kind}": PREFIX + f"infra{index}/" + suffix
          for side, index in (("left", 1), ("right", 2))
          for kind, suffix in (("image", "image_rect_raw"), ("info", "camera_info"))}


def analyze(stamps, fps):
    if not math.isfinite(fps) or fps <= 0:
        raise ValueError("fps must be finite and positive")
    for name in TOPICS:
        values = stamps[name]
        if len(values) < 2 or any(type(v) is not int or v < 0 for v in values):
            raise ValueError(f"{name}: need at least two nonnegative integer headers")
        if any(b <= a for a, b in zip(values, values[1:])):
            raise ValueError(f"{name}: non-increasing headers; no sorting or deduplication")
    period = 1e9 / fps
    first = min(v[0] for v in stamps.values())
    last = max(v[-1] for v in stamps.values())
    expected = round((last - first) / period) + 1
    streams = {}
    for side in ("left", "right"):
        images, info = stamps[f"{side}_image"], stamps[f"{side}_info"]
        missing = len(set(info) - set(images))
        gaps = sum(max(0, round((b - a) / period) - 1)
                   for a, b in zip(images, images[1:]))
        streams[side] = {
            "image_count": len(images), "camera_info_count": len(info),
            "first_source_ns": images[0], "last_source_ns": images[-1],
            "missing_images_with_recorded_camera_info": missing,
            "missing_fraction_vs_camera_info": missing / len(info),
            "images_without_camera_info": len(set(images) - set(info)),
            "nominal_internal_gap_frames_estimate": gaps,
            "nominal_common_span_missing_fraction_estimate": max(0, expected - len(images)) / expected,
            "observed_header_rate_hz": (len(images) - 1) * 1e9 / (images[-1] - images[0]),
        }
    left, right = set(stamps["left_image"]), set(stamps["right_image"])
    target = (left == right and all(s["missing_fraction_vs_camera_info"] < .01
              and s["nominal_common_span_missing_fraction_estimate"] < .01
              and s["images_without_camera_info"] == 0 for s in streams.values()))
    return {"nominal_fps": fps, "nominal_common_span_expected_estimate": expected,
            "streams": streams, "left_only_headers": len(left - right),
            "right_only_headers": len(right - left), "counts_equal": len(left) == len(right),
            "identical_image_headers": left == right, "observed_target_met": target,
            "scope": "Recorded headers/CameraInfo and nominal-rate estimate only; no hardware frame counter, unknown unrecorded prefix/tail, no proof of zero upstream loss."}


def read_headers(bag):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=""),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    for topic in TOPICS.values():
        expected = "sensor_msgs/msg/CameraInfo" if topic.endswith("camera_info") else "sensor_msgs/msg/Image"
        if types.get(topic) != expected:
            raise ValueError(f"missing or wrong topic type: {topic}")
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(TOPICS.values())))
    keys = {topic: key for key, topic in TOPICS.items()}
    classes = {topic: get_message(types[topic]) for topic in keys}
    stamps = {key: [] for key in TOPICS}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        msg = deserialize_message(data, classes[topic])
        stamps[keys[topic]].append(msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec)
    return stamps


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--fps", type=float, required=True)
    args = parser.parse_args()
    result = analyze(read_headers(args.bag), args.fps)
    result["bag"] = str(args.bag.resolve())
    with args.output.open("x") as stream:
        json.dump(result, stream, indent=2)
        stream.write("\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
