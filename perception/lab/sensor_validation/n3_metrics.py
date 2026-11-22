"""ROS-independent streaming evidence calculations; no hardware imports or access."""
from __future__ import annotations

import hashlib
import array
import json
import math
import struct
from pathlib import Path


def read_topics(path=None):
    path = Path(path) if path else Path(__file__).with_name("n3_topics.json")
    specs = json.loads(path.read_text())["topics"]
    names = [spec["name"] for spec in specs]
    if not specs or len(names) != len(set(names)):
        raise ValueError("Empty or duplicated topic manifest")
    if any(not name.startswith("/") for name in names):
        raise ValueError("Topic names must be absolute")
    return specs


def save_json(path, value):
    # Never replace an existing experiment artifact.
    with Path(path).open("x") as output:
        json.dump(value, output, indent=2, allow_nan=False)
        output.write("\n")


def finite(values):
    return all(math.isfinite(float(value)) for value in values)


def digest_message(digest, value):
    """Hash every decoded field deterministically, independent of CDR padding.

    Sequence boundaries/types and exact floating-point bits are retained. Wire
    hashes are separately preserved, since DDS may rewrite CDR alignment bytes.
    """
    if isinstance(value, (bytes, bytearray)) or isinstance(value, array.array) and value.typecode == 'B':
        digest.update(b'bytes' + struct.pack('<Q', len(value)) + bytes(value))
    elif isinstance(value, str):
        encoded = value.encode('utf-8'); digest.update(b'str' + struct.pack('<Q', len(encoded)) + encoded)
    elif isinstance(value, bool):
        digest.update(b'true' if value else b'false')
    elif isinstance(value, int):
        encoded = str(value).encode(); digest.update(b'int' + struct.pack('<Q', len(encoded)) + encoded)
    elif isinstance(value, float):
        digest.update(b'float' + struct.pack('<d', value))
    elif hasattr(value, 'get_fields_and_field_types') or hasattr(value, '__dict__'):
        fields = value.get_fields_and_field_types() if hasattr(value, 'get_fields_and_field_types') else vars(value)
        digest.update(b'message' + struct.pack('<Q', len(fields)))
        for field in sorted(fields):
            digest_message(digest, field); digest_message(digest, getattr(value, field))
    else:
        digest.update(b'sequence' + struct.pack('<Q', len(value)))
        for item in value:
            # ROS fixed numeric arrays can contain numpy scalars.
            digest_message(digest, item.item() if hasattr(item, 'item') else item)


class TopicStats:
    """Bounded-memory message accumulator; nanoseconds retain exact header stamps."""

    def __init__(self, spec):
        self.spec = spec
        self.count = self.repeated = self.backward = self.invalid = 0
        self.zero_stamps = 0
        self.first_stamp = self.last_stamp = None
        self.first_arrival = self.last_arrival = None
        self.max_stamp_gap = self.max_arrival_gap = 0.0
        self.frames = set()
        self.tf_edges = set()
        self.errors = set()
        self.header_digest = hashlib.sha256()
        self.payload_digest = hashlib.sha256()
        self.wire_digest = hashlib.sha256()
        self.wire_sample_hex = None
        self.sample = None

    def add(self, msg, raw, arrival):
        self.count += 1
        if self.first_arrival is None:
            self.first_arrival = arrival
        if self.last_arrival is not None:
            self.max_arrival_gap = max(self.max_arrival_gap, arrival - self.last_arrival)
        self.last_arrival = arrival
        self.wire_digest.update(struct.pack("<Q", len(raw)))
        self.wire_digest.update(raw)
        digest_message(self.payload_digest, msg)
        if self.wire_sample_hex is None and self.spec['role'] == 'camera_info':
            self.wire_sample_hex = raw.hex()
        header = getattr(msg, "header", None)
        if header is not None:
            stamp = int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)
            frame = str(header.frame_id)
            self.frames.add(frame)
            encoded = frame.encode("utf-8")
            self.header_digest.update(struct.pack("<qI", stamp, len(encoded)) + encoded)
            if self.first_stamp is None:
                self.first_stamp = stamp
            if self.last_stamp is not None:
                delta = stamp - self.last_stamp
                self.repeated += delta == 0
                self.backward += delta < 0
                self.max_stamp_gap = max(self.max_stamp_gap, delta / 1e9)
            self.last_stamp = stamp
            self.zero_stamps += stamp <= 0
            if not frame:
                self.errors.add("empty frame_id")
        role = self.spec["role"]
        valid = True
        if role == "image":
            valid = (msg.width == 640 and msg.height == 480 and msg.encoding == "mono8"
                     and msg.step == 640 and len(msg.data) == 640 * 480)
            if self.sample is None:
                self.sample = {key: getattr(msg, key) for key in
                               ("width", "height", "encoding", "step", "is_bigendian")}
        elif role == "camera_info":
            values = list(msg.k) + list(msg.p) + list(msg.r) + list(msg.d)
            valid = (msg.width == 640 and msg.height == 480 and finite(values)
                     and len(msg.k) == 9 and len(msg.p) == 12 and len(msg.r) == 9
                     and msg.k[0] > 0 and msg.k[4] > 0 and abs(msg.k[8] - 1) < 1e-6
                     and msg.p[0] > 0 and msg.p[5] > 0
                     and 0 <= msg.k[2] < 640 and 0 <= msg.k[5] < 480)
            if self.sample is None:
                self.sample = {"width": msg.width, "height": msg.height,
                               "distortion_model": msg.distortion_model,
                               "k": list(msg.k), "p": list(msg.p),
                               "r": list(msg.r), "d": list(msg.d)}
        elif role in ("gyro", "accel"):
            vector = msg.angular_velocity if role == "gyro" else msg.linear_acceleration
            valid = finite((vector.x, vector.y, vector.z))
        elif role == "imu_info":
            values = list(msg.data) + list(msg.noise_variances) + list(msg.bias_variances)
            valid = finite(values) and len(msg.data) == 12
            if self.sample is None:
                self.sample = {"data": list(msg.data), "noise_variances": list(msg.noise_variances),
                               "bias_variances": list(msg.bias_variances),
                               "calibration_adequacy": "NOT_QUALIFIED_BY_TRANSPORT_TEST"}
        elif role == "extrinsics":
            rotation, translation = list(msg.rotation), list(msg.translation)
            valid = len(rotation) == 9 and len(translation) == 3 and finite(rotation + translation)
            if valid:
                # The message uses column-major rotation; columns must be orthonormal.
                for column in range(3):
                    for other in range(3):
                        product = sum(rotation[3 * column + i] * rotation[3 * other + i] for i in range(3))
                        valid &= abs(product - int(column == other)) < 0.01
            if self.sample is None:
                self.sample = {"rotation": rotation, "translation": translation}
        elif role == "tf_static":
            valid = bool(msg.transforms)
            for transform in msg.transforms:
                parent, child = transform.header.frame_id, transform.child_frame_id
                t, q = transform.transform.translation, transform.transform.rotation
                valid &= (bool(parent) and bool(child) and parent != child
                          and finite((t.x, t.y, t.z, q.x, q.y, q.z, q.w))
                          and abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) < 0.01)
                self.tf_edges.add((parent, child))
        if not valid:
            self.invalid += 1

    def report(self, start, end):
        window = end - start
        return {"type": self.spec["type"], "role": self.spec["role"], "count": self.count,
                "window_s": window, "whole_window_hz": self.count / window if window > 0 else None,
                "first_stamp_ns": self.first_stamp, "last_stamp_ns": self.last_stamp,
                "stamp_span_hz": ((self.count - 1) * 1e9 / (self.last_stamp - self.first_stamp)
                                  if self.count > 1 and self.last_stamp is not None
                                  and self.last_stamp > self.first_stamp else None),
                "nonpositive_stamp_steps": self.repeated + self.backward,
                "repeated_stamps": self.repeated, "backward_stamps": self.backward,
                "nonpositive_stamps": self.zero_stamps,
                "max_stamp_gap_s": self.max_stamp_gap, "max_arrival_gap_s": self.max_arrival_gap,
                "start_boundary_gap_s": self.first_arrival - start if self.first_arrival is not None else None,
                "end_boundary_gap_s": end - self.last_arrival if self.last_arrival is not None else None,
                "frame_ids": sorted(self.frames), "invalid_messages": self.invalid,
                "errors": sorted(self.errors), "sample": self.sample,
                "tf_edges": sorted(self.tf_edges), "header_sha256": self.header_digest.hexdigest(),
                "payload_sha256": self.payload_digest.hexdigest(),
                "payload_hash_method": 'all decoded fields, sequence order and exact numeric bits; excludes wire padding',
                "serialized_payload_sha256": self.wire_digest.hexdigest(),
                "first_camera_info_wire_hex": self.wire_sample_hex}


def evaluate(topics, specs, live=False):
    failures = []
    all_edges = {tuple(edge) for result in topics.values() for edge in result.get("tf_edges", [])}
    graph = {}
    for parent, child in all_edges:
        graph.setdefault(parent, set()).add(child)
        graph.setdefault(child, set()).add(parent)
    image_frames = set()
    info_frames = set()
    stream_frames = set()
    for spec in specs:
        name = spec["name"]
        result = topics.get(name, {})
        if not result.get("count"):
            if spec.get("required", True):
                failures.append(f"{name}: missing required messages")
            continue
        if result.get("type") != spec["type"]:
            failures.append(f"{name}: unexpected type")
        if result.get("invalid_messages") or result.get("errors"):
            failures.append(f"{name}: invalid message content/frame")
        if "rate" in spec:
            if result.get("nonpositive_stamp_steps") or result.get("nonpositive_stamps"):
                failures.append(f"{name}: nonpositive/backward/repeated timestamp")
            if result.get("max_stamp_gap_s", math.inf) > spec["max_gap_s"]:
                failures.append(f"{name}: timestamp gap exceeds limit")
            rate = result.get("whole_window_hz")
            if live and (rate is None or not spec["rate"][0] <= rate <= spec["rate"][1]):
                failures.append(f"{name}: whole-window rate outside acceptance band")
            if live:
                for metric in ("max_arrival_gap_s", "start_boundary_gap_s", "end_boundary_gap_s"):
                    if result.get(metric) is None or result[metric] > spec["max_gap_s"]:
                        failures.append(f"{name}: {metric} exceeds limit")
            frames = result.get("frame_ids", [])
            if len(frames) != 1 or not frames[0]:
                failures.append(f"{name}: expected one stable frame_id")
            stream_frames.update(frames)
            if spec["role"] == "image":
                image_frames.update(frames)
            if spec["role"] == "camera_info":
                info_frames.update(frames)
    if image_frames != info_frames:
        failures.append("Image and CameraInfo frame IDs differ")
    if stream_frames:
        seen, pending = set(), [next(iter(stream_frames))]
        while pending:
            frame = pending.pop()
            if frame not in seen:
                seen.add(frame)
                pending.extend(graph.get(frame, set()) - seen)
        if not stream_frames.issubset(seen) or not stream_frames.issubset(graph):
            failures.append("TF static graph does not connect all image/IMU/CameraInfo frames")
    return failures


def compare_replay(stored, replay, specs):
    failures = evaluate(stored["topics"], specs) + evaluate(replay["topics"], specs)
    for name, expected in stored["topics"].items():
        actual = replay["topics"].get(name, {})
        for field in ("type", "count", "first_stamp_ns", "last_stamp_ns", "header_sha256", "payload_sha256"):
            if expected.get(field) != actual.get(field):
                failures.append(f"{name}: replay {field} differs from stored evidence")
    return failures
