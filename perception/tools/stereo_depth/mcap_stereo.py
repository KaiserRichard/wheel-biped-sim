"""Streaming reader and calibration extraction for the recorded ROS 2 stereo MCAP."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterator

import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import Decoder

LEFT_IMAGE = "/camera/camera/infra1/image_rect_raw"
RIGHT_IMAGE = "/camera/camera/infra2/image_rect_raw"
LEFT_INFO = "/camera/camera/infra1/camera_info"
RIGHT_INFO = "/camera/camera/infra2/camera_info"


@dataclass(frozen=True)
class Calibration:
    fx: float
    cx: float
    cy: float
    baseline_m: float
    width: int
    height: int

    @property
    def fb(self) -> float:
        return self.fx * self.baseline_m


@dataclass
class StereoPair:
    stamp_ns: int
    left: np.ndarray
    right: np.ndarray


def stamp_ns(message) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


def calibration_from_camera_info(left_info, right_info) -> Calibration:
    """Read infra1 intrinsics and baseline from infra2 P (Tx = -fx * B)."""
    fx = float(left_info.p[0])
    cx = float(left_info.p[2])
    cy = float(left_info.p[6])
    right_fx = float(right_info.p[0])
    tx = float(right_info.p[3])
    if fx <= 0 or right_fx <= 0 or tx >= 0:
        raise ValueError(f"Invalid rectified stereo projection: fx={fx}, right_fx={right_fx}, Tx={tx}")
    # The requested convention uses infra1 fx with the infra2 Tx term.
    baseline = -tx / fx
    return Calibration(fx, cx, cy, baseline, int(left_info.width), int(left_info.height))


def pair_by_stamp(messages: Iterator[tuple[str, int, np.ndarray]]) -> tuple[list[StereoPair], dict[str, int]]:
    """Pair (topic, stamp, image) records; returns pairs and unpaired counts."""
    pending: dict[int, dict[str, np.ndarray]] = {}
    pairs: list[StereoPair] = []
    counts = {"left": 0, "right": 0, "unpaired_left": 0, "unpaired_right": 0}
    for topic, stamp, image in messages:
        side = "left" if topic == LEFT_IMAGE else "right" if topic == RIGHT_IMAGE else None
        if side is None:
            continue
        counts[side] += 1
        record = pending.setdefault(int(stamp), {})
        if side in record:
            # Preserve duplicate-stamp visibility as an unpaired item rather than
            # silently replacing image data. This recording has unique stamps.
            counts[f"unpaired_{side}"] += 1
            record.pop(side)
            if not record:
                del pending[int(stamp)]
            continue
        record[side] = image
        if "left" in record and "right" in record:
            pairs.append(StereoPair(int(stamp), record["left"], record["right"]))
            del pending[int(stamp)]
    counts["unpaired_left"] += sum("left" in v for v in pending.values())
    counts["unpaired_right"] += sum("right" in v for v in pending.values())
    return pairs, counts


def image_to_array(message) -> np.ndarray:
    if message.encoding != "mono8":
        raise ValueError(f"Expected mono8, got {message.encoding}")
    arr = np.frombuffer(message.data, dtype=np.uint8)
    row_step = int(message.step)
    if row_step == int(message.width):
        return arr.reshape(int(message.height), int(message.width)).copy()
    return arr.reshape(int(message.height), row_step)[:, : int(message.width)].copy()


def read_calibration(path: str | Path) -> Calibration:
    decoder = Decoder()
    found = {}
    with open(path, "rb") as stream:
        reader = make_reader(stream)
        for schema, channel, msg in reader.iter_messages(topics=[LEFT_INFO, RIGHT_INFO]):
            side = "left" if channel.topic == LEFT_INFO else "right"
            if side not in found:
                found[side] = decoder.decode(schema, msg)
            if len(found) == 2:
                break
    if len(found) != 2:
        raise ValueError("MCAP is missing infra1/infra2 CameraInfo")
    return calibration_from_camera_info(found["left"], found["right"])


def iter_stereo_pairs(path: str | Path) -> tuple[Iterator[StereoPair], dict[str, int]]:
    """Return stream-backed generator and counts populated as it is consumed."""
    counts = {"left": 0, "right": 0, "unpaired_left": 0, "unpaired_right": 0}

    def generate():
        decoder = Decoder()
        pending: dict[int, dict[str, np.ndarray]] = {}
        with open(path, "rb") as stream:
            reader = make_reader(stream)
            for schema, channel, msg in reader.iter_messages(topics=[LEFT_IMAGE, RIGHT_IMAGE]):
                side = "left" if channel.topic == LEFT_IMAGE else "right"
                counts[side] += 1
                decoded = decoder.decode(schema, msg)
                stamp = stamp_ns(decoded)
                record = pending.setdefault(stamp, {})
                record[side] = image_to_array(decoded)
                if "left" in record and "right" in record:
                    yield StereoPair(stamp, record["left"], record["right"])
                    del pending[stamp]
        counts["unpaired_left"] += sum("left" in v for v in pending.values())
        counts["unpaired_right"] += sum("right" in v for v in pending.values())

    return generate(), counts


def read_stereo(path: str | Path) -> tuple[Calibration, list[StereoPair], dict[str, int]]:
    """Read calibration and all paired image messages, matched by header stamp."""
    calibration = read_calibration(path)
    stream, counts = iter_stereo_pairs(path)
    pairs = list(stream)
    return calibration, pairs, counts
