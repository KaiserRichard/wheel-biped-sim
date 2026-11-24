#!/usr/bin/env python3
"""Explicit TUM/ORB eight-column input plus per-pose epoch metadata -> JSONL."""
import argparse
from decimal import Decimal, DecimalException, localcontext
import hashlib
import json
from pathlib import Path
import sys

if __package__:
    from .frames import Transform, Sample, convert, sdk_ir1_to_imu, base_imu_camera
else:
    from frames import Transform, Sample, convert, sdk_ir1_to_imu, base_imu_camera


def parse_stamp(text, unit):
    if unit not in {"s", "ns"}:
        raise ValueError("time unit must be s or ns")
    try:
        with localcontext() as ctx:
            ctx.prec = max(60, len(text) + 20)
            ns = Decimal(text) * (1_000_000_000 if unit == "s" else 1)
            if not ns.is_finite() or ns != ns.to_integral_value():
                raise ValueError("timestamp must resolve to whole finite nanoseconds")
            if not -(2**63) <= ns < 2**63:
                raise ValueError("timestamp outside int64 range")
            return int(ns)
    except DecimalException as exc:
        raise ValueError("invalid timestamp") from exc


def read_samples(path, epochs_path, *, unit, sensor, direction, order):
    if sensor not in {"camera_optical", "imu_camera"}:
        raise ValueError("unsupported sensor frame")
    if direction not in {"sensor-to-world", "world-to-sensor"}:
        raise ValueError("explicit pose direction required")
    rows = [line.split("#", 1)[0].split() for line in Path(path).read_text().splitlines()]
    rows = [row for row in rows if row]
    epochs = [json.loads(line) for line in Path(epochs_path).read_text().splitlines() if line.strip()]
    if not rows or len(rows) != len(epochs):
        raise ValueError("need one epoch record per pose, nonempty input")
    samples = []
    for index, (row, epoch) in enumerate(zip(rows, epochs), 1):
        try:
            if len(row) != 8:
                raise ValueError("expected timestamp tx ty tz q1 q2 q3 q4")
            stamp = parse_stamp(row[0], unit)
            if type(epoch["stamp_ns"]) is not int or epoch["stamp_ns"] != stamp:
                raise ValueError("epoch stamp_ns must exactly match trajectory row")
            parent, child = "slam_world", sensor
            if direction == "world-to-sensor":
                parent, child = child, parent
            pose = Transform.from_pose(parent, child, [float(v) for v in row[1:4]],
                                       [float(v) for v in row[4:]], order=order)
            if direction == "world-to-sensor":
                pose = pose.inverse()
            samples.append(Sample(stamp, epoch["clock_domain"], epoch["map_id"],
                                  epoch["reset_counter"], pose))
        except (ValueError, KeyError, TypeError) as exc:
            raise ValueError(f"pose {index}: {exc}") from exc
    return samples


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("trajectory", type=Path)
    p.add_argument("--epochs", type=Path, required=True)
    p.add_argument("--mount", type=Path, required=True)
    p.add_argument("--time-unit", choices=["s", "ns"], required=True)
    p.add_argument("--sensor-frame", choices=["camera_optical", "imu_camera"], required=True)
    p.add_argument("--pose-direction", choices=["sensor-to-world", "world-to-sensor"], required=True)
    p.add_argument("--quaternion-order", choices=["xyzw", "wxyz"], required=True)
    p.add_argument("--world-frame", default="slam_world")
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args(argv)
    try:
        mount = json.loads(args.mount.read_text())
        if mount["status"] not in {"UNKNOWN", "PROPOSED", "MEASURED"} or not mount["provenance"]:
            raise ValueError("mount requires status and provenance")
        T_base_sensor = Transform("base_link", "camera_optical", mount["T_base_camera"])
        if args.sensor_frame == "imu_camera":
            sdk = mount["sdk_ir1_to_imu"]
            if not sdk["provenance"] or sdk["status"] not in {"UNKNOWN", "PROPOSED", "MEASURED"}:
                raise ValueError("SDK extrinsic requires status and provenance")
            T_base_sensor = base_imu_camera(T_base_sensor, sdk_ir1_to_imu(
                sdk["rotation_column_major"], sdk["translation_m"]))
        samples = read_samples(args.trajectory, args.epochs, unit=args.time_unit,
                               sensor=args.sensor_frame, direction=args.pose_direction,
                               order=args.quaternion_order)
        output = convert(samples, T_base_sensor, world=args.world_frame)
        provenance = {"mount": mount, "input_sensor_frame": args.sensor_frame,
                      "input_pose_direction": args.pose_direction,
                      "input_quaternion_order": args.quaternion_order,
                      "input_time_unit": args.time_unit,
                      "input_sha256": {name: hashlib.sha256(path.read_bytes()).hexdigest()
                                       for name, path in [("trajectory", args.trajectory),
                                                          ("epochs", args.epochs), ("mount", args.mount)]},
                      "velocity_semantics": "diagnostic backward position difference; not qualified odometry"}
        encoded = "".join(json.dumps({**row, "provenance": provenance}, allow_nan=False) + "\n"
                          for row in output)
        if args.output.resolve() in {args.trajectory.resolve(), args.epochs.resolve(), args.mount.resolve()}:
            raise ValueError("output must not overwrite an input")
        args.output.write_text(encoded)
    except (ValueError, KeyError, TypeError, OSError) as exc:
        p.exit(2, f"error: {exc}\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
