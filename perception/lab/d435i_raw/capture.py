#!/usr/bin/env python3
"""Child SDK capture, bounded by check.py. Never imported on analyze-only paths."""
import argparse
import csv
import json
import queue
import re
import threading
import time
from pathlib import Path

from common import FIELDS, STREAMS, host, load_config, utc, value, write_json


class Unavailable(RuntimeError):
    """Device/runtime/profile unavailable; not evidence of a measured violation."""


def domain_name(raw):
    return str(raw).split(".")[-1].lower().replace(" ", "_")


def describe_profile(rs, profile):
    kind = profile.stream_type()
    if kind == rs.stream.infrared:
        video = profile.as_video_stream_profile()
        name = {1: "ir_left", 2: "ir_right"}.get(profile.stream_index(), "other_ir")
        return {"stream": name, "fps": profile.fps(), "format": str(profile.format()).split(".")[-1],
                "width": video.width(), "height": video.height()}
    return {"stream": "gyro" if kind == rs.stream.gyro else "accel" if kind == rs.stream.accel else "other",
            "fps": profile.fps(), "format": str(profile.format()).split(".")[-1]}


def choose_profiles(rs, sensors, cfg):
    """Only enable exact offered native profiles; never interpolate raw accel."""
    offered = []
    candidates = {s: [] for s in STREAMS}
    for sensor in sensors:
        for p in sensor.get_stream_profiles():
            if p.stream_type() not in (rs.stream.infrared, rs.stream.gyro, rs.stream.accel):
                continue
            d = describe_profile(rs, p)
            offered.append(d)
            if d["stream"] not in candidates:
                continue
            if d["stream"].startswith("ir_"):
                matches = (d["format"] == "y8" and d["width"] == value(cfg, "profile", "width")
                           and d["height"] == value(cfg, "profile", "height")
                           and d["fps"] == value(cfg, "profile", "ir_hz"))
            else:
                rates = [value(cfg, "profile", "gyro_hz")] if d["stream"] == "gyro" else value(cfg, "profile", "accel_hz_preference")
                matches = d["format"] == "motion_xyz32f" and d["fps"] in rates
            if matches:
                candidates[d["stream"]].append((sensor, p, d))
    selected = []
    for stream in STREAMS:
        choices = candidates[stream]
        if not choices:
            raise Unavailable(f"No offered profile matches {stream}; see offered_profiles")
        if stream == "accel":
            preferences = value(cfg, "profile", "accel_hz_preference")
            choices.sort(key=lambda item: preferences.index(item[2]["fps"]))
        selected.append(choices[0])
    return selected, offered


def sample_row(rs, frame, receipt_ns, realtime_ns):
    p = frame.get_profile()
    stream = describe_profile(rs, p)["stream"]
    domain = domain_name(frame.get_frame_timestamp_domain())
    stamp = frame.get_timestamp()
    hardware, source = "", "UNKNOWN: SDK hardware metadata unavailable"
    # get_timestamp() is hardware ms only when the SDK says hardware_clock.
    if domain == "hardware_clock":
        hardware, source = stamp, "SDK get_timestamp hardware_clock (ms)"
    elif frame.supports_frame_metadata(rs.frame_metadata_value.frame_timestamp):
        hardware = frame.get_frame_metadata(rs.frame_metadata_value.frame_timestamp) / 1000
        source = "SDK FRAME_TIMESTAMP device microseconds / 1000; raw counter, no unwrap/mapping"
    row = dict.fromkeys(FIELDS, "")
    row.update(stream=stream, frame_number=frame.get_frame_number(), sdk_timestamp_ms=stamp,
               sdk_timestamp_domain=domain, hardware_timestamp_ms=hardware,
               hardware_timestamp_source=source,
               sdk_global_system_timestamp_ms=stamp if domain in ("global_time", "system_time") else "",
               host_clock_monotonic_ns=receipt_ns, host_clock_realtime_ns=realtime_ns)
    if stream in ("gyro", "accel"):
        v = frame.as_motion_frame().get_motion_data()
        row.update(x=v.x, y=v.y, z=v.z)
    return row


def run(out, cfg, seconds, static, rs=None):
    """Use direct sensor callbacks, avoiding a video frameset's IMU decimation."""
    meta = {"schema_version": 1, "started_utc": utc(), "host": host(), "static_confirmed": static,
            "evidence_label": "MEASURED SDK raw capture" if rs is None else "ASSUMED / SYNTHETIC injected SDK fixture",
            "sensor": "D435i camera IMU / imu_camera, never STM32 body IMU",
            "requested_duration_s": seconds, "complete": False, "outcome": "BLOCKED",
            "errors": [], "clock_mapping": None,
            "clock_notes": "SDK reported domains retained. CLOCK_REALTIME is host readout, not mapped device time.",
            "profiles": [], "offered_profiles": []}
    opened, started, restore = [], [], []
    accepting = threading.Event()
    lock = threading.Lock()
    samples = queue.Queue(maxsize=value(cfg, "capture", "queue_capacity"))
    next_image = {s: 0 for s in ("ir_left", "ir_right")}
    received = {s: 0 for s in STREAMS}
    overflow = 0
    callback_errors = []
    window_start = None
    setup_complete = False
    try:
        if rs is None:
            import pyrealsense2 as rs
        meta["sdk_version"] = getattr(rs, "__version__", "UNKNOWN: pyrealsense2 module has no __version__")
        rs.log_to_file(rs.log_severity.info, str(out / "sdk.log"))
        context = rs.context('{"dds":{"enabled":false}}')
        devices = list(context.query_devices())
        expected = value(cfg, "identity", "serial")
        device = next((d for d in devices if d.get_info(rs.camera_info.serial_number) == expected), None)
        if device is None:
            meta["enumerated_sdk_serials"] = [d.get_info(rs.camera_info.serial_number) for d in devices]
            raise Unavailable("Configured SDK serial absent; no camera stream opened")
        def info(key):
            return device.get_info(key) if device.supports(key) else None
        meta["identity"] = {"serial": info(rs.camera_info.serial_number),
                            "firmware": info(rs.camera_info.firmware_version),
                            "usb_type": info(rs.camera_info.usb_type_descriptor)}
        identity = meta["identity"]
        if identity["firmware"] is None or identity["usb_type"] is None:
            raise Unavailable("SDK identity fields unavailable")
        if (identity["firmware"] != value(cfg, "identity", "firmware")
                or not re.fullmatch(value(cfg, "identity", "usb_type_regex"), identity["usb_type"])):
            raise RuntimeError("Device firmware/USB identity mismatch; no camera stream opened")
        sensors = list(device.query_sensors())
        # Keep the full offered inventory even if selection fails.
        meta["offered_profiles"] = [describe_profile(rs, p) for s in sensors for p in s.get_stream_profiles()
                                     if p.stream_type() in (rs.stream.infrared, rs.stream.gyro, rs.stream.accel)]
        selected, _ = choose_profiles(rs, sensors, cfg)
        meta["profiles"] = [d for _, _, d in selected]
        groups = []
        for sensor in sensors:
            profiles = [p for s, p, _ in selected if s is sensor]
            if profiles:
                groups.append((sensor, profiles))
        emitter_values = []
        meta["global_time_options"] = []
        for sensor, profiles in groups:
            if sensor.supports(rs.option.global_time_enabled):
                meta["global_time_options"].append(sensor.get_option(rs.option.global_time_enabled))
            if any(p.stream_type() == rs.stream.infrared for p in profiles):
                if not sensor.supports(rs.option.emitter_enabled):
                    raise Unavailable("Cannot set/read emitter OFF on the IR sensor")
                old = sensor.get_option(rs.option.emitter_enabled)
                restore.append((sensor, old))
                sensor.set_option(rs.option.emitter_enabled, value(cfg, "profile", "emitter"))
                emitter_values.append(sensor.get_option(rs.option.emitter_enabled))
        meta["emitter_readback"] = emitter_values
        if not emitter_values or any(v != value(cfg, "profile", "emitter") for v in emitter_values):
            raise RuntimeError("Emitter OFF readback failed")
        # The SDK startup calls are outside the measured interval. The parent bounds all calls.
        for sensor, profiles in groups:
            sensor.open(profiles)
            opened.append(sensor)

        def callback(frame):
            nonlocal overflow
            receipt_ns = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
            realtime_ns = time.clock_gettime_ns(time.CLOCK_REALTIME)
            if not accepting.is_set():
                return
            if not window_start <= receipt_ns < window_end:
                return
            try:
                row = sample_row(rs, frame, receipt_ns, realtime_ns)
                image = None
                with lock:
                    stream = row["stream"]
                    received[stream] += 1
                    if stream in next_image:
                        slot = next_image[stream]
                        total = value(cfg, "capture", "ir_samples_per_stream")
                        if slot < total and receipt_ns - window_start >= slot * seconds * 1e9 / total:
                            video = frame.as_video_frame()
                            width, height = value(cfg, "profile", "width"), value(cfg, "profile", "height")
                            stride = video.get_stride_in_bytes()
                            if video.get_width() != width or video.get_height() != height or stride < width:
                                raise RuntimeError("Invalid Y8 image shape/stride")
                            buf = memoryview(video.get_data()).cast("B")
                            if len(buf) != stride * height:
                                raise RuntimeError("Invalid Y8 pixel buffer length")
                            pixels = b"".join(bytes(buf[y * stride:y * stride + width]) for y in range(height))
                            image = f"P5\n{width} {height}\n255\n".encode() + pixels
                            row["image_file"] = f"{stream}-{slot}.pgm"
                            next_image[stream] += 1
                    try:
                        samples.put_nowait((row, image))
                    except queue.Full:
                        overflow += 1
            except Exception as error:
                with lock:
                    # Bounded error memory; one callback failure is enough to fail.
                    if not callback_errors:
                        callback_errors.append(str(error))

        for sensor, _ in groups:
            sensor.start(callback)
            started.append(sensor)
        window_start = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
        window_end = window_start + round(seconds * 1e9)
        meta.update(window_start_host_clock_monotonic_ns=window_start,
                    window_end_host_clock_monotonic_ns=window_end)
        write_json(out / "capture.json", meta)
        setup_complete = True
        with (out / "samples.csv").open("x", newline="") as handle:
            writer = csv.DictWriter(handle, FIELDS)
            writer.writeheader()
            accepting.set()
            while time.clock_gettime_ns(time.CLOCK_MONOTONIC) < window_end:
                try:
                    row, image = samples.get(timeout=value(cfg, "capture", "poll_s"))
                except queue.Empty:
                    continue
                writer.writerow(row)
                if image is not None:
                    (out / row["image_file"]).write_bytes(image)
            accepting.clear()
            # Stop joins SDK callbacks before final drain; a callback already
            # extracting its row when clear() ran must not be silently lost.
            while started:
                sensor = started[-1]
                sensor.stop()
                started.pop()
            while not samples.empty():
                row, image = samples.get_nowait()
                writer.writerow(row)
                if image is not None:
                    (out / row["image_file"]).write_bytes(image)
        meta["outcome"] = "PASS"
    except (ImportError, Unavailable) as error:
        meta["errors"].append(str(error))
    except Exception as error:
        meta["outcome"] = "FAIL" if setup_complete or meta.get("identity") else "BLOCKED"
        meta["errors"].append(str(error))
    finally:
        accepting.clear()
        for sensor in reversed(started):
            try:
                sensor.stop()
            except Exception as error:
                meta["errors"].append(f"SDK stop: {error}")
        for sensor in reversed(opened):
            try:
                sensor.close()
            except Exception as error:
                meta["errors"].append(f"SDK close: {error}")
        for sensor, old in reversed(restore):
            try:
                sensor.set_option(rs.option.emitter_enabled, old)
                if sensor.get_option(rs.option.emitter_enabled) != old:
                    raise RuntimeError("restore readback mismatch")
            except Exception as error:
                meta["errors"].append(f"Emitter restore: {error}")
        meta.update(ended_utc=utc(), queue_overflow=overflow, callback_errors=callback_errors,
                    received_counts=received)
        if meta["outcome"] == "PASS" and (meta["errors"] or overflow or callback_errors):
            meta["outcome"] = "FAIL"
        meta["complete"] = meta["outcome"] == "PASS"
        write_json(out / "capture.json", meta)
        print(json.dumps(meta, allow_nan=False))
    return {"PASS": 0, "FAIL": 1, "BLOCKED": 2}[meta["outcome"]]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--outdir", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--seconds", type=float, required=True)
    parser.add_argument("--static", action="store_true")
    args = parser.parse_args()
    return run(args.outdir, load_config(args.config), args.seconds, args.static)


if __name__ == "__main__":
    raise SystemExit(main())
