#!/usr/bin/env python3
"""Run YOLO11n ONNX on the live Pi Camera v2 (rpicam-vid pipe) and measure FPS.

No depth on this camera: overlays show class and confidence only. Distance
fusion uses the D435i bags (run_p1.py).
"""
from __future__ import annotations

import argparse
import json
import platform
import subprocess
import time
from pathlib import Path

import cv2
import numpy as np

from run_p1 import COCO_NAMES, decode_detections, letterbox_chw_uint8, model_session, onnx_input, sha256

HERE = Path(__file__).resolve().parent


def draw(image: np.ndarray, detections, text: str) -> np.ndarray:
    canvas = image.copy()
    for class_id, confidence, (x1, y1, x2, y2) in detections:
        label = f"{COCO_NAMES[class_id]} {confidence:.2f}"
        cv2.rectangle(canvas, (x1, y1), (x2, y2), (40, 220, 80), 2)
        cv2.putText(canvas, label, (x1, max(18, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (70, 255, 110), 2, cv2.LINE_AA)
    cv2.putText(canvas, text, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2, cv2.LINE_AA)
    return canvas


def throttled() -> str:
    try:
        return subprocess.run(["vcgencmd", "get_throttled"], capture_output=True, text=True, timeout=5).stdout.strip()
    except (OSError, subprocess.TimeoutExpired):
        return "n/a"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=HERE / "models/yolo11n.onnx")
    parser.add_argument("--out-dir", type=Path, default=HERE / "out-live")
    parser.add_argument("--threads", type=int, default=4)
    parser.add_argument("--frames", type=int, default=100)
    parser.add_argument("--warmup", type=int, default=5)
    parser.add_argument("--save-every", type=int, default=20)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--confidence", type=float, default=0.25)
    parser.add_argument("--iou", type=float, default=0.45)
    args = parser.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    session = model_session(args.model, args.threads)
    # Ubuntu Pi 5 has no Picamera2; read I420 frames from rpicam-vid (/opt/rpi-cam, camera_setup.sh).
    # Width must be a multiple of 64 so the YUV stride has no padding.
    frame_bytes = args.width * args.height * 3 // 2
    camera = subprocess.Popen(
        ["rpicam-vid", "-t", "0", "-n", "--codec", "yuv420", "--width", str(args.width), "--height", str(args.height),
         "--framerate", "30", "-o", "-"],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=frame_bytes,
    )

    def capture() -> np.ndarray:
        raw = camera.stdout.read(frame_bytes)
        if len(raw) != frame_bytes:
            raise RuntimeError("rpicam-vid stream ended")
        yuv = np.frombuffer(raw, np.uint8).reshape(args.height * 3 // 2, args.width)
        return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)

    capture_s = infer_s = 0.0
    counts: dict[str, int] = {}
    saved = []
    start = time.perf_counter()
    try:
        for i in range(args.warmup + args.frames):
            t0 = time.perf_counter()
            frame = capture()
            t1 = time.perf_counter()
            chw, meta = letterbox_chw_uint8(frame)
            name, tensor = onnx_input(session, chw)
            outputs = session.run(None, {name: tensor})
            detections = decode_detections(outputs, frame.shape[:2], meta, args.confidence, args.iou)
            t2 = time.perf_counter()
            if i < args.warmup:
                start = time.perf_counter()
                continue
            capture_s += t1 - t0
            infer_s += t2 - t1
            for class_id, _, _ in detections:
                counts[COCO_NAMES[class_id]] = counts.get(COCO_NAMES[class_id], 0) + 1
            n = i - args.warmup
            if n % args.save_every == 0:
                path = args.out_dir / f"live-t{args.threads}-{n:04d}.jpg"
                cv2.imwrite(str(path), draw(frame, detections, f"Pi 5 CPU t={args.threads} | {1.0 / (t2 - t1):.1f} FPS infer"))
                saved.append(path.name)
        wall_s = time.perf_counter() - start
    finally:
        camera.terminate()
        camera.wait(timeout=5)

    result = {
        "platform": platform.platform(),
        "model_sha256": sha256(args.model),
        "threads": args.threads,
        "frames": args.frames,
        "resolution": [args.width, args.height],
        "capture_ms_mean": 1000 * capture_s / args.frames,
        "infer_plus_nms_fps": args.frames / infer_s,
        "end_to_end_fps": args.frames / wall_s,
        "detections_by_class": counts,
        "overlays": saved,
        "throttled_after": throttled(),
    }
    (args.out_dir / f"live-t{args.threads}.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
