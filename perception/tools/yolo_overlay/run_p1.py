#!/usr/bin/env python3
"""Run a COCO YOLO ONNX detector on the four 2026-10-24 D435i rosbag2 runs."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import platform
import sys
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import onnxruntime as ort
from PIL import Image
from rosbags.rosbag2 import Reader as Rosbag2Reader
from rosbags.typesys import Stores, get_typestore


HERE = Path(__file__).resolve().parent
DEFAULT_BAG_ROOT = Path.home() / "wbr-evidence-archive/mobile-demo-20261024-01"
COLOR_TOPIC = "/camera/camera/color/image_raw"
DEPTH_TOPIC = "/camera/camera/aligned_depth_to_color/image_raw"
IMAGE_SIZE = 640
SYNC_SLOP_NS = 30_000_000

# Ultralytics COCO-80 detection class order.
COCO_NAMES = (
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
    "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
    "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote",
    "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book",
    "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush",
)

RUNS = ("B-H13", "B-H05", "S1-empty", "W1")
RUN_CONTEXT = {
    "B-H13": "Two 13.3 cm-high box stacks; front edge at 1.00 m by tape.",
    "B-H05": "Books 4.50 cm high; front edge at 1.00 m by tape.",
    "S1-empty": "Empty tabletop; nearest logged obstacle was the lamp pole at about 1.42 m.",
    "W1": "Wall at 1.00 m by tape; no COCO wall class exists.",
}
# Only B-H05 has a COCO-representable target (books) for a class-matched range
# comparison. B-H13's target is a cardboard box stack, which has no COCO class.
TAPE_FRONT_M = {"B-H05": 1.00}


def image_stamp_ns(msg: Any, fallback_ns: int) -> int:
    stamp = msg.header.stamp
    value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return value if value else int(fallback_ns)


def read_rosbag_images(bag_path: Path, typestore: Any) -> tuple[list[tuple[int, Any]], list[tuple[int, Any]]]:
    colors: list[tuple[int, Any]] = []
    depths: list[tuple[int, Any]] = []
    with Rosbag2Reader(bag_path) as reader:
        connections = [c for c in reader.connections if c.topic in (COLOR_TOPIC, DEPTH_TOPIC)]
        if not any(c.topic == COLOR_TOPIC for c in connections):
            raise RuntimeError(f"{bag_path}: missing {COLOR_TOPIC}")
        if not any(c.topic == DEPTH_TOPIC for c in connections):
            raise RuntimeError(f"{bag_path}: missing {DEPTH_TOPIC}")
        for connection, bag_stamp_ns, raw in reader.messages(connections=connections):
            msg = typestore.deserialize_cdr(raw, connection.msgtype)
            entry = (image_stamp_ns(msg, bag_stamp_ns), msg)
            if connection.topic == COLOR_TOPIC:
                colors.append(entry)
            elif connection.topic == DEPTH_TOPIC:
                depths.append(entry)
    colors.sort(key=lambda item: item[0])
    depths.sort(key=lambda item: item[0])
    return colors, depths


def pair_color_depth(
    colors: list[tuple[int, Any]], depths: list[tuple[int, Any]], slop_ns: int = SYNC_SLOP_NS
) -> tuple[list[tuple[int, Any, Any]], int, int]:
    """Greedily pair each color/depth image once by ROS header timestamp."""
    pairs: list[tuple[int, Any, Any]] = []
    ci = di = 0
    while ci < len(colors) and di < len(depths):
        color_ns, color_msg = colors[ci]
        depth_ns, depth_msg = depths[di]
        delta = color_ns - depth_ns
        if abs(delta) <= slop_ns:
            pairs.append((color_ns, color_msg, depth_msg))
            ci += 1
            di += 1
        elif delta < 0:
            ci += 1
        else:
            di += 1
    return pairs, len(colors) - len(pairs), len(depths) - len(pairs)


def color_bgr(msg: Any) -> np.ndarray:
    height, width, step = int(msg.height), int(msg.width), int(msg.step)
    encoding = str(msg.encoding).lower()
    channels = 4 if encoding in ("rgba8", "bgra8") else 3
    if encoding in ("mono8", "8uc1"):
        channels = 1
    if encoding not in ("rgb8", "bgr8", "rgba8", "bgra8", "mono8", "8uc1", "8uc3"):
        raise ValueError(f"unsupported color encoding {msg.encoding!r}")
    rows = np.frombuffer(msg.data, dtype=np.uint8)
    if rows.size < height * step:
        raise ValueError(f"short color payload: {rows.size} bytes for {height} rows x {step}")
    rows = rows[: height * step].reshape(height, step)
    image = rows[:, : width * channels].reshape(height, width, channels).copy()
    if channels == 1:
        return cv2.cvtColor(image[:, :, 0], cv2.COLOR_GRAY2BGR)
    if encoding in ("rgb8", "rgba8"):
        code = cv2.COLOR_RGB2BGR if channels == 3 else cv2.COLOR_RGBA2BGR
        image = cv2.cvtColor(image, code)
    elif channels == 4:
        image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    return image


def depth_meters(msg: Any, default_scale: float) -> np.ndarray:
    height, width, step = int(msg.height), int(msg.width), int(msg.step)
    encoding = str(msg.encoding).lower()
    if encoding in ("16uc1", "mono16"):
        dtype = np.dtype(">u2" if msg.is_bigendian else "<u2")
        row_bytes = width * 2
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        if raw.size < height * step:
            raise ValueError(f"short depth payload: {raw.size} bytes for {height} rows x {step}")
        rows = raw[: height * step].reshape(height, step)[:, :row_bytes].copy()
        return rows.reshape(height, width, 2).view(dtype).reshape(height, width).astype(np.float32) * default_scale
    if encoding == "32fc1":
        dtype = np.dtype(">f4" if msg.is_bigendian else "<f4")
        row_bytes = width * 4
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        if raw.size < height * step:
            raise ValueError(f"short depth payload: {raw.size} bytes for {height} rows x {step}")
        rows = raw[: height * step].reshape(height, step)[:, :row_bytes].copy()
        return rows.reshape(height, width, 4).view(dtype).reshape(height, width).astype(np.float32)
    raise ValueError(f"unsupported depth encoding {msg.encoding!r}")


def letterbox_chw_uint8(image_bgr: np.ndarray) -> tuple[np.ndarray, tuple[float, int, int]]:
    height, width = image_bgr.shape[:2]
    ratio = min(IMAGE_SIZE / height, IMAGE_SIZE / width)
    resized_w, resized_h = int(round(width * ratio)), int(round(height * ratio))
    pad_w, pad_h = IMAGE_SIZE - resized_w, IMAGE_SIZE - resized_h
    left, top = int(round(pad_w / 2 - 0.1)), int(round(pad_h / 2 - 0.1))
    resized = cv2.resize(image_bgr, (resized_w, resized_h), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((IMAGE_SIZE, IMAGE_SIZE, 3), 114, dtype=np.uint8)
    canvas[top : top + resized_h, left : left + resized_w] = resized
    rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
    return np.ascontiguousarray(rgb.transpose(2, 0, 1)), (ratio, left, top)


def onnx_input(session: ort.InferenceSession, chw_uint8: np.ndarray) -> tuple[str, np.ndarray]:
    meta = session.get_inputs()[0]
    if len(meta.shape) != 4:
        raise ValueError(f"expected a 4-D NCHW model input, got {meta.shape}")
    if isinstance(meta.shape[1], int) and meta.shape[1] != 3:
        raise ValueError(f"expected a 3-channel model input, got {meta.shape}")
    if meta.type == "tensor(float)":
        tensor = chw_uint8.astype(np.float32)[None, ...] / 255.0
    elif meta.type == "tensor(float16)":
        tensor = (chw_uint8.astype(np.float32)[None, ...] / 255.0).astype(np.float16)
    elif meta.type == "tensor(uint8)":
        tensor = chw_uint8[None, ...]
    else:
        raise ValueError(f"unsupported ONNX input type {meta.type}")
    return meta.name, np.ascontiguousarray(tensor)


def unletterbox_box(box: tuple[float, float, float, float], image_shape: tuple[int, int], meta: tuple[float, int, int]) -> tuple[int, int, int, int]:
    ratio, left, top = meta
    height, width = image_shape
    x1, y1, x2, y2 = box
    x1 = int(np.clip(round((x1 - left) / ratio), 0, width - 1))
    x2 = int(np.clip(round((x2 - left) / ratio), 0, width - 1))
    y1 = int(np.clip(round((y1 - top) / ratio), 0, height - 1))
    y2 = int(np.clip(round((y2 - top) / ratio), 0, height - 1))
    return x1, y1, x2, y2


def decode_detections(outputs: list[np.ndarray], image_shape: tuple[int, int], letterbox_meta: tuple[float, int, int], conf_threshold: float, iou_threshold: float) -> list[tuple[int, float, tuple[int, int, int, int]]]:
    pred = np.asarray(outputs[0]).squeeze()
    if pred.ndim != 2:
        raise ValueError(f"expected 2-D detection output after batch squeeze, got {pred.shape}")

    # Ultralytics export with nms=True returns xyxy, confidence, class rows.
    if pred.shape[1] == 6 and pred.shape[0] <= 1000:
        detections = []
        for row in pred:
            score, class_id = float(row[4]), int(row[5])
            if score < conf_threshold or not 0 <= class_id < len(COCO_NAMES):
                continue
            box = unletterbox_box(tuple(float(v) for v in row[:4]), image_shape, letterbox_meta)
            if box[2] > box[0] and box[3] > box[1]:
                detections.append((class_id, score, box))
        return detections

    # Default YOLOv8/YOLO11 export is raw cx,cy,w,h plus class scores (84 columns for COCO).
    if pred.shape[0] in (84, 85) and pred.shape[1] > pred.shape[0]:
        pred = pred.T
    if pred.shape[1] not in (84, 85):
        raise ValueError(f"unsupported COCO detection output shape {pred.shape}; export a YOLOv8n/YOLO11n detector")
    if pred.shape[1] == 85:
        objectness = pred[:, 4]
        class_scores = pred[:, 5:] * objectness[:, None]
    else:
        class_scores = pred[:, 4:]
    class_ids = np.argmax(class_scores, axis=1)
    scores = class_scores[np.arange(len(class_scores)), class_ids]
    candidates = np.flatnonzero(scores >= conf_threshold)
    if not len(candidates):
        return []

    boxes_xywh = pred[candidates, :4]
    boxes_xyxy = np.empty_like(boxes_xywh)
    boxes_xyxy[:, 0] = boxes_xywh[:, 0] - boxes_xywh[:, 2] / 2
    boxes_xyxy[:, 1] = boxes_xywh[:, 1] - boxes_xywh[:, 3] / 2
    boxes_xyxy[:, 2] = boxes_xywh[:, 0] + boxes_xywh[:, 2] / 2
    boxes_xyxy[:, 3] = boxes_xywh[:, 1] + boxes_xywh[:, 3] / 2
    nms_boxes = [[float(x1), float(y1), float(x2 - x1), float(y2 - y1)] for x1, y1, x2, y2 in boxes_xyxy]
    kept: list[int] = []
    candidate_class_ids = class_ids[candidates]
    for class_id in np.unique(candidate_class_ids):
        positions = np.flatnonzero(candidate_class_ids == class_id)
        class_kept = cv2.dnn.NMSBoxes(
            [nms_boxes[int(i)] for i in positions],
            scores[candidates[positions]].astype(float).tolist(),
            conf_threshold,
            iou_threshold,
        )
        if len(class_kept):
            kept.extend(int(positions[int(i)]) for i in np.asarray(class_kept).reshape(-1))
    if not kept:
        return []
    result = []
    for i in sorted(kept, key=lambda index: float(scores[candidates[index]]), reverse=True):
        class_id = int(class_ids[candidates[i]])
        if not 0 <= class_id < len(COCO_NAMES):
            continue
        box = unletterbox_box(tuple(float(v) for v in boxes_xyxy[i]), image_shape, letterbox_meta)
        if box[2] > box[0] and box[3] > box[1]:
            result.append((class_id, float(scores[candidates[i]]), box))
    return result


def box_depth(depth: np.ndarray, box: tuple[int, int, int, int]) -> tuple[float | None, int]:
    x1, y1, x2, y2 = box
    roi = depth[y1 : y2 + 1, x1 : x2 + 1]
    valid = roi[np.isfinite(roi) & (roi > 0)]
    return (float(np.median(valid)), int(valid.size)) if valid.size else (None, 0)


def annotated(image: np.ndarray, detections: list[tuple[int, float, tuple[int, int, int, int]]], depth: np.ndarray) -> tuple[np.ndarray, list[dict[str, Any]]]:
    canvas = image.copy()
    details = []
    for class_id, confidence, box in detections:
        distance_m, valid_pixels = box_depth(depth, box)
        label = COCO_NAMES[class_id]
        distance_text = f"{distance_m:.2f} m" if distance_m is not None else "depth n/a"
        text = f"{label} {confidence:.2f} | {distance_text}"
        x1, y1, x2, y2 = box
        cv2.rectangle(canvas, (x1, y1), (x2, y2), (40, 220, 80), 2)
        cv2.putText(canvas, text, (x1, max(18, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (20, 20, 20), 3, cv2.LINE_AA)
        cv2.putText(canvas, text, (x1, max(18, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (70, 255, 110), 1, cv2.LINE_AA)
        details.append({"class_id": class_id, "label": label, "confidence": confidence, "box_xyxy": box, "median_depth_m": distance_m, "depth_valid_pixels": valid_pixels})
    return canvas, details


def largest_remainder(total: int, weights: list[int]) -> list[int]:
    weight_sum = sum(weights)
    if total <= 0 or weight_sum <= 0:
        return [0] * len(weights)
    exact = [total * weight / weight_sum for weight in weights]
    counts = [math.floor(value) for value in exact]
    for index in sorted(range(len(weights)), key=lambda i: exact[i] - counts[i], reverse=True)[: total - sum(counts)]:
        counts[index] += 1
    return counts


def sampled_indices(length: int, count: int) -> list[int]:
    if length <= 0 or count <= 0:
        return []
    if count == 1:
        return [length // 2]
    return sorted(set(int(round(v)) for v in np.linspace(0, length - 1, min(length, count))))


def model_session(model: Path, threads: int) -> ort.InferenceSession:
    if not model.is_file():
        raise FileNotFoundError(f"missing ONNX model: {model}\nExport YOLO11n as described in README.md first.")
    options = ort.SessionOptions()
    options.intra_op_num_threads = max(0, int(threads))
    options.inter_op_num_threads = 1
    return ort.InferenceSession(str(model), sess_options=options, providers=["CPUExecutionProvider"])


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def connection_counts(bag_path: Path) -> tuple[int, int]:
    with Rosbag2Reader(bag_path) as reader:
        color_count = sum(int(c.msgcount) for c in reader.connections if c.topic == COLOR_TOPIC)
        depth_count = sum(int(c.msgcount) for c in reader.connections if c.topic == DEPTH_TOPIC)
    return color_count, depth_count


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag-root", type=Path, default=DEFAULT_BAG_ROOT)
    parser.add_argument("--model", type=Path, default=HERE / "models/yolo11n.onnx")
    parser.add_argument("--out-dir", type=Path, default=HERE / "out")
    parser.add_argument("--threads", type=int, default=0, help="ONNX Runtime intra-op threads; 0 selects its default")
    parser.add_argument("--confidence", type=float, default=0.25)
    parser.add_argument("--iou", type=float, default=0.45)
    parser.add_argument("--depth-scale", type=float, default=0.001, help="assumed meters per uint16 depth unit; verify for the recording")
    parser.add_argument("--sync-slop-ms", type=float, default=30.0)
    parser.add_argument("--pi-frame-count", type=int, default=50)
    args = parser.parse_args()

    out_dir = args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)
    pi_frames_dir = out_dir / "pi-bench/frames"
    pi_frames_dir.mkdir(parents=True, exist_ok=True)
    run_bags = {run: args.bag_root / run / "rgbd-bag" for run in RUNS}
    for run, path in run_bags.items():
        if not path.is_dir():
            raise FileNotFoundError(f"missing {run} rosbag2 directory: {path}")

    # Fail before loading large bags if the required runtime/model is unavailable.
    session = model_session(args.model, args.threads)
    input_name = session.get_inputs()[0].name
    typestore = get_typestore(Stores.ROS2_JAZZY)
    counts = [connection_counts(run_bags[run]) for run in RUNS]
    pi_allocations = largest_remainder(args.pi_frame_count, [max(1, c) for c, _ in counts])
    sync_slop_ns = int(args.sync_slop_ms * 1_000_000)
    rows: list[dict[str, Any]] = []
    overlay_gif: list[Image.Image] = []
    model_seconds = 0.0
    pipeline_seconds = 0.0
    timed_inferences = 0
    exported_pi_frames = 0
    run_summaries: dict[str, Any] = {}
    warmed_up = False

    for run_index, run in enumerate(RUNS):
        colors, depths = read_rosbag_images(run_bags[run], typestore)
        pairs, unmatched_color, unmatched_depth = pair_color_depth(colors, depths, sync_slop_ns)
        if not pairs:
            raise RuntimeError(f"{run}: no color/aligned-depth pairs within {args.sync_slop_ms:g} ms")
        overlay_at = set(sampled_indices(len(pairs), 2))
        pi_at = set(sampled_indices(len(pairs), pi_allocations[run_index]))
        total_detections = 0
        local_model_s = local_pipeline_s = 0.0

        for frame_index, (stamp_ns, color_msg, depth_msg) in enumerate(pairs):
            pipeline_start = time.perf_counter()
            bgr = color_bgr(color_msg)
            depth = depth_meters(depth_msg, args.depth_scale)
            if depth.shape[:2] != bgr.shape[:2]:
                raise ValueError(f"{run} frame {frame_index}: color {bgr.shape[:2]} != aligned depth {depth.shape[:2]}")
            chw_uint8, letterbox_meta = letterbox_chw_uint8(bgr)
            input_name, tensor = onnx_input(session, chw_uint8)
            if not warmed_up:
                session.run(None, {input_name: tensor})
                warmed_up = True
            infer_start = time.perf_counter()
            outputs = session.run(None, {input_name: tensor})
            infer_elapsed = time.perf_counter() - infer_start
            detections = decode_detections(outputs, bgr.shape[:2], letterbox_meta, args.confidence, args.iou)
            overlay, details = annotated(bgr, detections, depth)
            pipeline_elapsed = time.perf_counter() - pipeline_start
            model_seconds += infer_elapsed
            pipeline_seconds += pipeline_elapsed
            local_model_s += infer_elapsed
            local_pipeline_s += pipeline_elapsed
            timed_inferences += 1
            total_detections += len(details)

            tape_front_m = TAPE_FRONT_M.get(run)
            for detection in details:
                expected_tape = tape_front_m if detection["label"] == "book" else None
                measured = detection["median_depth_m"]
                rows.append({
                    "run": run,
                    "frame_index": frame_index,
                    "stamp_ns": stamp_ns,
                    "label": detection["label"],
                    "confidence": f"{detection['confidence']:.5f}",
                    "x1": detection["box_xyxy"][0],
                    "y1": detection["box_xyxy"][1],
                    "x2": detection["box_xyxy"][2],
                    "y2": detection["box_xyxy"][3],
                    "median_box_depth_m": "" if measured is None else f"{measured:.5f}",
                    "valid_depth_pixels": detection["depth_valid_pixels"],
                    "tape_front_edge_m": "" if expected_tape is None else f"{expected_tape:.3f}",
                    "depth_minus_tape_m": "" if (expected_tape is None or measured is None) else f"{measured - expected_tape:.5f}",
                })

            if frame_index in overlay_at:
                filename = f"overlay-{run}-{frame_index:04d}.png"
                path = out_dir / filename
                if not cv2.imwrite(str(path), overlay):
                    raise OSError(f"failed to write {path}")
                overlay_gif.append(Image.fromarray(cv2.cvtColor(overlay, cv2.COLOR_BGR2RGB)))
            if frame_index in pi_at:
                np.save(pi_frames_dir / f"frame_{exported_pi_frames:03d}.npy", chw_uint8, allow_pickle=False)
                exported_pi_frames += 1
        run_summaries[run] = {
            "context_from_lab_log": RUN_CONTEXT[run],
            "color_frames": len(colors),
            "aligned_depth_frames": len(depths),
            "paired_frames": len(pairs),
            "unmatched_color_frames": unmatched_color,
            "unmatched_depth_frames": unmatched_depth,
            "detections": total_detections,
            "model_seconds": round(local_model_s, 6),
            "model_fps": round(len(pairs) / local_model_s, 3) if local_model_s else None,
            "pipeline_seconds": round(local_pipeline_s, 6),
            "pipeline_fps": round(len(pairs) / local_pipeline_s, 3) if local_pipeline_s else None,
        }
        print(f"{run}: paired={len(pairs)} detections={total_detections} model_fps={run_summaries[run]['model_fps']} pipeline_fps={run_summaries[run]['pipeline_fps']}")

    csv_path = out_dir / "detections.csv"
    fieldnames = ["run", "frame_index", "stamp_ns", "label", "confidence", "x1", "y1", "x2", "y2", "median_box_depth_m", "valid_depth_pixels", "tape_front_edge_m", "depth_minus_tape_m"]
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    gif_path = out_dir / "p1-detections.gif"
    if overlay_gif:
        overlay_gif[0].save(gif_path, save_all=True, append_images=overlay_gif[1:], duration=400, loop=0, optimize=True)

    total_pairs = sum(int(value["paired_frames"]) for value in run_summaries.values())
    summary = {
        "status": "MEASURED",
        "model": str(args.model),
        "model_sha256": sha256(args.model),
        "python_version": sys.version.split()[0],
        "platform": platform.platform(),
        "machine": platform.machine(),
        "onnxruntime_version": ort.__version__,
        "numpy_version": np.__version__,
        "providers": session.get_providers(),
        "intra_op_threads": args.threads,
        "confidence_threshold": args.confidence,
        "iou_threshold": args.iou,
        "depth_scale_m_per_unit_for_16UC1": args.depth_scale,
        "sync_slop_ms": args.sync_slop_ms,
        "frames": total_pairs,
        "mac_model_only_seconds": round(model_seconds, 6),
        "mac_model_only_fps": round(timed_inferences / model_seconds, 3) if model_seconds else None,
        "mac_pipeline_seconds_excluding_bag_decode_and_overlay_write": round(pipeline_seconds, 6),
        "mac_pipeline_fps_excluding_bag_decode_and_overlay_write": round(total_pairs / pipeline_seconds, 3) if pipeline_seconds else None,
        "pi_benchmark_frames_exported": exported_pi_frames,
        "runs": run_summaries,
        "outputs": {"detections_csv": str(csv_path), "gif": str(gif_path), "pi_frames": str(pi_frames_dir)},
    }
    (out_dir / "results.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({k: summary[k] for k in ("frames", "mac_model_only_seconds", "mac_model_only_fps", "mac_pipeline_fps_excluding_bag_decode_and_overlay_write", "pi_benchmark_frames_exported", "model_sha256")}, indent=2))
    if exported_pi_frames != args.pi_frame_count:
        print(f"WARNING: requested {args.pi_frame_count} Pi frames, exported {exported_pi_frames}.")
    print(f"Wrote {len(overlay_gif)} overlays, {gif_path}, {csv_path}, and results.json under {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
