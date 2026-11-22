#!/usr/bin/env python3
"""Benchmark ONNX Runtime CPU inference on preprocessed CHW uint8 NumPy frames."""

from __future__ import annotations

import argparse
import hashlib
import json
import platform
import sys
import time
from pathlib import Path

import numpy as np
import onnxruntime as ort


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def prepare_input(chw: np.ndarray, kind: str) -> np.ndarray:
    if chw.dtype != np.uint8 or chw.ndim != 3 or chw.shape[0] != 3:
        raise ValueError(f"expected uint8 CHW RGB frame, got dtype={chw.dtype}, shape={chw.shape}")
    if kind == "tensor(float)":
        return np.ascontiguousarray(chw.astype(np.float32)[None, ...] / 255.0)
    if kind == "tensor(float16)":
        return np.ascontiguousarray((chw.astype(np.float32)[None, ...] / 255.0).astype(np.float16))
    if kind == "tensor(uint8)":
        return np.ascontiguousarray(chw[None, ...])
    raise ValueError(f"unsupported ONNX input type {kind}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--frames-dir", type=Path, required=True)
    parser.add_argument("--threads", type=int, default=0, help="ONNX Runtime intra-op threads; 0 selects its default")
    parser.add_argument("--passes", type=int, default=10, help="repeat the frame set to reduce timer noise")
    parser.add_argument("--warmup", type=int, default=5)
    args = parser.parse_args()
    if not args.model.is_file():
        raise FileNotFoundError(args.model)
    files = sorted(args.frames_dir.glob("frame_*.npy"))
    if not files:
        raise FileNotFoundError(f"no frame_*.npy files in {args.frames_dir}")
    frames = [np.load(path, allow_pickle=False) for path in files]
    options = ort.SessionOptions()
    options.intra_op_num_threads = max(0, args.threads)
    options.inter_op_num_threads = 1
    session = ort.InferenceSession(str(args.model), sess_options=options, providers=["CPUExecutionProvider"])
    input_meta = session.get_inputs()[0]
    input_name = input_meta.name

    def invoke(frame: np.ndarray) -> tuple[float, float]:
        prepare_start = time.perf_counter()
        tensor = prepare_input(frame, input_meta.type)
        prepare_elapsed = time.perf_counter() - prepare_start
        infer_start = time.perf_counter()
        session.run(None, {input_name: tensor})
        infer_elapsed = time.perf_counter() - infer_start
        return prepare_elapsed, infer_elapsed

    for index in range(max(0, args.warmup)):
        invoke(frames[index % len(frames)])

    prepare_seconds = infer_seconds = 0.0
    timed_frames = 0
    for _ in range(args.passes):
        for frame in frames:
            prepare_elapsed, infer_elapsed = invoke(frame)
            prepare_seconds += prepare_elapsed
            infer_seconds += infer_elapsed
            timed_frames += 1
    result = {
        "status": "MEASURED",
        "model": str(args.model),
        "model_sha256": sha256(args.model),
        "python_version": sys.version.split()[0],
        "platform": platform.platform(),
        "machine": platform.machine(),
        "onnxruntime_version": ort.__version__,
        "numpy_version": np.__version__,
        "providers": session.get_providers(),
        "threads": args.threads,
        "warmup_inferences": max(0, args.warmup),
        "unique_frames": len(frames),
        "passes": args.passes,
        "timed_inferences": timed_frames,
        "prepare_seconds": round(prepare_seconds, 6),
        "inference_seconds": round(infer_seconds, 6),
        "inference_fps": round(timed_frames / infer_seconds, 3) if infer_seconds else None,
        "prepare_plus_inference_fps": round(timed_frames / (prepare_seconds + infer_seconds), 3) if prepare_seconds + infer_seconds else None,
    }
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
