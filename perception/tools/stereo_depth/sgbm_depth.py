"""Offline dense stereo depth from the project's rectified D435 MCAP."""
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import cv2
import numpy as np

from mcap_stereo import LEFT_IMAGE, Calibration, iter_stereo_pairs, read_calibration

DEFAULT_MCAP = Path(
    "./wbr-evidence-archive/camera-recording-20261021-01/"
    "mcap-marked-loop-01/sensor-bag/sensor-bag_0.mcap"
)
DEFAULT_OUTPUT = Path(
    "./Desktop/Research/wbr-data/derived/stereo-depth-20261023"
)


def disparity_count(calibration: Calibration, min_depth_m: float = 0.3) -> int:
    """Smallest OpenCV-compatible search range covering fB/min_depth."""
    return max(16, int(math.ceil((calibration.fb / min_depth_m) / 16.0) * 16))


def compute_depth(left: np.ndarray, right: np.ndarray, calibration: Calibration,
                  *, num_disparities: int | None = None,
                  consistency_px: float = 1.5) -> tuple[np.ndarray, np.ndarray]:
    """Return metric depth and a valid mask using left-right consistency.

    SGBM uses 5x5 blocks, P1=8*blockSize^2, P2=32*blockSize^2,
    disp12MaxDiff=1, preFilterCap=31, uniquenessRatio=10,
    speckleWindowSize=100 and speckleRange=2. Right disparity is computed
    on swapped images with a negative disparity search range.
    """
    if left.ndim != 2 or right.ndim != 2 or left.shape != right.shape:
        raise ValueError("left and right must be same-size grayscale images")
    nd = num_disparities or disparity_count(calibration)
    if nd % 16:
        raise ValueError("num_disparities must be a multiple of 16")
    block = 5
    common = dict(
        blockSize=block,
        P1=8 * block * block,
        P2=32 * block * block,
        disp12MaxDiff=1,
        preFilterCap=31,
        uniquenessRatio=10,
        speckleWindowSize=100,
        speckleRange=2,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
    )
    left_matcher = cv2.StereoSGBM_create(minDisparity=0, numDisparities=nd, **common)
    right_matcher = cv2.StereoSGBM_create(minDisparity=-nd, numDisparities=nd, **common)
    disp_l = left_matcher.compute(left, right).astype(np.float32) / 16.0
    disp_r = right_matcher.compute(right, left).astype(np.float32) / 16.0

    h, w = left.shape
    x = np.broadcast_to(np.arange(w, dtype=np.float32), (h, w))
    xr = np.rint(x - disp_l).astype(np.int32)
    in_bounds = (xr >= 0) & (xr < w)
    xr_safe = np.clip(xr, 0, w - 1)
    rows = np.broadcast_to(np.arange(h)[:, None], (h, w))
    sampled_r = disp_r[rows, xr_safe]
    valid = (
        in_bounds & (disp_l > 0.0) & (disp_r[rows, xr_safe] < 0.0)
        & (np.abs(disp_l + sampled_r) <= consistency_px)
    )
    depth = np.zeros(left.shape, dtype=np.float32)
    depth[valid] = (calibration.fb / disp_l[valid]).astype(np.float32)
    depth[~np.isfinite(depth)] = 0.0
    return depth, valid


def colorize_depth(depth: np.ndarray, max_depth_m: float) -> np.ndarray:
    valid = depth > 0
    gray = np.zeros(depth.shape, dtype=np.uint8)
    gray[valid] = np.clip(depth[valid] / max_depth_m * 255.0, 0, 255).astype(np.uint8)
    colored = cv2.applyColorMap(gray, cv2.COLORMAP_TURBO)
    # Invalid pixels are black; otherwise they share the colour of near depth.
    colored[~valid] = 0
    return colored


def _save_contact_sheet(items: list[tuple[int, np.ndarray, np.ndarray]], path: Path,
                        max_depth_m: float) -> None:
    if not items:
        raise RuntimeError("No frames selected; cannot make contact sheet")
    tiles = []
    target_w = 320
    for index, left, depth in items:
        h, w = left.shape
        scale = target_w / w
        left_small = cv2.resize(left, (target_w, int(h * scale)))
        left_bgr = cv2.cvtColor(left_small, cv2.COLOR_GRAY2BGR)
        preview = cv2.resize(colorize_depth(depth, max_depth_m), (target_w, int(h * scale)))
        label = f"pair {index}"
        cv2.putText(left_bgr, label, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        tiles.append(np.hstack((left_bgr, preview)))
    cols = 2
    rows = math.ceil(len(tiles) / cols)
    tile_h, tile_w = tiles[0].shape[:2]
    sheet = np.zeros((rows * tile_h, cols * tile_w, 3), dtype=np.uint8)
    for i, tile in enumerate(tiles):
        y, x = divmod(i, cols)
        sheet[y * tile_h:(y + 1) * tile_h, x * tile_w:(x + 1) * tile_w] = tile
    if not cv2.imwrite(str(path), sheet):
        raise OSError(f"Failed to write {path}")


def run(mcap: Path, output: Path, every: int = 15, max_depth_m: float = 8.0,
        frames: int | None = None) -> dict:
    calibration = read_calibration(mcap)
    nd = disparity_count(calibration)
    output.mkdir(parents=True, exist_ok=True)
    pair_stream, counts = iter_stereo_pairs(mcap)
    selected: list[tuple[int, np.ndarray, np.ndarray]] = []
    frame_stats = []
    full_times, half_times = [], []
    pair_count = 0
    for pair_index, pair in enumerate(pair_stream):
        pair_count += 1
        if pair_index % every != 0:
            continue
        if frames is not None and len(frame_stats) >= frames:
            # Continue consuming so the unpaired counts reflect the full bag.
            continue
        t0 = time.perf_counter()
        depth, valid = compute_depth(pair.left, pair.right, calibration, num_disparities=nd)
        full_times.append((time.perf_counter() - t0) * 1000.0)
        half_left = cv2.resize(pair.left, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
        half_right = cv2.resize(pair.right, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
        half_cal = Calibration(calibration.fx / 2, calibration.cx / 2, calibration.cy / 2,
                               calibration.baseline_m, half_left.shape[1], half_left.shape[0])
        t0 = time.perf_counter()
        compute_depth(half_left, half_right, half_cal,
                      num_disparities=max(16, int(math.ceil((nd / 2) / 16) * 16)))
        half_times.append((time.perf_counter() - t0) * 1000.0)

        stamp = pair.stamp_ns
        stem = f"frame_{len(frame_stats):04d}_pair_{pair_index:04d}_{stamp}"
        npz_path = output / f"{stem}.npz"
        np.savez_compressed(npz_path, depth_m=depth)
        png_path = output / f"{stem}_depth_mm.png"
        depth_mm = np.clip(np.rint(depth * 1000), 0, 65535).astype(np.uint16)
        if not cv2.imwrite(str(png_path), depth_mm):
            raise OSError(f"Failed to write {png_path}")
        preview_path = output / f"{stem}_preview.png"
        if not cv2.imwrite(str(preview_path), colorize_depth(depth, max_depth_m)):
            raise OSError(f"Failed to write {preview_path}")
        vals = depth[valid]
        stats = {
            "pair_index": pair_index,
            "stamp_ns": stamp,
            "valid_fraction": float(valid.mean()),
            "median_depth_m": float(np.median(vals)) if vals.size else None,
            "p5_depth_m": float(np.percentile(vals, 5)) if vals.size else None,
            "p95_depth_m": float(np.percentile(vals, 95)) if vals.size else None,
            "npz": npz_path.name,
            "depth_mm_png": png_path.name,
            "preview_png": preview_path.name,
        }
        frame_stats.append(stats)
        # Keep roughly 12 evenly spaced examples after the selected frame count is known.
        selected.append((pair_index, pair.left.copy(), depth))
    if pair_count == 0:
        raise RuntimeError("No stereo pairs found")
    if frames is None and len(selected) > 12:
        indices = np.linspace(0, len(selected) - 1, 12).round().astype(int)
        selected = [selected[i] for i in indices]
    _save_contact_sheet(selected[:12], output / "contact_sheet.png", max_depth_m)
    params = {
        "algorithm": "OpenCV StereoSGBM",
        "num_disparities": nd,
        "min_depth_target_m": 0.3,
        "block_size": 5,
        "P1": 200,
        "P2": 800,
        "disp12_max_diff": 1,
        "pre_filter_cap": 31,
        "uniqueness_ratio": 10,
        "speckle_window_size": 100,
        "speckle_range": 2,
        "left_right_consistency_px": 1.5,
        "max_preview_depth_m": max_depth_m,
        "process_every_nth_pair": every,
        "frames_limit": frames,
        "half_resolution_benchmark": "images resized by 0.5; f,cx,cy and disparity range halved",
    }
    summary = {
        "input_mcap": str(mcap),
        "pair_counts": {**counts, "paired": pair_count},
        "calibration_measured_from_file": {
            "fx_px": calibration.fx, "cx_px": calibration.cx, "cy_px": calibration.cy,
            "baseline_m": calibration.baseline_m, "fB_px_m": calibration.fb,
            "width": calibration.width, "height": calibration.height,
        },
        "parameters": params,
        "processed_frames": len(frame_stats),
        "timing_ms_per_frame": {
            "full_resolution_mean": float(np.mean(full_times)) if full_times else None,
            "full_resolution_median": float(np.median(full_times)) if full_times else None,
            "half_resolution_mean": float(np.mean(half_times)) if half_times else None,
            "half_resolution_median": float(np.median(half_times)) if half_times else None,
        },
        "frames": frame_stats,
        "contact_sheet": "contact_sheet.png",
        "notes": [
            "Calibration values are measured from this MCAP CameraInfo.",
            "Depth accuracy is UNKNOWN because the recording has no ground-truth depth.",
            "Passive IR with emitter off can produce invalid depth on untextured or low-texture surfaces.",
        ],
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(f"MEASURED fx={calibration.fx:.6f}px cx={calibration.cx:.6f}px cy={calibration.cy:.6f}px "
          f"B={calibration.baseline_m:.9f}m fB={calibration.fb:.6f}px*m")
    print(f"Pairs: {pair_count}; unpaired left={counts['unpaired_left']} right={counts['unpaired_right']}; "
          f"processed={len(frame_stats)}")
    print(f"Timing mean ms/frame: full={summary['timing_ms_per_frame']['full_resolution_mean']:.2f}, "
          f"half={summary['timing_ms_per_frame']['half_resolution_mean']:.2f}")
    print(f"Output: {output}")
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mcap", type=Path, default=DEFAULT_MCAP)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--every", type=int, default=15, help="process every Nth paired image")
    parser.add_argument("--frames", type=int, help="limit processed frames (still scans the whole bag)")
    parser.add_argument("--max-depth", type=float, default=4.0, help="preview color scale in metres")
    args = parser.parse_args()
    if args.every < 1 or (args.frames is not None and args.frames < 1):
        parser.error("--every and --frames must be positive")
    run(args.mcap, args.output, args.every, args.max_depth, args.frames)


if __name__ == "__main__":
    main()
