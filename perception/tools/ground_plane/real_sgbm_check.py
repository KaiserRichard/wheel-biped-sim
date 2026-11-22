"""Fit the floor plane on offline SGBM depth frames from a real IR stereo bag.

No ground truth exists for these frames: the output is plane validity, fitted
camera height/pitch/roll and inlier support, for inspection only.
"""
import argparse, glob, json, re, sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from plane import estimate_plane  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--depth-dir", default="./Desktop/Research/wbr-data/derived/stereo-depth-20261023")
    ap.add_argument("--out", default=str(Path(__file__).resolve().parent / "results" / "real_sgbm.json"))
    a = ap.parse_args()
    # Floor prior: camera-facing normal roughly "up" (-y in the optical frame),
    # any height up to ~1.2 m, at most 30 deg of camera tilt.
    prior = (np.array([0.0, -1.0, 0.0]), 0.2)
    rows = []
    for f in sorted(glob.glob(f"{a.depth_dir}/*.npz")):
        d = np.load(f)["depth_m"]
        d = np.where((d > 0.2) & (d < 4.0), d, 0).astype(np.float32)
        e = estimate_plane(d, stride=4, iterations=300, threshold=0.015, seed=1, prior=prior,
                           max_deviation=(np.radians(30), 1.0), min_inliers=150)
        valid_px = int((d[::4, ::4] > 0).sum())
        rows.append({"pair": int(re.search(r"pair_(\d+)", f).group(1)), "valid": bool(e.valid),
                     "camera_height_m": float(e.camera_height) if e.valid else None,
                     "pitch_deg": float(np.degrees(e.pitch)) if e.valid else None,
                     "roll_deg": float(np.degrees(e.roll)) if e.valid else None,
                     "inlier_fraction": e.inlier_count / valid_px if e.valid and valid_px else None})
    ok = [r for r in rows if r["valid"]]
    pct = lambda k: [round(float(x), 3) for x in np.percentile([r[k] for r in ok], [5, 50, 95])] if ok else None
    summary = {"frames": len(rows), "valid": len(ok), "camera_height_m_p5_p50_p95": pct("camera_height_m"),
               "pitch_deg_p5_p50_p95": pct("pitch_deg"), "inlier_fraction_p5_p50_p95": pct("inlier_fraction"),
               "note": "No ground truth; handheld low-height loop over glossy tiles. Inspection only."}
    Path(a.out).write_text(json.dumps({"summary": summary, "frames": rows}, indent=1))
    print(json.dumps(summary, indent=1))


if __name__ == "__main__":
    main()
