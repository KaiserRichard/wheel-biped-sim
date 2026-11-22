# C2-M1A ground-plane geometry report

**Date:** 2026-10-23  
**Scope:** offline synthetic NumPy implementation only.

## What was built

- Analytic pinhole renderer for a pitched/rolled camera, a horizontal floor, and
  axis-aligned boxes, with invalid pixels, range cutoff, and disparity-derived
  axial-depth noise.
- Seeded three-point RANSAC over back-projected samples, optional previous-plane
  gating, SVD refinement, camera-facing plane normal, camera height, pitch,
  roll, support count, and validity.
- Per-pixel signed height and threshold/range obstacle labels, plus a
  fixed-mount baseline. The angular residual model is
  `height_error = d * tan(delta_pitch)`.
- Quick and full evaluation sweeps; JSON metrics and four plots are stored in
  `results/`.

## Claims and assumptions

- Intrinsics 640x480, fx=fy=386.953 px, cx=321.034 px, cy=241.040 px:
  **MEASURED** factory values supplied by the task.
- Stereo baseline 0.050 m: **SOURCE-SUPPORTED** datasheet parameter supplied by
  the task; treated as a configurable value.
- Nominal camera height 0.40 m, obstacle threshold 0.05 m, stereo noise model,
  scene dimensions and evaluation sampling: **ASSUMED** parameters.
- Plane fit and sweep metrics: **CALCULATED** from rendered frames.
- Timing: host measurements for this run only; no Pi timing claim.

## Full sweep results

The full sweep contains 45 scenes (five pitches: 0, 1.5, 3, 4.5, 6 degrees;
three disparity standard deviations: 0.05, 0.1, 0.2 px; three seeds) and 90
method rows. Every scene contains four boxes, each 3–15 cm tall and placed
0.6–3.0 m forward. Plane fitting and obstacle scores use stride 4.

| Method | Invalid planes | Mean normal-angle error | Mean height error | Pooled obstacle precision | Pooled obstacle recall |
|---|---:|---:|---:|---:|---:|
| Per-frame plane | 0/45 (0%) | 0.0191 deg | 0.000326 m | 0.995 | 0.984 |
| Fixed mount (0 deg, 0.40 m) | N/A (no fit) | = true pitch | see note | 0.069 | 0.995 |

Scoring (corrected by the supervisor, see below): positive = box pixel whose
true height is at least the 5 cm threshold; negative = floor pixel; box pixels
below 5 cm are ignored. Counts are pooled over all 45 scenes. Fixed-mount
"height error" is not comparable with the per-frame camera-height error; its
expected point-height error is `d * tan(pitch)` (CALCULATED).

Per pitch (pooled precision / recall):

| True pitch | Per-frame plane | Fixed mount |
|---:|---:|---:|
| 0 deg | 0.994 / 0.972 | 0.995 / 0.976 |
| 1.5 deg | 0.995 / 0.991 | 0.162 / 1.000 |
| 3 deg | 0.995 / 0.988 | 0.067 / 1.000 |
| 4.5 deg | 0.996 / 0.983 | 0.049 / 1.000 |
| 6 deg | 0.996 / 0.988 | 0.031 / 1.000 |

By axial depth Z (pooled precision / recall):

| Method | 0–1 m | 1–2 m | 2–3 m |
|---|---:|---:|---:|
| Per-frame plane | 1.000 / 0.987 | 1.000 / 0.984 | 1.000 / 0.970 |
| Fixed mount | 0.113 / 0.994 | 0.065 / 0.999 | 0.043 / 0.993 |

Reading: once the body pitches by 1.5 deg or more, a fixed-mount assumption
flags large areas of floor as obstacles (precision 0.16 down to 0.03), while the
per-frame plane keeps precision near 0.99. This is the easy case: flat floor,
independent Gaussian disparity noise, many floor pixels.

Supervisor correction (2026-10-23): the runner's first scoring counted every box
pixel as positive, including box sides below 5 cm, which capped recall at about
0.55, and its per-bin recall divided all predictions (including false
positives) by predictions plus misses. Both are fixed in `evaluate.py`; the old
"any box pixel" recall is kept as `recall_any_box_pixel` (per-frame 0.554,
fixed mount 0.874).

Full-run host timings after the correction rerun (three repetitions per stride,
Mac, Python, 80 RANSAC iterations on one benchmark frame):

| Pixel stride | Mean ms/frame | Samples (ms) |
|---:|---:|---:|
| 1 | 69.97 | 87.08, 52.38, 70.46 |
| 4 | 11.30 | 12.92, 10.25, 10.75 |

These are not a Raspberry Pi estimate.

Plots: [normal error](results/normal_error.png),
[height error](results/height_error.png),
[recall by range](results/recall_by_range.png),
[timing](results/timing.png). Raw per-scene records are in
[metrics.json](results/metrics.json).

## Tests and evaluator runs

Final unit command:

```text
python3 -m unittest discover -s tools/ground_plane/tests -v
```

Actual output:

```text
test_fixed_mount_angular_height_error (test_geometry.GroundPlaneTests.test_fixed_mount_angular_height_error) ... ok
test_floor_depth_at_bottom_row (test_geometry.GroundPlaneTests.test_floor_depth_at_bottom_row) ... ok
test_noiseless_plane_recovery (test_geometry.GroundPlaneTests.test_noiseless_plane_recovery) ... ok
test_ransac_with_half_depth_outliers (test_geometry.GroundPlaneTests.test_ransac_with_half_depth_outliers) ... ok

----------------------------------------------------------------------
Ran 4 tests in 0.119s

OK
```

Quick command:

```text
MPLCONFIGDIR=/private/tmp/wbr-ground-plane-mpl python3 tools/ground_plane/evaluate.py --quick
```

Actual result: exit 0, 18 method rows (9 scenes), metrics written to
`tools/ground_plane/results/metrics.json`; final quick timing was 137.19
ms/frame at stride 1 and 19.42 ms/frame at stride 4 (one sample each; not used
in the full results table).

Full command:

```text
MPLCONFIGDIR=/private/tmp/wbr-ground-plane-mpl python3 tools/ground_plane/evaluate.py
```

Actual result: exit 0, 90 method rows (45 scenes). Metrics JSON and all four
PNGs were generated. Full-run timing and aggregate metrics appear above.

## Failure retained

The first quick CLI attempt exited 1 after Matplotlib initialized. The direct
script import fallback loaded `obstacles.py`, whose unused package-relative
import of `plane` then failed with `ImportError: attempted relative import with
no known parent package`. The unused import was removed. The subsequent quick
and full direct-script invocations both exited 0.

## Limitations

- Synthetic geometry only; no RealSense frames or measured robot pitch/roll.
- Flat floor, pinhole optics and axis-aligned boxes; no floor texture, sloped or
  uneven ground, camera calibration error, rolling shutter, multipath,
  occlusion artifacts, or real stereo matching failure patterns.
- Disparity noise is independent Gaussian noise with the supplied idealized
  `sigma_Z = Z^2 * sigma_d / (f * B)` relation; invalid pixels are random.
- Plane support and object distribution are scene-dependent. The current
  precision/recall scores include only rendered valid pixels.
- No Pi timing or end-to-end ROS integration was measured.

## Real-data first contact (supervisor, 2026-10-23)

Input: 148 offline SGBM depth frames (`tools/stereo_depth`, branch
`claude2/20261023-stereo-depth`) from the 21 Oct handheld marked-loop IR bag.
The camera was held low over a glossy tiled floor. Command:
`python3 tools/ground_plane/real_sgbm_check.py`
(writes `results/real_sgbm.json`).

Two plane-gate bugs found on real data and fixed in `plane.py`:
1. The prior gate accepted sign-flipped normals (`min(ang, pi-ang)`), so seat
   and desk undersides above the camera were accepted as "floor" (pitch about
   178 deg). The camera-facing normal must now agree with the prior direction.
2. Planes passing through the camera centre (a near wall filling the view)
   had an undefined side. Planes closer than `min_height` (0.05 m) are rejected.
Synthetic results above are unchanged after both fixes (rerun: per-frame
0.995 / 0.984, fixed mount 0.069 / 0.995, 0/45 invalid).

Result (no ground truth, inspection only): 148/148 frames gave a plane; fitted
camera height p5/p50/p95 = 0.103 / 0.231 / 0.326 m, pitch -5.4 / 0.3 / 22.7 deg,
inlier fraction 0.065 / 0.103 / 0.197 of valid pixels. Low inlier support is
expected from SGBM on passive IR (emitter off): the tiles have little texture
and mirror the chairs, so floor depth is sparse and partly wrong. Whether each
fitted plane is the actual floor is UNKNOWN without a measured camera height.
