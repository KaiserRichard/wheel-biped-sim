# Camera tilt sweep: obstacle height and advisory

## Result

**MEASURED on synthetic data:** across the full requested grid, per-frame plane fitting kept height error below 2.6 mm for all detected obstacles and all 486 fitted planes were valid. The known-pose fixed-mount height map had 121 mm mean absolute height error, and its advisory matched the expected advisory in 204/486 cases. At +5° pitch and the 0.132 m box (the lab-day 3-box13 measured reference at approximately 1.0 m), mean fixed-mount height bias was +93 mm at 1.0 m; the plane-fit bias was -0.14 mm. The synthetic object-height/pose combination is not a remeasurement of the laboratory detector result.

**CALCULATED analytic check:** the first-order angular vertical residual `d * sin(delta_theta)` at 5° is 43.6 mm, 87.2 mm, and 113.3 mm at 0.5 m, 1.0 m, and 1.3 m. This matches the measured fixed-mount trend: the +5°/0-roll/0.132 m group mean errors are +50.1 mm, +93.4 mm, and +119.2 mm. The fixed-mount extraction uses the rendered obstacle mask to isolate obstacle returns, then applies the fixed plane and the normal advisory policy; it isolates pose-induced height error from detector clustering and is not a fully independent end-to-end detector.

## Method and command

The renderer and detector were read-only. The experiment used `tools/ground_plane/synth.py::render`, `tools/obstacle_decision/detect.py::process_frame`, and `tools/obstacle_decision/decide.py::update`. Plane fitting followed `tools/obstacle_decision/evaluate.py`: five repeated frames for temporal confirmation. Grid: pitch -15,-10,-5,-2,0,2,5,10,15°; roll 0,5,10°; box distance 0.5,1.0,1.3 m; height 0.046 or 0.132 m; seeds 0,1,2 (486 scenes, 972 method rows). The box is 0.30 m wide and 0.12 m deep. Synthetic depth noise is `sigma_d=0.1 px`, using the renderer's disparity-derived model.

Camera height 0.40 m, intrinsics fx=fy=386.953 px and cx=321.034, cy=241.040 px, and stereo baseline 0.050 m follow the defaults in `tools/obstacle_decision/evaluate.py` and `tools/ground_plane/synth.py`. The detector's default sampling/thresholds and decision parameters were retained; maximum jump height is 0.12 m. Expected label is JUMP_CANDIDATE for the 0.046 m box and STOP_CANDIDATE for the 0.132 m box, following the 0.12 m maximum jump-height threshold.

Run command:

```sh
MPLBACKEND=Agg MPLCONFIGDIR=/private/tmp/wbr-mpl-cache python3 tools/tilt_sweep/sweep.py
```

Runtime: approximately 5 minutes on this Mac. Outputs: `results/metrics.csv`, `results/summary.json`, and the three plots below.

## Aggregate measurements

Errors are absolute-error aggregates over detected boxes only. Advisory accuracy uses all 486 scenes per method; missing detections count as incorrect.

| Method | Height MAE / max | Distance MAE / max | Plane valid | Correct advisory |
|---|---:|---:|---:|---:|
| Per-frame plane fit | 0.00021 / 0.00258 m (n=435 detected) | 0.0152 / 0.1165 m (n=435) | 486/486 | 435/486 |
| Fixed mount (0° pitch, 0° roll) | 0.1214 / 0.3613 m (n=435 detected) | 0.0685 / 0.2122 m (n=435) | Assumed nominal plane | 204/486 |

The 51 missed plane-fit detections all had a valid plane and returned NONE. Representative missed cases (all three seeds): pitch=-15°, roll=0°, distance=0.5 m, height=0.046 m; pitch=-15°, roll=0°, distance=0.5 m, height=0.132 m; pitch=-10°, roll=0°, distance=0.5 m, height=0.046 m. The complete case rows are in `results/metrics.csv`. These failures arise when upward pitch places the close box at/near the lower image boundary and leaves little visible floor. Plane validity alone therefore does not guarantee obstacle visibility or detection. At the most extreme upward tilt, some planes remain valid while the box is missed.

For the 0.132 m box at +5° pitch, roll=0°, the nine noise samples at each distance yielded:

| Distance | Plane-fit mean height error | Fixed-mount mean height error | Fixed-mount range error |
|---:|---:|---:|---:|
| 0.5 m | -0.00013 m | +0.0501 m | -0.0384 m |
| 1.0 m | -0.00014 m | +0.0934 m | -0.0446 m |
| 1.3 m | -0.00014 m | +0.1192 m | -0.0510 m |

The fixed-mount method produced only 204 correct advisories because pose error shifts obstacle heights across the 0.03 m minimum and 0.12 m maximum jump-height thresholds. The 0.132 m box should produce STOP_CANDIDATE; height underestimation can instead produce JUMP_CANDIDATE or UNCERTAIN. Upward and rolled poses can also produce positive floor residuals under a fixed plane, so fixed-pose false positives are possible; the main table scores the rendered box advisory, not floor false-positive rate.

## Evidence status and limits

- **MEASURED on synthetic data:** numerical values, detection misses, plane validity and advisory outcomes above, generated from the saved seeded CSV.
- **CALCULATED analytic check:** `d*sin(5°)` values above; `tools/ground_plane/obstacles.py::angular_height_error` uses `d*tan(delta)` as its exact planar residual model. The sine form is the small-angle vertical component requested for comparison.
- **MEASURED reference (separate lab data):** lab-day 3-box13 reports approximately 0.132 m height at approximately 1.0 m. This is context only; the sweep did not ingest lab data.
- This uses flat ground, ideal pinhole geometry, axis-aligned boxes and independent Gaussian disparity noise. It does not model body angular acceleration, rolling-shutter timing, stereo matching artifacts, uneven floors, or camera calibration changes. The fixed-mount path uses synthetic labels to select obstacle returns and does not measure end-to-end false-positive rates. Advisory is informational and not jump permission.

Plots:

- [Height error vs pitch](results/height_error_vs_pitch.png)
- [Advisory correctness heatmap](results/advisory_correctness.png)
- [Rendered obstacle support](results/obstacle_support.png)

## Test

Command:

```sh
MPLBACKEND=Agg MPLCONFIGDIR=/private/tmp/wbr-mpl-cache python3 tools/test_support/run_unittest.py tools/tilt_sweep
```

The targeted test checks the 0° and 5° cases at 1.0 m for the 0.132 m box. It asserts plane-fit height error below 1 cm and fixed-mount error within 2.5 cm of `d*sin(5°)`. Actual command output is recorded after execution.

Actual test output:

```text
test_pose_compensation_and_fixed_mount_error (tests.test_sweep.TiltSweepTests.test_pose_compensation_and_fixed_mount_error) ... ok
----------------------------------------------------------------------
Ran 1 test in 1.150s
OK
DISCOVERED: 1 test files, 1 tests, 0 skipped
```

The first runner attempt exited 2 with `BLOCKED: no tests discovered from tests/test_sweep.py`; unittest discovery required `tests/__init__.py` in this repository's runner. Adding that package marker fixed discovery, and the exact runner command above then exited 0.

## Repository handoff limitation

The runner sandbox could not write git metadata. The maintainer committed and opened the PR.
