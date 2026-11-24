# phase-18 report — Kalibr result acceptance gate

## Built

- Added `check_kalibr_result.py` and a single `thresholds.yaml` policy file.
- Reused `summarize_kalibr.py` parsers, transform validation, direction
  conventions, and factory stereo camchain values.
- Added result-contract JSON with criterion-level PASS/FAIL/BLOCKED status,
  provenance strings, input SHA-256 hashes, a human-readable table, raw input
  snapshots, and a factory diff table.
- Added saved-input `--analyze-only` mode.
- Added synthetic perturbation, provenance mismatch, missing-input and replay
  tests. Added the real TUM-VI smoke report and camchain as parser-only fixture
  material; it is not D435i calibration evidence.

## Test results

`python3 -m unittest discover -s lab/calib_result/tests -v` passes 8 tests.
Coverage includes fixture PASS, residual/extrinsic/time/baseline/intrinsic/
distortion FAIL cases, IMU noise provenance FAIL, missing-input BLOCKED, and
offline saved-input reanalysis, missing/invalid saved threshold policy BLOCKED
behavior, and usage exit-code handling. The real TUM-VI result verifies parser
behavior only.

## Not verified and open risks

- No Pi, camera, hardware, or production Kalibr run was used for this work.
- No real D435i stereo Kalibr result was present in the worktree or the shared
  checkout experiment tree. The checked-in real report is a TUM-VI fisheye
  smoke run and cannot pass this D435i gate.
- Thresholds for reprojection mean/std, gyro/accel residuals, time shift,
  intrinsic drift and distortion are **PROPOSED** pending real D435i review.
  The documented 5 mm / 2° extrinsic and 1 mm baseline limits remain ASSUMED.
- The current factory camchain YAML contains factory intrinsics/distortion and
  stereo transform but no camera–IMU transform. The checker uses the rounded
  factory/CAD camera-to-IMU reference constant already defined in the existing
  summary tool; this is not independent metrology.
- Single-run time-shift magnitude does not establish repeatability across
  three takes. Scalar residuals do not reveal spatial patterns or target/data
  problems. ORB-SLAM3 use still requires review of PDFs, receipts, clocks and
  runtime behavior.

## Proposed commit message

`Add Kalibr result acceptance gate`
