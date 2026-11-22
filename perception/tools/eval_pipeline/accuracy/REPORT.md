# phase-5 accuracy pipeline report

## Built

- `run_euroc.sh`: controlled, repeated Pi invocation of upstream `stereo_inertial_euroc`, with a non-overlap file lock, executable/library/vocabulary/settings/timestamp SHA-256, `ldd`, per-run UTC, throttling snapshots, command, exit status and stdout/stderr.
- `evaluate.py`: discovers supported ORB trajectory names, reuses `tools/trajectory_eval/traj_eval.py`, evaluates SE3 primary and Sim3 secondary ATE plus RPE at 1 m and 10 frames, counts initialization/loss/reset log phrases and pose coverage, aggregates metrics and writes `summary.json` and `report.md`.
- `thresholds.json`, README, unit tests and real MH01 validation evidence.

## Verification

- Real saved N4A trajectories and ground truth reproduce SE3 ATE RMSE 0.041461294, 0.036844344 and 0.041603883 m, matching the historical report to its rounding. The result is PASS against proposed 0.046 m ATE and 0.95 pose-fraction thresholds. Historical initialization lines were present in each stdout. Pose fraction uses the saved report's camera count (3,682); the original timestamp file was unavailable in the evidence bundle.
- Validation artifacts are [report.md](validation/report.md) and [summary.json](validation/summary.json). They refer to saved experiment inputs by absolute path and hash their ground truth and trajectories; no raw data was copied into this worktree.
- Review fixes reject nonzero/missing engine exit status as FAIL/BLOCKED, use a host-wide lock for the runner, reject ambiguous trajectory files, and fail pose-count ratios above 1.0. Legacy saved runs resolve their exit codes from the bundle's `runs.log`.
- Unit tests: `python3 -m unittest discover -s tools/eval_pipeline/accuracy/tests -v` — 10 tests passed, including real-data validation; exact output is in [last-run.log](tests/last-run.log).
- Shell syntax: `bash -n tools/eval_pipeline/accuracy/run_euroc.sh` — passed.

## Not verified / risks

- The Pi runner was not executed here; no hardware, camera, serial device, remote host or network was accessed. Pi library paths, `flock`, `vcgencmd`, executable behavior, storage and real run timing remain unverified.
- Phrase-count diagnostics cannot prove absence of tracking loss or map resets. Pose fraction is a count proxy, not temporal coverage. The source ground-truth/body and inertial frame labels do not independently prove frame equivalence; no extrinsics are applied.
- Dataset clock-sharing is source-supported for EuRoC files, not a live clock mapping. The proposed thresholds require project review and do not promote any project gate.

Proposed commit message: `Add repeated EuRoC accuracy evaluation pipeline`
