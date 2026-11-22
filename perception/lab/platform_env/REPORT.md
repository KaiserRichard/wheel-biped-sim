# N0 platform environment check report

## Built

- Added a standard-library-only, read-only Pi environment collector and offline `--analyze-only` evaluator.
- Added a single-source manifest with policies and evidence/rationale for each pin. It describes the captured 4 GB Pi 5 stand-in; model and RAM are INFO pending re-pin from reviewed 8 GB target evidence.
- Writes per-command stdout/stderr, `collection.json`, `snapshot.json`, `result.json`, criterion evidence pointers, and SHA-256 records. `--write-manifest-candidate` writes only into the selected output directory.
- The firmware pin is informational because this check is camera-free. Unknown OpenCV/Eigen/clocksource values remain INFO. Storage free bytes are recorded without a made-up threshold.

## Validation

See `tests/last-run.log` for the unittest results. Fixtures include the measured October 13 platform and ORB build provenance from `experiments/2026-10-13-pi5-env-snapshot/` and `experiments/2026-10-13-n4a-euroc-mh01/`. Tests cover matching fields, a required mismatch, missing required data, missing command behavior, and offline analysis.

Fixture success validates the evaluator only and is not an N0 PASS.

## Not verified and risks

- No Pi check was run; the Mac worktree does not represent the deployed ARM environment.
- The expected snapshot describes a 4 GB stand-in. Board model and RAM are recorded as INFO pending a reviewed current snapshot from the 8 GB target. Other environment pins may also need review against that board's current state.
- The camera was not opened. Firmware is recorded as expected, not re-measured at run time; ROS launch/streaming and N0 power/thermal behavior are outside this checker.
- The saved source evidence does not provide OpenCV, Eigen, or clocksource pins. These values are collected when available and marked INFO.
- ORB source/artifact evidence came from saved N4A build provenance. The current tool expects the source tree under `~/wbr-perception-phase1-runtime/third_party/ORB_SLAM3` and built artifacts under `~/orbslam3/ORB_SLAM3`; other layouts yield BLOCKED required fields.
- Free space is descriptive only. This checker does not qualify write throughput or prescribe a storage threshold.

Proposed commit message: `Add camera-free N0 platform environment check`
