# phase-9 N4B offline toolchain report

## Built

- Promoted and attributed the 2026-10-14 DB3 converter, changed timestamp selection to ROS `header.stamp`, ignored bag receive timestamps, preserved integer nanoseconds, validates CDR/mono8 payloads and source stream ordering, writes EuRoC PNG/CSV and nanosecond time files, and emits metadata with clock and IMU-combination policy.
- Added `qualify_euroc.py` and one JSON threshold file with source/rationale labels. Qualification supports `--analyze-only` and the required PASS/FAIL/BLOCKED contract.
- Added `make_orb_yaml.py` reusing the existing calibration converter, with provenance labels per generated field and source hashes.
- Added Pi-side `run_n4b.sh` with input/binary hash capture, dynamic-link check, GNU time receipt, optional profiler invocation, and refusal when patched-build identity is unknown/mismatched unless explicitly overridden.

## Verification

Six N4B unittest cases pass: synthetic DB3 round-trip/qualification PASS, injected IMU order reversal FAIL, missing camera BLOCKED, usage exit-code contract, runner library-path/provenance capture, and a MEASURED saved-stream summary fixture check. All 16 existing `calib_tools` tests also pass. Exact test output is in `tests/last-run.log`. `make_orb_yaml.py` was exercised against the repository calibration inputs and emitted 45 per-field provenance entries. `bash -n` passed for `run_n4b.sh`.

A real `.db3` bag was not present in the shared checkout. The real-evidence fixture is a saved summary (447 IR frames per camera at 29.992 Hz, gyro 2971 at 200.159 Hz, accel 1507 at 100.978 Hz) and is not used as a synthetic gate PASS. No camera, Pi, SSH, ORB binary or hardware was accessed. N4B remains OPEN.

## Not verified / open risks

- Real bag schema variants, compressed bags, or alternate ROS CDR encodings.
- Driver unite-IMU method where no combined IMU messages exist; exported accel interpolation is independently stated.
- Patched Pi ORB rebuild and binary SHA-256. The saved evidence says its rebuild was interrupted, so the manifest is `UNKNOWN` and the runner refuses by default.
- ORB settings parser/runtime, actual image tracking, inertial initialization, calibration accuracy, time offset, long-take memory/runtime, or Pi power/thermal/storage behavior.
- Proposed qualification limits are fixture-oriented screening values and require review against the eventual recording profile.

Proposed commit message: `Add offline D435i N4B bag export and qualification tools`
