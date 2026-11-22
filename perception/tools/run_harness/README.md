# Repeatable run harness — C8 / Q3

INFERRED implementation: a Python 3.11 CLI using NumPy and PyYAML. Launches a
caller-supplied headless engine repeatedly in new working directories, with
stdin closed, display variables removed, and no shell argument interpretation.
The engine must itself disable its viewer; `headless: true` is an attestation,
not an automatic ORB patch. No engine installation or hardware access is needed
for tests. Nothing here establishes a gate PASS.

```sh
PYTHONDONTWRITEBYTECODE=1 python3 tools/run_harness/harness.py \
  tools/run_harness/examples/orb_pi.yaml --out tools/run_harness/runs/NEW_RUN
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tools/run_harness/tests -v
```

Fill every `TO-FILL` path/domain in the example before using it on the Pi.
All relative input paths resolve against the spec directory; `--out` resolves
against the caller's working directory and must be new. Output cannot overlap
inputs or lie inside the dataset. Inputs remain in place; data is never copied
into the repository. Engine output belongs in its per-repeat working directory.
The harness assumes a trusted executable that respects the input paths as read
only; process isolation here is a working directory, not a filesystem sandbox.

## Spec

[examples/orb_pi.yaml](examples/orb_pi.yaml) is the complete minimal example.
Required keys: `binary`, `args` (list), `vocabulary`, `settings`, `dataset`,
`times`, `dataset_manifest`, `clock_domain`, `headless: true`, `repeats`, and
`trajectory` with `path`, `time_unit`, `sensor_frame`, `pose_direction`,
`quaternion_order`. Timeouts default to 300 s; jump threshold defaults to 1 m.
Duplicate YAML keys, invalid counts/units/templates/paths and input/output
collisions fail before output creation. Settings are hashed as opaque bytes;
OpenCV YAML does not need to be parsable by PyYAML.

Only `args` strings expand placeholders: `{vocabulary}`, `{settings}`,
`{dataset}`, `{times}`, `{binary}`, `{repeat}` (one-based), `{run_dir}`.
Each string stays one argv element, including spaces. Artifact filenames are
literal relative names, never templates or glob expressions. Declare optional
`extra_trajectories` to hash additional trajectory artifacts. All engine files
inside a repeat directory remain available even if undeclared.

The primary trajectory has eight whitespace-separated columns: stamp, position
in metres, quaternion. Units (`s`/`ns`), order (`xyzw`/`wxyz`), pose direction
and sensor frame (`camera_optical`/`imu_camera`) are explicit. Blank lines and
`#` comments are allowed. Decimal stamps become exact int64 ns; fractional ns,
invalid/nonfinite poses and nonunit quaternions are rejected through `tools/frames`.

Optional `trajectory.epochs` names the engine-produced JSONL sidecar required by
[tools/frames](../frames/README.md), one exact stamp/domain/map/reset record per
pose. The spec clock domain must agree with every epoch. Epoch changes split
segments; no differences cross boundaries. Without the sidecar, actual reset
and segment counts remain null/UNKNOWN; timestamp reversals produce separately
labelled diagnostic segments. Log phrases and position jumps cannot establish
pose epoch identities.

Optional `mount` refers to the existing frame tool's JSON schema with honest
status/provenance and `T_base_camera`. For `imu_camera`, its SDK IR1-to-IMU
extrinsic is also required. Mount conversion requires observed epochs and reuses
`tools.frames.convert()`, writing `base_link.jsonl`. `T_A_B` maps B into A;
`T_world_base = T_world_sensor @ inverse(T_base_sensor)`. Both raw-sensor and
base_link continuity metrics are retained. `slam_world` remains diagnostic;
no odom/TF, mounted calibration, velocity qualification or world ENU is claimed.

Optional `ground_truth` requires `path`, `evaluator`, `clock_domain`, `frame`,
and a nonempty `clock_assumption`. Formats: `tum`/`orbslam3`/`euroc_gt`; units:
`s`/`ns`. Choose the actual child frame, `base_link` after a mount conversion.
Domains must match; any clock mapping happens externally. Defaults: SE3
alignment, 0.01 s association tolerance, RPE delta 1 s, 60 s evaluation timeout.
`align`, `t_max_diff_s`, `rpe_delta_s`, `timeout_s` override these defaults.
Sim3 is diagnostic; it can absorb scale errors.

The CLI reuses the existing `tools/trajectory_eval/traj_eval.py single` interface,
with plots disabled. This worktree does not contain that tool: supply its path,
for example the read-only reference checkout's path on the Mac. No evaluator
code is copied or replaced. The estimate is normalized to sensor-to-world TUM
seconds/xyzw after optional mount conversion, then ATE/RPE is written under
`evaluation/`. Quaternion conversion uses a symmetric eigenproblem, retaining
nonidentity rotations and full integer-ns text precision.

ATE/RPE across observed map/reset changes or nonincreasing timestamps is refused:
the ground-truth schema does not bind references to those epochs. Supply separate
aligned-epoch runs for that case. Evaluator failures are recorded independently
of engine status; partial/crashed trajectories are evaluated when valid, and all
trials remain in the denominator. Ground-truth results are diagnostic and retain
the evaluator's assumptions, hashes, association coverage and warnings.

## Receipts and status

The output contains `resolved_spec.json`, incremental `summary.json`, and
`repeat_001/`, etc. Each repeat contains stdout/stderr, every engine-written
artifact, and `receipt.json`. Receipts preserve argv, UTC and host-monotonic
start/end domains, monotonic wall duration, exit code, host/architecture/Python,
Git HEAD read directly from refs (no Git commands), input/output SHA-256, frame
conventions, continuity/log diagnostics and optional evaluator outcome.
The Git revision describes this repository; binary source revision and dirty
state remain UNKNOWN. Hashes capture the engine bytes, not shared libraries.

Input hashes include spec, binary, vocabulary, settings, times, dataset manifest,
and optional mount/reference/evaluator. The dataset hash covers manifest bytes
only: it is not independent verification of every dataset image/IMU file.
Keep inputs immutable; changed input hashes are reported per repeat and cause
CLI failure. There is no promise of bitwise deterministic asynchronous engine
output or protection against changes that occur and revert between checks.

INFERRED status precedence:

| Status | Meaning |
|---|---|
| TIMEOUT | Engine exceeded timeout; its POSIX process group was killed. |
| CRASH | Nonzero exit/signal or launch error, even if a partial trajectory exists. |
| NO_TRAJECTORY | Exit 0 but primary trajectory/required epoch sidecar missing, empty or invalid. |
| OK | Exit 0 and valid nonempty primary trajectory. |

`OK` does not establish full input coverage, initialized tracking, accuracy or a
gate PASS. A partial exit-0 export remains OK with its count ratio visible.
Continuity records path length, jumps strictly above threshold, observed epoch
segment/reset changes and timestamp-order breaks. The count ratio of output
poses to nonblank times-file rows is only a coverage proxy, never a temporal
coverage guarantee. Missing reset observations can contaminate path length.

Tracking diagnostics count `IMU initialized`, `Reseting`/`Resetting`,
`Fail to track`, map-creation ID messages and final explicit map-count reports.
`distinct_logged_map_count` is a log-derived lower bound, not Atlas size.
`New Map created with N points` does not identify a map. Phrase counts are not
per-frame tracking state durations or authoritative resets; verbosity, multiple
subsystems and buffering can hide or duplicate events.

The summary preserves planned repeats as denominator, all four status counts,
OK fraction, analysis/input-change failures and individual trials. No aggregate
accuracy average drops failed runs silently. Exit 0 means every trial OK with
successful requested analyses and stable hashes; exit 1 means a completed batch
with a failed trial/analysis/input change; exit 2 means invalid spec or I/O error.
An interrupted batch keeps completed receipts and the last incremental summary;
there is no resume/restart or disk-failure recovery protocol.

MEASURED verification is recorded in [REPORT.md](REPORT.md). SOURCE-SUPPORTED
interfaces and log evidence are listed in [SOURCES.md](SOURCES.md).
