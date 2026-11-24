# Offline trajectory evaluation

`traj_eval.py` is a reusable CLI and importable module for calculated absolute
trajectory error (ATE) and relative pose error (RPE). It reads existing files on
the Mac. It does not start estimators, capture sensors, access hardware, use the
network, or establish a gate PASS. Version: `1.0.0`.

## Dependencies and execution

Python 3.11 or newer (tested on 3.11 and 3.13) and NumPy are required. Matplotlib is optional and imported only when
writing plots. The validated environment has Python 3.11.0rc2, NumPy 1.26.4, and
Matplotlib 3.10.9. No build or package installation is needed.

From the repository root:

```sh
PYTHONDONTWRITEBYTECODE=1 python3 tools/trajectory_eval/traj_eval.py --help
PYTHONDONTWRITEBYTECODE=1 python3 tools/trajectory_eval/traj_eval.py single --help
PYTHONDONTWRITEBYTECODE=1 python3 tools/trajectory_eval/traj_eval.py batch --help
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tools/trajectory_eval -p 'test_*.py' -v
```

`single` may be omitted when the first argument is an evaluation option.
`--clock-assumption`, `--est-frame`, `--ref-frame`, both input paths, and `--out`
are mandatory. Empty assumption/frame strings are also rejected. Clock text and
frame labels are stored verbatim in JSON.

## Formats and timestamp precision

| Format | Columns | Default timestamp unit |
|---|---|---|
| `tum` | `t tx ty tz qx qy qz qw` | seconds |
| `orbslam3` | Same eight columns, including ORB-SLAM3 EuRoC float-printed ns | nanoseconds |
| `euroc_gt` | `timestamp, px,py,pz, qw,qx,qy,qz, ...` | nanoseconds |

Use `--est-format` and `--ref-format`; both default to `tum`. Optional explicit
`--est-time-unit {s,ns}` and `--ref-time-unit {s,ns}` override format defaults.
For each input, JSON records the effective unit and whether it was explicit.
`#` comments and blank lines are ignored. EuRoC extra columns are ignored; TUM
and ORB-SLAM3 rows must have exactly eight fields. Decimal scientific notation is
supported. Positions are in metres; quaternion components are dimensionless.

Timestamp strings go directly through `decimal.Decimal` to signed int64
nanoseconds. There is no epoch float conversion:
`1403636579963555584.000000` in ns becomes exactly `1403636579963555584`.
This preserves supplied decimal text; precision already lost by an input writer
or historical conversion cannot be recovered.
Sub-nanosecond input precision, present in historical converted TUM files, is
rounded to the nearest nanosecond with ties to even. JSON records the policy,
rounded timestamp count, and maximum rounding amount. CLI offsets, tolerances,
and RPE second deltas must be exactly representable in whole nanoseconds.

Input poses must be finite and timestamps strictly increasing after conversion.
Duplicates/reversals fail with file, line, and timestamp details; no sorting or
deduplication occurs. Quaternion norm deviation must be at most `1e-3`; accepted
quaternions are normalized afterward and maximum original deviation is recorded.
The internal quaternion order is `(x,y,z,w)`. Rotations are active body-to-world
rotations; input translations are body origins in each trajectory's world.

## Association and frame/clock assumptions

`--t-offset` defaults to `0` seconds and is **added to estimate timestamps**.
It is a user-declared relationship, never estimated. `--t-max-diff` defaults to
`0.01` seconds; the MH01 comparison below uses `0.02`.

Pair selection follows `evo.sync.associate_trajectories(reference, estimate)`:
iterate the shorter trajectory (estimate on equal lengths, as evo does), find the nearest
timestamp in the other trajectory, keep it if `abs(dt) <= tolerance`. An exact
tie selects the earliest timestamp. Samples can be reused; there is no one-to-one
enforcement, interpolation, or resampling. This implementation uses exact integer
nanosecond comparisons; evo's floating-second arithmetic can differ for inputs
at rounding-sensitive tolerance boundaries/ties. The provided MH01 association
and ATE match its historical evo results.

Association output includes original counts, matched pairs, unique matched
samples, unmatched counts, max/mean absolute time difference, iteration side,
and reuse flags/counts on both sides. Unmatched means original samples never
used; it is not necessarily original count minus pair count when reuse occurs.

Different textual frame labels print a `WARNING` and remain in the output.
**No extrinsics are applied.** A global alignment cannot correct a body/camera
lever arm or establish that two labels describe the same physical frame.
Even matching labels do not prove frame compatibility. Clock text likewise
records an assumption and does not establish synchronization.

## Alignment and metrics

`--align {se3,sim3,none}` defaults to `se3`. SE3 fits a rotation and translation;
Sim3 also fits a positive scale. Umeyama's 1991 closed form includes the reflection
fix so `det(R)=+1`. The fit maps estimate positions into reference coordinates:

```text
p_est_aligned = s * R * p_est + t
R_est_aligned = R * R_est
```

JSON records `R`, `t` [m], `s`, fit method, and fit pair count. `--n-to-align N`
uses the first N associated pairs (default all), while metrics still evaluate
all pairs. N must be at least 3 and at most the number of matches. A covariance
rank below 2 (stationary/collinear data) fails rigid/similarity alignment rather
than reporting an arbitrary orientation. `none` uses identity with fit count 0;
its `--n-to-align`, if supplied, is validated but has no fitting effect.
`--align-both` evaluates SE3 and Sim3 independently on the same pairs; it replaces
the single alignment selection and cannot be combined with `--align none`.

ATE is the per-pair aligned position error norm, equivalent to evo APE translation.
It reports RMSE, mean, median, population std (`ddof=0`), min, max, SSE [m²], and n.
`--rotation-ate` adds a separate rotation block [deg] using the principal angle
of `R_ref.T @ R_est_aligned`.

RPE defaults to `--rpe-delta 1 --rpe-unit s`. Each eligible start index produces
one pair; pairs may overlap and their endpoints may be reused:

- `frames`: associated indices `i, i+delta`; delta must be a positive integer.
- `s`: first `j > i` whose associated **reference** timestamp is at least delta
  seconds later. This remains well-defined if association reuses a reference pose.
- `m`: first `j > i` with at least delta metres of cumulative distance along
  the **matched reference polyline**. Stationary segments contribute zero.

There is no endpoint interpolation and actual separation can exceed delta.
Selection differs from evo's configurable RPE filters; only ATE is compared to
historical evo evidence here. The error transform is
`E = (Ref_i^-1 Ref_j)^-1 (Est_i^-1 Est_j)`. Translation norm [m] and principal
rotation angle [deg, 0–180] each report RMSE, mean, median, population std, min,
max, and n. Global rigid alignment cancels in SE(3) RPE. Sim3 scaling is applied
to estimate translations before RPE and the applied scale is recorded.
If no endpoints meet delta, RPE has n=0 and JSON null statistics, with a warning.
No timestamp matches at all is an error.

Trajectory metadata includes original estimate/reference time spans, shifted
estimate span, both matched spans, reference matched duration, reference length
using **all original reference samples between first and last match**, and the
matched-polyline length used for RPE distance selection. These lengths can differ.

## Outputs and overwrite

`--out DIR` must be new unless `--overwrite` is explicit. Inputs cannot reside
inside that directory; output artifact symlinks are rejected. Overwrite replaces
named outputs without recursively deleting unrelated files. Old artifacts from
earlier alignment/plot selections can remain; the current `result.json` lists
the plots created by this invocation. Prefer a new scratch directory per run.

- `result.json`: tool/version, UTC evaluation time, Python/NumPy versions,
  SHA-256 of the exact bytes parsed for each input, full invoked CLI arguments
  and parsed options, assumptions, parsing provenance, association, alignment,
  spans/lengths, metrics, warnings, and plot availability. Numbers have evidence
  label `CALCULATED`. This evaluation time is not a capture timestamp.
- `summary.md`: one compact metrics table and a plain-text assumption block,
  including parsing, association, length/time information, warnings, and plots.
- `ate_errors.csv`: `t_ns,error_m` for one alignment. With `--align-both`, use
  `t_ns,error_m_se3,error_m_sim3`.
- `rpe_errors.csv`: reference start/end timestamps and associated start/end
  indices, translation/rotation errors; alignment-suffixed columns for both modes.
- Three PNGs per alignment: `trajectory_xy_MODE.png`, `ate_over_time_MODE.png`,
  `rpe_over_time_MODE.png`. Plot times are relative to the first reference match,
  and RPE uses start time. CLI Matplotlib cache resides within `DIR/.matplotlib/`
  and fontconfig cache within `DIR/.cache/`. Cache environment settings are scoped
  to rendering; direct Figure/Agg ownership avoids global pyplot/backend changes.
  Missing Matplotlib is noted in JSON; core outputs still work. `--no-plots`
  explicitly disables plots.

Batch mode accepts repeated `--est NAME=PATH` with unique safe directory names;
all runs share reference, format/unit choices, and evaluation settings. It
produces a directory per run, combined `summary.md`, and combined `result.json`.
The combined table includes ATE RMSE/mean/median/max, RPE translation/rotation
RMSE and matched pair count. Across-run mean/min/max are unweighted summaries
of each run's metric, computed separately per alignment, with null RPE runs
excluded. They are not pooled-pose statistics or confidence intervals. Inputs
and metrics for all runs are validated before output writing begins.

## Reproduce N4A EuRoC MH01 validation

Read-only evidence: `experiments/2026-10-13-n4a-euroc-mh01/`, including its
`REPORT.md`, raw trajectories, CSV/TUM reference and `eval/ape_summary.txt`.
These evidence files are not committed to the repository, so this validation
runs only on a checkout that has them; elsewhere the REAL-DATA test is skipped.
The commands write to the ignored `_scratch/` directory.
Do not write outputs into `experiments/`.

From the repository root, run these exact commands with a new output directory:

```sh
traj_evidence=experiments/2026-10-13-n4a-euroc-mh01
traj_scratch=tools/trajectory_eval/_scratch/mh01-validation
PYTHONDONTWRITEBYTECODE=1 python3 tools/trajectory_eval/traj_eval.py batch \
  --ref "$traj_evidence/eval/gt_data.csv" --ref-format euroc_gt --ref-time-unit ns \
  --est "run1=$traj_evidence/run1/f_mh01_stereoi.txt" \
  --est "run2=$traj_evidence/run2/f_mh01_stereoi.txt" \
  --est "run3=$traj_evidence/run3/f_mh01_stereoi.txt" \
  --est-format orbslam3 --est-time-unit ns \
  --t-max-diff 0.02 --t-offset 0 --align-both --rotation-ate \
  --rpe-delta 1 --rpe-unit s \
  --clock-assumption 'EuRoC dataset timestamps are assumed to share one clock domain; estimate offset is explicitly 0 s. This does not establish live-device synchronization.' \
  --est-frame 'imu_body (ORB-SLAM3 body)' \
  --ref-frame 'imu_body (EuRoC state_groundtruth_estimate0)' \
  --out "$traj_scratch"

PYTHONDONTWRITEBYTECODE=1 python3 tools/trajectory_eval/traj_eval.py \
  --est "$traj_evidence/eval/run1.tum" --ref "$traj_evidence/eval/gt_data.tum" \
  --est-format tum --ref-format tum --est-time-unit s --ref-time-unit s \
  --t-max-diff 0.02 --t-offset 0 --align-both --rotation-ate \
  --rpe-delta 1 --rpe-unit s \
  --clock-assumption 'EuRoC dataset timestamps are assumed to share one clock domain; estimate offset is explicitly 0 s. This does not establish live-device synchronization.' \
  --est-frame 'imu_body (ORB-SLAM3 body)' \
  --ref-frame 'imu_body (EuRoC state_groundtruth_estimate0)' \
  --out tools/trajectory_eval/_scratch/mh01-tum-crosscheck
```

The frame-label warning is expected: these descriptive labels differ textually.
Neither label nor matching results independently verifies body-frame/extrinsics.
If repeating these commands, choose new output names or explicitly add `--overwrite`.

Historical evo comparison (metres, rounded to its six reported decimals):

| Run | SE3 RMSE | SE3 mean | SE3 median | SE3 max | Sim3 RMSE |
|---|---:|---:|---:|---:|---:|
| run1 | 0.041461 | 0.038090 | 0.042723 | 0.081394 | 0.018874 |
| run2 | 0.036844 | 0.034089 | 0.036917 | 0.076001 | 0.023794 |
| run3 | 0.041604 | 0.038548 | 0.042754 | 0.087709 | 0.022288 |

Each run has 3,678 estimate poses, 36,382 reference poses, 3,639 matched pairs,
39 unmatched estimate poses, 32,743 unmatched reference poses, and no reuse.
Maximum absolute dt is 0.02 s. Run1 raw CSV/ns and converted TUM/s ATE metrics
and error arrays agree within 1e-12 m despite differences in timestamp text.
The REAL-DATA test compares all recorded SE3 fields and Sim3 RMSE against the
read-only summary with tolerance 5e-5 m. It skips clearly if evidence is missing;
the SYNTHETIC suite remains runnable.

## Importable API

With the repository root on Python's import path:

```python
from tools.trajectory_eval.traj_eval import read_trajectory, evaluate, write_evaluation

estimate = read_trajectory("estimate.txt", "orbslam3", "ns")
reference = read_trajectory("reference.csv", "euroc_gt", "ns")
evaluation = evaluate(
    estimate, reference,
    clock_assumption="Caller-declared common dataset clock; offset 0 s.",
    est_frame="imu_body", ref_frame="imu_body",
    t_max_diff="0.02", align_both=True,
)
# evaluation.result is JSON-compatible; evaluation.series holds NumPy arrays.
write_evaluation(evaluation, "tools/trajectory_eval/_scratch/api-example")
```

Other public helpers include `Trajectory`, `parse_timestamp`, `associate`,
`umeyama`, `quaternion_matrices`, `rpe_pair_indices`, `relative_pose_errors`, and
`statistics`. `umeyama(source,target)` returns `(R,t,s)` mapping source to target.
File-backed trajectories carry input hashes; caller-created `Trajectory` objects
must supply their own provenance in `metadata`. Invoked CLI provenance is added
by `main`; direct API evaluations do not pretend to have CLI arguments.

## Test plan, limits and next steps

The suite has 30 tests: 29 SYNTHETIC tests and one REAL-DATA test. With the N4A
evidence present all 30 pass; in a clean checkout 29 pass and the REAL-DATA test
is skipped, so MH01 agreement cannot be reproduced from the repository alone. Synthetic tests cover exact transforms, scale,
reflection/degeneracy, parsing/order/units, timestamps, association boundaries,
offset/reuse, analytic drift, independent homogeneous-matrix RPE checks, rigid
invariance, output provenance, CLI requirements, batch aggregation and missing
Matplotlib behavior. Offline fixtures prove tooling behavior only.

Manual validation: run the commands above, inspect both summary tables and JSON,
confirm the expected counts/numbers, and open the PNGs. Expect overlapping aligned
top-view paths and finite ATE/RPE curves. Missing required assumptions, invalid
timestamps/quaternions, no matches, or degenerate alignment must return exit 2
with a specific diagnostic; existing output must fail without `--overwrite`.
RPE n=0 must show null metrics and a warning rather than zero error.

This validation proves agreement with the supplied historical evo ATE evidence,
not a fresh evo execution, estimator correctness, calibration, frame identity,
clock synchronization, dataset truth quality, Pi performance, camera readiness,
or any gate PASS. No performance improvement is claimed. Sim3 can absorb scale
error and should not replace SE3 evidence for metric-scale estimators. Alignment
can hide global gauge differences; reported assumptions remain essential.

Next use: supply explicit clock/frame declarations for another offline dataset,
keep outputs in a dedicated scratch location, and archive reviewed results with
their input hashes and tool version when an experiment is authorized.

Location: this tool lives in `tools/trajectory_eval/`, not the `scripts/evaluation/`
and `results/` layout in `docs/NATIVE_PI5_BASELINE_ARCHITECTURE.md` section 3.
Move it if that layout is adopted.
