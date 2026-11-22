# Headless stereo replay on an Intel Mac through Linux Docker

This path consumes a finalized stereo MCAP without camera/Pi access. The
recorded images go directly to native `ORB_SLAM3::System::STEREO`, viewer off;
there is no ROS graph, DDS transport, motor interface or calibration write.
The existing ROS/Pi launch path is unchanged. This measures another platform,
not Raspberry Pi performance.

Do not install/build/convert/replay on the Mac while the owner's PPO v2 batch
is active. The queued task requires terminal supervisor/batch receipts and no
remaining training/evaluation workers. Pi use needs a fresh owner release.

## Runtime and execution

`Dockerfile` captures the dependency/source provisioning. The actual session
used the same steps interactively in an exclusive container; setup failures and
repair logs remain in the archive. To reproduce the provisioning image:

```bash
docker build -t wbr-offline-stereo scripts/camera_day/offline
```


Use an isolated Ubuntu 22.04 container with bounded resources (the actual run
uses 4 CPUs / 5 GiB), a read-only input archive, and a read-only repository mount.
Inside the container install `build-essential cmake git ca-certificates
libopencv-dev libeigen3-dev libboost-serialization-dev libssl-dev libglew-dev
libgl1-mesa-dev libepoxy-dev python3-pip python3-numpy python3-opencv zlib1g-dev
libjpeg-dev libpng-dev`. Install `rosbags==0.11.0` for offline CDR/MCAP decoding.
No ROS installation or RealSense SDK is required for visual replay.

Create `/work/Pangolin` at v0.9.1, commit
`7364b59a58045d1c429b146e44f6f088fd1cc518`, and `/work/ORB_SLAM3` at
`4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4` (the repository submodule
pin). The Pi local build revision `727f3cbd` is not an upstream tree. Preserve exact
commit IDs, local compatibility diff and library hashes
in the result archive. The builder applies the same C++17/portable-ISA choices
as the existing Pi build, only in this isolated source copy. It also changes
the upstream private `mnFullBAIdx` generation counter from bool to int because
C++17 rejects incrementing bool. The full diff is retained, and this is a source
compatibility difference from the accepted Pi binary. A second local fix skips
the diagnostic camera2 print for Rectified input: upstream never allocates that
object for this mode. This fixes a pre-frame startup crash without changing the
calibration/model used for tracking. Build g2o explicitly before linking ORB:

```bash
bash /repo/scripts/camera_day/offline/build_runtime.sh /work /repo
python3 /repo/scripts/camera_day/offline/extract_stereo.py \
  /evidence/mcap-marked-loop-01/sensor-bag /work/clean-pairs-03 --expected-pairs 2215
python3 /repo/scripts/camera_day/offline/run_batch.py \
  /work/runner-build/replay_stereo /work/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  /evidence/mcap-marked-loop-01/stereo.yaml /work/clean-pairs-03 /work/three-replays \
  --pace-rate 1
python3 /repo/scripts/camera_day/offline/analyze_batch.py \
  /work/three-replays /work/analysis
```

All evidence output directories are exclusive-created. Retain failed logs and
use a different path for any repair/retry. Input timestamps remain integer
nanoseconds in manifests/poses; ORB's interface receives double seconds. Decode
only IR1/IR2; both header lists must be identical and contain2215frames. Mono8
pixels are stored as lossless PNGs; each PNG is decoded again and compared with
the original pixels. No sorting, trimming, dropped-frame filling or calibration
transformation occurs. The saved YAML must match640×480/15Hz/800features and the
borrowed D435i's read-only SDK-derived intrinsics/baseline.

Each replay starts a separate process/System/atlas. Every pair is processed once
in reader order, paced against recorded source differences. Slow processing can
run behind schedule, but this runner never drops pairs to catch up. Outputs:

- `attempts.csv`: flushed immediately before each Track call, so an incomplete
  call is distinguishable from a completed frame row.
- `frames.csv`: source timestamp, ORB tracking state, publication flag, cumulative
  processed count, zero application drops, steady Track start/end and map ID.
- `CameraTrajectory.txt`: finite tracked online camera poses, original source
  stamps. World frame may jump at resets/merges/closure; no continuous odometry.
- `map-events.csv`: current-map ID changes (including initial map), which may
  reflect resets or merges. Native stdout reset/new-map/merge markers are counted
  separately; a current-map change is not automatically a reset.
- `FinalAtlasTrajectory.txt` and `KeyFrameTrajectory.txt`: separate final exports
  for context; they may cover only an atlas subset. Online endpoint metrics use
  the explicitly named online trajectory, not a stitched final map.
- `summary.json`, batch commands/exits/progress and complete stdout logs.

Tracked FPS = tracked calls divided by the measured first Track start to last
Track end, including pacing/IO gaps. Track-compute FPS = tracked calls divided
by the sum of all TrackStereo call durations, including nontracked calls but
excluding pacing/IO. Neither is substituted with nominal15Hz or recorded source
rate. Loop detection does not alone prove completed loop correction.

Endpoint = first-to-last online pose distance; path = sum of consecutive online
pose distances; percent=100×endpoint/path. Physical start/end alignment is
owner-reported, not ground truth. Resets/merges/loop closure can jump coordinates
and inflate path; do not interpret these as pure continuous-map drift accuracy.

## Verification and limitations

Actual validation must include successful native compile/link, MCAP extraction
receipt/hash,2215equal image headers and pixel round-trips, three zero exits,
2215contiguous processed rows per run, finite trajectories and source boundaries.
Failure symptoms include mismatched pairs/types/size, malformed timestamps,
nonzero process exits, missing summaries, runner shutdown timeout or differing
pose/publication counts. Keep the failed run as evidence.

Rollback: use the existing ROS/Pi replay tools when that platform is authorized;
new files/container can be left idle. No original source, bag, camera settings,
training environment or robot runtime is overwritten. Large datasets, extracted
images, binaries, dependency downloads and raw logs stay outside git.

For the decoder's small MCAP fixture tests (inside this container):

```bash
python3 -m unittest discover -s /repo/scripts/camera_day/offline \
  -p test_extract_stereo.py -v
```

Tests cover stride-aware pixel preservation and integer source stamps, duplicate
and mismatched header rejection, wrong type/count rejection, and exclusive output
creation. They do not establish native tracking or endpoint accuracy.

## Archived TrackStereo frame budget

Analyze the three already completed runs without starting a replay or Docker
workload. The input directory must contain `replay-1/frames.csv` through
`replay-3/frames.csv`; output is exclusive-created:

```bash
python3 scripts/camera_day/offline/track_budget.py \
  /path/to/three-replays-02 /path/to/new-j-budget-output
```

The reader rejects a changed header, malformed rows, non-increasing source
stamps, non-contiguous processed counters, and non-positive or overlapping
serial Track intervals. It counts all calls strictly above 66.7 ms using
integer nanoseconds and also reports the exact 1/15 s test
`15 * duration_ns > 1,000,000,000`. Each run's directory contains a JSON
summary plus a histogram CSV and SVG. Histogram bins are shared across runs,
lower-inclusive/upper-exclusive milliseconds, and account for every call.
Percentiles use nearest rank; tracked-only counts select
`pose_published=true` as logged by the replay runner.

These no-drop 1× offline replays absorb compute overruns. Counts are budget
misses, not observed live camera drops or a direct forecast of latest-image
queue drops. Measurements are from Intel Mac x86_64 Docker and do not transfer
to the Pi.

Focused tests:

```bash
python3 -m unittest discover -s scripts/camera_day/offline -p test_track_budget.py -v
```

## Exact-stamp rigid replay agreement

`align_replays.py` compares **online camera-to-world translations** in two replay
directories. Each trajectory must agree exactly with its `frames.csv` publication
stamps. Decimal trajectory seconds are converted to integer nanoseconds without
floating-point epoch conversion. Inputs must already be strictly increasing;
duplicate, malformed and out-of-order stamps are rejected. Unequal tracked/source
schedules are allowed and explicitly counted: only exact shared stamps enter the
fit, with no interpolation, sorting, scale fitting or trimming.

For replay B positions $x_i$ and A positions $y_i$, the fit minimizes
$\sum_i\|Rx_i+t-y_i\|^2$, with $R^TR=I$, $\det R=+1$ and scale fixed to 1.
Given $U\Sigma V^T=\frac{1}{N}\sum_i(y_i-\bar y)(x_i-\bar x)^T$,
$R=U\,\mathrm{diag}(1,1,\det(UV^T))V^T$ and $t=\bar y-R\bar x$.
The determinant correction prevents fitting a reflection. Stationary/collinear
positions and rank-deficient correspondences below rank two are rejected at
relative singular-value tolerance $10^{-10}$; noncollinear planar motion is valid.
For example, $y=R_z(90^\circ)x+[5,-7,11]^T$ gives zero residual in the rigid-transform
fixture; reflecting a nonplanar fixture cannot give zero residual with a proper
rotation. Scaling a fixture by two also retains nonzero residual: stereo scale
is never adjusted.

Run from this checkout, with NumPy installed. Each output directory must be new:

```bash
ARCHIVE=<local workspace>/wbr-evidence-archive/camera-recording-20261021-01
BATCH="$ARCHIVE/clean-loop-mac-01/three-replays-02"
K_ANALYSIS="$ARCHIVE/AI-assistance-followup-01/k-analysis"
mkdir -p "$K_ANALYSIS"
python3 scripts/camera_day/offline/align_replays.py \
  "$BATCH/replay-1" "$BATCH/replay-2" "$K_ANALYSIS/alignment-02"
```

`report.json` includes the fitted B-to-A rotation/translation, determinant, fixed
scale, ranks, source-schedule equality, unmatched counts, input/code SHA256 hashes
and NumPy version. `residuals.csv` retains each exact shared stamp and before/after
Euclidean position differences. RMS is $\sqrt{\mathrm{mean}(\|e_i\|^2)}$;
median, p95 (NumPy linear percentile), maximum and count are also reported.
The fit and residuals use the same samples. These measure agreement between
replays, not ground-truth accuracy; loop corrections/map discontinuities remain
in the online data. Before-fit errors also depend on the arbitrary world frames.

## Single disabled-loop replay analysis

The inspected ORB source reads `loopClosing` in `System.cc:101–105`, passes it
to `LoopClosing` at line 213, and returns false immediately from
`NewDetectCommonRegions()` at `LoopClosing.cc:324–328` when zero. `Run()` gates
both loop correction and merging on this return value. No runner flag/rebuild
is necessary to request this upstream mode. It also disables place recognition;
local mapping/BA, tracking loss and new atlas maps remain possible.

The existing upstream implementation has a potential disabled-mode defect:
`mpCurrentKF` is not initialized in the constructor/header; the early return
precedes its assignment, yet `Run()` copies it into `mpLastCurrentKF` and may
dereference it on a subsequent iteration. Static control-flow inspection does
not establish successful runtime operation. maintainer owns the single native
run and any runtime repair; runner analysis does not modify this source.

For the 2026-10-21 follow-up, maintainer-only exact dispatch/config commands and
binary/source hashes are in `/private/tmp/wbr-camera-review-20261021/k-runtime-ready.md`.
The dispatch exclusive-creates its directory, appends only `loopClosing: 0` to
the original settings bytes, and uses the existing headless binary for exactly
one full 2,215-pair replay at 1x. No batch wrapper, rerun-to-good or trimming.

```bash
python3 scripts/camera_day/offline/analyze_openloop.py \
  "$K_ANALYSIS/native-openloop-01/replay-1" \
  "$K_ANALYSIS/native-openloop-01/pairs.csv" \
  "$K_ANALYSIS/openloop-analysis-01" \
  --settings "$K_ANALYSIS/native-openloop-01/stereo-loopClosing-0.yaml" \
  --log "$K_ANALYSIS/native-openloop-01/replay-1.log" \
  --expected-pairs 2215 --jump-threshold-m 0.25 --jump-threshold-deg 45 \
  --physical-return owner-reported
```

The analyzer requires every manifest stamp, sequential processed counts and zero
application drops; missing input coverage fails rather than being silently
trimmed. It partitions tracked segments at loss/unavailable pose, map/state
change, translation step >0.25 m or orientation step >45 degrees. Quaternion
sign changes are not orientation jumps. Thresholds are declared diagnostic
assumptions, not validated motion bounds; smaller corrections can remain.
JSON reports every state span, segment endpoint/path, break stamp/reason and
per-map counts. Separate segments in the same map are never joined.

The first tracked segment ends at the first loss, map change or detected jump.
Its displacement is not a closed-loop drift candidate unless it spans the entire
physical-start-to-return record, including the recorded stationary tail. Missing
initial tracking or any break therefore disqualifies it. Settings must explicitly
disable loop closing; observed loop/merge/reset or extra-map log markers also
disqualify it. Physical return must be supplied as owner-reported; it is not
independently measured. Even an eligible endpoint is conditional on that return,
local BA and the jump thresholds, never a ground-truth accuracy result. maintainer
still verifies native exit/provenance and complete logs independently.

Focused CPU-only tests, requiring NumPy and no native runtime:

```bash
python3 -m unittest discover -s scripts/camera_day/offline -p test_align_replays.py -v
python3 -m unittest discover -s scripts/camera_day/offline -p test_analyze_openloop.py -v
```

These focused suites are not currently discovered by `tools/run_all_tests.sh`;
run them explicitly. Fixtures cover rigid transforms, reflections, fixed scale,
degenerate/unequal/malformed timestamps and trajectories, losses, map changes,
pose jumps, conditional return coverage and refusal to overwrite reports.
