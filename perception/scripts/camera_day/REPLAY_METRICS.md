# Offline replay metrics

`replay_metrics.py` uses Python 3 standard library only. It reads three labeled
replays, preserves input bytes, and creates `replay_metrics.json` plus a Markdown
table in a **new directory outside git checkouts**. It never opens ROS, a camera,
or SSH. This is endpoint consistency, **not absolute accuracy without ground
truth**. No real camera replay was evaluated during development.

```bash
python3 scripts/camera_day/replay_metrics.py \
  --replay replay1 /evidence/replay1/CameraTrajectory.txt \
  --replay replay2 /evidence/replay2/CameraTrajectory.txt \
  --replay replay3 /evidence/replay3/CameraTrajectory.txt \
  --coverage replay1 complete --coverage replay2 partial \
  --same-start-end replay1 yes \
  --frames replay1 /evidence/replay1/frames.csv \
  --frames-coverage replay1 complete \
  --orb-log replay1 /evidence/replay1/stdout.log \
  --log-coverage replay1 complete \
  --output-dir /private/tmp/NEW-replay-summary
```

Paths above are examples, not existing evidence. Each optional argument takes
`LABEL VALUE`; it can be repeated for each replay. Exactly three unique labels
are required. Unspecified coverage and equal-physical-start/end default to
`unknown`. Set `--same-start-end LABEL yes|no|unknown` from physical run evidence,
never from estimated pose proximity. Coverage values are `complete`, `partial`,
`unknown`, declared independently for trajectory export, frame capture and log.
`complete` means the supplied artifact covers the entire intended replay; the
program cannot establish that declaration from a trajectory alone. Complete
trajectory export still does not establish tracking at the physical endpoints.

Exit 0 means analysis completed, including legitimate unavailable metrics.
Exit 2 means bad arguments, an output refusal or invalid/missing evidence.
Evidence errors are retained per replay in JSON so other replays still appear.
Inputs include absolute paths and SHA-256 hashes. Existing output directories
are refused. Retain the input artifacts alongside the reports; do not edit an
original artifact to fit a supported format.

## Trajectory and interpretation

Input is actual TUM text, one pose per line:

```text
# timestamp_seconds x_m y_m z_m qx qy qz qw
0 0 0 0 0 0 0 1
1 4 0 0 0 0 0 1
2 1 0 0 0 0 0 1
```

This example is **synthetic**. Blank/comment lines and trailing `#` comments are
supported. All eight values must be finite, orientation must be nonzero, and
timestamps must strictly increase. Timestamps use decimal comparisons to avoid
collapsing adjacent epochs through float conversion. Orientations are checked
for finite/nonzero values but are not used to compute translation metrics.
Repeated or backward stamps are errors: no automatic sorting, deduplication or
map-reset stitching. No PoseStamped YAML/rosbag parser is included. A separate
export of valid ROS PoseStamped messages can produce TUM: use the original
`header.stamp.sec + nanosec / 10^9` (preserve decimal precision), position in
metres and quaternion in `x y z w` order, retaining capture order. Never export
only poses and infer successful tracking for missing frames.

For saved camera positions \(p_i\) in one coordinate system:

\[
d = \lVert p_{N-1}-p_0\rVert_2,\qquad
L = \sum_{i=1}^{N-1}\lVert p_i-p_{i-1}\rVert_2,\qquad
r = 100\,d/L.
\]

The first expression is endpoint displacement; it measures closed-path endpoint
drift only if the camera physically returned to the same start position. The
second is the sampled estimated path, not ground-truth distance. The ratio is
a percentage of that estimated path. In the synthetic example, \(L=4+3=7\) m,
\(d=1\) m and \(r=14.285714\%\). A real full-loop interpretation additionally
needs complete replay coverage and tracked poses at both physical endpoints.
Partial exports still get geometry for their observed span, clearly labeled.

Fewer than two poses makes distance metrics unavailable. If
\(L \leq\texttt{--min-path-m}\) (default \(10^{-6}\) m), the report retains
endpoint/path distances but marks the path degenerate and the percentage
unavailable. This is a numerical guard, not an accepted motion-quality limit.
Repeated static frames with zero drift are never evidence of a successful
moving loop. Spatially subsampled keyframes underestimate traveled distance;
use full frame trajectories when available and retain export provenance.

ORB resets, relocalization and loop corrections can change the coordinate
system or introduce jumps. A monotonic time series cannot rule these out.
Do not concatenate map epochs. Published online poses and final optimized TUM
exports have different correction histories; their path/endpoint metrics need
not agree. Equal physical endpoints also do not prove correct scale or map.

## Optional per-frame CSV

This is an independent conservative interchange format, not a claim about an
unfinished native logger. One row represents **one completed tracking call**:

| Header | Meaning |
|---|---|
| `source_ns` (required) | Nonnegative integer input image epoch; strictly increasing. Not host receipt/processing time. |
| `tracking_state` (required) | ORB numeric enum or exact state name below. |
| `pose_published` | Optional `true/false/1/0`; whether this call emitted a valid pose. |
| `processing_start_steady_ns`, `processing_end_steady_ns` | Optional pair of nonnegative integer timestamps from the same measured host steady clock/session. Both required if either is present. End must exceed start; serial calls cannot overlap. |
| `processed` | Optional one-based cumulative completed-call counter, strictly increasing; detects missing status rows. |
| `application_dropped` | Optional cumulative nonnegative nondecreasing application counter; not DDS/driver loss. |

When an optional supported column is present, every row must supply a valid
value. Extra headers are ignored, never guessed as alternate timing/state
fields. Bad row widths, duplicate headers, invalid states and reversed/reset
counters fail explicitly. Split sessions at clock changes or process resets.
The analyzer cannot prove that a timestamp named `steady` was really measured
on a steady clock: keep the instrumentation/source provenance.

Supported ORB states (verified in Pi `Tracking.h`): `-1 SYSTEM_NOT_READY`,
`0 NO_IMAGES_YET`, `1 NOT_INITIALIZED`, `2 OK`, `3 RECENTLY_LOST`, `4 LOST`,
`5 OK_KLT`. Only `OK` and `OK_KLT` count as tracked. Loss episodes count observed
transitions from either successful state into `RECENTLY_LOST` or `LOST`;
consecutive lost frames count once. Initialization loss before success does
not count, and initialization resets the transition context. Missing rows or
partial/unknown captures make this an observed transition count only, not a
complete replay loss count. Dropped input frames are a separate quantity.

`frames_without_pose` counts explicit false publication flags;
`successful_frames_without_pose` highlights successful state without a valid
published pose. Without publication flags these fields are unavailable.
Trajectory/published-count disagreement and nontracking capture boundaries
produce warnings, not fabricated poses. No timestamp joining between TUM and
CSV is attempted: native relative TUM stamps and ROS source epochs may differ.

`missing_status_rows` uses counter gaps (plus a missing prefix when frame
coverage is declared complete). Optional `--expected-processed LABEL COUNT`
declares the known total number of tracking calls to expose missing prefix and
tail rows. This is **not raw camera frame count** or nominal Hz times duration.
Without counters/expected total, missing status rows remain unknown. Loss at
DDS/driver/before callback is not inferred from application counters or stamps.

With steady intervals, let \(T=(e_{N-1}-s_0)/10^9\) seconds. `processedFPS` is
\(N/T\), `trackedFPS` is \(N_{\mathrm{OK\ or\ OK\_KLT}}/T\). The span includes
gaps/idle time between calls, not just summed tracking CPU durations. These
are observed processing throughput and successful-tracking throughput over
the **same measured span**; `trackedFPS` is the explicit tracking-FPS measure
here, not the nominal camera rate or inverse mean TrackStereo/TrackRGBD time.
Counter gaps mark these observed-row rates as lower bounds for their span.
Partial captures describe their captured span only. No steady measurements
means both FPS metrics are unavailable; source duration never substitutes.
There is no subtraction across source, ROS, receipt or steady clock domains.

Read-only inspection of `ros2/wbr_orb_node/src/d435_node.cpp` found its status
fields `source_ns`, `second_ns`, `tracking_state`, `pose_published`, `processed`,
`application_dropped`. It publishes only finite `OK`/`OK_KLT` poses. Status has
**no steady timing**, so exporting those fields alone supports counts but
cannot establish real FPS. It is JSON in `std_msgs/String`, not this CSV; an
external export is required. No native-runner schema is assumed or modified.

## Loop evidence

Read-only SSH inspection on 2026-10-21 used the Pi source
`<local workspace>/orbslam3/ORB_SLAM3/src/LoopClosing.cc`, checkout commit
`727f3cbd79482af0360447a26b906950c84ce61c`, file SHA-256
`50c5a4ca6e5618bec11759f87610c56aaea1bef7bd4c82909f37d0bc0ff2b7b1`.
This verifies strings/control flow in that source, not the loaded binary or
logging completeness of another build. Messages must match a whole stripped
line, optionally prefixed by bracketed launch/logger tags.

| Exact upstream stdout | Interpretation |
|---|---|
| `*Loop detected` | Positive detection event; `loop_closure_detected=yes`. |
| `BAD LOOP!!!` | Rejection after detection; not a completed correction. |
| `*Merge detected` / `Merge finished!` | Map merge detection/completion, separate from same-map loop detection. |
| `Global Bundle Adjustment finished` / `Map updated!` | Separate optimization/map-update events; not proof that a specific correction completed. |

`PR: Loop detected with Reffine Sim3` is emitted while the coincidence threshold
has **not** been reached. It is ignored. `Loop detected!` and `Loop closed.`
are not accepted: the inspected correction function has only commented-out
text, and its end has no direct correction-completed stdout marker. Therefore
`correction_completed=unknown`, even after a detection or bundle adjustment.
Detection can be followed by `BAD LOOP!!!`; the table's **Loop detected** is
deliberately a detection yes/no/unknown, not a correction-success claim.

With no recognized positive event, a missing, partial or unknown-coverage log
yields `unknown` (not observed). Only a supplied, declared-complete saved log
yields `no`, meaning **no recognized detection observed in that log**, not proof
that closure never happened. Different binaries, disabled loop closing,
verbosity and lost/interleaved output can hide events. A trajectory returning
near its start, static repetitions, candidates and merges never set loop=yes.

## Offline verification

```bash
python3 -m unittest discover -s scripts/camera_day/tests -p test_replay_metrics.py -v
```

Tests cover numeric 3D distances, short/empty paths, finite/monotonic checks,
partial coverage, clock separation, measured throughput, loss transitions,
missing outputs/counters, loop/candidate/merge evidence, invalid replay isolation
and exclusive output creation. The development session records an actual CLI
run over explicit synthetic inputs outside git. No hardware/accuracy claim follows.
