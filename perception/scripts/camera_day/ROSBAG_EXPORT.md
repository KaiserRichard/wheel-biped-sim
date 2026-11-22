# Export one replay's recorded ROS outputs

Use ROS-installed Python after sourcing the Pi's saved Jazzy setup. Export a
**finalized, new output bag** containing only that replay's `/d435_orb/pose` and
`/d435_orb/status`. The maintainer records those topics separately while playing
only sensor topics from the original bag; do not mix old estimator outputs into
new replay evidence. This tool opens bags read-only and never starts ROS nodes.

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/camera_day/rosbag_to_tum.py \
  /absolute/path/replay-output-bag /absolute/path/NEW-export-dir \
  --input-bag /absolute/path/original-sensor-bag \
  --image-topic /camera/camera/infra1/image_rect_raw
```

`--input-bag` is optional. For RGB-D use `--image-topic
/camera/camera/color/image_raw`. Override output topic names with `--pose-topic`
and `--status-topic` if necessary. Output must be a new directory outside Git and
outside both input bags. Existing directories/files are never overwritten.
Hashing reads every file in each supplied bag; finalize recording first.

- `CameraTrajectory.txt`: TUM `t x y z qx qy qz qw`, metres. Seconds/nanoseconds
  come directly from `PoseStamped.header.stamp`, formatted with nine fractional
  digits using integers. Coordinates/quaternions use round-trip float text;
  no transforms, renormalization, resampling or inferred poses.
- `frames.csv`: canonical `source_ns,tracking_state,pose_published,processed,
  application_dropped`, followed by `second_ns`. One row per recorded status,
  preserving integer values, numeric ORB states and boolean publication flags.
  States: -1 system not ready, 0 no images, 1 initializing, 2 OK, 3 recently lost,
  4 lost, 5 OK_KLT. Published poses require state 2 or 5; those states may still
  have a false flag when the node rejects a nonfinite result.
- `export.json`: SHA-256/size/path for bag files, exporter and exported TUM/CSV;
  counts, first/last source stamps, published frame ID, counter coverage, links
  and optional original-image comparison. No processing timing is invented:
  `processedFPS` and `trackedFPS` are null. Bag record timestamps are **not**
  monotonic processing timestamps; source time spans are not measured runtime.

These are **online camera-optical poses in the published SLAM map/world frame**.
Loop correction, relocalization or resets can jump that frame. They are not a
final-optimized atlas trajectory or ground truth. Endpoint displacement describes
only the exported tracked span; equal physical endpoints and full replay
coverage require separate evidence.

Validation rejects malformed/missing/extra status fields, duplicate JSON keys,
unknown states, wrong ROS message types/serialization, nonfinite poses, zero
quaternions, changing frame IDs, repeated/backward pose or source stamps,
regressing counters and pose/publication contradictions. A published flag must
match exactly one recorded pose header; missing either side fails with exit 2
before creating outputs. Cross-topic interleaving is allowed; each topic's reader
order is retained. No sorting, stitching, timestamp resets or deduplication.
Entirely absent outputs produce header-only files and explicit absent counts,
never synthetic poses or tracking states. Exit 0 means valid export, not tracking
success. Other I/O failures can leave partial new files; only a completed
`export.json` identifies a finished export, and retries need a fresh directory.

Processed counter skips and a first counter above one expose missing status rows.
The tail remains unknown even at bag EOF; contiguous counters never establish
full capture. `application_dropped` is the node's cumulative **input-message** drop
counter, not a count of missing pose/status records or all DDS losses.

With `--input-bag`, the selected image headers must strictly increase and each
status source must equal an original image stamp. The report counts original
images, statuses and matched first/last stamps, plus runs of images without an
observed status (zero-based image indices and source stamps). It never subtracts
clock domains or infers missing images from nominal FPS. Unmatched images may
reflect pairing, processing, DDS or recorder losses; matching all images still
does not certify runtime capture completeness. A wrong original bag/topic fails
if any observed status source has no matching image.

`replay_metrics.py` accepts the TUM/CSV directly: pass `--frames LABEL frames.csv`
with `--replay LABEL CameraTrajectory.txt` for each of its three replay entries.
Keep coverage unknown/partial unless independently established. No steady timing
columns are supplied, so its FPS stays unavailable. Counter gaps limit observed
loss transitions; missing pose/status pairs intentionally require repairing the
capture rather than generating a misleading endpoint report.

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  python3 -m unittest discover -s scripts/camera_day/tests -p test_rosbag_to_tum.py -v
```

The pure tests run without ROS on Mac. Actual Pi serialization/read/export smoke
uses explicitly synthetic bags outside Git; commands/results and preserved
paths are in [the runner session](../../ops/worklog/sessions/2026-10-21-camera-replay-export.md).
No real camera, estimator, replay or final atlas export is validated by that smoke.
