# D435 sparse ORB map points

`d435_node` publishes `/d435_orb/map_points` (`sensor_msgs/msg/PointCloud2`)
from the ORB map selected by `Atlas::GetCurrentMap()` at snapshot start, not
from RealSense depth or camera motion.
The existing pose and per-frame status contents are unchanged. The camera launch
still disables the RealSense point cloud. No sensor launch/recording script changes
are needed. This also compiles for the existing RGB-D mode; that mode has not been
replayed for this feature.

## Topics and bounds

| Item | Contract |
| --- | --- |
| `~/map_points` | Full replacement snapshot, unordered XYZ in metres; one row; float32 x/y/z at offsets 0/4/8; 12 bytes/point; little endian; dense (all emitted coordinates finite). Empty snapshots are valid. |
| Header | Triggering first/left image stamp, exactly as on pose/status; `world_frame` parameter, default `slam_world`. No wall-time restamping or TF. |
| `~/map_points_info` | JSON with matching `source_ns`, per-process `sequence`, `map_id`, `init_kf_id`, `big_change`, `map_change`, `tracked`, `candidates`, `points`, `truncated`. Record this alongside clouds. |
| QoS | Both new topics reliable, transient-local, keep-last 1. Late subscribers can obtain the latest snapshot **while the publisher lives**. |
| `map_points_hz` | Default/max 1 Hz; 0 disables; otherwise 0.001–1 Hz. Both steady elapsed time and source time must reach the period. Rate check occurs before any map access/copying. No timer/runner or snapshots without incoming pairs. |
| `map_points_max` | Default 50,000, allowed 1–200,000 **candidates**. Output may be smaller after filtering. On overflow, visit the first bounded subset in upstream pointer-set order and set `truncated=true`; this is not uniform spatial sampling. |

The wrapper holds at most the candidate cap in temporary pointers and XYZ values,
plus one serialized cloud and bounded DDS history. It does not bound ORB's own
map, allocator, or external recorder memory. A busy map-update mutex skips that
sampling opportunity. Other upstream mutex waits and reliable publication are
not hard real-time bounded; no latency/capacity claim is made. No ORB lock is held
during byte serialization or ROS publication.

If the triggering frame lacks a finite tracked pose, a successful due snapshot
is empty (`tracked=false`), replacing stale geometry. Lock contention can delay
that empty update. If input stops, no new message clears the last cloud.

## Frames, correction, reset and merge

ORB returns camera-from-world `Tcw`:

\[
p_c=R_{cw}p_w+t_{cw},\qquad
T_{wc}=T_{cw}^{-1},\quad R_{wc}=R_{cw}^{T},\quad
 t_{wc}=-R_{cw}^{T}t_{cw}.
\]

The existing PoseStamped uses `Twc.translation()` and its quaternion. Each
`MapPoint::GetWorldPos()` is already `p_w`, so no additional transform is applied.
For example, with identity rotation and `t_cw=(-1,0,0)`, the camera pose is
`(1,0,0)` in world; a world landmark `(2,0,3)` remains `(2,0,3)` in the cloud
and has camera coordinates `(1,0,3)`.

Stereo starts the first camera at identity; its initial world axes therefore
follow optical x-right/y-down/z-forward. Scale comes from the configured stereo
baseline. This is not gravity alignment, robot-body pose, continuous odometry,
or a globally surveyed map. Local optimization, loop closure, reset and merge
can change coordinates. A snapshot contains landmarks accumulated over many
frames; asynchronous ORB corrections may occur after the triggering pose was
computed. Matching headers mean a common source trigger, **not an atomic
pose/cloud optimization state**.

**Treat every message as a replacement, including an empty one. Do not append
clouds into a cumulative map.** Active reset can retain both map object and
numeric ID; merge can transfer the old numeric ID to a different map. Initial
keyframe ID and change counters help inspect sampled history but are not an
exhaustive event log or persistent frame identity. Same ID/frame is not proof of
continuity. Separate process sessions also reuse `slam_world`; keep recordings
separate. This policy preserves the established pose frame without inventing a
TF between disconnected maps.

## Audited source and locking

Repository pin: `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`. Pi native build:
`727f3cbd79482af0360447a26b906950c84ce61c`. Their System/Atlas/Map/MapPoint headers
and Atlas/Map/MapPoint implementations are byte-identical. The inspected System
implementation differences are pre-existing empty-map trajectory-save guards.

- `System::GetTrackedMapPoints()` is just the current tracked frame, not the map.
  System has no public Atlas accessor. Reuse the existing
  [`orb_access.hpp`](../../scripts/camera_day/native/orb_access.hpp) typed-member
  compatibility mechanism; no upstream edits, layout changes or private/public
  macros. Two additional typed pointers are isolated in `orb_map_snapshot.hpp`
  for the map container and its mutex, because public `GetAllMapPoints()` copies
  the entire unbounded pointer set before a caller could truncate it.
- After the rate gate: obtain the map once through `Atlas::GetCurrentMap()`;
  try-lock its `mMutexMapUpdate`; copy capped candidate pointers under
  `mMutexMap`; release the container mutex; filter non-bad points still in this
  map and copy finite `GetWorldPos()` values; release the update lock. Getters
  lock point state. Do not re-enter Atlas while holding the update lock: the
  pinned source has paths that take these locks in the opposite order.
- At this pinned source, retired `Map` allocations are not deleted by
  `Atlas::RemoveBadMaps()`. If a background merge switches the active map after
  selection, the snapshot can therefore represent the just-retired map; the
  sampled `map_id` and change fields identify the source map but are not a
  continuity guarantee. Pose and cloud are not an atomic ORB transaction.
  This follows the upstream update→container order. Never hold the Atlas mutex
  while acquiring the update mutex (merge takes those in the opposite order).
- The audited `Map::EraseMapPoint` only removes pointers. `Map::clear` and
  `Atlas::RemoveBadMaps` leave deletion commented out. The snapshot runs between
  Track calls on the same single ROS owner, so Tracking's reset/clear cannot run
  concurrently. ORB's background mapper and loop closer still run. New/culling
  points can change between getters; this is a sampled visualization, not a
  frozen database transaction. Re-audit ownership/deletion and lock ordering
  before an ORB upgrade or moving snapshots to another executor/thread.

Primary source: pinned [System](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/System.cc),
[Map](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Map.cc),
[MapPoint](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/MapPoint.cc),
and [Atlas](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Atlas.cc).

## Retain and display after the process exits

Use the installed ROS MCAP recorder in a **separate new output directory**, in
the same ROS domain as the node. Start recording before SLAM when possible:

```bash
source /opt/ros/jazzy/setup.bash
ros2 bag record --storage mcap --output /absolute/new-map-bag --topics \
  /d435_orb/map_points /d435_orb/map_points_info /d435_orb/pose /d435_orb/status
```

For offline replay, the recorder may use wall time while the SLAM node follows
the replay clock; each cloud still carries its original camera-source stamp in
the message header. Set `--use-sim-time` only if you intentionally need simulated
receive timestamps and have arranged `/clock` before starting and synchronizing
the recorder. Stop the recorder gracefully with Ctrl-C so its
MCAP index and metadata finish. These clouds remain usable after SLAM exits;
transient-local DDS alone does not. Abrupt process/power loss can lose queued or
unflushed data and may require recovery; no shutdown export or atlas checkpoint
is promised. This is visualization geometry, not a reloadable ORB atlas: it does
not retain descriptors, observations or keyframes. Existing sensor MCAPs remain
unchanged; replay one through the new node to derive a separate cloud MCAP.

In Foxglove, connect to the existing bridge or open the new MCAP. Add a 3D panel,
set fixed/display frame to `slam_world` (or the configured world frame), enable
`/d435_orb/map_points`, choose flat color and a visible point size, and set decay
time to **0** to show the latest replacement. Enable the pose topic if desired;
no TF is needed between topics already in the same world frame. Inspect
`map_points_info` in Raw Messages. Coordinates may appear sideways because the
world follows the initial optical axes. Schema/settings reference:
[Foxglove 3D panel](https://docs.foxglove.dev/docs/visualization/panels/3d).
Live bridge delivery and GUI appearance remain unverified for this change.

## Build and focused offline validation

Host (no ROS/ORB): `make -C ros2/wbr_orb_node test`. This runs the cloud byte-writer,
finite/header/empty/bounds/rate assertions plus 19 existing wrapper tests and a SIGTERM cleanup test using sleeping stubs. The
host cloud type is a fixture; Pi CTest uses the actual ROS PointCloud2 type.

Build on Pi from a private source snapshot, preserving the native ORB install
and the existing camera build. Use a **fresh** build/install prefix:

```bash
source /opt/ros/jazzy/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
orb=<local workspace>/orbslam3/ORB_SLAM3
run=<local workspace>/NEW-orb-map-build
mkdir "$run"
timeout --signal=INT --kill-after=10s 170s colcon --log-base "$run/log" build \
  --base-paths /absolute/private-source/ros2/wbr_orb_node --packages-select wbr_orb_node \
  --executor sequential --parallel-workers 1 --build-base "$run/build" --install-base "$run/install" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DWITH_ORB_SLAM3=ON \
  "-DORB_SLAM3_ROOT=$orb" "-DORB_EXTRA_INCLUDE_DIRS=/usr/include/eigen3;$orb/Thirdparty/Sophus"
ctest --test-dir "$run/build/wbr_orb_node" --output-on-failure
python3 ros2/wbr_orb_node/tests/replay_map_points.py \
  --node "$run/install/wbr_orb_node/lib/wbr_orb_node/d435_node" \
  --bag <local workspace>/wbr-evidence/camera-day-20261021-01/mcap-marked-loop-01/sensor-bag \
  --settings <local workspace>/wbr-evidence/camera-day-20261021-01/mcap-marked-loop-01/stereo.yaml \
  --vocabulary "$orb/Vocabulary/ORBvoc.txt" --output /absolute/NEW-replay-evidence --domain 187
```

The harness defaults to the first **20 bag seconds at 0.3×**, private domain 187,
localhost transport and remapped test image topics; no camera process. Select a
different prefix and playback rate with `--seconds N --rate R`. `--seconds 0`
reads the full recording duration from its `metadata.yaml` and replays that
duration. The receipt and successful validation JSON record requested seconds,
resolved bag seconds, rate and the computed playback deadline. The deadline is
`ceil(bag_seconds / rate + 30)` seconds; the harness extends its work and overall
windows to accommodate the requested playback while retaining the original
default windows. For example:

```bash
python3 ros2/wbr_orb_node/tests/replay_map_points.py \
  --node /path/to/d435_node --bag /path/to/stereo-bag \
  --settings /path/to/stereo.yaml --vocabulary /path/to/ORBvoc.txt \
  --output /path/to/new-full-map-replay --seconds 0 --rate 0.3
```

It starts
the SLAM node and recorder, waits for all requested topics to subscribe, then
starts the player. The derived bag uses receive-time while cloud headers preserve
source-time. It keeps commands/PIDs/exits/logs, stops its
own process groups (also on supervisor SIGTERM), requires clean child exits, then
reads the closed MCAP with `check_map_points_bag.py`. Startup and cleanup
allowances remain bounded, and playback receives its computed duration-based
deadline.
The check requires nonempty real map points, exact XYZ layout, finite coordinates,
matching pose/status/metadata stamps, source rate ≤1 Hz and timestamps present in
the input bag. It is not a trajectory accuracy, live throughput, reset/merge
stress, cap-overflow or Foxglove GUI test. Full-bag map replay itself remains
unverified until run against a complete recording.

The validator can be repeated without running SLAM:

```bash
python3 ros2/wbr_orb_node/scripts/check_map_points_bag.py /absolute/replay-evidence/map-bag \
  --source-bag <local workspace>/wbr-evidence/camera-day-20261021-01/mcap-marked-loop-01/sensor-bag
```
