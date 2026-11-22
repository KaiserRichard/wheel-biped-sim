# wbr_orb_node

**ROS 2 Jazzy / native ORB build verified on the Pi, 2026-10-21; tracking unverified.** This
`ament_cmake` package's original `orb_node` drives the existing `tools/vio_adapter` boundary from
recorded or live stereo + combined **D435i camera IMU** messages. The STM32 body
IMU is a different sensor and must never be substituted. N4B offline comes
before N4C live. Passing these host fixtures is **not a hardware gate PASS**.

The default build uses the existing scripted fake engine; its poses, tracking
states and map changes are **ASSUMED synthetic**, even with real bag input.
`WITH_ORB_SLAM3=ON` links an existing native ORB build through the existing
`orb_engine.cpp`; it constructs `IMU_STEREO` with its viewer disabled. This
package installs no dependencies and changes no upstream tool.

**2026-10-20 camera preparation:** this same package now also provides
`d435_node` when `WITH_ORB_SLAM3=ON`, with visual-only `STEREO` and `RGBD` input,
headless ORB, source-stamped tracked optical poses, camera/SLAM launches and SDK
calibration conversion. See [camera-day commands and limitations](../../scripts/camera_day/README.md).
This path has no IMU dependency or fake engine. Native Pi build/install and
camera-free `d435_node` startup now pass; startup intentionally stops at the
missing-vocabulary/settings error, before ORB construction. No live camera
execution or tracked pose is claimed. See the
[build session](../../ops/worklog/sessions/2026-10-21-camera-ros-build.md).
**Sparse active-map output:** `d435_node` now also exposes bounded, source-stamped
`PointCloud2` snapshots at at most 1 Hz, with companion map metadata. See
[map frames, Foxglove, separate MCAP persistence, build and bounded replay](MAP_POINTS.md).

The rest of this README documents the original
stereo-inertial `orb_node`, whose IMU contract remains unchanged.

Callbacks validate MONO8 (`mono8` / `8UC1`) images, preserve integer header
stamps and enqueue owning values with sensor-data QoS. Both rectified and raw
image topic names are parameters; the pixels must match the ORB calibration
and rectification policy. Combined IMU acceleration is m/s² and angular velocity
rad/s. Its interpolation/timing must already be qualified; subscribing to
`/imu` does not establish that qualification. Separate accel and gyro topics
cannot be supplied directly to this node.

One runner owns the synchronous adapter and engine. Transport pairs require
exact equal source header epochs. The final atomic stereo validation, IMU
interpolation, coverage gate, engine-state/reset semantics and snapshots reuse
`tools/vio_adapter`. IMU samples are supplied through the first gyro epoch at or
after each image, with co-stamped accel brackets; no extrapolation is introduced.
Pairing waits for coverage, including when IMU arrives after both images.

All application input buffers are bounded and drop oldest on overflow, with
separate counts for ingress and each sensor buffer. A callback uses `try_lock`;
contention rejects that incoming message with its own count. Dropped IMU,
nonfinite motion, wrong clock, source reversal or excessive IMU gap latches a
session fault: tracking needs a new process/session. Image loss alone does not
invent an IMU gap. The adapter's rejection guards remain in force; only one
covered pair is submitted at a time, avoiding an accumulating adapter image
queue. A slow engine or disk can stall the runner, while the executor continues
to enqueue/drop. These are bounded memory policies, not hard real-time promises.
DDS keep-last loss, driver loss and pre-callback loss are **UNKNOWN** and are
not included in application drop counters.

All proposed numeric bounds and their rationales live in
[config/defaults.json](config/defaults.json). Build configuration generates the
C++ defaults. ROS overrides should be saved in a reviewed parameter file with
their rationale; `run_manifest.yaml` retains actual parameter values. None of
these defaults is an accepted Pi latency, accuracy or sensor-rate budget.

## Host checks

From the repository root (C++17 compiler, make and Python 3.11+ stdlib; no ROS,
OpenCV, ORB, camera, serial device or package installation):

```sh
make -C ros2/wbr_orb_node test
# Direct C++ assertions, using the same compiled fake engine:
make -C ros2/wbr_orb_node core-test
# Save a new software-check evidence directory, then re-evaluate its raw files:
python3 ros2/wbr_orb_node/scripts/check_core.py \
  --outdir ros2/wbr_orb_node/tests/software-check
python3 ros2/wbr_orb_node/scripts/check_core.py \
  --analyze-only ros2/wbr_orb_node/tests/software-check
```

The core links only the C++ standard library and platform threading/clock APIs.
Python generates defaults and runs unittest. Configured clockmap preparation
uses NumPy **only through the existing** `tools.clockmap`; that test is skipped
if NumPy is unavailable. The default invalid-mapping path needs no NumPy.
The real fixture copies eight integer source epochs from the saved 2026-10-14
RAM-take association evidence, with SHA-256 in `tests/fixtures/provenance.json`.
Its pixels, motion values and estimated poses remain synthetic.

The software checker writes the COMMON result contract (`result.json`, hashes,
raw stdout/stderr and command receipts). Its three criteria mean: `all` passed
C++ regression assertions; `real` preserved saved epochs; `emit` completed
synthetic health/runtime emission. Missing executable is BLOCKED; an assertion
failure is FAIL; bad arguments/raw metadata are error exit 3. Exit codes are
0 PASS, 1 FAIL, 2 BLOCKED, 3 usage/internal error. Analyze-only re-evaluates saved
command results; it cannot establish that a ROS package built or a Pi gate passed.
The runtime node itself is an estimator, not a lab gate check.

## Build on an already provisioned Pi

Choose a dedicated task checkout and evidence parent on the Pi. The path below
is proposed, not verified. Do not use or switch the shared runtime checkout.
No live sensor is needed for this build or replay with recorded camera IMU.

```sh
export WBR_ROOT=<local workspace>/wbr-pi-orbslam-pose
cd "$WBR_ROOT"
source /opt/ros/jazzy/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=1
export MAKEFLAGS=-j1
colcon build --base-paths ros2/wbr_orb_node --packages-select wbr_orb_node \
  --executor sequential --parallel-workers 1 \
  --cmake-args -DWITH_ORB_SLAM3=OFF
source install/setup.bash
```

For actual ORB, use a separate colcon build/install base to avoid confusing the
fake and ORB binaries. Replace these paths with the existing qualified build's
paths and required transitive libraries. On the current 4 GB Pi keep compilation
serial. Fix/install an ordinary missing dependency only after its actual build
error, preserving existing builds and logs:

```sh
export ORB_ROOT=<local workspace>/orbslam3/ORB_SLAM3
colcon --log-base log-orb build --base-paths ros2/wbr_orb_node \
  --packages-select wbr_orb_node --build-base build-orb --install-base install-orb \
  --executor sequential --parallel-workers 1 \
  --cmake-args -DWITH_ORB_SLAM3=ON -DORB_SLAM3_ROOT="$ORB_ROOT" \
  "-DORB_EXTRA_INCLUDE_DIRS=/usr/include/eigen3;$ORB_ROOT/Thirdparty/Sophus" \
  '-DORB_EXTRA_LIBRARIES='
source install-orb/setup.bash
```

`ORB_EXTRA_INCLUDE_DIRS` and `ORB_EXTRA_LIBRARIES` are CMake lists. Existing
OpenCV/Eigen, ROS rclcpp/sensor_msgs/nav_msgs and yaml_cpp_vendor development
packages and the Pangolin CMake package must be available. The native camera-day
build and installed `d435_node` linking were verified on 2026-10-21 using
`scripts/camera_day/build_pi.sh`; stereo-inertial runtime remains unverified.
Record source revision, patch hashes, library/executable
hashes and the selected ORB settings/vocabulary using the existing run harness.

## Offline first: rosbag replay

Use the accepted bag's exact topic names. A dedicated playback process/session
must not share live publishers. Start the node first; in another terminal play
the existing bag with `/clock`. Do not use looping playback; backward seek,
clock mode change or source regression requires restarting this node with a new
session. A paused ROS clock does not rewrite timestamps or renew perception.

```sh
# Terminal 1, after sourcing the intended fake or ORB install:
ros2 run wbr_orb_node orb_node --ros-args \
  --params-file "$WBR_ROOT/ros2/wbr_orb_node/config/replay.yaml" \
  -p source_session:=bag-run-unique-id \
  -p output_directory:=<local workspace>/wbr-evidence/NEW-orb-replay \
  -p left_topic:=/camera/camera/infra1/image_rect_raw \
  -p right_topic:=/camera/camera/infra2/image_rect_raw \
  -p imu_topic:=/camera/camera/imu
# ORB build additionally requires:
# -p orb_vocabulary:=/absolute/path/ORBvoc.txt
# -p orb_settings:=/absolute/path/qualified-D435i-stereoi.yaml
# Terminal 2, same ROS environment, camera publisher stopped:
ros2 bag play /absolute/path/ACCEPTED_BAG --clock
```

To publish diagnostic Odometry, additionally pass a reviewed calibration
parameter file via `--params-file /absolute/path/camera-imu-geometry.yaml`.
Its `wbr_orb_node.ros__parameters` must contain:

- `T_camera_left_optical_imu_camera`: 16 **actual** row-major SE(3) values,
  mapping camera-IMU coordinates into left optical coordinates, metres.
- `extrinsic_provenance`: actual unit/profile evidence path and hash.

Use `tools.frames.frames.sdk_ir1_to_imu(rotation_column_major, translation_m)`
then `.inverse().matrix.reshape(-1).tolist()` to obtain that matrix from the
measured SDK IR1-to-IMU readback. The helper's generic `camera_optical` frame is
this adapter's `camera_left_optical`; `imu_camera` is the camera IMU. There is no
assumed identity or robot mount default. Missing geometry withholds Odometry
while health events and tracking still run. Fake smoke tests may explicitly
supply an identity matrix with provenance `ASSUMED synthetic identity only`;
that is never camera calibration evidence.

## Source clocks and optional live mapping

`header.stamp` is treated as a bound source domain, by default `ros_header`.
No camera epoch is recovered from receipt time. The parameter
`source_header_is_d435i_time` must remain false unless independent evidence
shows that the header holds the selected D435i hardware/global-time epoch
without restamping. Global time is not monotonic time. Replay always writes
`mapping.valid=false`, even when a clockmap file was provided; a saved
camera-to-old-Pi mapping cannot establish source age in a new playback session.
Fake estimates always have invalid mapping.

For a future qualified **live** run only, prepare an existing clockmap offline.
The sender-compatible input JSON contains `valid:true`, `camera_domain`,
`camera_session`, `pi_domain:"CLOCK_MONOTONIC_RAW"`, `pi_session` (Linux boot ID)
and `exchanges_ns` (four typed integer epochs per exchange). These must be real
session-bound exchanges, not observer regressions or fabricated timestamp pairs.
For sources after the exchange window, the input config must also declare
`valid_source_min_ns`, `valid_source_max_ns` and a `validity_rationale` for the
finite extrapolation interval; without those it uses only the observed exchange
midpoint interval. The helper reuses `tools.clockmap`, including its extrapolation
uncertainty, and refuses invalid fits/identities:

```sh
python3 ros2/wbr_orb_node/scripts/prepare_mapping.py \
  /absolute/path/trusted-camera-pi-map.json /absolute/path/NEW-prepared-map.json \
  --source-domain global_time --source-session actual-camera-session \
  --pi-session actual-linux-boot-id
```

A live node uses the same `ros2 run` command with `use_sim_time:=false`, actual
`source_domain`/`source_session`, the ORB vocabulary/settings and geometry, plus
`clockmap_file`, `pi_clock_session` and verified
`source_header_is_d435i_time:=true`. This task supplies no camera launch and
has performed no live run. The node checks the Pi boot ID and a finite configured
source interval from the prepared file. Outside that interval mapping is
invalid. The unchanged exchanges travel to the health sender, which reconstructs
and evaluates the existing `ClockMap`. A static file is not an online mapping
service: renew it and restart the node after session/clock changes. Undetected
clock steps remain **UNKNOWN**. Observer events use Linux `CLOCK_MONOTONIC`
(`host_steady`), while the health sender uses `CLOCK_MONOTONIC_RAW`; never
subtract those epochs or use a map targeting one as a map targeting the other.

## Outputs and their meaning

- `diagnostic_odometry` (`nav_msgs/Odometry`, sensor-data QoS): parent `odom`,
  child **`imu_camera`**, following `tools/frames` naming. The pose composes
  `T_slam_world_camera_left_optical * T_camera_left_optical_imu_camera`.
  `mapping.hpp` supplies quaternion and integer ROS stamp extraction. The stamp
  is always the original image header, never `now()` and never the mapped Pi
  epoch. Missing geometry, invalid tracking and reset boundaries suppress it.
  After any reset/epoch change, Odometry stays withheld until process restart;
  health continues to describe later epochs. Covariance and all twist entries
  are NaN (UNKNOWN); none is synthesized as zero confidence or velocity.
- `pose_events.jsonl`: sender-compatible six required fields, plus source
  clock/session, sequence/epoch, adapter state, validity and evidence metadata.
  States map STARTING/FAULT → SYSTEM_NOT_READY, INITIALIZING → NOT_INITIALIZED,
  TRACKING → OK, LOST → LOST. A FAULT is additionally identified in the adapter
  state. Invalid results still name the attempted image source epoch; they
  represent no valid pose. The adapter supplies neither engine IMU initialization
  nor tracked-feature count, so `imu_initialized=false`, `tracked_features=0`
  are conservative **UNKNOWN** sentinels, and `degraded=true` is always set.
  No tracking-state inference claims inertial initialization. The file matches
  sender input; it does not open UART, follow a growing file or transmit packets.
- `events.csv`: existing `event_recorder.h` format, using receipt time before
  the pixel copy, queued, processing start/end, output-write time, drops and
  ingress queue depth. The left image ID identifies the tracking lifecycle;
  right and IMU receipt/queue events have distinct IDs. Output time means the
  health write/optional ROS publish returned, not subscriber receipt. Since all
  sensor messages are recorded, receive throughput is not camera frame rate.
  Recorder overwrite/contention/invalid/clock-error counters are in the CSV
  header. Counts remain authoritative if trace events were lost. Use
  `python3 tools/runtime_observer/aggregate.py /path/to/events.csv` for offline
  analysis; incomplete per-message lifecycles are expected for right/IMU inputs.
- `counts.json`, `run_manifest.yaml`: application drop/fault/tail counts and
  configured parameters. SIGINT stops callbacks, joins the runner, finishes
  buffers and saves the recorder. Abrupt process death can lose the in-memory
  event trace. Runtime evidence output failure is reported and exits nonzero.

**task-specific diagnostic frame policy:** `tools/ros2_bridge/DESIGN.md`
prohibits relabeling corrected `slam_world` as continuous navigation `odom`.
phase-14 explicitly requests `odom` output; this package supplies that named view
only on `diagnostic_odometry`, preserving unknown SLAM world axes. It publishes
no TF and establishes no `map -> odom`, body mount, ENU alignment, continuous
odometry or control input. Even initial-map loop corrections may jump. The
`odom` label alone does not qualify REP-105 semantics. Do not connect this topic
to the wheel/body EKF or a controller. Those limitations cannot be removed by
renaming a frame or restamping a pose.
