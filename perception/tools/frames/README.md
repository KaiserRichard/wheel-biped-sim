# Diagnostic sensor pose to robot pose (B5 / C5)

Small Python library and CLI using stdlib + numpy. Converts camera or camera-IMU
poses to `base_link` in `slam_world`, retaining integer nanosecond stamps, clock
domain, map ID, reset counter and independent segment IDs. No ROS publisher.

## Conventions and evidence

**SOURCE-SUPPORTED:** `T_A_B` maps B coordinates into A. The pinned ORB revision
is `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`. Its
[System.h lines 106–110](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/include/System.h#L106-L110)
declares `TrackStereo`; [System.cc lines 316–325](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/System.cc#L316-L325)
returns tracker `Tcw`. [Tracking.cc lines 1493–1516](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Tracking.cc#L1493-L1516)
returns the current frame pose; [Frame.cc lines 474–478 and 523](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Frame.cc#L474-L523)
establish world-to-camera direction. Invert live `Tcw` before composing the mount.

**SOURCE-SUPPORTED:** the eight-column layouts mimicked by the CLI come from
[System.cc lines 617–624](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/System.cc#L617-L624)
(TUM: seconds, camera-to-world translation, `qx qy qz qw`) and
[lines 757–772](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/System.cc#L757-L772)
(EuRoC/ORB export: nanoseconds, `qx qy qz qw`; inertial mode exports IMU-to-world,
non-inertial mode camera-to-world). Inertial “body” means the camera IMU, never
the robot body. Eigen quaternion constructor order is `wxyz`, whereas Eigen
`coeffs()` and these ORB text exports are `xyzw`; order is always an explicit
argument. CLI `wxyz` is an adapter input option, not a claim about ORB text files.

**SOURCE-SUPPORTED:** [REP-103 axis conventions, lines 95–124](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst#L95-L124)
give body forward/left/up and optical right/down/forward. At a common origin,
`optical_to_body()` maps optical x to body -y, optical y to body -z, and optical z
to body x. Its inverse performs the reverse conversion.
[REP-105 lines 46–76](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst#L46-L76)
requires continuous `odom` and permits jumps in `map`.

**INFERRED implementation:**

```text
T_world_base = T_world_camera @ inverse(T_base_camera)
T_base_imu_camera = T_base_camera @ inverse(T_imu_camera_camera)
T_world_base = T_world_imu_camera @ inverse(T_base_imu_camera)
```

Here the frame name `imu_camera` identifies the camera IMU; the SDK helper returns
`Transform("imu_camera", "camera_optical", ...)`. SDK IR1->IMU acts on points in
IR1 to produce IMU coordinates. [librealsense v2.58.4 rs_sensor.h lines 105–109](https://github.com/realsenseai/librealsense/blob/v2.58.4/include/librealsense2/h/rs_sensor.h#L105-L109)
specifies column-major rotation and metres; the helper explicitly reshapes with
Fortran order. See also [rs_sensor.h extrinsic query, lines 444–451](https://github.com/realsenseai/librealsense/blob/v2.58.4/include/librealsense2/h/rs_sensor.h#L444-L451).
Local direction/readback evidence is in the read-only reference checkout's
`experiments/2026-10-13-n4b-d435i-prep/calibration_notes.md`, “Transform direction
and math.” No unit-specific readback values are embedded as defaults here.

**UNKNOWN / PROPOSED:** `examples/mount_proposed.json` assumes a co-located,
forward-facing camera solely for synthetic use. Actual mount rotation and lever
arm require the final robot mount. It is mandatory to provide a mount, and its
status/provenance are retained in output. A static mount assumes a rigid camera;
moving-link geometry needs a timestamped mount implementation outside this tool.

**INFERRED ownership from R1 §2:** the local wheel/body EKF owns
`odom -> base_link`; a qualified SLAM bridge owns `map -> odom`, calculated at the
same epoch as `T_map_base @ inverse(T_odom_base)`. Robot-state publisher owns the
mount/joint chain; camera driver owns internal sensor geometry. This converter
publishes no TF and provides no alignment or continuous local estimator.
It requires exactly `slam_world` and refuses `odom` or any alternate world name.
It preserves engine world axes; it does not claim world ENU or gravity alignment.

## CLI usage

Run from the repository root:

```sh
python3 tools/frames/convert_trajectory.py tools/frames/examples/camera.tum \
  --epochs tools/frames/examples/epochs.jsonl \
  --mount tools/frames/examples/mount_proposed.json \
  --time-unit s --sensor-frame camera_optical \
  --pose-direction sensor-to-world --quaternion-order xyzw \
  --output tools/frames/examples/base.synthetic.jsonl
```

TUM uses the flags above. For a serialized live `Tcw` adapter, use
`--pose-direction world-to-sensor` with its explicitly chosen quaternion order
and timestamp unit. For non-inertial ORB EuRoC output choose `--time-unit ns`
and `--sensor-frame camera_optical`. For inertial ORB EuRoC output choose
`--time-unit ns --sensor-frame imu_camera`; the mount must additionally contain:

```json
"sdk_ir1_to_imu": {
  "status": "MEASURED",
  "provenance": "actual unit/profile SDK readback artifact and hash",
  "rotation_column_major": [9 actual SDK coefficients],
  "translation_m": [3 actual SDK coefficients]
}
```

This schema illustration is not literal JSON data or calibration. Supply actual
numeric arrays and honest status/provenance. No fabricated IMU default is supplied.

Input has exactly eight whitespace-delimited columns; blank lines and `#` comments
are permitted. Quaternion norm must be within `1e-3` of unity and is normalized
after validation. Timestamp conversion uses Decimal without a float round trip;
sub-nanosecond stamps and int64 overflow are rejected. Input is never sorted.

The epoch sidecar is a **custom INFERRED JSONL contract**, not an ORB format.
One record per emitted pose is required, in the same order, with an exact stamp
match. `map_id` and `clock_domain` are nonempty strings; reset counter is a
nonnegative integer:

```json
{"stamp_ns":1,"clock_domain":"device_session_a","map_id":"map_a","reset_counter":0}
```

Stock trajectory files omit epoch events. Obtain these through a C4 adapter;
never reconstruct reset/map identity from position jumps. Do not guess metadata
for an unaudited export, and account for skipped lost frames when pairing rows.

Every change of map ID or reset counter starts a new segment, including returning
to a previously seen pair. Timestamps must increase strictly inside each segment;
they may restart at a boundary. Clock-domain changes are rejected, including at
boundaries; map clocks explicitly before conversion. No time subtraction or pose
difference crosses a boundary. `slam_world` in two different segments need not
denote aligned coordinates; retain segment identity downstream.

Output is custom JSONL with `T_world_base` (row-major 4x4), reference/child frames,
epoch/clock fields, segment ID, `velocity_valid`, and optional
`diagnostic_linear_velocity_world_m_s`. At each segment start velocity is `null`
and invalid. Other velocities are backward differences in world axes, useful
only for inspection: loop corrections can jump within a segment. No tracking
health, qualified navigation velocity, angular velocity or covariance is inferred.
Provenance includes input hashes, conventions and the complete mount record.

## Library usage and verification

Import `Transform`, `Sample`, `convert`, `optical_to_body`, `sdk_ir1_to_imu` and
`base_imu_camera` from `tools.frames.frames`. `Transform(parent, child, matrix)`
validates SE(3); composition checks adjacent names, and inverse swaps names and
uses `-R.T @ t`. `Transform.from_pose(..., order="xyzw")` makes quaternion order
mandatory. `convert()` accepts sensor-to-world samples and a sensor-to-base mount
in the named convention (sensor points map into base).

```sh
python3 -m unittest discover -s tools/frames/tests -v
```

**Verified: synthetic tests only**, 10 passing tests: non-identity inverse/frame
checks, optical basis, explicit quaternion orders, rotation with lever arm and
pose round trip, non-identity SDK column-major extrinsic and camera/IMU route
equivalence, reset/map splits, invalid boundary velocities, clock/order rejection,
SE(3) validation, exact Decimal stamps, CLI conversion and `odom` rejection.

**NOT verified:** real TUM/ORB trajectories, live ORB tracking/reset observations,
actual robot mount, SDK readback ingestion on hardware, calibration accuracy,
world alignment, moving mounts, health/covariance or navigation qualification.
No hardware was used, no packages installed, no gate promoted.
