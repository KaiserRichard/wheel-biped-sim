# Proposed N3 camera helpers

**UNTESTED ON HARDWARE. Profile chosen by the owner on 2026-10-19.** Profile `n3_640x480_30`
is additive; no default config or gate state changes. Do not execute these
ROS scripts until the owner authorizes CAMERA AVAILABLE,
and power/USB and destination storage qualification prerequisites are satisfied.
See [storage draft](../../docs/STORAGE_QUALIFICATION_DRAFT.md), which itself needs review.

The intended target is realsense2_camera **4.58.4**, as pinned in
[manifest.yaml](../platform_env/manifest.yaml). From a sourced Jazzy Pi terminal,
after native SDK capture exits, use separate new directories whose parents exist:

```sh
bash lab/camera/launch_n3_camera.sh /path/to/qualified/evidence/camera-new
# In another Pi terminal, with the same ROS environment:
bash lab/camera/readback_n3_camera.sh /path/to/qualified/evidence/readback-new
```

Both scripts force ROS_DOMAIN_ID=42 and ROS_LOCALHOST_ONLY=1 and refuse an
existing evidence directory. Launch runs `ros2 run realsense2_camera
realsense2_camera_node` in the foreground with namespace `/camera`, node name
`camera`, and parameters generated from the saved raw profile by `n3_params.py`.
It saves the profile, arguments, command and combined `camera.log` via `tee`;
Ctrl-C ends the foreground pipeline. A driver or logging failure is nonzero.
Using `ros2 run` permits passing the dynamic emitter option directly.

The intended graph is `/camera/camera`, both `infra{1,2}/image_rect_raw` and
`infra{1,2}/camera_info`, separate `gyro/sample`, `accel/sample`, and `/tf_static`,
as listed in [the bag profile](../rosbag_qual/profiles/n3_640x480_30.json).
IR1+IR2 Y8 640x480/30, gyro 200, accel 100, emitter OFF, unite method 0;
depth, color, infra0, pointcloud and enable_sync are disabled, publish_tf enabled.
Topic publication, device support, effective settings and delivery remain UNKNOWN.

Readback saves node list, typed topic list, parameter dump, and verbose topic info
for every configured topic (including `/tf_static`), with separate stderr files.
`topics.txt` maps zero-based topic-info filenames to names. It attempts every
read even after a command failure. Missing, malformed or mismatching required
parameters, or failed ROS commands, produce a nonzero exit. `comparison.txt`
checks profile, both Y8 formats, rates, emitter, unite method, stream enables,
sync and TF policy. It accepts nested or dotted parameter keys. Parameter matches
are policy evidence only; manually review exact topic names/types, publishers,
QoS, firmware/serial and actual resolved streams before recording. Parameter
values do not prove sensor rate, physical emitter state, or global-time policy.
No global-time policy is added by this draft; retain and review those dump fields.
PyYAML is needed for dump comparison (present in the offline test environment;
use the intended ROS environment, no installation is performed by the scripts).

## Offline parameter-name check

Checked local source on 2026-10-19 at
`<local workspace>/Desktop/Research/wbr-perception-phase1-runtime/third_party/realsense-ros/`.
`realsense2_camera/package.xml:5` declares 4.58.4; this is source inspection,
not verification of the installed Pi binary or an unmodified upstream checkout.

| Names used | Offline source evidence (relative to that checkout) | Status |
|---|---|---|
| `camera_name`, `enable_color`, `enable_depth`, `enable_infra`, `enable_infra1`, `enable_infra2`, `depth_module.infra_profile`, `depth_module.infra1_format`, `depth_module.infra2_format` | `realsense2_camera/launch/rs_launch.py:23-49`; `src/profile_manager.cpp:450-486` parses/normalizes WxHxFPS | SOURCE-CHECKED |
| `enable_sync`, `enable_gyro`, `enable_accel`, `gyro_fps`, `accel_fps`, `unite_imu_method` | `realsense2_camera/launch/rs_launch.py:60-69`; `src/profile_manager.cpp:611` registers motion FPS | SOURCE-CHECKED |
| `pointcloud.enable`, `publish_tf` | `realsense2_camera/launch/rs_launch.py` configurable parameters | SOURCE-CHECKED |
| `depth_module.emitter_enabled` | `README.md:420-422` names the option; `src/sensor_params.cpp:170-208,229-297` dynamically registers supported module options | SOURCE-CHECKED; availability/type on current device requires readback |

**UNVERIFIED parameter names: none** in the command/comparator after this offline
source check. All runtime behavior is **UNTESTED ON HARDWARE**. Namespace/node
remaps use ROS arguments; no `camera_namespace` parameter is passed by `ros2 run`.

Offline verification: `python3 tools/test_support/run_unittest.py lab/camera/tests`.
The hand-written YAML is synthetic, not captured hardware output.
