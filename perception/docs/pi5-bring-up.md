# Pi 5 bring-up

**FROM FILE.** The Phase 1 baseline used Raspberry Pi 5, Ubuntu 24.04 ARM64, ROS 2 Jazzy, a USB 3 RealSense D435i, and its built-in IMU. Start with the native ROS environment, camera SDK and driver. Record driver and firmware versions, negotiated USB speed, temperature and throttling flags before using a run as performance evidence. [Phase 1 bring-up guide](https://github.com/KaiserRichard/wbr-perception-phase1-runtime/blob/fde5721af0627eaf60c78d95296ab26924425e37/docs/RPI5_NATIVE_PERCEPTION_BRINGUP.md)

**STANDARD PRACTICE.** From the main checkout, source Jazzy, build the available wrapper packages, and source the resulting overlay. Keep the perception directory path available to nodes that load the Python tools.

```bash
source /opt/ros/jazzy/setup.bash
export WBR_PERCEPTION_ROOT="$PWD/perception"
export WBR_REPO_ROOT="$WBR_PERCEPTION_ROOT"
colcon build --base-paths "$WBR_PERCEPTION_ROOT/ros2" \
  --packages-select wbr_obstacle_node wbr_planner_node twin_cmd_bridge
source install/setup.bash
ros2 launch wbr_obstacle_node obstacle.launch.py algo_path:="$WBR_PERCEPTION_ROOT"
```

**FROM FILE.** For ORB-SLAM3, follow the [wrapper build and replay steps](../ros2/wbr_orb_node/README.md). Provide the external ORB checkout, vocabulary and camera settings explicitly. First verify camera info and synchronized stereo or aligned colour/depth, then image/IMU clocks and rosbag replay in its own ROS domain. Run a known EuRoC sequence and a local bag before live mapping. Camera identity, calibration and ROS overlays are deployment inputs.

**FROM FILE.** Set `CAMERA_SERIAL` locally for native ORB settings preparation and Allan recording. Set a local SDK serial and firmware version in the raw capture and calibration qualification policy before using it with the connected camera. The checked-in policy values are fixture aliases.
