# WBR live lateral path planner

Advice-only ROS 2 Jazzy package. The NumPy core imports no ROS modules and reuses
`tools/ground_plane/plane.py` and `tools/local_map/{fuse,grid}.py`. Those live
paths need NumPy only; `cv2` and `rosbags` in `from_bag.py` are offline-only.

## Build on the Pi

Link or copy this package into a colcon source workspace and build in the same
overlay as the camera stack:

```bash
source /opt/ros/jazzy/setup.bash
source <local workspace>/wbr-camera-ros-build/install/setup.bash
cd <local workspace>/wbr-planner-build
colcon build --packages-select wbr_planner_node
source install/setup.bash
```

The live launcher adds the repository root to `PYTHONPATH`, required because
the pure core directly reuses the repository's `tools` modules. Keep that source
tree available at runtime. Run the scenario with:

```bash
lab-day-20261025/path_live.sh <local workspace>/wbr-evidence/path-live-20261025-01 90
```

The script uses ROS domain 201 and localhost discovery, starts RGB-D capture,
ORB-SLAM3, the planner, Foxglove bridge, and a SQLite bag, and records logs and
one-second CPU/memory samples. `CAMERA_SETUP`, `PLANNER_SETUP`, and `ROS_SETUP`
can override setup paths. The output directory must be new for rosbag recording.

## Topics and display

The node subscribes to aligned 16-bit millimetre depth, color camera intrinsics,
and `/d435_orb/pose` (`T_wc` in `slam_world`). It processes at up to 3 Hz,
keeps depth subscription depth one, requires a nearest pose within 50 ms, and
rejects motion above 1 m/s or 60 degrees/s. Suggestions are JSON on
`/wbr_planner_node/suggestion`; the map and visualization are on
`/wbr_planner_node/grid` and `/wbr_planner_node/markers`. TF publishes
`wbr_map -> slam_world` after the first valid ground plane. No motor or serial
interface exists in this package.

In Foxglove, connect to `ws://<pi-address>:8765`, add a 3D panel with fixed
frame `wbr_map`, enable `/wbr_planner_node/markers` and
`/wbr_planner_node/grid`, and add a Raw Messages panel for
`/wbr_planner_node/suggestion`. The markers show start, goal, current position,
selected corridor, and obstacle cells. The occupancy grid uses -1 unknown, 0
free, and 100 obstacle.

Candidate corridor centers span -0.60 to +0.60 m in 0.05 m increments. Each
candidate is tested from current forward position to the fixed 3.00 m goal with
the configured robot half-width. Obstacle cells exceed 0.03 m elevation.
Unknown-space fraction adds a 0.5 weighted cost; absolute lateral displacement
adds 1.0 weighted cost. This is a straight-corridor lateral-shift suggestion,
not a trajectory controller.
