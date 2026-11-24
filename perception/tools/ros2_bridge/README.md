# Q5 ROS 2 output bridge — UNBUILT

Diagnostic camera PoseStamped in `slam_world` and diagnostic_updater skeleton.
No Odometry or TF output. See [DESIGN.md](DESIGN.md) for clock, reset, covariance,
QoS, replay and controller authority requirements; [REPORT.md](REPORT.md) for
verification limits. ROS 2 is absent; no ROS package build is claimed.

From the repository root:

```sh
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tools/ros2_bridge/tests -v
make -C tools/ros2_bridge test
```

The first command needs Python stdlib only; optional C++ parity is skipped.
The second also compiles a ROS-free C++17 probe against Q1's `adapter.h` and
enables parity. It never builds rclcpp or ORB. Outputs stay in this directory.
`make -C tools/ros2_bridge clean` removes the probe binary.

`mapping.py` consumes Q1's typed snapshot JSON projection. `src/mapping.hpp`
uses the actual C++ EstimateSnapshot. `src/bridge_node.cpp` is explicitly
UNBUILT and has no snapshot producer or clock mapper connected. Missing session,
age budgets or mapping evidence suppress all poses; the executable is a skeleton,
not a functional estimator or a controller input.
