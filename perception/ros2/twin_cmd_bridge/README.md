# AGV-style /cmd_vel rehearsal bridge

This `ament_python` package forwards ROS 2 commands to the Mac phase-1 physics twin
using JSON over UDP. **The real WBR DM-jump firmware uses PS2 + UART, has no
micro-ROS and has no `/cmd_vel`** (SOURCE-SUPPORTED by owner sessions).
`/cmd_vel` and micro-ROS belong to the AGV firmware. This package is an
**AGV-style /cmd_vel rehearsal**, not a WBR hardware command interface.

## Package behavior

`ros2 run twin_cmd_bridge bridge` subscribes to `geometry_msgs/Twist` on
`/cmd_vel` and `std_msgs/Empty` on `/twin/jump`. It forwards `linear.x` as
`v` (m/s), `angular.z` as `yaw_rate` (rad/s), and each `Empty` as a jump edge.
Other Twist axes are ignored. The planar twin logs yaw but cannot rotate.
The [protocol and assumptions](https://github.com/KaiserRichard/wbr-perception-phase1-runtime/blob/fde5721af0627eaf60c78d95296ab26924425e37/docs/twin3d_cmd/README.md) are shared with
the Mac sender/listener.

| Parameter / policy | Default | Evidence |
| --- | --- | --- |
| `host` | `localhost` | ASSUMED; set to Mac LAN address when running on Pi |
| `port` | `9876` | ASSUMED UDP port |
| `src` | Empty selects unique `agv-ros-<process-start-time-ns>` | ASSUMED namespace using MEASURED process-start wall time |
| Velocity clamp | ±0.6 m/s | ASSUMED; receiver also clamps |
| Local Twist freshness for jump packets | <0.5 s, receiver-monotonic | ASSUMED; stale Twist produces zero velocity/yaw |
| Subscription queue depth | 1 | ASSUMED; reduces queued command backlog |
| UDP sequence | Starts at zero, increments once per successfully sent packet | ASSUMED protocol |
| UDP timestamp | Current Unix seconds | MEASURED sender wall time |

No replay timer is used. Without new ROS callbacks, the Mac watchdog expires
after 0.5 s (ASSUMED). A jump callback writes velocity too: it uses the most
recent locally fresh Twist, otherwise zero. Successive `Empty` messages are
independent request edges; the twin rejects them while its jump is active.
phase-1 `SETTLE` stays locked until Mac reset. A fixed `src` must have a continued
sequence or be changed after process restart; default unique sources avoid
sequence restarts colliding. Host/port/source parameters are read at startup.

## Owner-run Pi commands (not executed by this runner)

From a repository checkout on the Pi with ROS 2 Jazzy, `colcon`, and
`teleop_twist_keyboard` already available:

```sh
source /opt/ros/jazzy/setup.bash
colcon --log-base /tmp/t2-ros-log build --base-paths ros2/twin_cmd_bridge --packages-select twin_cmd_bridge --build-base /tmp/t2-ros-build --install-base /tmp/t2-ros-install
source /tmp/t2-ros-install/setup.bash
export TWIN_MAC_HOST=pi-host
ros2 run twin_cmd_bridge bridge --ros-args -p host:="$TWIN_MAC_HOST" -p port:=9876
```

`pi-host` is an **ASSUMED example Mac address**, not a discovered address;
replace it with the actual reachable Mac LAN IP. The Mac listener must run
with `--host localhost`. Clocks must satisfy the ASSUMED receiver age policy
(≤0.5 s old, ≤0.1 s future). These commands do not install software or modify
firmware. This runner did not SSH or execute any Pi command.

In a second Pi terminal, source ROS and run keyboard teleop:

```sh
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p repeat_rate:=10.0 -p key_timeout:=0.3
```

The 10 Hz repeat and 0.3 s teleop key timeout are ASSUMED rehearsal settings.
The repeat period (CALCULATED 0.1 s) stays below the Mac watchdog interval.
`teleop_twist_keyboard` prints its key bindings; its default topic is `/cmd_vel`.
The Mac source overlay should show `agv-ros-...`. To request one simulated
jump from a third ROS terminal:

```sh
source /opt/ros/jazzy/setup.bash
ros2 topic pub --once /twin/jump std_msgs/msg/Empty '{}'
```

Stop ROS input to observe watchdog braking; use R on the Mac to reset/rearm.
PS2/UART mapping is documented in the main README and is conceptual only.

## Tests without ROS

`core.py` uses only Python's standard library. `node.py` imports `rclpy` and
message types only inside `main()`, so import/serialization/forwarding tests
run without ROS installed:

```sh
export PYTHON=./.venv/bin/python
"$PYTHON" tools/test_support/run_unittest.py ros2/twin_cmd_bridge
```

The real localhost test skips on sandbox `PermissionError` at bind. A skipped
socket test does not establish delivery. Building the package with `colcon`,
ROS callback behavior and actual Pi-to-Mac networking remain unverified here.
