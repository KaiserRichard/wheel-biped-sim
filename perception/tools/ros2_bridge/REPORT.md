# Q5 report — ROS 2 bridge design and skeleton

Status: **completed design/skeleton task; ROS package UNBUILT**.
No ROS 2, ORB-SLAM3, camera, Pi, MCU, navigation or safety gate PASS is claimed.

## Built

- [DESIGN.md](DESIGN.md): actual Q1 EstimateSnapshot boundary; diagnostic camera
  PoseStamped in `slam_world`; R1 TF ownership and odometry exclusion; per-topic
  QoS; replay `/clock`; explicit mapping evidence, age/uncertainty and session
  identity; reset/stale behavior; covariance policy; fixed MCU authority.
  REP-103/105, Jazzy QoS, diagnostic_updater and nav_msgs/message/API sources
  have URLs and source line citations.
- `package.xml`, `CMakeLists.txt`, `src/bridge_node.cpp`: **UNBUILT** rclcpp/Jazzy
  skeleton, with diagnostic_updater and PoseStamped publisher. Includes the
  actual sibling adapter header. It links neither ORB nor the adapter engine.
  Public submit seam has no producer or clock mapper wired in main.
- `mapping.py`: Python stdlib ROS-free reference field conversion and gate.
  `src/mapping.hpp`: ROS-free C++ counterpart used by the skeleton.
- `tests/test_mapping.py`, `tests/core_probe.cpp`, Makefile and README:
  hardware-free mapping/policy checks and optional cross-language field parity.
  Build artifacts are local and ignored under `build/`.

## Exact commands and results

Commands ran from `<local workspace>/Desktop/Research/wbr-sw-foundation`:

```sh
make -C tools/ros2_bridge test
```

**MEASURED, synthetic:** exit 0. Apple host compiler built `build/core_probe`
with `-std=c++17 -Wall -Wextra -Werror -pedantic` and the real Q1 `adapter.h`.
C++ policy assertions passed. Python unittest ran **18 tests, all passed**,
including C++/Python output parity for the nonidentity pose and identity/half-turn
rotation fixtures. Final run: `Ran 18 tests in 0.032s`, `OK`.

```sh
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tools/ros2_bridge/tests -v
```

**MEASURED, synthetic:** exit 0. `Ran 18 tests in 0.002s`,
`OK (skipped=1)`: **17 passed**, optional compiled C++ parity skipped because the
probe environment variable was absent. This command is pure Python/stdlib and
requires neither ROS nor a compiler.

Coverage: exact integer-ns split and ROS stamp limits; parent/child names;
nonidentity transform direction and translation; xyzw quaternion order and half
turns; proper finite SE(3); loss/initialization/fault/reasons; frozen heartbeat
age; invalid/reset events with unchanged sequence; map/epoch changes; sequence
and source regressions; clock session/domain/generation mismatch; mapping
uncertainty; missing source/pose/mapping; foreign-domain watermarks; available
versus unavailable covariance metadata; replay binding; unconfigured budgets.

**MEASURED, static only:** Python AST and package XML parse; local Markdown
targets resolve; rclcpp skeleton has only a PoseStamped publisher and no TF,
Odometry, MCU or engine output call. These checks do not compile ROS APIs.
The initial host probe build found a test helper named `clock()` conflicting
with the C library; it was renamed `clock_evidence()` before the passing runs.

## Verified versus NOT verified

**MEASURED / verified:** synthetic ROS-free reference mapping, listed gate
behavior, host C++ core assertions and selected field parity against Python.
**SOURCE-SUPPORTED:** primary ROS schemas/conventions and inspected APIs cited
in DESIGN.md. **INFERRED:** output policies, mailbox fault latch, QoS choices,
wall polling and replay/controller isolation design.

**UNKNOWN / NOT verified:** rclcpp/ament package compilation, ROS message
serialization, installed Jazzy diagnostics version, real DDS QoS compatibility,
timer/executor behavior, paused diagnostics, node mailbox fault handling,
complete clock-jump detection, real mapper uncertainty/source age, native
adapter/ORB runtime integration, world alignment, final robot mount/covariance,
Pi throughput/CPU/latency, hardware loss/reset/bridge-death trials, MCU behavior.
ROS node source remains **UNBUILT**, even though its ROS-free helper compiled.

## Open integration items

Connect a serialized snapshot owner that forwards feed/reset/fault events.
Supply a trusted session/generation and characterized clock mapper; configure
reviewed age/uncertainty limits. Add complete jump/seek invalidation and retained
epoch sidecar records. Build on an already provisioned Jazzy host and verify
QoS/remaps, clock pause/seek, reset/malformed/stall/death behavior and bounded
ingress. Measure native versus native plus bridge load on the Pi only under the
existing hardware authorization/gates. Review placeholder package
maintainer/license before distribution. Continuous local odometry and qualified
map alignment are absent, so odometry/TF publication stays excluded.

## Scope and process

All authored files and test build outputs are under `tools/ros2_bridge/`.
The reference checkout and real data were not modified; real data were not
copied into the repository. No SSH, installs, package downloads or hardware
commands ran. Primary upstream documentation/source was read through browsing.

Process deviation: one read-only `git status --short` ran in the initial
inventory call before COMMON_QUEUE.md's no-git rule had been read. No later git
command or git mutation ran; the runner retains commit responsibility.
