# C2-RN implementation report

## Implemented

- Created the `ament_python` package, setup metadata, launch file, and ROS 2 node.
- Kept perception, temporal decision policy, rate limiting, depth conversion,
  marker specifications, point generation, and proposed byte-frame generation
  in ROS-independent `core.py`.
- The node subscribes to depth and camera info, drops over-rate frames, then
  publishes JSON advisory, proposed `TerrainAdvisory` bytes, optical-frame
  markers, and optionally downsampled depth points. It contains no serial,
  motor-topic, or STM32 transport code.
- Algorithm modules are imported from the repository at runtime. Set `algo_path`
  to the repository root containing `tools/obstacle_decision`.

## Commands and results

From the repository root:

```text
$ python3 setup.py build  (cwd: ros2/wbr_obstacle_node)
running build
running build_py
copying wbr_obstacle_node/core.py -> build/lib/wbr_obstacle_node
copying wbr_obstacle_node/node.py -> build/lib/wbr_obstacle_node
warning: build_py: byte-compiling is disabled, skipping.
exit status: 0

$ python3 -m unittest discover -s ros2/wbr_obstacle_node/test -v
test_16uc1_millimetres_and_zeros ... ok
test_marker_spec_contents ... ok
test_rate_limiter_drops_without_queue ... ok
test_synthetic_five_cm_box ... ok
Ran 4 tests
OK

$ python3 -m py_compile $(find ros2/wbr_obstacle_node -name '*.py' -print)
exit status: 0 (no output)
```

The first unittest invocation failed because discovery puts `test/` rather than
the package directory on `sys.path`. The test file now adds the package and
repository roots; the exact requested command then passed.

## Limitations and verification status

- `setup.py build` verifies the Python package copy only. ROS 2 Jazzy is not
  installed on this Mac, so `colcon build`, message conversion, launch, and
  publication were not executed.
- Pi build/run and Foxglove visualization are **UNVERIFIED**.
- The obstacle box marker is centered on the optical axis because the current
  detector reports cluster width but not lateral centroid. Its orientation is
  axis-aligned in the optical frame. The point cloud contains valid sampled
  depth points, not an obstacle-only semantic cloud.
- The codec is the existing **PROPOSED** format. Unavailable floats are sent as
  zero because the codec rejects non-finite values; JSON uses `null` for those
  fields. This remains informational only.
- `processing_ms` measures the detector/decision core call; ROS serialization
  and publish overhead are excluded.
