# Planner implementation report

## Commands and results

Host validation (Mac has no ROS 2 installation):

```text
python3 -m unittest discover -s ros2/wbr_planner_node/test -v
Ran 11 tests in 2.663s
OK
box sequence core runtime ms/frame: mean=95.38, max=111.52
clear sequence core runtime ms/frame: mean=95.89, max=99.88
```

The test renderer provides a 15 cm box on the 1.5 m line, clear-floor sequence,
missing pose, >1 m/s pose jump, and no-floor frame. In the box case the chosen
lateral shift is at least 0.30 m; the clear case returns `GO_STRAIGHT`. Shell syntax validation and the launch trap regression test passed. Python
syntax compilation passed for core, ROS adapter, launch file, and setup file.

The core uses only NumPy and repository modules `tools/ground_plane/plane.py`
and `tools/local_map/{fuse,grid}.py`. Runtime imports in those files do not
require SciPy, OpenCV, or Matplotlib. Bag replay utility dependencies do not
enter the live path. Host timing is a Mac measurement, not a Pi performance
claim.

## Limitations

- Pi build, CPU load with ORB-SLAM3, live synchronization and Foxglove display
  have not been tested; ROS 2 is absent on this Mac.
- The planner searches straight parallel corridors with one fixed 3 m goal.
  It does not generate a dynamically feasible curve or optimize speed.
- Dynamic obstacles and uncertainty-aware obstacle tracking are not modeled;
  the elevation grid accumulates observed cells.
- Gravity alignment depends on seeing enough floor in the first valid frame.
  Large pose discontinuities require three locally consistent poses and then
  start a new map epoch; subtler loop-closure changes are not independently
  detected.
- A no-pose frame yields `NO_POSE` and is not fused. The ROS adapter publishes
  that advisory but map outputs require an initialized plane.

## Review fixes

1. **Adapter-only `max_rate_hz` passed into `PlannerCore`:** added pure
   `core_parameters()` filtering and use it before core construction. Tested by
   constructing `PlannerCore` from the adapter's parameter set without ROS.
2. **ROS setup under nounset:** moved the three setup sources into
   `source_ros_setups()`, which disables `-u` for all three and restores it.
   The shell regression sources three stubs that expand unset
   `AMENT_TRACE_SETUP_FILES`, then verifies nounset is enabled again.
3. **INT/TERM cleanup resumed launch execution:** INT and TERM handlers now exit
   with 130 and 143; the EXIT trap owns idempotent cleanup. The shell regression
   sends INT to a stub session and verifies exit 130 and exactly one cleanup.
4. **Pose tolerance also acted as processing deadline:** timestamp matching
   remains 50 ms; a separate bounded 500 ms pending-frame timeout allows delayed
   matching poses to arrive. ROS-free adapter tests verify a pose at 80 ms is
   accepted and an unmatched frame expires at 500 ms.
5. **Collision checks omitted lateral transition:** candidate checking covers
   the swept footprint near the robot and the target corridor for the remaining
   route. A regression places an obstacle at `(0.025, 0.30)` for a camera at
   `(0, 0.60)` and verifies the planner does not shift right through it; the
   existing box avoidance test still passes.
6. **Rejected pose discontinuities could eventually pass:** a discontinuity now
   latches fusion and requires three consecutive locally consistent poses,
   after which the map is reinitialized as a new epoch. Regression reproduces
   the 2 m jump and long timestamp gap and verifies the jump is not fused.
7. **No terminal goal handling:** positions at or beyond the goal return
   `GOAL_REACHED` with zero distance before corridor selection. A fully occupied
   grid at `x=3.1 m` verifies terminal output.
8. **Current arrow translated twice:** its marker pose is now identity while
   endpoints remain absolute map coordinates. A pure adapter test verifies the
   arrow at `(1, 0.2)` starts at `(1, 0.2, 0.05)`.
9. **Recording omitted planner map and transforms:** added
   `/wbr_planner_node/grid`, `/tf`, and `/tf_static` to the bag topic list.
   Verified by inspection alongside `bash -n` for the launcher.

Regression command:

```text
WBR_REPO_ROOT=$PWD python3 -m unittest discover -s ros2/wbr_planner_node/test -v
bash -n lab-day-20261025/path_live.sh
bash ros2/wbr_planner_node/lab_test_traps.sh
```
