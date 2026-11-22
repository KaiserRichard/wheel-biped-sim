# ORB-SLAM3 map export

Export the latest `/d435_orb/map_points` replacement snapshot, camera poses, and status counts from a rosbag2 directory backed by MCAP or SQLite3 (`db3`). The reader uses the installed `rosbags` type store to decode ROS 2 messages; no ROS runtime is required.

```bash
python3 tools/map_export/export_map.py \
  /path/to/map-bag --output /path/to/output
```

The default voxel width is 0.02 m; use `--voxel 0` to preserve every point in the selected snapshot. Output files are `map.ply`, `trajectory.csv`, `topdown.png`, `side.png`, and `summary.json`.

## Snapshot and frame interpretation

Every cloud message replaces the previous cloud, including an empty message. The exporter therefore uses the last cloud in bag log order and never unions snapshots. This preserves reset/merge replacement behavior and avoids mixing disconnected map states. `MAP_POINTS.md` defines landmarks as world-frame ORB map points, already expressed in `slam_world`; the exporter applies no transform. Stereo world axes follow the initial optical camera axes: x right, y down, z forward. The top-down plot uses x-z; the side plot uses z-y and labels optical-down y.

Pose positions and orientations are copied from `/d435_orb/pose`; `t_ns` uses the message source stamp, falling back to bag log time only when that stamp is zero. Status JSON `tracking_state` values are counted separately; the raw status JSON is not modified.

Point density is exported point count divided by the axis-aligned bounding-box volume, so it is a descriptive sparse-cloud statistic, not physical coverage. For a zero-volume box it is `null`.

Run the focused tests with:

```bash
python3 -m unittest discover -s tools/map_export/tests -v
```
