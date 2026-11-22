# Offline ORB-SLAM3 run checks

These scripts read rosbag2 directories with `rosbags` and need no ROS runtime. Use the perception virtual environment and a headless Matplotlib backend:

```bash
export MPLBACKEND=Agg
export MPLCONFIGDIR=/private/tmp/wbr-mpl-cache
PY=python3
```

## L1 3.00 m line

```bash
$PY tools/orb_checks/l1_line.py /path/to/l1-replay-bag --tape 3.00 --output /path/to/l1-check
```

Writes `l1_line.json` and `l1_line.png`. It finds the first and last pose segments with adjacent speed below 0.02 m/s lasting at least 1 s, uses each segment's median position, and reports their straight-line separation, full pose path length, max/RMS distance from the start-to-end line, status OK percentage, >1 m/s pose jumps, and signed/absolute tape error. The plot is top view in the initial optical x-z plane.

## M1 lab map

```bash
$PY tools/orb_checks/m1_summary.py /path/to/m1-replay-bag --output /path/to/m1-check
```

Writes `m1_summary.json` and calls `tools/map_export/export_map.py`'s export function to write `topdown.png`, `side.png`, `map.ply`, `trajectory.csv`, and `summary.json` in that output directory. The JSON reports status counts and LOST/RECENTLY_LOST sample times and transitions, map-change count and times when `/d435_orb/map_points_info` exists, >1 m/s pose jumps, trajectory length, and pose endpoint return distance. The sensor-only bag does not contain poses/status; run the replay first and point this command at the derived output bag. Map changes cannot be inferred from pose/status alone, so the JSON marks their availability explicitly.

## Tests

```bash
MPLBACKEND=Agg MPLCONFIGDIR=/private/tmp/wbr-mpl-cache \
  python3 -m unittest discover -s tools/orb_checks/tests -v
```

Tests write temporary SQLite rosbag2 fixtures using `rosbags.rosbag2.Writer` and remove them after the run.
