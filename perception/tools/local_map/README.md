# Offline robot-centric local elevation map

NumPy-only mapping core for fusing axial depth points into a scrolling 2.5D
height grid. Matplotlib is used only to make evaluation plots. There is no ROS,
GPU, network access, or install step.

## Frames

Depth uses the camera optical frame: x right, y down, z forward; depth is axial
Z in metres. `T_world_camera` maps camera points into ORB-SLAM3's `slam_world`
(whose origin is the first camera pose). The renderer's synthetic world is
(right, forward, up).

The first valid depth frame is passed to `tools/ground_plane/plane.py`. Its
camera-facing floor normal is transformed by the first `T_world_camera` and
defines map +z. The first camera-right direction projected onto the floor
defines map +x, and +y is +z cross +x. The map origin is the projection of the
first camera centre onto the estimated floor. Thus the initial ORB world is
rotated and translated into a local gravity-aligned frame; later poses and
points use that same fixed transform. This assumes a rigid, locally planar
floor and a valid first-frame floor estimate.

## API and behaviour

- `grid.py`: `ElevationGrid` stores per-cell mean/variance, max observed height,
  point count and last-seen time. Each frame's same-cell samples are inverse
  variance averaged, then incorporated with a scalar Kalman update. `StereoNoise`
  uses the camera axial range with `sigma_Z = Z^2 sigma_d / (f B)`. Scrolling moves the grid window while
  preserving world-cell contents. Variance grows for stale observations;
  cells older than `unknown_after_s` become unknown in outputs.
- Grid classes are `0=unknown`, `1=free`, `2=obstacle`; obstacle means maximum
  observed height above the floor exceeds the configured threshold.
- `fuse.py`: stride-samples depth, backprojects with the shared ground-plane
  helper, transforms points through pose and gravity frame, then updates the
  grid. `pose_bias` applies accumulated end-of-sequence translation and
  rotation drift while anchoring the first pose.
- `synth_sequence.py`: 16 frames, 1.5 m forward travel, 0.30 m camera height,
  pitch oscillation ±3°, and floor-resting boxes of 3, 5 and 12 cm. Rendering
  uses `tools/ground_plane/synth.py`.
- `evaluate.py`: produces `results/metrics.json` and three PNGs.

## Run

From the repository root:

```sh
MPLBACKEND=Agg MPLCONFIGDIR=/private/tmp/wbr-mpl-cache \
  python3 \
  -m unittest discover -s tools/local_map/tests -v
MPLBACKEND=Agg MPLCONFIGDIR=/private/tmp/wbr-mpl-cache \
  python3 tools/local_map/evaluate.py
```

The evaluation stride is 4 pixels and the grid is 4 m at 5 cm resolution.
See [REPORT.md](REPORT.md) for the recorded run and measured host timing.

## Map a recorded RGB-D bag

`from_bag.py` reads a rosbag2 directory with the ROS 2 Jazzy typestore (SQLite
or MCAP). It pairs aligned 16UC1 millimetre depth images with the nearest
`/d435_orb/pose` header stamp within 50 ms, using the documented `T_wc` camera
pose in `slam_world`. CameraInfo supplies the aligned image intrinsics. The
first usable floor plane establishes the fixed gravity-aligned map frame; pose
translation jumps above 1 m/s and rotation jumps above 60 degrees/s are skipped.
The output directory contains `height_map.png`, `obstacle_classes.png` and
`summary.json` (including frame counts, map bounds, obstacle centroid and
processing time).

```sh
MPLBACKEND=Agg MPLCONFIGDIR=/private/tmp/wbr-mpl-cache \
  python3 tools/local_map/from_bag.py \
  /path/to/rosbag2-directory --out /path/to/local-map
# Optional expected centroid in gravity-aligned map coordinates (x,y,height):
python3 tools/local_map/from_bag.py \
  /path/to/rosbag2-directory --out /path/to/local-map --expected-box 0.0,1.5,0.05
```

Run the SQLite bag integration test and existing grid tests with:

```sh
MPLBACKEND=Agg MPLCONFIGDIR=/private/tmp/wbr-mpl-cache \
  python3 \
  -m unittest discover -s tools/local_map/tests -v
```

## Limits

This is synthetic-only evidence. The per-frame ORB pose quality on the robot's
camera data has not been validated for elevation mapping. Plane alignment can
fail on a frame without enough visible floor, a non-planar floor, or moving
objects dominating the frame. The height estimator currently fuses surface
returns rather than ray-clearing free space, and its scalar variance ignores
pose covariance and spatial correlation. It is a local mapping prototype, not
a safety-rated obstacle detector or motor command source.
