# `wbr_obstacle_node`

ROS 2 Jazzy Python node that publishes depth-based ground/obstacle advisories. It
is informational only: it has no serial transport, motor topics, or actuation
authority. The actual detection and temporal policy are imported from
`tools/obstacle_decision` in the repository, so `algo_path` must name the
repository root on the Pi.

## Topics

Subscriptions default to aligned RealSense RGB-D topics:

- `/camera/camera/aligned_depth_to_color/image_raw` (`sensor_msgs/Image`)
- `/camera/camera/color/camera_info` (`sensor_msgs/CameraInfo`)

Private outputs (`/wbr_obstacle_node/...` when launched with the default name):

- `~/advisory` (`std_msgs/String`): JSON with advisory, confidence, distance,
  obstacle height/width, plane status/height/pitch, frame stamp and processing ms.
- `~/advisory_bytes` (`std_msgs/UInt8MultiArray`): proposed
  `TerrainAdvisory` codec frame. Informational only; firmware review is required
  before any firmware use.
- `/terrain_advisory` (`std_msgs/Float64MultiArray`, Best Effort depth 1): fixed
  eight-value advisory for the STM32. The topic name is configurable with the
  `advisory_topic` parameter; see `docs/interfaces/TERRAIN_ADVISORY_TOPIC.md`
  for field order and receiver freshness rules.
- `~/markers` (`visualization_msgs/MarkerArray`): ground patch, obstacle cube
  and height text, plus corridor lines in the depth message's optical frame.
- `~/obstacle_points` (`sensor_msgs/PointCloud2`): optionally published,
  downsampled valid depth points in the source optical frame.

## Build and run on Pi (UNVERIFIED)

These commands are for Ubuntu 24.04 with ROS 2 Jazzy and the repository at
`~/wbr-lab-ops`. No Pi build or live run has been performed for this package.

```bash
source /opt/ros/jazzy/setup.bash
cd ~/wbr-lab-ops
colcon build --packages-select wbr_obstacle_node
source install/setup.bash
ros2 launch wbr_obstacle_node obstacle.launch.py \
  algo_path:=<local workspace>/wbr-lab-ops
```

The launch file selects the default aligned depth and color camera-info topics.
For direct launch instead:

```bash
ros2 run wbr_obstacle_node obstacle_node --ros-args \
  -p algo_path:=<local workspace>/wbr-lab-ops \
  -p depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
  -p info_topic:=/camera/camera/color/camera_info
```

Foxglove: add `/wbr_obstacle_node/markers` as a Marker Array panel,
`/wbr_obstacle_node/obstacle_points` as a Point Cloud panel, and
`/wbr_obstacle_node/advisory` plus `/wbr_obstacle_node/advisory_bytes` as raw
topic panels. Set the visualization fixed frame to the incoming depth optical
frame (often `camera_color_optical_frame`).

## Parameters

`stride`, `min_obstacle_height`, `corridor_half_width`, `min_forward`,
`max_forward`, `cell_size`, `min_cells`, `plane_max_age`, `plane_gate_deg`,
`jump_min_h`, `jump_max_h`, `confirm_n`, `confirm_m`, `ema_alpha`, `near_m`,
`hysteresis_frames`, `max_rate_hz`, `publish_obstacle_points`, `point_stride`,
and `corridor_length_m` map directly to detector, decision, rate, and output
settings. `advisory_topic` selects the STM32 advisory output topic. Frames
arriving before the rate interval expires are dropped in the subscription
callback, with no queue or backlog.

## Mac core checks

With the repository's perception virtual environment:

```bash
python3 -m unittest discover \
  -s ros2/wbr_obstacle_node/test -v
python3 -m py_compile $(find ros2/wbr_obstacle_node -name '*.py' -print)
```

The core tests do not import ROS. ROS packaging, Pi build/run, image transport
timing, and Foxglove visualization remain unverified.
