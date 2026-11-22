#!/usr/bin/env bash
# Record D435i data on a borrowed ROS 2 computer (e.g. the lab robot's Pi)
# without touching its workspace, services or shell configuration.
#
# Usage:  source /opt/ros/$DISTRO/setup.bash
#         bash borrowed_pi_capture.sh stereo|rgbd RUN_NAME SECONDS
#
# Writes only under ~/wbr-capture-20261024/RUN_NAME. Uses its own ROS domain
# (WBR_DOMAIN_ID, default 87) so it cannot talk to the host robot's nodes.
# UNVERIFIED on the borrowed machine; parameter names follow realsense-ros 4.x.
set -eo pipefail

if [[ $# != 3 || ( "$1" != stereo && "$1" != rgbd ) ]]; then
  echo "Usage: $0 stereo|rgbd RUN_NAME SECONDS" >&2; exit 2
fi
mode=$1; name=$2; secs=$3
: "${ROS_DISTRO:?source /opt/ros/<distro>/setup.bash first}"
export ROS_DOMAIN_ID=${WBR_DOMAIN_ID:-87}
root="$HOME/wbr-capture-20261024"
out="$root/$name"
test ! -e "$out" || { echo "$out exists; choose a new RUN_NAME" >&2; exit 1; }
mkdir -p "$out"

{
  date -Is; uname -a; lsb_release -ds 2>/dev/null || true
  echo "ROS_DISTRO=$ROS_DISTRO ROS_DOMAIN_ID=$ROS_DOMAIN_ID mode=$mode seconds=$secs"
  dpkg -l | grep -E 'realsense|rosbag2-storage-mcap' || true
  rs-enumerate-devices -s 2>/dev/null || echo "rs-enumerate-devices not available"
  lsusb -t; df -h "$root"; vcgencmd get_throttled 2>/dev/null || true
} > "$out/env.txt" 2>&1

common=(-p enable_gyro:=false -p enable_accel:=false -p enable_sync:=true)
if [[ $mode == stereo ]]; then
  # Same as our ORB-SLAM3 stereo captures: IR pair, 640x480 at 15 Hz, emitter off.
  params=(-p enable_infra1:=true -p enable_infra2:=true -p enable_color:=false
          -p enable_depth:=false -p depth_module.emitter_enabled:=0
          -p depth_module.infra_profile:=640x480x15)
  topics=(/camera/camera/infra1/image_rect_raw /camera/camera/infra1/camera_info
          /camera/camera/infra2/image_rect_raw /camera/camera/infra2/camera_info)
else
  # Box / local-map captures: colour + depth + aligned depth at 15 Hz, emitter on.
  params=(-p enable_color:=true -p enable_depth:=true -p enable_infra1:=false
          -p enable_infra2:=false -p align_depth.enable:=true
          -p depth_module.emitter_enabled:=1
          -p depth_module.depth_profile:=640x480x15 -p rgb_camera.color_profile:=640x480x15)
  topics=(/camera/camera/color/image_raw /camera/camera/color/camera_info
          /camera/camera/depth/image_rect_raw /camera/camera/depth/camera_info
          /camera/camera/aligned_depth_to_color/image_raw
          /camera/camera/aligned_depth_to_color/camera_info)
fi

ros2 run realsense2_camera realsense2_camera_node --ros-args \
  -r __ns:=/camera -r __node:=camera "${common[@]}" "${params[@]}" \
  > "$out/camera.log" 2>&1 &
cam=$!
trap 'kill -INT $cam 2>/dev/null || true; wait $cam 2>/dev/null || true' EXIT
sleep 8
kill -0 $cam || { echo "camera node exited; see $out/camera.log" >&2; exit 1; }
ros2 param dump /camera/camera > "$out/camera-parameters.yaml" 2>&1 || true
ros2 topic list > "$out/topics.txt"

storage=sqlite3
ros2 bag record --help 2>/dev/null | grep -q mcap && dpkg -l | grep -q rosbag2-storage-mcap && storage=mcap
echo "recording ${secs}s, storage=$storage" | tee -a "$out/env.txt"
timeout -s INT "$secs" ros2 bag record -s "$storage" -o "$out/bag" "${topics[@]}" \
  > "$out/record.log" 2>&1 || true
sleep 2
ros2 bag info "$out/bag" > "$out/bag-info.txt" 2>&1 || true
(cd "$out" && find bag -type f -exec sha256sum {} + > SHA256SUMS)
cat "$out/bag-info.txt"
echo "done: $out"
