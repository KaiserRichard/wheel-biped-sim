#!/usr/bin/env bash
# Foreground, explicit topics; Ctrl-C lets rosbag finish its metadata.
set -euo pipefail
if [[ $# != 2 || ( "$1" != stereo && "$1" != rgbd ) ]]; then
  echo "Usage: $0 stereo|rgbd NEW_BAG_DIRECTORY" >&2
  exit 2
fi
if [[ -e "$2" || -L "$2" ]]; then
  echo "Refusing existing bag path: $2" >&2
  exit 2
fi
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
topics=(
  /camera/camera/depth/image_rect_raw /camera/camera/depth/camera_info
  /camera/camera/depth/metadata
  /tf_static /tf /d435_orb/pose /d435_orb/status
)
if [[ "$1" == stereo ]]; then
  topics+=(
    /camera/camera/infra1/image_rect_raw /camera/camera/infra1/camera_info
    /camera/camera/infra1/metadata
    /camera/camera/infra2/image_rect_raw /camera/camera/infra2/camera_info
    /camera/camera/infra2/metadata
  )
else
  topics+=(
    /camera/camera/color/image_raw /camera/camera/color/camera_info
    /camera/camera/color/metadata
    /camera/camera/aligned_depth_to_color/image_raw
    /camera/camera/aligned_depth_to_color/camera_info
  )
fi
exec ros2 bag record --storage sqlite3 --output "$2" \
  --qos-profile-overrides-path "$script_dir/record_qos.yaml" "${topics[@]}"
