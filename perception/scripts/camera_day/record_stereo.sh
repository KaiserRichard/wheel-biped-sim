#!/usr/bin/env bash
# Stereo-only capture; publishers must offer reliable image QoS.
set -euo pipefail
if [[ $# != 1 ]]; then
  echo "Usage: $0 NEW_BAG_DIRECTORY" >&2
  exit 2
fi
if [[ -e "$1" || -L "$1" ]]; then
  echo "Refusing existing bag path: $1" >&2
  exit 2
fi
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec ros2 bag record --storage mcap --output "$1" \
  --qos-profile-overrides-path "$script_dir/stereo_record_qos.yaml" \
  --topics /camera/camera/infra1/image_rect_raw /camera/camera/infra2/image_rect_raw \
  /camera/camera/infra1/camera_info /camera/camera/infra2/camera_info
