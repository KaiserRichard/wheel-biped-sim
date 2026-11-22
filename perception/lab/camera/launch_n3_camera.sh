#!/usr/bin/env bash
# Profile n3_640x480_30 (owner decision 2026-10-19); UNTESTED ON HARDWARE.
set -euo pipefail
[[ $# == 1 ]] || { echo "Usage: $0 NEW_EVIDENCE_DIR" >&2; exit 2; }
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
# mkdir without -p refuses existing paths, including symlinks; parent must exist.
mkdir -- "$1"
OUT="$(cd "$1" && pwd)"
cp "$HERE/../d435i_raw/profiles/n3_640x480_30.yaml" "$OUT/profile.yaml"
python3 "$HERE/n3_params.py" --profile "$OUT/profile.yaml" --arguments > "$OUT/parameters.txt"
command=(ros2 run realsense2_camera realsense2_camera_node --ros-args -r __ns:=/camera -r __node:=camera)
while IFS= read -r parameter; do command+=(-p "$parameter"); done < "$OUT/parameters.txt"
printf '%q ' "${command[@]}" > "$OUT/command.txt"
printf '\nROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1\nUNTESTED ON HARDWARE\n' >> "$OUT/command.txt"
# Foreground only. pipefail preserves driver/tee failures; Ctrl-C stops the pipeline.
"${command[@]}" 2>&1 | tee "$OUT/camera.log"
