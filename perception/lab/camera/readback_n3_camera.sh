#!/usr/bin/env bash
# Profile n3_640x480_30 (owner decision 2026-10-19); UNTESTED ON HARDWARE.
set -euo pipefail
[[ $# == 1 ]] || { echo "Usage: $0 NEW_EVIDENCE_DIR" >&2; exit 2; }
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
mkdir -- "$1"
OUT="$(cd "$1" && pwd)"
cp "$HERE/../d435i_raw/profiles/n3_640x480_30.yaml" "$OUT/profile.yaml"
cp "$HERE/../rosbag_qual/profiles/n3_640x480_30.json" "$OUT/bag_profile.json"
printf 'ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1\nUNTESTED ON HARDWARE\n' > "$OUT/context.txt"
python3 - "$OUT/bag_profile.json" > "$OUT/topics.txt" <<'PY'
import json, sys
for topic in json.load(open(sys.argv[1]))['topics']:
    print(topic['name'])
PY
failed=0
ros2 node list > "$OUT/node_list.txt" 2> "$OUT/node_list.stderr" || failed=1
ros2 topic list -t > "$OUT/topic_list.txt" 2> "$OUT/topic_list.stderr" || failed=1
ros2 param dump /camera/camera > "$OUT/parameter_dump.yaml" 2> "$OUT/parameter_dump.stderr" || failed=1
index=0
while IFS= read -r topic; do
  ros2 topic info -v "$topic" > "$OUT/topic_info_${index}.txt" 2> "$OUT/topic_info_${index}.stderr" || failed=1
  index=$((index + 1))
done < "$OUT/topics.txt"
python3 "$HERE/n3_params.py" --profile "$OUT/profile.yaml" --dump "$OUT/parameter_dump.yaml" > "$OUT/comparison.txt" 2>&1 || failed=1
cat "$OUT/comparison.txt"
exit "$failed"
