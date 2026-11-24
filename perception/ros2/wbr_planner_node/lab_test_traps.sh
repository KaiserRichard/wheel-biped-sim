#!/usr/bin/env bash
set -Eeuo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
for i in 1 2 3; do printf ': "${AMENT_TRACE_SETUP_FILES}"\n' >"$tmp/setup$i.bash"; done
PATH_LIVE_CLEANUP_TEST_MARKER=1 bash -c '
set -Eeuo pipefail
source "$1/ros2/wbr_planner_node/path_live_traps.sh"
source_ros_setups "$2/setup1.bash" "$2/setup2.bash" "$2/setup3.bash"
[[ $- == *u* ]]
' _ "$ROOT" "$tmp"
bash -c '
set -Eeuo pipefail
RUN_DIR=$1
pids=()
monitor_pid=""
source "$2/ros2/wbr_planner_node/path_live_traps.sh"
install_path_traps
start_job() { setsid sleep 30 & pids+=("$!"); }
start_job
kill -INT "$$"
sleep 0.1
' _ "$tmp" "$ROOT" >"$tmp/out" 2>&1 && exit 1
status=$?
[[ $status -eq 130 ]]
[[ $(grep -c '^cleanup-run$' "$tmp/out") -eq 1 ]]
echo 'INT exits 130 and EXIT cleanup runs once'
