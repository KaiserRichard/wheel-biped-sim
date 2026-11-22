#!/usr/bin/env bash
# Run ON the Pi from its private A1 checkout, not on the training Mac.
set -euo pipefail
if [[ $# != 2 ]]; then
  echo "Usage: $0 EXISTING_ORB_SLAM3_ROOT NEW_BUILD_RUN_DIRECTORY" >&2
  exit 2
fi
if [[ "$(uname -s)" != Linux || "$(uname -m)" != aarch64 ]]; then
  echo "This serial build script is for the aarch64 Linux Pi." >&2
  exit 2
fi
task_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
if [[ "$(basename "$task_root")" == wbr-perception-phase1-runtime ]]; then
  echo "Use a dedicated task checkout; refusing shared runtime checkout." >&2
  exit 2
fi
orb_root="$(cd "$1" && pwd)"
mkdir -- "$2"
run_dir="$(cd "$2" && pwd)"
# ROS setup scripts can reference unset variables.
set +u
source /opt/ros/jazzy/setup.bash
set -u
export CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
{
  hostname
  uname -m
  head -5 /proc/meminfo
  df -h /
  dpkg-query -W ros-jazzy-librealsense2 ros-jazzy-realsense2-camera
} 2>&1 | tee "$run_dir/environment.log"
colcon --log-base "$run_dir/log" build \
  --base-paths "$task_root/ros2/wbr_orb_node" --packages-select wbr_orb_node \
  --build-base "$run_dir/build" --install-base "$run_dir/install" \
  --executor sequential --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DWITH_ORB_SLAM3=ON \
  "-DORB_SLAM3_ROOT=$orb_root" \
  "-DORB_EXTRA_INCLUDE_DIRS=/usr/include/eigen3;$orb_root/Thirdparty/Sophus" \
  2>&1 | tee "$run_dir/build.log"
