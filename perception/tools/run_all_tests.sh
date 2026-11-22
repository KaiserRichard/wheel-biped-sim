#!/usr/bin/env bash
set -euo pipefail

PYTHON=${PYTHON:-python3}

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"
export PYTHONDONTWRITEBYTECODE=1

declare -a suite_names=()
declare -a suite_statuses=()
declare -a suite_details=()
failed=0

record_result() {
  suite_names+=("$1")
  suite_statuses+=("$2")
  suite_details+=("$3")
  if [[ "$2" == "FAIL" || "$2" == "BLOCKED" ]]; then
    failed=1
  fi
}

run_python_suite() {
  local suite_dir="$1"
  local suite_name="python:${suite_dir}"
  if "$PYTHON" tools/test_support/run_unittest.py "$suite_dir"; then
    record_result "$suite_name" "PASS" "unittest discovery"
  else
    local status=$?
    if [[ "$status" == 2 ]]; then
      record_result "$suite_name" "BLOCKED" "missing discovery/execution evidence; see output"
    else
      record_result "$suite_name" "FAIL" "unittest exited ${status}"
    fi
  fi
}

run_make_suite() {
  local suite_dir="$1"
  local makefile="${suite_dir}/Makefile"
  if [[ ! -f "$makefile" ]]; then
    record_result "make:${suite_dir}:test" "BLOCKED" "required Makefile is absent"
    return
  fi

  # These five targets were reviewed: each test target exercises host-only C/C++
  # and/or Python code. The ROS bridge target builds only its ROS-free core probe;
  # VIO's default test uses the fake engine and does not enable its optional ORB binding.
  if [[ "$suite_dir" == "tools/vio_adapter" ]]; then
    if make -C "$suite_dir" CXX="${CXX:-c++}" WBR_WITH_ORBSLAM3=0 test; then
      record_result "make:${suite_dir}:test" "PASS" "host fake-engine target"
    else
      local status=$?
      record_result "make:${suite_dir}:test" "FAIL" "make exited ${status}"
    fi
  else
    if make -C "$suite_dir" test; then
      record_result "make:${suite_dir}:test" "PASS" "host target"
    else
      local status=$?
      record_result "make:${suite_dir}:test" "FAIL" "make exited ${status}"
    fi
  fi
}

declare -a python_dirs=()
add_python_dir() {
  local suite_dir="$1"
  local existing
  [[ -d "$suite_dir" ]] || return 0
  for existing in "${python_dirs[@]-}"; do
    [[ "$existing" == "$suite_dir" ]] && return 0
  done
  python_dirs+=("$suite_dir")
}

if [[ "${1:-}" == --core ]]; then
  for suite in tools/calib_tools/tests tools/clockmap/tests tools/frames/tests \
    tools/ground_plane/tests tools/local_map/tests/test_grid.py \
    tools/obstacle_decision/tests/test_obstacle_decision.py \
    tools/ros2_bridge/tests tools/scene_twin/tests tools/sim_scenarios/tests \
    tools/trajectory_eval tools/wbr_pose/tests ros2/wbr_obstacle_node/test ros2/wbr_planner_node/test; do
    run_python_suite "$suite"
  done
elif (( $# )); then
  echo "Usage: $0 [--core]" >&2
  exit 2
else
# Required baseline inventory from origin/main at the phase-8 review. A removed or
# renamed suite must update this inventory deliberately, rather than disappear
# from a successful glob. New suites below are still discovered generically.
for suite_dir in tools/calib_tools/tests tools/clockmap/tests tools/frames/tests \
  tools/link_bench/tests tools/link_proto/tests tools/ros2_bridge/tests \
  tools/run_harness/tests tools/runtime_observer/tests tools/take_tools/tests \
  tools/vio_adapter/tests tools/trajectory_eval calibration/allan/tests \
  calibration/lab/tests tools/test_support/tests; do
  if [[ -d "$suite_dir" ]]; then
    add_python_dir "$suite_dir"
  else
    record_result "python:${suite_dir}" "BLOCKED" "required suite directory is absent"
  fi
done

for suite_dir in tools/*/tests; do
  [[ -d "$suite_dir" ]] && add_python_dir "$suite_dir"
done
for suite_dir in ros2/*/tests ros2/*/test; do
  [[ -d "$suite_dir" ]] && add_python_dir "$suite_dir"
done
add_python_dir tools/trajectory_eval
for suite_dir in calibration/*/tests lab/*/tests tools/eval_pipeline/tests tools/eval_pipeline/*/tests tools/perception_health/tests tools/perception_health/integration scripts/tests scripts/camera_day/tests scripts/camera_day/native scripts/camera_day/offline lab/tests_session; do
  [[ -d "$suite_dir" ]] && add_python_dir "$suite_dir"
done
add_python_dir tests/fuzz

# Simulation infrastructure only; learner exercises intentionally remain separate.
for suite_dir in sim/*/tests; do
  [[ -d "$suite_dir" ]] && add_python_dir "$suite_dir"
done

for suite_dir in tools/link_proto tools/link_bench tools/runtime_observer tools/vio_adapter tools/ros2_bridge; do
  run_make_suite "$suite_dir"
done

# Optional host-only Makefile suites added after the phase-8 baseline.
for suite_dir in tools/perception_health/c/mcu tests/fuzz; do
  [[ -d "$suite_dir" && -f "$suite_dir/Makefile" ]] || continue
  run_make_suite "$suite_dir"
done

# ROS packages with host-only Makefile tests are discoverable without making
# the root runner depend on a ROS installation.  A package's own target keeps
# its test command authoritative, just as the explicitly listed tool suites do.
for suite_dir in ros2/*; do
  [[ -d "$suite_dir" && -f "$suite_dir/Makefile" ]] || continue
  run_make_suite "$suite_dir"
done

for suite_dir in "${python_dirs[@]-}"; do
  [[ -n "$suite_dir" ]] || continue
  run_python_suite "$suite_dir"
done
fi

printf '\n%-48s %-8s %s\n' "SUITE" "STATUS" "DETAIL"
printf '%-48s %-8s %s\n' "------------------------------------------------" "--------" "------------------------------"
for i in "${!suite_names[@]}"; do
  printf '%-48s %-8s %s\n' "${suite_names[$i]}" "${suite_statuses[$i]}" "${suite_details[$i]}"
done

if (( failed )); then
  exit 1
fi
