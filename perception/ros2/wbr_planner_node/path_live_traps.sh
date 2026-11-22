# Shared process cleanup and signal handling for path_live.sh.
source_ros_setups() {
  set +u
  source "$1"
  source "$2"
  source "$3"
  set -u
}

cleanup_done=0
cleanup() {
  (( cleanup_done )) && return 0
  cleanup_done=1
  for pid in "${pids[@]}"; do kill -INT -- "-$pid" 2>/dev/null || true; done
  for pid in "${pids[@]}"; do
    for _ in {1..30}; do
      kill -0 "$pid" 2>/dev/null || break
      sleep 0.2
    done
    kill -TERM -- "-$pid" 2>/dev/null || true
  done
  for pid in "${pids[@]}"; do wait "$pid" 2>/dev/null || true; done
  if [[ -n ${monitor_pid:-} ]]; then
    kill "$monitor_pid" 2>/dev/null || true
    wait "$monitor_pid" 2>/dev/null || true
  fi
  echo "Stopped live path session at $(date -Is)" >> "$RUN_DIR/session.log"
  if [[ ${PATH_LIVE_CLEANUP_TEST_MARKER:-0} == 1 ]]; then echo 'cleanup-run'; fi
}
handle_int() { trap - INT; exit 130; }
handle_term() { trap - TERM; exit 143; }
install_path_traps() {
  trap cleanup EXIT
  trap handle_int INT
  trap handle_term TERM
}
