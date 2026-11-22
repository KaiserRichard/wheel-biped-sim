#!/usr/bin/env bash
# One command to put the WBR AGV micro-ROS image back (agv_f446_home_canfix01, source d163063).
# Stop the micro-ROS agent first. Set IMAGE=... SHA=... to restore a different image, e.g. the
# pre-fix agv_f446_home_loopback.bin (fcc293593a9ece26a306fb21581cc70a9508ca59eb204f84a54995db83c2517e).
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
IMAGE=${IMAGE:-$HOME/course-stage/fw/wbr-agv_f446_home_canfix01.bin}
SHA=${SHA:-f614cde00541a15f80873a72981618dfc65e3f923a1d9309e1fb732831ecf296}
exec "$here/flash.sh" "$IMAGE" "$SHA" "$HOME/wbr-evidence/imu-can-bench-20261029/restore-$(date +%H%M%S)"
