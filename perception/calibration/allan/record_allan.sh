#!/bin/bash
# OPEN: future Pi-local capture only; build nothing. Usage: record_allan.sh HOURS NAME
set -euo pipefail
if [ "$#" -ne 2 ]; then
    echo "Usage: record_allan.sh HOURS NAME" >&2
    exit 2
fi
SCRIPT_DIR=$(cd -- "$(dirname -- "$0")" && pwd)
HOURS=$1
CAMERA_SERIAL=${CAMERA_SERIAL:?Set CAMERA_SERIAL to the intended camera serial or ANY}
NAME=$2
if [[ ! "$NAME" =~ ^[A-Za-z0-9][A-Za-z0-9_-]{0,79}$ ]]; then
    echo "NAME must be 1..80 letters/digits/underscore/hyphen and start with a letter/digit" >&2
    exit 2
fi
RECORDER=${ALLAN_RECORDER:-"$SCRIPT_DIR/d435i_imu_record"}
[ -x "$RECORDER" ] || { echo "Prebuilt recorder missing: $RECORDER" >&2; exit 1; }
command -v vcgencmd >/dev/null || { echo "vcgencmd missing; Pi power evidence unavailable" >&2; exit 1; }
# A conservative text row budget: <=24 chars each SDK stamp/XYZ, <=20 each
# domain/frame/host, seven separators/newline = <=163 B; round upward to 192.
# 300 samples/s * 3600 * 192 = 207,360,000 CSV bytes/hour, before small logs.
BUDGET=$(python3 - "$HOURS" <<'PY'
import math, sys
hours = float(sys.argv[1])
if not math.isfinite(hours) or not 0.01 <= hours <= 30:
    raise SystemExit("HOURS must be finite in 0.01..30")
csv_hour = 300 * 3600 * 192
print(math.ceil(hours * (csv_hour + 1024 * 1024)))
print(f"CALCULATED budget: 300 rows/s × 192 B/row × 3600 = {csv_hour:,} CSV bytes/hour "
      f"({csv_hour/1e6:.2f} MB/h); +1 MiB/h small-log allowance; "
      f"{hours:g} h projected {math.ceil(hours*(csv_hour+1024*1024)):,} bytes.", file=sys.stderr)
PY
)
EVIDENCE="$HOME/wbr-evidence"
mkdir -p -- "$EVIDENCE"
RUN_DIR="$EVIDENCE/allan-$NAME"
# Refuse all pre-existing run paths, even empty ones, to prevent ownership races.
[ ! -e "$RUN_DIR" ] && [ ! -L "$RUN_DIR" ] || { echo "Run path exists: $RUN_DIR" >&2; exit 1; }
FREE=$(df -Pk "$EVIDENCE" | awk 'END {print $4 * 1024}')
python3 - "$FREE" "$BUDGET" <<'PY'
import sys
free, budget = map(float, sys.argv[1:])
print(f"Free {free:,.0f} bytes; required 3× projection = {3*budget:,.0f} bytes.", flush=True)
if free < 3*budget:
    raise SystemExit("Insufficient free space; capture refused")
PY
mkdir -- "$RUN_DIR"
# Logs remain in a private sibling until stop so recorder sees an empty run dir.
CONTROL=$(mktemp -d "$EVIDENCE/.allan-$NAME-control.XXXXXXXX")
REC_PID=""
POWER_PID=""
REQUESTED_STOP=0
on_signal() {
    REQUESTED_STOP=1
    if [ -n "$REC_PID" ]; then kill -TERM "$REC_PID" 2>/dev/null || true; fi
}
cleanup() {
    local original_rc=$?
    trap - EXIT INT TERM
    if [ -n "$REC_PID" ]; then
        kill -TERM "$REC_PID" 2>/dev/null || true
        wait "$REC_PID" 2>/dev/null || true
    fi
    if [ -n "$POWER_PID" ]; then
        kill -TERM "$POWER_PID" 2>/dev/null || true
        wait "$POWER_PID" 2>/dev/null || true
    fi
    for file in "$CONTROL"/*; do
        [ -f "$file" ] && mv -n -- "$file" "$RUN_DIR/" || true
    done
    rmdir -- "$CONTROL" 2>/dev/null || true
    exit "$original_rc"
}
trap on_signal INT TERM
trap cleanup EXIT
sha256sum "$RECORDER" "$SCRIPT_DIR/record_allan.sh" "$SCRIPT_DIR/power_monitor.py" > "$CONTROL/tool-sha256.txt"
python3 - "$HOURS" "$NAME" "$RECORDER" > "$CONTROL/capture_request.json" <<'PY'
import datetime, json, platform, sys
print(json.dumps({"status": "OPEN", "hours": float(sys.argv[1]), "name": sys.argv[2],
                  "recorder": sys.argv[3], "host": platform.node(), "os": platform.platform(),
                  "start_utc": datetime.datetime.now(datetime.timezone.utc).isoformat()}))
PY
echo "OPEN: Pi-local IMU-only run $RUN_DIR. Use nohup or tmux to survive SSH disconnect."
python3 "$SCRIPT_DIR/power_monitor.py" "$CONTROL/power-10s.jsonl" "$CONTROL/power_summary.json" "$CONTROL/power.ready" &
POWER_PID=$!
# Wait for the actual before-run baseline; a missing monitor is a failure.
for ((i=0; i<200; i++)); do
    [ -f "$CONTROL/power.ready" ] && break
    kill -0 "$POWER_PID" 2>/dev/null || { echo "Power monitor failed" >&2; exit 1; }
    sleep 0.1
done
[ -f "$CONTROL/power.ready" ] || { echo "Power baseline timed out" >&2; exit 1; }
[ "$REQUESTED_STOP" -eq 0 ] || { echo "Stopped before recorder start" >&2; exit 130; }
"$RECORDER" "$RUN_DIR" "$HOURS" --serial "$CAMERA_SERIAL" > "$CONTROL/record.log" 2>&1 &
REC_PID=$!
if [ "$REQUESTED_STOP" -ne 0 ]; then on_signal; fi
set +e
wait "$REC_PID"
RC=$?
# A trapped signal may interrupt wait before the recorder has drained its queue.
while kill -0 "$REC_PID" 2>/dev/null; do wait "$REC_PID"; RC=$?; done
REC_PID=""
set -e
echo "recorder_exit=$RC" >> "$CONTROL/record.log"
kill -TERM "$POWER_PID" 2>/dev/null || true
set +e
wait "$POWER_PID"
POWER_RC=$?
set -e
POWER_PID=""
if [ "$POWER_RC" -ne 0 ] || [ ! -f "$CONTROL/power_summary.json" ]; then
    echo "UNKNOWN: power monitor failed; preserve raw logs" >&2
    [ "$RC" -ne 0 ] || RC=1
else
    python3 - "$CONTROL/power_summary.json" <<'PY'
import json, sys
p = json.load(open(sys.argv[1]))
print("Under-voltage during run:", "YES" if p["undervoltage_occurred"] else
      ("NO OBSERVED" if p["monitoring_complete"] else "UNKNOWN"))
print(p["label"])
if p["initial_sticky_undervoltage"]:
    print("Initial sticky bit already set: historical event; a new sticky-only event cannot be distinguished.")
PY
fi
echo "OPEN: recorder exit=$RC; evidence $RUN_DIR"
exit "$RC"
