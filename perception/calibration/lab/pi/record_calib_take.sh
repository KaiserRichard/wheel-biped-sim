#!/usr/bin/env bash
# OPEN: future Pi-local capture only. Do not execute before CAMERA AVAILABLE.
set -euo pipefail
fail() { printf 'OPEN: %s\n' "$*" >&2; exit 2; }
[[ $# -eq 2 ]] || fail 'Usage: record_calib_take.sh NAME SECONDS'
[[ "$1" =~ ^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$ ]] || fail 'NAME must be 1..64 safe alphanumeric/underscore/hyphen characters'
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)
REQUIRED=$(python3 "$HERE/take_diagnostics.py" ram-required "$2") || exit 2
[[ "${WBR_CAMERA_AVAILABLE:-}" == 'CAMERA AVAILABLE' ]] || fail 'Set WBR_CAMERA_AVAILABLE="CAMERA AVAILABLE" only after explicit owner authorization and prerequisite review'
[[ -r /proc/device-tree/model ]] || fail 'This script runs only on the Raspberry Pi'
[[ "$(tr -d '\0' < /proc/device-tree/model)" == *'Raspberry Pi'* ]] || fail 'This script runs only on the Raspberry Pi'
BUNDLE="$HOME/wbr-runtime/n4b-20261013T080913Z"
RECORDER="$BUNDLE/bin/d435i_record"
ANALYZER="$BUNDLE/bin/analyze_take.py"
[[ -x "$RECORDER" && -f "$ANALYZER" && -f "$HERE/take_diagnostics.py" ]] || fail 'Missing recorder/analyzer/diagnostic helper'
for tool in vcgencmd journalctl sha256sum cp sync df find sort xargs du tail tr tee; do
    command -v "$tool" >/dev/null || fail "Missing $tool"
done
R="$HOME/wbr-evidence/2026-10-15-calib-$1"
S="/dev/shm/wbr-calib-$1"
[[ ! -e "$R" && ! -L "$R" && ! -e "$S" && ! -L "$S" ]] || fail 'Evidence or RAM directory already exists; use a fresh NAME'
FREE=$(df -B1 --output=avail /dev/shm | tail -n 1 | tr -d ' ')
[[ "$FREE" =~ ^[0-9]+$ ]] || fail 'Could not read /dev/shm free bytes'
[[ "$FREE" -ge "$REQUIRED" ]] || fail "/dev/shm free=$FREE bytes < required=$REQUIRED (18.4 MB/s * seconds * 1.2 * 1.5)"
mkdir -p "$(dirname "$R")"
# ASSUMED 2 GiB OS reserve plus twice the overhead-adjusted take budget.
# This checks capacity only; sustained-write/storage qualification is still OPEN.
DISK_REQUIRED=$(python3 -c 'import math,sys; print(math.ceil(2*18_400_000*float(sys.argv[1])*1.2)+2*1024**3)' "$2")
DISK_FREE=$(df -B1 --output=avail "$(dirname "$R")" | tail -n 1 | tr -d ' ')
[[ "$DISK_FREE" =~ ^[0-9]+$ && "$DISK_FREE" -ge "$DISK_REQUIRED" ]] || fail "Evidence storage free=$DISK_FREE bytes < required=$DISK_REQUIRED"
mkdir "$R"
printf 'OPEN: name=%s seconds=%s RAM_free=%s RAM_required=%s\n' "$1" "$2" "$FREE" "$REQUIRED" > "$R/record.log"
printf 'Profile: IR1+IR2 Y8 640x480@30; gyro 200; accel 100; emitter OFF; recorder enforces SDK 99005.\n' >> "$R/record.log"
printf 'storage_free=%s storage_required=%s (ASSUMED OS reserve 2 GiB)\n' "$DISK_FREE" "$DISK_REQUIRED" >> "$R/record.log"
sha256sum "$RECORDER" "$ANALYZER" "$0" "$HERE/take_diagnostics.py" > "$R/tools-sha256.txt"
P='' K='' C='' INTERRUPTED=0
stop_logs() {
    if [[ -n "$P" ]]; then kill "$P" 2>/dev/null || true; wait "$P" 2>/dev/null || true; P=''; fi
    if [[ -n "$K" ]]; then kill "$K" 2>/dev/null || true; wait "$K" 2>/dev/null || true; K=''; fi
}
cleanup() {
    # Never remove RAM in a trap. The normal checksum path owns deletion.
    if [[ -n "$C" ]]; then kill -TERM "$C" 2>/dev/null || true; wait "$C" 2>/dev/null || true; fi
    stop_logs
}
interrupt() { INTERRUPTED=1; [[ -z "$C" ]] || kill -TERM "$C" 2>/dev/null || true; }
trap cleanup EXIT
trap interrupt INT TERM
sample_power() {
    printf '%s phase=%s ' "$(date -u +%FT%TZ)" "$1"
    local command
    for command in 'get_throttled' 'pmic_read_adc EXT5V_V' 'measure_temp' 'measure_clock arm'; do
        # These words are fixed, without caller-controlled shell expansion.
        if ! vcgencmd $command; then printf 'POWER_COMMAND_FAILED: %s\n' "$command"; fi
    done
}
sample_power before > "$R/power-1hz.log" 2>&1
(while true; do sample_power during; sleep 1; done) >> "$R/power-1hz.log" 2>&1 & P=$!
journalctl -k -f -n0 -o short-iso-precise > "$R/kernel.log" 2>&1 & K=$!
sleep 1
kill -0 "$K" 2>/dev/null || fail 'Kernel logger exited before recording; see kernel.log'
[[ "$INTERRUPTED" -eq 0 ]] || fail 'Interrupted before capture; logs retained'
printf 'take-start=%s\n' "$(date -u +%FT%TZ)" >> "$R/record.log"
"$RECORDER" "$S" "$2" >> "$R/record.log" 2>&1 & C=$!
RC=0
wait "$C" || RC=$?
if [[ "$INTERRUPTED" -ne 0 ]]; then
    # Interrupted wait may return before recorder finalizes its database.
    wait "$C" 2>/dev/null || true
    RC=130
fi
C=''
printf 'take-end=%s rc=%s interrupted=%s\n' "$(date -u +%FT%TZ)" "$RC" "$INTERRUPTED" >> "$R/record.log"
LOGGER_OK=yes
if ! kill -0 "$P" 2>/dev/null || ! kill -0 "$K" 2>/dev/null; then
    LOGGER_OK=no
    printf 'WARN: power/kernel logger exited during recording\n' >> "$R/record.log"
fi
stop_logs
sample_power after >> "$R/power-1hz.log" 2>&1
if [[ "$LOGGER_OK" != yes ]]; then printf 'POWER_COMMAND_FAILED: incomplete logging\n' >> "$R/power-1hz.log"; fi
ARC=0
python3 "$ANALYZER" "$S/timestamps.csv" > "$R/analysis.json" 2> "$R/analysis.stderr.log" || ARC=$?
VERIFIED=no
if [[ -d "$S" ]]; then
    # Hash all recorder files before copying; no glob omission of hidden files.
    (cd "$S" && find . -type f -print0 | sort -z | xargs -0 -r sha256sum) > "$R/ram-sha256.txt"
    [[ -s "$R/ram-sha256.txt" ]] || fail 'RAM take has no files; retained for inspection'
    phase-0=$(date +%s)
    cp -a "$S" "$R/take"
    sync
    printf 'copy_to_card_s=%s\n' "$(( $(date +%s) - phase-0 ))" >> "$R/record.log"
    if (cd "$R/take" && sha256sum -c ../ram-sha256.txt) > "$R/copy-verify.log" 2>&1; then
        VERIFIED=yes
        # S is constructed from a validated NAME, never caller-supplied as a path.
        rm -rf -- "$S"
    else
        printf 'WARN: checksum mismatch; RAM preserved at %s\n' "$S" >&2
    fi
    du -B1 "$R/take" > "$R/sizes.txt"
else
    printf 'WARN: recorder created no RAM take\n' >&2
fi
python3 "$HERE/take_diagnostics.py" summary "$R/analysis.json" "$R/power-1hz.log" "$RC" "$ARC" "$VERIFIED" | tee "$R/summary.txt"
