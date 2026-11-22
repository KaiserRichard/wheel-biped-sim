#!/usr/bin/env bash
# Back up the whole F446 flash (read twice, must match), then write and verify an image.
# Usage: flash.sh <image.bin> <expected-sha256> <archive-dir>
# Uses tools/link_bench/pi/stm32boot.py (copied next to this script on the Pi).
# Stop the micro-ROS agent first. When it says "waiting for bootloader": FIRST fit BOOT0
# (CN7 pin 5 to 7), THEN press RESET. A running bench image prints the letter "y" (0x79),
# which the sync loop would take for a bootloader ACK.
# The firmware's "BOOTLOADER" jump is not used: on 29 Oct the jump happened but the ROM
# bootloader never answered on USART1 (see README).
set -euo pipefail
img=$1; want=$2; out=$3; port=${PORT:-/dev/ttyAMA0}
here=$(cd "$(dirname "$0")" && pwd)
got=$(sha256sum "$img" | cut -d' ' -f1)
[ "$got" = "$want" ] || { echo "REFUSE: $img sha256 $got != $want"; exit 2; }
if fuser "$port" >/dev/null 2>&1; then echo "REFUSE: $port is in use (stop the micro-ROS agent first)"; fuser -v "$port"; exit 2; fi
mkdir -p "$out"
python3 "$here/stm32boot.py" --port "$port" --out "$out" --flash "$img" --sync-seconds "${SYNC_SECONDS:-900}" 2>&1 | tee "$out/flash.log"
echo "image sha256 $got" | tee -a "$out/flash.log"
echo "Remove BOOT0, then press RESET."
