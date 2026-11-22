#!/usr/bin/env bash
# OPEN: run later on the Mac. No hardware acquisition.
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
[[ $# -eq 2 || ( $# -eq 3 && "$3" == --diagnostic ) ]] || fail 'Usage: take_to_kalibr_bag.sh TAKE_DIR OUT_DIR [--diagnostic]'
[[ -d "$1" ]] || fail 'TAKE_DIR must be a finalized recorder directory'
TAKE=$(cd "$1" && pwd -P)
INPUT=$(absolute_file "$TAKE/stereo_imu.db3")
TOOL="$FOUNDATION/tools/take_tools/take_tools.py"
require_file "$TOOL"
command -v docker >/dev/null || fail 'Docker unavailable; reviewer must supply the image'
new_output "$2"
EXPORT=(python3 "$TOOL" export-euroc "$INPUT" "$OUT/euroc")
if [[ $# -eq 3 ]]; then
    printf '\n*** WARNING: DIAGNOSTIC EXPORT OVERRIDE; REJECTED DATA MAY BE INCLUDED; ALL GATES OPEN ***\n\n' >&2
    EXPORT+=(--allow-diagnostic)
    LAB_DIAGNOSTIC=yes
fi
receipt --input "$TAKE" --input "$TOOL" --input "$LAB_DIR/mac" -- "${EXPORT[@]}"
[[ ! -e "$OUT/euroc" && ! -L "$OUT/euroc" ]] || fail 'Refusing existing EuRoC directory'
"${EXPORT[@]}" 2>&1 | tee "$OUT/export.log"
receipt --input "$OUT/euroc/manifest.json" -- python3 "$LAB_DIR/mac/lab_files.py" prepare "$OUT/euroc" "$OUT/bagcreater"
python3 "$LAB_DIR/mac/lab_files.py" prepare "$OUT/euroc" "$OUT/bagcreater"
run_container 'rosrun kalibr kalibr_bagcreater --folder /data/bagcreater --output-bag /data/take.bag'
[[ -s "$OUT/take.bag" ]] || fail 'Missing/empty ROS1 bag; evidence retained'
receipt --input "$OUT/bagcreater" --input "$OUT/take.bag" -- conversion-complete
cat >> "$OUT/README.txt" <<'EOF'
Topics (SOURCE-SUPPORTED, pinned bagcreater): /cam0/image_raw /cam1/image_raw /imu0.
Layout: bagcreater/cam{0,1}/<integer-ns>.png hard-linked to euroc/mav0/cam{0,1}/data.
imu0.csv: one skipped header, ns,w_x,w_y,w_z,a_x,a_y,a_z; rad/s and m/s^2.
Original SDK epoch retained; no time shift, resampling, or additional interpolation.
Physical accuracy / ROS1 header-count-pixel round trip: OPEN, manual verification required.
Hard-linked PNGs are immutable evidence; editing either path changes both.
EOF
printf 'OPEN: bag prepared at %s; physical/header round-trip validation remains OPEN.\n' "$OUT/take.bag"
