#!/usr/bin/env bash
# OPEN: fixed factory camera chain; default temporal calibration stays enabled.
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
[[ $# -eq 3 || $# -eq 4 ]] || fail 'Usage: run_kalibr_camimu.sh BAG TARGET_YAML OUT_DIR [IMU_YAML]'
IMU=$(absolute_file "${4:-$LAB_DIR/config/imu_d435i_provisional.yaml}")
CONVERTER="$FOUNDATION/tools/calib_tools/calib_convert.py"
require_file "$CONVERTER"
stage_calibration "$1" "$2" "$3"
receipt --input "$IMU" --input "$LAB_DIR/config/camchain_factory_serial-alias-6.yaml" --input "$CONVERTER" -- "$0" "$@"
cp -p "$IMU" "$OUT/work/imu.yaml"
cp -p "$LAB_DIR/config/camchain_factory_serial-alias-6.yaml" "$OUT/work/factory-camchain.yaml"
run_container 'rosrun kalibr kalibr_calibrate_imu_camera --bag /data/work/take.bag --cams /data/work/factory-camchain.yaml --imu /data/work/imu.yaml --target /data/work/target.yaml --imu-models calibrated --dont-show-report'
collect_result imucam camchain-imucam
CONVERT=(python3 "$CONVERTER" --camchain "$OUT/take-camchain-imucam.yaml" --imu "$OUT/work/imu.yaml" --output-dir "$OUT/orbslam3" --target orb --fps 30 --units-si)
receipt -- "${CONVERT[@]}"
"${CONVERT[@]}" 2>&1 | tee "$OUT/convert.log"
python3 "$LAB_DIR/mac/summarize_kalibr.py" camimu --chain "$OUT/take-camchain-imucam.yaml" --results "$OUT/take-results-imucam.txt" | tee "$OUT/summary.txt"
receipt --input "$OUT/orbslam3" --input "$OUT/summary.txt" -- calibration-complete
