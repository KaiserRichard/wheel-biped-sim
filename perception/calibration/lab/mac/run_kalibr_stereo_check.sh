#!/usr/bin/env bash
# OPEN: optical validation only; fitted values never replace factory settings.
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
[[ $# -eq 3 ]] || fail 'Usage: run_kalibr_stereo_check.sh BAG TARGET_YAML OUT_DIR'
stage_calibration "$1" "$2" "$3"
cp -p "$LAB_DIR/config/camchain_factory_serial-alias-6.yaml" "$OUT/work/factory-camchain.yaml"
receipt --input "$OUT/work/factory-camchain.yaml" -- stereo-validation-only
run_container 'rosrun kalibr kalibr_calibrate_cameras --bag /data/work/take.bag --target /data/work/target.yaml --models pinhole-radtan pinhole-radtan --topics /cam0/image_raw /cam1/image_raw --dont-show-report'
collect_result cam camchain
python3 "$LAB_DIR/mac/summarize_kalibr.py" stereo --chain "$OUT/take-camchain.yaml" --results "$OUT/take-results-cam.txt" --factory "$OUT/work/factory-camchain.yaml" | tee "$OUT/summary.txt"
receipt --input "$OUT/summary.txt" -- validation-complete
