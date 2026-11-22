#!/usr/bin/env bash
# OPEN. Sourced helpers; no Docker command runs merely by sourcing this file.
set -euo pipefail
LAB_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)
FOUNDATION=${WBR_CALIBRATION_FOUNDATION:-./wbr-sw-foundation}
IMAGE=kalibr:noetic-1f60227
KALIBR_SRC=${KALIBR_SRC:-./kalibr-src}
LAB_DIAGNOSTIC=no

fail() { printf 'OPEN: %s\n' "$*" >&2; exit 2; }
require_file() { [[ -f "$1" ]] || fail "Missing file: $1"; }
absolute_file() {
    require_file "$1"
    python3 -c 'import pathlib,sys; print(pathlib.Path(sys.argv[1]).resolve(strict=True))' "$1"
}
new_output() {
    [[ ! -e "$1" && ! -L "$1" ]] || fail "Refusing existing output: $1"
    mkdir -p "$(dirname "$1")"
    mkdir "$1"
    OUT=$(cd "$1" && pwd -P)
    # Docker -v uses colons as separators; reject ambiguous mount paths.
    [[ "$OUT" != *:* && "$OUT" != *$'\n'* && "$OUT" != *,* ]] || fail 'Unsupported mount path punctuation'
    printf 'Status: OPEN\nImage tag: %s (source pin supplied by reviewer: 1f60227)\n' "$IMAGE" > "$OUT/README.txt"
}
receipt() {
    # Bash 3.2 with nounset cannot expand a declared-but-empty array. Keep the
    # argv array populated and append the optional flag only when requested.
    local args=(python3 "$LAB_DIR/mac/lab_files.py" receipt --file "$OUT/README.txt")
    if [[ "$LAB_DIAGNOSTIC" == yes ]]; then args+=(--diagnostic); fi
    args+=("$@")
    "${args[@]}"
}
run_container() {
    local command=$1
    # Pinned Dockerfile uses shell-form ENTRYPOINT and has setup sourcing commented
    # out. Override explicitly; one -c string, with fixed container-only paths.
    local invocation="export KALIBR_MANUAL_FOCAL_LENGTH_INIT=1; source /catkin_ws/devel/setup.bash && cd /data && $command"
    local argv=(docker run --rm --pull=never --network none -v "${OUT}:/data" -e MPLBACKEND=Agg
        --entrypoint /bin/bash "$IMAGE" -c "$invocation")
    local source
    local sources=(--input "$KALIBR_SRC/Dockerfile_ros1_20_04")
    for source in kalibr_bagcreater kalibr_calibrate_imu_camera kalibr_calibrate_cameras \
        kalibr_imu_camera_calibration/IccUtil.py kalibr_camera_calibration/CameraUtils.py; do
        sources+=(--input "$KALIBR_SRC/aslam_offline_calibration/kalibr/python/$source")
    done
    receipt "${sources[@]}" -- "${argv[@]}"
    # Capture the local image identity/architecture, without pulling or building.
    local inspect=(docker image inspect --format '{{.Id}} {{.Os}}/{{.Architecture}} {{json .RepoDigests}}' "$IMAGE")
    receipt -- "${inspect[@]}"
    "${inspect[@]}" > "$OUT/docker-image.txt"
    receipt -- "${argv[@]}"
    # Executed only by the future operator. Offline/no automatic image download.
    "${argv[@]}" 2>&1 | tee "$OUT/kalibr.log"
}
stage_calibration() {
    BAG=$(absolute_file "$1")
    TARGET=$(absolute_file "$2")
    command -v docker >/dev/null || fail 'Docker unavailable; reviewer must supply the image'
    python3 -c 'import yaml,numpy'
    new_output "$3"
    receipt --input "$BAG" --input "$TARGET" --input "$LAB_DIR/mac" -- "$0" "$@"
    mkdir "$OUT/work"
    cp -p "$BAG" "$OUT/work/take.bag"
    cp -p "$TARGET" "$OUT/work/target.yaml"
    # Kalibr outputs follow the staged bag basename, preserving the original.
    python3 -c 'import sys; sys.path.insert(0,sys.argv[1]); from lab_files import sha256; assert sha256(sys.argv[2]) == sha256(sys.argv[3]), "staged bag hash mismatch"' \
        "$LAB_DIR/mac" "$BAG" "$OUT/work/take.bag"
}
collect_result() {
    local suffix=$1
    local chain=$2
    local file
    for file in "take-report-$suffix.pdf" "take-results-$suffix.txt" "take-$chain.yaml"; do
        [[ -s "$OUT/work/$file" ]] || fail "Kalibr did not produce $file; partial evidence retained"
        cp -p "$OUT/work/$file" "$OUT/$file"
    done
    if [[ "$suffix" == imucam ]]; then
        [[ -s "$OUT/work/take-imu.yaml" ]] || fail 'Missing fitted IMU YAML; evidence retained'
        cp -p "$OUT/work/take-imu.yaml" "$OUT/take-imu.yaml"
    fi
    receipt --input "$OUT/work" --input "$OUT/kalibr.log" -- collect-result "$suffix" "$chain"
}
