#!/usr/bin/env bash
set -euo pipefail
# Pi-side offline run. Does not open a camera. --allow-unpatched is an explicit diagnostic override.
ALLOW_UNPATCHED=0
if [[ ${1:-} == --allow-unpatched ]]; then ALLOW_UNPATCHED=1; shift; fi
if [[ $# -ne 5 ]]; then echo "usage: $0 [--allow-unpatched] ORB_ROOT DATA_DIR TIMES_FILE SETTINGS_YAML OUTDIR" >&2; exit 2; fi
ORB_ROOT=$1 DATA_DIR=$2 TIMES_FILE=$3 SETTINGS_YAML=$4 OUTDIR=$5
BIN="$ORB_ROOT/Examples/Stereo-Inertial/stereo_inertial_euroc"
VOCAB="$ORB_ROOT/Vocabulary/ORBvoc.txt"
ORB_LIB="$ORB_ROOT/lib/libORB_SLAM3.so"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
MANIFEST="$SCRIPT_DIR/patched_build.json"
[[ -x "$BIN" && -f "$VOCAB" && -d "$DATA_DIR" && -f "$TIMES_FILE" && -f "$SETTINGS_YAML" ]] || { echo 'required binary/input is missing' >&2; exit 2; }
EXPECTED=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("sha256") or "")' "$MANIFEST")
ACTUAL=$(sha256sum "$BIN" | awk '{print $1}')
if [[ -z "$EXPECTED" || "$ACTUAL" != "$EXPECTED" ]]; then
  if [[ $ALLOW_UNPATCHED -ne 1 ]]; then echo "refusing binary sha256=$ACTUAL; recorded patched sha256=${EXPECTED:-UNKNOWN}; supply a verified manifest or --allow-unpatched" >&2; exit 2; fi
  echo "WARNING: explicit --allow-unpatched; binary sha256=$ACTUAL expected=${EXPECTED:-UNKNOWN}" >&2
fi
[[ ! -e "$OUTDIR" ]] || { echo "output exists: $OUTDIR" >&2; exit 2; }
mkdir -p "$OUTDIR"
cd "$OUTDIR"
export LD_LIBRARY_PATH="$ORB_ROOT/lib:$ORB_ROOT/Thirdparty/DBoW2/lib:$ORB_ROOT/Thirdparty/g2o/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
START=$(date -u +%FT%TZ)
printf '%s\n' "$START" > started_utc.txt
HASH_INPUTS=("$BIN" "$VOCAB" "$TIMES_FILE" "$SETTINGS_YAML" "$0" "$MANIFEST")
if [[ -f "$ORB_LIB" ]]; then HASH_INPUTS+=("$ORB_LIB"); fi
sha256sum "${HASH_INPUTS[@]}" > provenance.sha256
ldd "$BIN" > ldd.txt
if grep -q 'not found' ldd.txt; then echo 'unresolved dynamic library' >&2; exit 2; fi
PROFILE="$SCRIPT_DIR/../eval_pipeline/profile"
PROFILE_PID=''
CMD=("$BIN" "$VOCAB" "$SETTINGS_YAML" "$DATA_DIR" "$TIMES_FILE" n4b)
printf '%q ' /usr/bin/time -v "${CMD[@]}" > command.txt; printf '\n' >> command.txt
if [[ -x "$PROFILE" ]]; then sha256sum "$PROFILE" >> provenance.sha256; fi
/usr/bin/time -v "${CMD[@]}" > run.log 2>&1 & RUN_PID=$!
if [[ -x "$PROFILE" ]]; then
  "$PROFILE" "$RUN_PID" > "$OUTDIR/profile.log" 2>&1 & PROFILE_PID=$!
fi
if wait "$RUN_PID"; then RC=0; else RC=$?; fi
if [[ -n "$PROFILE_PID" ]]; then kill "$PROFILE_PID" 2>/dev/null || true; wait "$PROFILE_PID" 2>/dev/null || true; fi
printf '%s\n' "$RC" > exit-status.txt
date -u +%FT%TZ > ended_utc.txt
printf 'offline run rc=%s; see %s/run.log\n' "$RC" "$OUTDIR"
exit "$RC"
