#!/usr/bin/env bash
set -euo pipefail

usage() { echo "Usage: $0 BINARY VOCABULARY SETTINGS DATASET TIMES SEQUENCE RUNS OUTDIR" >&2; exit 2; }
[[ $# == 8 ]] || usage
BIN=$(realpath "$1"); VOCAB=$(realpath "$2"); YAML=$(realpath "$3"); DATA=$(realpath "$4"); TIMES=$(realpath "$5")
SEQ=$6; RUNS=$7; OUT=$(realpath -m "$8")
[[ $RUNS =~ ^[1-9][0-9]*$ ]] || usage
[[ -x $BIN && -f $VOCAB && -f $YAML && -d $DATA && -f $TIMES ]] || { echo "missing/non-executable input" >&2; exit 2; }
mkdir -p "$OUT"
LOCK_FILE="${TMPDIR:-/tmp}/wbr-stereo-inertial-euroc.lock"
exec 9>"$LOCK_FILE"
flock -n 9 || { echo "another EuRoC pipeline run owns $LOCK_FILE" >&2; exit 2; }
for ((i=1; i<=RUNS; i++)); do
  RUN=$(printf '%s/run%02d' "$OUT" "$i")
  [[ ! -e $RUN ]] || { echo "run output already exists: $RUN" >&2; exit 2; }
done
ORB_ROOT=${ORB_ROOT:-$(dirname "$(dirname "$(dirname "$BIN")")")}
export LD_LIBRARY_PATH="$ORB_ROOT/lib:$ORB_ROOT/Thirdparty/DBoW2/lib:$ORB_ROOT/Thirdparty/g2o/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
for input in "$BIN" "$ORB_ROOT/lib/libORB_SLAM3.so" "$VOCAB" "$YAML" "$TIMES"; do
  [[ -f $input ]] || { echo "missing provenance input: $input" >&2; exit 2; }
  sha256sum "$input"
done > "$OUT/input.sha256"
ldd "$BIN" > "$OUT/ldd.txt" 2>&1 || { echo "ldd failed" >&2; exit 2; }
if grep -q 'not found' "$OUT/ldd.txt"; then echo "unresolved shared library; see $OUT/ldd.txt" >&2; exit 2; fi
echo '{"sequence":'"$(printf '%s' "$SEQ" | python3 -c 'import json,sys; print(json.dumps(sys.stdin.read()))')"',"runs":'"$RUNS"'}' > "$OUT/manifest.json"
for ((i=1; i<=RUNS; i++)); do
  RUN=$(printf '%s/run%02d' "$OUT" "$i"); mkdir "$RUN"
  PREFIX="${SEQ}_run${i}"
  printf '%s\n' "$(date -u +%FT%TZ)" > "$RUN/start_utc.txt"
  (vcgencmd get_throttled 2>&1 || true) > "$RUN/throttled_before.txt"
  printf '%q ' "$BIN" "$VOCAB" "$YAML" "$DATA" "$TIMES" "$PREFIX" > "$RUN/command.txt"; printf '\n' >> "$RUN/command.txt"
  set +e
  (cd "$RUN" && "$BIN" "$VOCAB" "$YAML" "$DATA" "$TIMES" "$PREFIX") > "$RUN/stdout.log" 2> "$RUN/stderr.log"
  rc=$?
  set -e
  printf '%s\n' "$rc" > "$RUN/exit_code.txt"
  (vcgencmd get_throttled 2>&1 || true) > "$RUN/throttled_after.txt"
  printf '%s\n' "$(date -u +%FT%TZ)" > "$RUN/end_utc.txt"
  cp "$OUT/ldd.txt" "$RUN/ldd.txt"
  cp "$OUT/input.sha256" "$RUN/input.sha256"
  find "$RUN" -maxdepth 1 -type f ! -name sha256sums.txt -print0 | sort -z | xargs -0 sha256sum > "$RUN/sha256sums.txt"
done
echo "Completed $RUNS $SEQ run(s) in $OUT. No accuracy evaluation was performed."
