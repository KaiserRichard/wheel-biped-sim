#!/usr/bin/env bash
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo="$(cd "$here/../.." && pwd)"
outdir="${TMPDIR:-/tmp}"
args=()
while (($#)); do
  case "$1" in
    --allow-dirty) args+=(--allow-dirty); shift ;;
    --outdir) (($# >= 2)) || { echo "--outdir needs a path" >&2; exit 3; }; outdir="$2"; shift 2 ;;
    *) echo "usage: make_bundle.sh [--allow-dirty] [--outdir DIR]" >&2; exit 3 ;;
  esac
done
exec python3 "$here/deploy.py" make --repo "$repo" --outdir "$outdir" "${args[@]}"
