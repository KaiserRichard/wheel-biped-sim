#!/usr/bin/env bash
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
outdir=""
dest=""; dry=0; rollback=0; local_root=""; host=""; allow_dirty=0
while (($#)); do
  case "$1" in
    --dest) (($# >= 2)) || { echo "--dest needs a path" >&2; exit 3; }; dest="$2"; shift 2 ;;
    --dry-run) dry=1; shift ;;
    --allow-dirty) allow_dirty=1; shift ;;
    --rollback) rollback=1; shift ;;
    --local-root) (($# >= 2)) || { echo "--local-root needs a path" >&2; exit 3; }; local_root="$2"; shift 2 ;;
    --outdir) (($# >= 2)) || { echo "--outdir needs a path" >&2; exit 3; }; outdir="$2"; shift 2 ;;
    *) if [[ -z "$host" ]]; then host="$1"; shift; else echo "unexpected argument: $1" >&2; exit 3; fi ;;
  esac
done
if ((rollback)); then
  if [[ -n "$dest" && -n "$local_root" ]]; then echo "--dest cannot be used with local rollback" >&2; exit 3; fi
  if [[ -n "$local_root" ]]; then exec python3 "$here/deploy.py" local-rollback "$local_root"; fi
  [[ -n "$host" ]] || { echo "usage: deploy.sh <ssh-host> [--rollback] [--dest PATH]" >&2; exit 3; }
  [[ -n "$dest" ]] || dest="\$HOME/wbr-tools/current"
  cmd="set -euo pipefail; base=\$(dirname \"$dest\"); test -L \"\$base/current\" && test -L \"\$base/previous\"; a=\$(readlink \"\$base/current\"); b=\$(readlink \"\$base/previous\"); test -d \"\$base/\$b\"; ln -s \"\$b\" \"\$base/.current-new\"; mv -Tf \"\$base/.current-new\" \"\$base/current\"; ln -s \"\$a\" \"\$base/.previous-new\"; mv -Tf \"\$base/.previous-new\" \"\$base/previous\"; echo \"\$base/\$b\""
  if ((dry)); then printf 'ssh %q %q\n' "$host" "$cmd"; exit 0; fi
  exec ssh "$host" "$cmd"
fi
[[ -n "$host" || -n "$local_root" ]] || { echo "usage: deploy.sh <ssh-host> [--dest PATH] [--dry-run]" >&2; exit 3; }
if [[ -z "$outdir" ]]; then outdir="$(mktemp -d "${TMPDIR:-/tmp}/wbr-deploy.XXXXXX")"; trap 'rm -rf "$outdir"' EXIT; fi
bundle_args=(--outdir "$outdir")
((allow_dirty)) && bundle_args+=(--allow-dirty)
bundle="$("$here/make_bundle.sh" "${bundle_args[@]}")"
commit="$(python3 -c 'import json,sys,tarfile; t=tarfile.open(sys.argv[1]); m=json.load(t.extractfile("MANIFEST.json")); print(m["git_commit"])' "$bundle")"
[[ -n "$dest" ]] || dest="\$HOME/wbr-tools/$commit"
if [[ -n "$local_root" ]]; then
  tmp="$local_root/.incoming-$commit"
  [[ ! -e "$tmp" && ! -e "$local_root/$commit" ]] || { echo "release already exists" >&2; exit 1; }
  mkdir -p "$tmp"
  python3 - "$bundle" "$tmp" <<'PY'
import sys, tarfile
from pathlib import Path
archive, target = Path(sys.argv[1]), Path(sys.argv[2]).resolve()
with tarfile.open(archive, "r:gz") as t:
    for member in t.getmembers():
        p = (target / member.name).resolve()
        if target not in p.parents and p != target: raise SystemExit("unsafe archive path")
    t.extractall(target)
PY
  python3 "$here/deploy.py" verify "$tmp"
  python3 "$here/deploy.py" local-deploy "$tmp" "$local_root"
  python3 -c 'import shutil,sys; shutil.rmtree(sys.argv[1])' "$tmp"
  exit 0
fi
incoming="\$HOME/wbr-tools-bundle-$commit.tar.gz"
remote="set -euo pipefail; dest=\"$dest\"; incoming=\"$incoming\"; base=\$(dirname \"\$dest\"); test ! -e \"\$dest\"; mkdir -p \"\$base\"; mkdir \"\$dest\"; mv \"\$incoming\" \"\$dest.tar.gz\"; tar -xzf \"\$dest.tar.gz\" -C \"\$dest\"; python3 \"\$dest/tools/deploy/deploy.py\" verify \"\$dest\"; if test -L \"\$base/current\"; then old=\$(readlink \"\$base/current\"); ln -s \"\$old\" \"\$base/.previous-new\"; mv -Tf \"\$base/.previous-new\" \"\$base/previous\"; fi; ln -s \$(basename \"\$dest\") \"\$base/.current-new\"; mv -Tf \"\$base/.current-new\" \"\$base/current\"; rm -f \"\$dest.tar.gz\""
scp_dest="~/wbr-tools-bundle-$commit.tar.gz"
if ((dry)); then printf 'scp %q %q\n' "$bundle" "$host:$scp_dest"; printf 'ssh %q %q\n' "$host" "$remote"; exit 0; fi
scp "$bundle" "$host:$scp_dest"
exec ssh "$host" "$remote"
