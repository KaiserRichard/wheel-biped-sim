#!/usr/bin/env python3
"""Stdlib helpers for reproducible WBR Pi deployment."""
import argparse, hashlib, json, os, shutil, subprocess, sys, tarfile, tempfile
from pathlib import Path
from datetime import datetime, timezone
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
POLICY = json.loads((HERE / "policy.json").read_text())
VERSION = "1.0.0"

def sha(path):
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""): h.update(block)
    return h.hexdigest()

def files_in_bundle(repo):
    git_root = Path(subprocess.check_output(
        ["git", "-C", str(repo), "rev-parse", "--show-toplevel"], text=True).strip())
    paths = subprocess.check_output(["git", "-C", str(git_root), "ls-files", "--cached", "--others", "--exclude-standard", "-z"]).decode().split("\0")
    prefix = repo.resolve().relative_to(git_root.resolve()).as_posix()
    if prefix != ".":
        prefix += "/"
        paths = [p.removeprefix(prefix) for p in paths if p.startswith(prefix)]
    roots, excludes = tuple(POLICY["bundle_roots"]), tuple(POLICY["exclude_paths"])
    selected = [p for p in paths if p and p.startswith(roots) and not p.startswith(excludes)]
    selected.extend(p for p in POLICY["support_files"] if p in paths)
    return sorted(set(p for p in selected if (repo / p).is_file()))

def make_bundle(repo, outdir, allow_dirty=False):
    repo, outdir = repo.resolve(), outdir.resolve()
    status = subprocess.check_output(["git", "-C", str(repo), "status", "--porcelain", "--untracked-files=all"], text=True)
    if status.strip() and not allow_dirty: raise ValueError("working tree is dirty; use --allow-dirty to include the current files")
    commit = subprocess.check_output(["git", "-C", str(repo), "rev-parse", "HEAD"], text=True).strip()
    outdir.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="wbr-bundle-") as td:
        stage = Path(td) / "bundle"; stage.mkdir(); records = []
        for rel in files_in_bundle(repo):
            src, dst = repo / rel, stage / rel; dst.parent.mkdir(parents=True, exist_ok=True); shutil.copy2(src, dst)
            records.append({"path": rel, "sha256": sha(dst)})
        built = datetime.now(timezone.utc)
        built_utc = built.isoformat().replace("+00:00", "Z")
        manifest = {"format": 1, "tool_version": VERSION, "git_commit": commit,
                    "built_utc": built_utc,
                    "dirty_checkout": bool(status.strip()), "files": records}
        (stage / "MANIFEST.json").write_text(json.dumps(manifest, indent=2) + "\n")
        archive = outdir / f"wbr-tools-{commit}-{built.strftime('%Y%m%dT%H%M%SZ')}.tar.gz"
        if archive.exists(): raise FileExistsError(f"refusing to overwrite {archive}")
        with tarfile.open(archive, "w:gz") as tar:
            for path in sorted(stage.rglob("*")): tar.add(path, arcname=path.relative_to(stage).as_posix(), recursive=False)
    return archive

def verify_tree(root):
    root = Path(root).resolve(); manifest_path = root / "MANIFEST.json"
    try:
        manifest = json.loads(manifest_path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        return False, [f"manifest missing or unreadable: {exc}"], {}
    if not isinstance(manifest, dict) or not isinstance(manifest.get("files"), list):
        return False, ["manifest must be an object with a files list"], manifest if isinstance(manifest, dict) else {}
    seen, errors = set(), []
    for item in manifest.get("files", []):
        if not isinstance(item, dict):
            errors.append("invalid manifest file entry: expected object"); continue
        rel = item.get("path", "")
        if not isinstance(rel, str) or not rel or rel.startswith("/") or ".." in Path(rel).parts or rel in seen:
            errors.append(f"invalid or duplicate path: {rel}"); continue
        seen.add(rel); path = root / rel
        if not path.is_file(): errors.append(f"missing: {rel}")
        elif not isinstance(item.get("sha256"), str) or sha(path) != item.get("sha256"): errors.append(f"sha256 mismatch: {rel}")
    actual = {p.relative_to(root).as_posix() for p in root.rglob("*") if p.is_file() and p.name != "MANIFEST.json"}
    errors.extend(f"unlisted file: {rel}" for rel in sorted(actual - seen))
    return not errors, errors, manifest

def local_deploy(bundle, root):
    ok, errors, manifest = verify_tree(bundle)
    if not ok: raise ValueError("bundle verification failed: " + "; ".join(errors))
    root = Path(root).resolve(); root.mkdir(parents=True, exist_ok=True); dest = root / manifest["git_commit"]
    if dest.exists(): raise FileExistsError(f"refusing to overwrite existing release {dest}")
    shutil.copytree(bundle, dest); ok, errors, _ = verify_tree(dest)
    if not ok: shutil.rmtree(dest); raise ValueError("deployed copy verification failed: " + "; ".join(errors))
    cur, prev = root / "current", root / "previous"; old = os.readlink(cur) if cur.is_symlink() else None
    tmp = root / ".current-new"
    if tmp.exists() or tmp.is_symlink(): tmp.unlink()
    tmp.symlink_to(dest.name); tmp.replace(cur)
    if old:
        ptmp = root / ".previous-new"
        if ptmp.exists() or ptmp.is_symlink(): ptmp.unlink()
        ptmp.symlink_to(old); ptmp.replace(prev)
    return dest

def local_rollback(root):
    root = Path(root).resolve(); cur, prev = root / "current", root / "previous"
    if not cur.is_symlink() or not prev.is_symlink(): raise ValueError("rollback requires current and previous release symlinks")
    ct, pt = os.readlink(cur), os.readlink(prev)
    if not (root / pt).is_dir(): raise ValueError("previous release is missing")
    for link, target, tmp in ((cur, pt, root / ".current-new"), (prev, ct, root / ".previous-new")):
        if tmp.exists() or tmp.is_symlink(): tmp.unlink()
        tmp.symlink_to(target); tmp.replace(link)
    return root / pt

def main():
    p = argparse.ArgumentParser(); sub = p.add_subparsers(dest="cmd", required=True)
    b = sub.add_parser("make"); b.add_argument("--repo", type=Path, default=ROOT); b.add_argument("--outdir", type=Path, required=True); b.add_argument("--allow-dirty", action="store_true")
    v = sub.add_parser("verify"); v.add_argument("directory", type=Path)
    d = sub.add_parser("local-deploy"); d.add_argument("bundle", type=Path); d.add_argument("root", type=Path)
    r = sub.add_parser("local-rollback"); r.add_argument("root", type=Path)
    a = p.parse_args()
    try:
        if a.cmd == "make": print(make_bundle(a.repo, a.outdir, a.allow_dirty))
        elif a.cmd == "verify":
            ok, errors, manifest = verify_tree(a.directory); print(json.dumps({"valid": ok, "git_commit": manifest.get("git_commit"), "errors": errors}, indent=2)); return 0 if ok else 1
        elif a.cmd == "local-deploy": print(local_deploy(a.bundle, a.root))
        else: print(local_rollback(a.root))
    except Exception as exc: print(f"deploy: {exc}", file=sys.stderr); return 1
    return 0
if __name__ == "__main__": raise SystemExit(main())
