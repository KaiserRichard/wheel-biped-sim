#!/usr/bin/env python3
"""Bounded D435i raw capture or hardware-free re-analysis. See README.md."""
import argparse
import hashlib
import json
import math
import shutil
import subprocess
import sys
from pathlib import Path

from analyze import analyze
from common import HERE, host, load_config, utc, value, write_json

EXIT = {"PASS": 0, "FAIL": 1, "BLOCKED": 2}


class CheckParser(argparse.ArgumentParser):
    def error(self, message):
        self.print_usage(sys.stderr)
        self.exit(3, f"{self.prog}: error: {message}\n")


def copy_evidence(source, out):
    """Copy only this tool's raw format; never modify the input evidence."""
    if not source.is_dir():
        raise ValueError("Analyze-only directory does not exist")
    for path in source.iterdir():
        if path.name in ("capture.json", "capture.partial.json", "request.json", "samples.csv", "sdk.log", "capture.stdout", "capture.stderr", "timeout.json", "capture_config.yaml", "frames.csv", "command.json", "PROVENANCE.json") or path.suffix == ".pgm":
            if path.is_symlink() or not path.is_file():
                raise ValueError(f"Raw evidence must be a regular file: {path.name}")
            shutil.copyfile(path, out / path.name)


def capture(out, cfg, seconds, static):
    args = [sys.executable, str(HERE / "capture.py"), "--outdir", str(out),
            "--config", str(out / "capture_config.yaml"), "--seconds", str(seconds)]
    if static:
        args.append("--static")
    start = utc()
    write_json(out / "request.json", {"started_utc": start, "host": host(), "argv": args,
                                     "static_confirmed": static, "requested_duration_s": seconds,
                                     "config_sha256": hashlib.sha256((out / "capture_config.yaml").read_bytes()).hexdigest()})
    with (out / "capture.stdout").open("xb") as stdout, (out / "capture.stderr").open("xb") as stderr:
        child = subprocess.Popen(args, stdout=stdout, stderr=stderr)
        timed_out = False
        try:
            returncode = child.wait(timeout=seconds + value(cfg, "capture", "worker_grace_s"))
        except subprocess.TimeoutExpired:
            timed_out = True
            child.kill()
            returncode = child.wait()
        except BaseException:
            child.kill()
            child.wait()
            raise
    meta_path = out / "capture.json"
    if timed_out or not meta_path.exists():
        failure = {"started_utc": start, "ended_utc": utc(), "host": host(), "complete": False,
                   "outcome": "FAIL", "errors": ["SDK child exceeded bounded deadline" if timed_out else f"SDK child exited {returncode} without finalized metadata"],
                   "static_confirmed": static, "queue_overflow": None, "callback_errors": []}
        if timed_out:
            write_json(out / "timeout.json", failure)
        # Preserve any partial metadata in stdout/raw; never claim an orderly capture.
        if meta_path.exists():
            shutil.copyfile(meta_path, out / "capture.partial.json")
        write_json(meta_path, failure)
    elif returncode != EXIT.get(json.loads(meta_path.read_text()).get("outcome")):
        meta = json.loads(meta_path.read_text())
        meta.update(complete=False, outcome="FAIL")
        meta.setdefault("errors", []).append(f"SDK child exit {returncode} disagrees with metadata")
        write_json(meta_path, meta)


def main(argv=None):
    parser = CheckParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--capture", action="store_true", help="Open configured SDK device on the Pi")
    mode.add_argument("--analyze-only", type=Path, metavar="DIR", help="Saved capture from this check; never import/open SDK")
    parser.add_argument("--outdir", type=Path, required=True, help="New output directory; never overwrite")
    parser.add_argument("--config", type=Path, help="JSON-form YAML thresholds; saved capture config is default on replay")
    parser.add_argument("--seconds", type=float, help="Capture duration; default from config")
    parser.add_argument("--static", action="store_true", help="Operator confirms camera resting still for this live capture")
    args = parser.parse_args(argv)
    try:
        if args.analyze_only and (args.seconds is not None or args.static):
            raise ValueError("Replay uses saved duration and static confirmation; --seconds/--static are capture-only")
        source = args.analyze_only.resolve() if args.analyze_only else None
        if source and not source.is_dir():
            raise ValueError("Analyze-only directory does not exist")
        config_path = args.config or (source / "capture_config.yaml" if source and (source / "capture_config.yaml").is_file() else HERE / "thresholds.yaml")
        cfg = load_config(config_path)
        seconds = args.seconds if args.seconds is not None else value(cfg, "capture", "duration_s")
        if not math.isfinite(seconds) or not value(cfg, "capture", "min_duration_s") <= seconds <= value(cfg, "capture", "max_duration_s"):
            raise ValueError("Capture seconds outside configured finite bounds")
        out = args.outdir.resolve()
        if source and (out == source or source in out.parents or out in source.parents):
            raise ValueError("Analysis input/output must be separate, non-nested directories")
        out.mkdir(parents=True, exist_ok=False)
        if source:
            copy_evidence(source, out)
        else:
            write_json(out / "capture_config.yaml", cfg)
        write_json(out / "analysis_config.yaml", cfg)
        if args.capture:
            capture(out, cfg, seconds, args.static)
        result = analyze(out, cfg)
        print(f"{'CRITERION':<32} STATUS")
        for criterion in result["criteria"]:
            print(f"{criterion['id']:<32} {criterion['status']}")
        print(f"d435i_raw: {result['status']} | {out / 'result.json'}")
        return EXIT[result["status"]]
    except (ValueError, OSError, KeyError, TypeError, ArithmeticError) as error:
        print(f"d435i_raw usage/internal error: {error}", file=sys.stderr)
        return 3


if __name__ == "__main__":
    raise SystemExit(main())
