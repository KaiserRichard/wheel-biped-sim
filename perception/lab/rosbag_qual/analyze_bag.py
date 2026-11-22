#!/usr/bin/env python3
"""Analyze a rosbag2 bag offline, without a running ROS graph."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent))

from bag_core import analyze_database, evaluate_report, storage_files
from qual_common import (base_result, criterion, file_record, finalize_result,
                         load_config, sha256, utc, write_json, UsageParser)


def _config_for(output_dir: Path, supplied: Path | None) -> tuple[dict, Path]:
    config_path = Path(supplied) if supplied else output_dir / "config.json"
    if supplied is None and not config_path.is_file():
        config_path = Path(__file__).with_name("config.json")
    config = load_config(config_path)
    destination = output_dir / "config.json"
    if config_path.resolve() != destination.resolve():
        shutil.copy2(config_path, destination)
    return config, destination


def _blocked(output_dir: Path, result: dict, message: str, config: dict | None = None, *, error_code: int | None = None) -> int:
    error_path = output_dir / "error.json"
    write_json(error_path, {"error": message})
    source = (config or {}).get("storage", {}).get("threshold_source", "UNKNOWN")
    criteria = [criterion("offline_input", "Offline bag analysis completed",
                          None, "", message, "BLOCKED", "error.json", source)]
    return finalize_result(result, output_dir, criteria, [file_record(error_path, output_dir)], [message], error_code=error_code)


def analyze(args: argparse.Namespace) -> int:
    output_dir = Path(args.analyze_only or args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    started = utc()
    result = base_result("analyze_bag", started)
    if args.analyze_only and args.config is None and not (output_dir / "config.json").is_file():
        return _blocked(output_dir, result, "analyze-only requires saved config.json or explicit --config")
    try:
        config, config_path = _config_for(output_dir, args.config)
    except Exception as error:
        return _blocked(output_dir, result, f"config: {type(error).__name__}: {error}", error_code=3)

    if args.analyze_only:
        analysis_path = output_dir / "analysis.json"
        if not analysis_path.is_file():
            return _blocked(output_dir, result, "analyze-only requires saved analysis.json", config)
        try:
            report = json.loads(analysis_path.read_text())
            criteria = evaluate_report(report, config, "analysis.json")
        except Exception as error:
            return _blocked(output_dir, result, f"saved analysis: {type(error).__name__}: {error}", config)
        raw = [file_record(analysis_path, output_dir), file_record(config_path, output_dir)]
        return finalize_result(result, output_dir, criteria, raw,
                               ["ANALYZE_ONLY: re-evaluated saved topic and disk measurements; no ROS graph used."])

    if not args.bag:
        return _blocked(output_dir, result, "a bag path is required unless --analyze-only is used", config)
    try:
        report, criteria = analyze_database(Path(args.bag), config)
        report["analyzed_utc"] = utc()
        write_json(output_dir / "analysis.json", report)
        raw = [file_record(output_dir / "analysis.json", output_dir),
               file_record(config_path, output_dir)]
        raw.extend(file_record(item) for item in storage_files(Path(args.bag)))
        notes = ["Header-rate and stamp-order measurements use ROS header.stamp.",
                 "Bag-receive rate and disk demand use rosbag2 messages.timestamp.",
                 "Offline SQLite parsing does not establish physical sensor or ROS graph health."]
        return finalize_result(result, output_dir, criteria, raw, notes)
    except Exception as error:
        return _blocked(output_dir, result, f"bag: {type(error).__name__}: {error}", config)


def main(argv: list[str] | None = None) -> int:
    parser = UsageParser(description=__doc__)
    parser.add_argument("--bag", type=Path, help="rosbag2 directory or .db3/.sqlite3 file")
    parser.add_argument("--output-dir", type=Path, help="new or existing evidence directory")
    parser.add_argument("--config", type=Path)
    parser.add_argument("--analyze-only", type=Path, help="re-evaluate saved analysis.json in this directory")
    args = parser.parse_args(argv)
    if bool(args.analyze_only) == bool(args.output_dir):
        parser.error("provide exactly one of --output-dir or --analyze-only")
    return analyze(args)


if __name__ == "__main__":
    raise SystemExit(main())

