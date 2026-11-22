"""Print the phase-5 lab checklist and validate a completed evidence folder."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import sys
from typing import Iterable


ROOT_FILES = (
    "preflight/run-root.txt",
    "preflight/pi-status.txt",
    "preflight/camera.txt",
    "preflight/nucleo-image.txt",
    "preflight/flash-backup.sha256",
    "preflight/clocks.txt",
    "closeout/run-sheet.csv",
    "SHA256SUMS",
)

LIVE_CAPTURE_FILES = (
    "notes.md",
    "measurements.csv",
    "setup.jpg",
    "live/summary.json",
    "live/packets.csv",
    "live/camera.log",
    "live/record.log",
    "live/camera-bag/metadata.yaml",
    "live/camera-bag/camera-bag_0.mcap",
    "orb/slam.log",
    "orb/status.txt",
    "orb/pose-first.txt",
    "orb/record.log",
    "orb/pose-bag/metadata.yaml",
    "orb/pose-bag/pose-bag_0.mcap",
)


@dataclass(frozen=True)
class Scenario:
    label: str
    simulation: str
    files: tuple[str, ...]


SCENARIOS: dict[str, Scenario] = {
    "normal_approach": Scenario(
        "Clear-scene hand-carried approach", "lab_empty", LIVE_CAPTURE_FILES),
    "stop_box_0132": Scenario(
        "132 mm STOP box", "lab_box132 / lab_approach_box132", LIVE_CAPTURE_FILES),
    "jump_box_0046_or_0080": Scenario(
        "46 mm or 80 mm low JUMP box", "lab_books46 / step080", LIVE_CAPTURE_FILES),
    "jump_gate_injected": Scenario(
        "Synthetic jump phase over carried-camera advisories",
        "jump_box110 (optionally range_entry_jump)",
        (*LIVE_CAPTURE_FILES, "analysis/jump-gate.csv")),
    "link_loss": Scenario(
        "Advisory UART loss and recovery",
        "none in tools/sim_scenarios",
        (*LIVE_CAPTURE_FILES, "link-loss-video.mov")),
    "imu_sway": Scenario(
        "Measured imu_sensor sway and current gate replay",
        "lab_approach_box132 via tools/simcam_imu",
        (
            "notes.md",
            "measurements.csv",
            "setup.jpg",
            "camera/session-bag/metadata.yaml",
            "camera/session-bag/session-bag_0.mcap",
            "camera/slam.log",
            "camera/status.txt",
            "camera/pose-first.txt",
            "imu_view_run.csv",
            "analysis/tilt-source.json",
            "analysis/timeline.csv",
            "analysis/timeline_zero_tilt.csv",
            "analysis/timeline_comparison.csv",
            "analysis/timeline_metadata.json",
            "analysis/wobble-fix-comparison.md",
        ),
    ),
    "can_loss": Scenario(
        "CANH open/recovery on the isolated bench loop",
        "none in tools/sim_scenarios",
        (
            "notes.md",
            "measurements.csv",
            "setup.jpg",
            "image.txt",
            "baseline/raw.log",
            "baseline/summary.json",
            "baseline/checks.txt",
            "canh-open/raw.log",
            "canh-open/summary.json",
            "canh-open/checks.txt",
            "recovery/raw.log",
            "recovery/summary.json",
            "recovery/checks.txt",
            "counter-comparison.md",
        ),
    ),
}


def required_files() -> tuple[str, ...]:
    """Return all required paths once, preserving the printed checklist order."""
    paths: list[str] = list(ROOT_FILES)
    for scenario_id, scenario in SCENARIOS.items():
        paths.extend(
            f"scenarios/{scenario_id}/{relative}"
            for relative in scenario.files
        )
    return tuple(dict.fromkeys(paths))


def missing_artifacts(output_root: Path) -> list[str]:
    """List missing or empty expected files, relative to ``output_root``."""
    missing = []
    for relative in required_files():
        candidate = output_root / relative
        if not candidate.is_file() or candidate.stat().st_size == 0:
            missing.append(relative)
    return missing


def print_checklist(stream=sys.stdout) -> None:
    """Print the preflight and scenario checklist for an operator."""
    print("phase-5 D435i live bench twin checklist", file=stream)
    print("Preflight: Pi free and reachable; D435i enumerated; Nucleo image identified;", file=stream)
    print("           current flash backup hashed; clocks and measurements recorded.", file=stream)
    print("Run:      keep the rig hand-carried and motors untouched; capture RGB-D + ORB pose.", file=stream)
    print("Scenarios:", file=stream)
    for scenario_id, scenario in SCENARIOS.items():
        print(f"  [ ] {scenario_id}: {scenario.label} (sim: {scenario.simulation})", file=stream)
    print("Closeout: copy raw bags, logs, photos, analysis and run sheet; verify SHA-256.", file=stream)
    print("Validation: python3 tools/bench_twin_lab/checklist.py --validate OUTPUT_ROOT", file=stream)


def validate_output(output_root: Path, stream=sys.stdout) -> bool:
    """Print a validation report and return whether all expected files exist."""
    missing = missing_artifacts(output_root)
    if missing:
        print(f"FAIL: {len(missing)} expected files missing or empty under {output_root}", file=stream)
        for relative in missing:
            print(f"  - {relative}", file=stream)
        return False
    print(f"PASS: all expected files are present and nonempty under {output_root}", file=stream)
    return True


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--validate", type=Path, metavar="OUTPUT_ROOT",
                        help="validate a completed lab output folder")
    args = parser.parse_args(argv)
    print_checklist()
    if args.validate is None:
        return 0
    return 0 if validate_output(args.validate) else 1


if __name__ == "__main__":
    raise SystemExit(main())
