#!/usr/bin/env python3
"""Run the lab qualification checks in their documented dependency order."""

from __future__ import annotations

import argparse
import json
import platform
import shlex
import socket
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
EXIT_STATUS = {0: "PASS", 1: "FAIL", 2: "BLOCKED", 3: "ERROR"}


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def host_facts() -> dict[str, str]:
    return {"hostname": socket.gethostname(), "system": platform.system(),
            "release": platform.release(), "machine": platform.machine(),
            "python": platform.python_version()}


def git_commit(root: Path) -> str | None:
    try:
        proc = subprocess.run(["git", "rev-parse", "HEAD"], cwd=root, text=True,
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
        return proc.stdout.strip() if proc.returncode == 0 else None
    except OSError:
        return None


def ask(prompt: str, assume_yes: bool) -> bool:
    if assume_yes:
        return True
    try:
        answer = input(prompt + " [y/N] ").strip().lower()
    except EOFError:
        return False
    return answer in ("y", "yes")


def build_plan(args: argparse.Namespace, session: Path, root: Path) -> list[dict[str, Any]]:
    py = sys.executable
    check_root = Path(args.checks_dir).expanduser().resolve() if args.checks_dir else root / "lab"
    power_out = session / "01_power_usb"
    raw_out = session / "02_d435i_raw"
    record_out = session / "03_rosbag_record"
    replay_out = session / "04_rosbag_replay"
    calib_out = session / "05_calib_record"
    steps: list[dict[str, Any]] = [
        {"id": "power_usb", "name": "Power and USB", "directory": power_out,
         "command": [py, str(check_root / "power_usb/check_power_usb.py"), "--out", str(power_out)]},
        {"id": "d435i_raw", "name": "D435i raw streams", "directory": raw_out,
         "confirmation": "Place the camera on a still, rigid surface for the full raw capture.",
         "command": [py, str(check_root / "d435i_raw/check.py"), "--capture", "--static", "--outdir", str(raw_out)]},
        {"id": "rosbag_record", "name": "ROS record", "directory": record_out,
         "confirmation": "Start the configured RealSense ROS camera node in ROS domain 42 and ensure it is streaming.",
         "command": ["env", "ROS_DOMAIN_ID=42", "ROS_LOCALHOST_ONLY=1", py,
                     str(check_root / "rosbag_qual/record_check.py"), "--config",
                     str(check_root / "rosbag_qual/config.json"), "--output-dir", str(record_out)]},
        {"id": "rosbag_replay", "name": "ROS replay", "directory": replay_out,
         "confirmation": "Stop the live camera node before replay qualification.",
         "command": ["env", "ROS_DOMAIN_ID=42", "ROS_LOCALHOST_ONLY=1", py,
                     str(check_root / "rosbag_qual/replay_check.py"), "--config",
                     str(check_root / "rosbag_qual/config.json"), "--bag", str(record_out / "bag"),
                     "--output-dir", str(replay_out), "--live-domain", "42", "--replay-domain", "43"]},
        {"id": "calib_record", "name": "Calibration take", "directory": calib_out,
         "command": [py, str(check_root / "calib_record/check_calib_take.py"), "--take-dir",
                     str(Path(args.calib_take).expanduser().resolve()), "--kind", args.calib_kind, "--outdir", str(calib_out)]},
    ]
    if args.power_thresholds:
        steps[0]["command"] += ["--thresholds", str(Path(args.power_thresholds).expanduser().resolve())]
    if args.raw_config:
        steps[1]["command"] += ["--config", str(Path(args.raw_config).expanduser().resolve())]
    if args.ros_config:
        for index in (2, 3):
            command = steps[index]["command"]
            command[command.index(str(check_root / "rosbag_qual/config.json"))] = str(Path(args.ros_config).expanduser().resolve())
    if args.calib_config:
        steps[4]["command"] += ["--config", str(Path(args.calib_config).expanduser().resolve())]
    return steps


def read_result(directory: Path) -> tuple[dict[str, Any] | None, str | None]:
    path = directory / "result.json"
    if not path.is_file():
        return None, None
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return None, f"invalid result.json: {exc}"
    if not isinstance(value, dict):
        return None, "invalid result.json: top level must be an object"
    if value.get("status") not in ("PASS", "FAIL", "BLOCKED"):
        return None, "invalid result.json: missing or invalid check status"
    criteria = value.get("criteria")
    if not isinstance(criteria, list):
        return None, "invalid result.json: criteria must be an array"
    for index, criterion in enumerate(criteria):
        if not isinstance(criterion, dict):
            return None, f"invalid result.json: criterion {index} must be an object"
        if criterion.get("status") not in ("PASS", "FAIL", "BLOCKED"):
            return None, f"invalid result.json: criterion {index} has invalid status"
        evidence = criterion.get("evidence")
        evidence_path = Path(evidence) if isinstance(evidence, str) else Path("/")
        if not evidence or evidence_path.is_absolute() or ".." in evidence_path.parts:
            return None, f"invalid result.json: criterion {index} has invalid evidence path"
        if not (directory / evidence_path).is_file():
            return None, f"invalid result.json: criterion {index} evidence file is missing: {evidence}"
    return value, None


def aggregate_criteria(step: dict[str, Any], result: dict[str, Any] | None,
                       session: Path) -> list[dict[str, Any]]:
    if not result:
        return []
    entries = []
    for criterion in result.get("criteria", []):
        item = dict(criterion)
        item["step"] = step["id"]
        evidence = item.get("evidence")
        if evidence:
            item["evidence_path"] = str((step["directory"] / evidence).relative_to(session))
        entries.append(item)
    return entries


def render_markdown(session_data: dict[str, Any]) -> str:
    lines = ["# Lab session", "", f"- Status: **{session_data['status']}**",
             f"- Started (UTC): `{session_data['started_utc']}`",
             f"- Ended (UTC): `{session_data['ended_utc']}`",
             f"- Host: `{session_data['host'].get('hostname')}` ({session_data['host'].get('system')} / {session_data['host'].get('machine')})",
             f"- Repository commit: `{session_data.get('git_commit') or 'UNKNOWN'}`", "", "## Steps", "",
             "| Step | Status | Exit | Evidence |", "|---|---:|---:|---|"]
    for step in session_data["steps"]:
        evidence = f"`{step['directory']}/result.json`" if step.get("result_file") else "—"
        lines.append(f"| {step['name']} | **{step['status']}** | {step.get('exit_code', '—')} | {evidence} |")
    lines += ["", "## Criteria", "", "| Step | Criterion | Status | Measured | Threshold | Evidence |",
              "|---|---|---:|---|---|---|"]
    for item in session_data["criteria"]:
        lines.append("| {} | {} | **{}** | {} | {} | `{}` |".format(
            item.get("step", ""), item.get("id", ""), item.get("status", "UNKNOWN"),
            json.dumps(item.get("measured"), ensure_ascii=False), item.get("threshold", ""),
            item.get("evidence_path", "")))
    if not session_data["criteria"]:
        lines.append("| — | No check criteria were produced | BLOCKED | — | — | — |")
    lines += ["", "## Notes", ""]
    lines.extend(f"- {note}" for note in session_data.get("notes", []))
    lines += [""]
    return "\n".join(lines)


def write_session(session: Path, data: dict[str, Any]) -> None:
    data["ended_utc"] = utc_now()
    (session / "SESSION.json").write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    (session / "SESSION.md").write_text(render_markdown(data), encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session-dir", type=Path, required=True, help="New session evidence directory")
    parser.add_argument("--calib-take", type=Path, required=True, help="Existing saved calibration take")
    parser.add_argument("--calib-kind", choices=("camera-imu", "allan"), default="camera-imu")
    parser.add_argument("--power-thresholds", type=Path)
    parser.add_argument("--raw-config", type=Path)
    parser.add_argument("--ros-config", type=Path)
    parser.add_argument("--calib-config", type=Path)
    parser.add_argument("--continue-on-fail", action="store_true", help="Run steps after failed/blocked dependencies")
    parser.add_argument("--yes", action="store_true", help="Accept physical-action confirmations")
    parser.add_argument("--plan", action="store_true", help="Print commands and confirmations without executing")
    parser.add_argument("--checks-dir", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    root = Path(args.checks_dir).resolve().parent if args.checks_dir else ROOT
    session = args.session_dir.expanduser().resolve()
    steps = build_plan(args, session, root)

    if args.plan:
        print(f"Session directory: {session}")
        print("Commands are shown in execution order; ROS step prompts are operator actions.")
        for step in steps:
            if step.get("confirmation"):
                print(f"CONFIRM: {step['confirmation']}")
            print(f"{step['id']}: {shlex.join(step['command'])}")
        return 0

    if session.exists():
        print(f"session directory already exists: {session}", file=sys.stderr)
        return 3
    calibration_take = args.calib_take.expanduser().resolve()
    if not calibration_take.is_dir():
        print(f"calibration take does not exist: {args.calib_take}", file=sys.stderr)
        return 3
    if calibration_take == session or calibration_take in session.parents or session in calibration_take.parents:
        print("session directory and calibration take must not overlap", file=sys.stderr)
        return 3
    try:
        session.mkdir(parents=True)
    except OSError as exc:
        print(f"cannot create session directory: {exc}", file=sys.stderr)
        return 3

    started = utc_now()
    data: dict[str, Any] = {"schema_version": 1, "status": "BLOCKED", "started_utc": started,
                            "ended_utc": None, "host": host_facts(), "git_commit": git_commit(root),
                            "session_directory": str(session), "steps": [], "criteria": [], "notes": []}
    failed_dependencies: set[str] = set()
    process_error = False
    for step in steps:
        step_data: dict[str, Any] = {"id": step["id"], "name": step["name"],
                                     "directory": str(step["directory"]), "status": "BLOCKED"}
        deps = {"power_usb": set(), "d435i_raw": {"power_usb"},
                "rosbag_record": {"power_usb", "d435i_raw"},
                "rosbag_replay": {"power_usb", "d435i_raw", "rosbag_record"},
                "calib_record": {"power_usb"}}[step["id"]]
        failed = sorted(deps & failed_dependencies)
        if failed and not args.continue_on_fail:
            step_data["status"] = "BLOCKED"
            step_data["skipped"] = True
            step_data["notes"] = ["Skipped because dependency did not PASS: " + ", ".join(failed)]
            data["steps"].append(step_data)
            data["notes"].extend(step_data["notes"])
            continue
        if step.get("confirmation") and not ask(step["confirmation"], args.yes):
            step_data["skipped"] = True
            step_data["notes"] = ["Operator did not confirm required physical action."]
            failed_dependencies.add(step["id"])
            data["steps"].append(step_data)
            data["notes"].extend(step_data["notes"])
            continue
        step["directory"].mkdir(parents=True, exist_ok=False)
        stdout_path = step["directory"] / "orchestrator.stdout"
        stderr_path = step["directory"] / "orchestrator.stderr"
        step_data["command"] = step["command"]
        step_data["stdout"] = str(stdout_path.relative_to(session))
        step_data["stderr"] = str(stderr_path.relative_to(session))
        step_start = utc_now()
        try:
            with stdout_path.open("xb") as stdout, stderr_path.open("xb") as stderr:
                proc = subprocess.run(step["command"], cwd=root, stdin=subprocess.DEVNULL,
                                      stdout=stdout, stderr=stderr, check=False)
            code = proc.returncode
            step_data["exit_code"] = code
            step_data["status"] = EXIT_STATUS.get(code, "ERROR")
        except OSError as exc:
            step_data["exit_code"] = 3
            step_data["status"] = "ERROR"
            step_data["notes"] = [f"Could not launch check: {exc}"]
            process_error = True
        step_data["started_utc"] = step_start
        step_data["ended_utc"] = utc_now()
        result, result_error = read_result(step["directory"])
        if result is not None:
            step_data["check_status"] = result.get("status")
            step_data["result_file"] = str((step["directory"] / "result.json").relative_to(session))
            data["criteria"].extend(aggregate_criteria(step, result, session))
            if result.get("status") != step_data["status"] and step_data["exit_code"] in (0, 1, 2):
                step_data.setdefault("notes", []).append("Exit code status disagrees with result.json status.")
                step_data["status"] = "ERROR"
                process_error = True
        elif result_error:
            step_data.setdefault("notes", []).append(result_error)
            step_data["status"] = "ERROR"
            process_error = True
        elif step_data.get("exit_code") in (0, 1, 2):
            step_data.setdefault("notes", []).append("Check returned a qualification status without result.json.")
            step_data["status"] = "ERROR"
            process_error = True
        if step_data["status"] in ("FAIL", "BLOCKED", "ERROR"):
            failed_dependencies.add(step["id"])
        data["steps"].append(step_data)

    statuses = [item["status"] for item in data["steps"]]
    if process_error or "ERROR" in statuses:
        data["status"] = "ERROR"
    elif "FAIL" in statuses:
        data["status"] = "FAIL"
    elif "BLOCKED" in statuses:
        data["status"] = "BLOCKED"
    else:
        data["status"] = "PASS"
    write_session(session, data)
    print(f"{'STEP':<20} STATUS     EXIT")
    for item in data["steps"]:
        print(f"{item['id']:<20} {item['status']:<10} {item.get('exit_code', '—')}")
    print(f"lab session: {data['status']} | {session / 'SESSION.json'}")
    return {"PASS": 0, "FAIL": 1, "BLOCKED": 2, "ERROR": 3}[data["status"]]


if __name__ == "__main__":
    raise SystemExit(main())
