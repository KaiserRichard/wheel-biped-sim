#!/usr/bin/env python3
"""Offline Pi bundle smoke checks. No sensor, serial, ROS graph, or network access."""
import argparse, hashlib, json, os, platform, shutil, subprocess, sys, tempfile
import importlib
from datetime import datetime, timezone
from pathlib import Path
from deploy import POLICY, verify_tree, VERSION

def utc(): return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")

def overall_status(criteria):
    if any(item["status"] == "FAIL" for item in criteria): return "FAIL"
    if any(item["status"] == "BLOCKED" for item in criteria): return "BLOCKED"
    return "PASS"

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--root", type=Path, default=Path.cwd()); ap.add_argument("--outdir", type=Path); ap.add_argument("--verify-only", action="store_true"); args = ap.parse_args()
    root = args.root.resolve(); out = (args.outdir or Path(tempfile.gettempdir()) / f"wbr-pi-smoke-{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}").resolve()
    start = utc(); checks = []; raw = []
    def criterion(name, desc, measured, threshold, status, evidence, source):
        checks.append({"id": name, "description": desc, "measured": measured, "unit": "", "threshold": threshold, "status": status, "evidence": evidence, "threshold_source": source})
    ok, errors, manifest = verify_tree(root)
    out.mkdir(parents=True, exist_ok=True)
    (out / "manifest-check.json").write_text(json.dumps({"valid": ok, "errors": errors, "git_commit": manifest.get("git_commit")}, indent=2) + "\n")
    criterion("manifest", "All bundled files match the SHA-256 manifest", errors or "verified", "zero missing, changed, or unlisted files", "PASS" if ok else "FAIL", "manifest-check.json", "phase-16 bundle integrity contract")
    if not ok:
        # Do not execute a release whose contents cannot be authenticated. Missing
        # or unreadable manifest evidence is BLOCKED; a present but invalid
        # manifest or payload is a FAIL.
        manifest_status = "BLOCKED" if errors and errors[0].startswith("manifest missing or unreadable:") else "FAIL"
        checks[0]["status"] = manifest_status
        status = manifest_status
        result = {"check": "pi_deployment_smoke", "tool_version": VERSION, "status": status,
                  "started_utc": start, "ended_utc": utc(),
                  "host": {"hostname": platform.node(), "system": platform.system(), "machine": platform.machine(), "python": sys.version.split()[0]},
                  "criteria": checks, "raw_files": [],
                  "notes": ["Smoke checks stopped because the bundle manifest is unavailable or invalid.",
                            "No camera, serial device, ROS graph or network was accessed."]}
        (out / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        print("criterion             status")
        print(f"manifest              {status}")
        print(f"OVERALL               {status}")
        return {"BLOCKED": 2, "FAIL": 1}[status]
    if args.verify_only:
        print(json.dumps({"valid": ok, "errors": errors}, indent=2)); return 0 if ok else 1
    py = list(sys.version_info[:2]); minimum = [POLICY["python_minimum"]["major"], POLICY["python_minimum"]["minor"]]
    criterion("python", "Interpreter meets Pi minimum", ".".join(map(str, py)), f">={minimum[0]}.{minimum[1]}", "PASS" if tuple(py) >= tuple(minimum) else "BLOCKED", "python-version.txt", POLICY["python_minimum"]["source"])
    (out / "python-version.txt").write_text(sys.version + "\n")
    modules = {}
    for name in ("numpy", "yaml", "rclpy"):
        try:
            importlib.import_module(name)
            modules[name] = {"available": True, "detail": "import succeeded"}
        except (ImportError, ModuleNotFoundError) as exc:
            modules[name] = {"available": False, "detail": str(exc)}
    (out / "modules.json").write_text(json.dumps(modules, indent=2) + "\n")
    needs = {"lab/d435i_raw/check.py": [], "lab/d435i_raw/capture.py": [], "lab/calib_record/check_calib_take.py": [], "lab/power_usb/check_power_usb.py": [],
             "lab/rosbag_qual/record_check.py": [], "lab/rosbag_qual/replay_check.py": [], "lab/rosbag_qual/analyze_bag.py": [],
             "tools/eval_pipeline/accuracy/evaluate.py": ["numpy"], "tools/eval_pipeline/profile/profile.py": ["numpy"],
             "tools/trajectory_eval/traj_eval.py": ["numpy"], "tools/clockmap/analyze.py": ["numpy"],
             "tools/take_tools/take_tools.py": [], "tools/run_harness/harness.py": ["numpy", "yaml"],
             "tools/frames/convert_trajectory.py": ["numpy"], "tools/calib_tools/calib_convert.py": ["numpy", "yaml"],
             "calibration/lab/pi/take_diagnostics.py": [], "tools/eval_pipeline/accuracy/run_euroc.sh": []}
    usability = {name: {"usable": all(modules[m]["available"] for m in mods),
                        "missing_modules": [m for m in mods if not modules[m]["available"]],
                        "rclpy_optional_available": modules["rclpy"]["available"]}
                 for name, mods in needs.items()}
    (out / "tool-usability.json").write_text(json.dumps(usability, indent=2) + "\n")
    missing_required = any(not item["usable"] for item in usability.values())
    criterion("modules", "Imported modules are checked and usable tools are identified", usability, "tool imports succeed; rclpy optional", "BLOCKED" if missing_required else "PASS", "tool-usability.json", "phase-16 module inventory")
    scripts = ["lab/d435i_raw/check.py", "lab/d435i_raw/capture.py", "lab/calib_record/check_calib_take.py", "lab/power_usb/check_power_usb.py",
               "lab/rosbag_qual/record_check.py", "lab/rosbag_qual/replay_check.py", "lab/rosbag_qual/analyze_bag.py",
               "tools/eval_pipeline/accuracy/evaluate.py", "tools/eval_pipeline/profile/profile.py",
               "tools/trajectory_eval/traj_eval.py", "tools/clockmap/analyze.py", "tools/take_tools/take_tools.py",
               "tools/run_harness/harness.py", "tools/frames/convert_trajectory.py", "tools/calib_tools/calib_convert.py",
               "calibration/lab/pi/take_diagnostics.py"]
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
    outcomes = []
    for rel in scripts:
        proc = subprocess.run([sys.executable, str(root / rel), "--help"], cwd=root, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        path = out / (rel.replace("/", "_") + ".help.log"); path.write_text(proc.stdout); raw.append(path)
        outcomes.append({"tool": rel, "exit": proc.returncode})
    criterion("cli_help", "Pi-relevant lab and evaluation CLIs expose help", outcomes, "all exit 0", "PASS" if all(x["exit"] == 0 for x in outcomes) else "FAIL", "cli-help.json", "phase-16 smoke requirement")
    (out / "cli-help.json").write_text(json.dumps(outcomes, indent=2) + "\n")
    launcher = subprocess.run(["bash", str(root / "tools/eval_pipeline/accuracy/run_euroc.sh")], cwd=root, env=env,
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    (out / "run-euroc-usage.log").write_text(launcher.stdout); raw.append(out / "run-euroc-usage.log")
    usage_exit = POLICY["launcher_usage_exit_code"]["value"]
    launcher_result = {"tool": "tools/eval_pipeline/accuracy/run_euroc.sh", "exit": launcher.returncode,
                       "expected_usage_exit": usage_exit, "usage_only": "launcher has no --help or analyze-only mode; no arguments print usage before any run"}
    (out / "run-euroc-usage.json").write_text(json.dumps(launcher_result, indent=2) + "\n")
    criterion("launcher_usage", "Engine launcher exits through its no-argument usage path", launcher.returncode,
              f"{usage_exit} (documented no-argument usage); engine never starts", "PASS" if launcher.returncode == usage_exit else "FAIL",
              "run-euroc-usage.json", POLICY["launcher_usage_exit_code"]["source"])
    # Offline analyzers run against preserved fixtures; known diagnostic FAIL/BLOCKED outcomes are expected.
    fixture_runs = [
        ("lab/d435i_raw/check.py", ["--analyze-only", "lab/d435i_raw/tests/fixtures/old-unit-smoke"], "--outdir"),
        ("lab/calib_record/check_calib_take.py", ["--analyze-only", "lab/calib_record/tests/fixtures", "--kind", "camera-imu"], "--outdir"),
        ("lab/power_usb/check_power_usb.py", ["--analyze-only", "lab/power_usb/tests/fixtures/real-partial"], "--out"),
        ("lab/rosbag_qual/record_check.py", ["--analyze-only", "lab/rosbag_qual/tests"], None),
        ("lab/rosbag_qual/replay_check.py", ["--analyze-only", "lab/rosbag_qual/tests"], None),
        ("lab/rosbag_qual/analyze_bag.py", ["--analyze-only", "lab/rosbag_qual/tests"], None),
        ("tools/eval_pipeline/profile/profile.py", ["validity", "--analyze-only", "tools/eval_pipeline/profile/tests/fixtures"], "--out"),
    ]
    fixture_outcomes = []
    with tempfile.TemporaryDirectory(prefix="wbr-pi-smoke-") as td:
        for i, (script, argv, outflag) in enumerate(fixture_runs):
            dest = str(Path(td) / f"fixture-{i}")
            args_for_run = list(argv)
            if outflag is None:
                fixture_copy = Path(td) / f"input-{i}"
                shutil.copytree(root / argv[1], fixture_copy)
                args_for_run[1] = str(fixture_copy)
            cmd = [sys.executable, str(root / script), *args_for_run]
            if outflag is not None: cmd += [outflag, dest]
            proc = subprocess.run(cmd, cwd=root, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            logfile = out / f"fixture-{i}.log"; logfile.write_text(proc.stdout); raw.append(logfile)
            # These historical fixtures intentionally produce FAIL or BLOCKED. CLI contract codes are 0/1/2.
            fixture_outcomes.append({"tool": script, "exit": proc.returncode, "accepted_contract_exit": proc.returncode in (0, 1, 2)})
        trajectory_out = str(Path(td) / "trajectory-eval")
        trajectory_fixture = str(root / "tools/deploy/tests/fixtures/trajectory.tum")
        trajectory_cmd = [sys.executable, str(root / "tools/trajectory_eval/traj_eval.py"), "single",
                          "--est", trajectory_fixture, "--ref", trajectory_fixture,
                          "--est-format", "tum", "--ref-format", "tum", "--est-time-unit", "s", "--ref-time-unit", "s",
                          "--est-frame", "fixture_camera", "--ref-frame", "fixture_camera",
                          "--clock-assumption", "same bundled fixture file; no cross-clock mapping", "--out", trajectory_out, "--no-plots"]
        trajectory_proc = subprocess.run(trajectory_cmd, cwd=root, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        trajectory_log = out / "trajectory-eval-fixture.log"; trajectory_log.write_text(trajectory_proc.stdout); raw.append(trajectory_log)
        fixture_outcomes.append({"tool": "tools/trajectory_eval/traj_eval.py single", "exit": trajectory_proc.returncode,
                                 "accepted_contract_exit": trajectory_proc.returncode == 0})
        accuracy_root = Path(td) / "accuracy-fixture"; empty_run = accuracy_root / "run-empty"
        empty_run.mkdir(parents=True); gt = accuracy_root / "groundtruth.csv"; gt.write_text("")
        accuracy_out = str(Path(td) / "accuracy-output")
        accuracy_cmd = [sys.executable, str(root / "tools/eval_pipeline/accuracy/evaluate.py"),
                        "--sequence", "smoke-fixture", "--runs", str(empty_run), "--ground-truth", str(gt),
                        "--expected-frames", "1", "--out", accuracy_out]
        accuracy_proc = subprocess.run(accuracy_cmd, cwd=root, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        accuracy_log = out / "accuracy-eval-fixture.log"; accuracy_log.write_text(accuracy_proc.stdout); raw.append(accuracy_log)
        fixture_outcomes.append({"tool": "tools/eval_pipeline/accuracy/evaluate.py", "exit": accuracy_proc.returncode,
                                 "accepted_contract_exit": accuracy_proc.returncode in (0, 1, 2)})
    criterion("fixture_analysis", "Offline analyzers accept bundled historical fixtures", fixture_outcomes, "exit code in PASS/FAIL/BLOCKED set 0,1,2", "PASS" if all(x["accepted_contract_exit"] for x in fixture_outcomes) else "FAIL", "fixture-analysis.json", "COMMON.md lab result contract")
    (out / "fixture-analysis.json").write_text(json.dumps(fixture_outcomes, indent=2) + "\n")
    suites = ["lab/d435i_raw/tests", "lab/calib_record/tests", "lab/power_usb/tests", "lab/rosbag_qual/tests",
              "tools/eval_pipeline/accuracy/tests", "tools/eval_pipeline/profile/tests", "tools/trajectory_eval",
              "tools/clockmap/tests", "tools/take_tools/tests", "tools/run_harness/tests", "tools/frames/tests", "tools/calib_tools/tests",
              "tools/link_proto/tests", "tools/perception_health/tests", "tools/ros2_bridge/tests",
              "tools/runtime_observer/tests", "tools/vio_adapter/tests"]
    suite_results = []
    for i, suite in enumerate(suites):
        suite_root = root; suite_tmp = None
        if suite == "tools/vio_adapter/tests":
            # The documented suite builds ignored binaries under build/. Run a
            # disposable copy so smoke never modifies the verified release.
            suite_tmp = tempfile.TemporaryDirectory(prefix="wbr-vio-suite-")
            suite_root = Path(suite_tmp.name)
            shutil.copytree(root / "tools/vio_adapter", suite_root / "tools/vio_adapter")
            command = ["make", "-s", "-C", str(suite_root / "tools/vio_adapter"), "test"]
        else:
            command = [sys.executable, "-m", "unittest", "discover", "-s", suite, "-p", "test*.py"]
        proc = subprocess.run(command, cwd=suite_root, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        if suite_tmp is not None: suite_tmp.cleanup()
        log = out / f"unittest-{i}.log"; log.write_text(proc.stdout); raw.append(log)
        suite_results.append({"suite": suite, "exit": proc.returncode, "log": log.name})
    criterion("unittests", "Hardware-free lab and evaluation unit suites pass", suite_results, "all exit 0", "PASS" if all(x["exit"] == 0 for x in suite_results) else "FAIL", "unittest-summary.json", "phase-16 smoke requirement")
    (out / "unittest-summary.json").write_text(json.dumps(suite_results, indent=2) + "\n")
    status = overall_status(checks)
    result = {"check": "pi_deployment_smoke", "tool_version": VERSION, "status": status, "started_utc": start, "ended_utc": utc(),
              "host": {"hostname": platform.node(), "system": platform.system(), "machine": platform.machine(), "python": sys.version.split()[0]},
              "criteria": checks, "raw_files": [{"path": p.name, "sha256": hashlib.sha256(p.read_bytes()).hexdigest()} for p in raw],
              "notes": ["Offline fixture and unit checks only; this does not pass a hardware gate.", "No camera, serial device, ROS graph or network was accessed."]}
    (out / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    print("criterion             status")
    for item in checks: print(f"{item['id']:<21} {item['status']}")
    print("\nTool availability")
    for name, item in usability.items(): print(f"{name:<48} {'usable' if item['usable'] else 'unavailable'}")
    print(f"OVERALL               {status}")
    return {"PASS": 0, "FAIL": 1, "BLOCKED": 2}[status]

if __name__ == "__main__": raise SystemExit(main())
