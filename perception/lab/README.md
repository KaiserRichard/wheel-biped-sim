# Lab qualification tools

The `lab/` directory contains four bounded checks and a session orchestrator.
Each check writes its own `result.json` with criterion level
PASS/FAIL/BLOCKED evidence. Passing a fixture or analyzer test verifies tool
behavior only; it is not a physical gate PASS. See each linked README for its
criteria, evidence format, saved-file analysis mode and limits.

| Check | Purpose | Entry point | README |
|---|---|---|---|
| Power / USB | Host power, thermal and kernel/USB evidence over a bounded window | `lab/power_usb/check_power_usb.py` | [power_usb](power_usb/README.md) |
| D435i raw | Separate IR and IMU stream continuity, timestamps and plausibility | `lab/d435i_raw/check.py` | [d435i_raw](d435i_raw/README.md) |
| ROS bag qualification | Bounded ROS 2 record, offline analysis and isolated replay | `lab/rosbag_qual/record_check.py`, `replay_check.py`, `analyze_bag.py` | [rosbag_qual](rosbag_qual/README.md) |
| Calibration take | Qualify a saved camera–IMU or Allan take before heavier analysis | `lab/calib_record/check_calib_take.py` | [calib_record](calib_record/README.md) |
| Session orchestrator | Run the checks in dependency order and aggregate their evidence | `lab/run_lab_session.py` | This page |

## Orchestrated bench session

Run from the repository root on the Pi after sourcing ROS 2 Jazzy and preparing
the camera and an already saved calibration take. Use a new output directory.
The default session sequence is power/USB, still-camera raw capture, ROS record,
ROS replay with the live camera stopped, and calibration-take qualification.
The ROS camera node must be started in domain 42 before its record confirmation.
The calibration check analyzes the supplied saved take; it does not capture one.

```bash
source /opt/ros/jazzy/setup.bash
python3 lab/run_lab_session.py \
  --session-dir <local workspace>/wbr-evidence/session-20261018-01 \
  --calib-take <local workspace>/wbr-evidence/calib-C1 \
  --calib-kind camera-imu
```

The orchestrator prompts before physical actions: resting the camera still for
raw IMU checks, confirming the ROS camera node is streaming, and confirming
that it is stopped before replay. `--yes` accepts those confirmations for a
deliberately unattended run. `--continue-on-fail` runs steps beyond failed or
blocked dependencies; by default power/USB gates all later steps, raw gates ROS,
and a failed ROS recording gates replay. A saved calibration take is independent
of the ROS record/replay result and is still checked when its power prerequisite
passed. The command never starts the camera driver or sources ROS itself.

```bash
python3 lab/run_lab_session.py \
  --session-dir /tmp/wbr-session-plan \
  --calib-take <local workspace>/wbr-evidence/calib-C1 \
  --plan
```

`--plan` prints the resolved commands and prompts without creating the session
directory or launching checks. Optional `--power-thresholds`, `--raw-config`,
`--ros-config` and `--calib-config` pass reviewed policy files to their checks.
The session directory contains numbered per-check folders, verbatim
`orchestrator.stdout`/`orchestrator.stderr`, `SESSION.json`, and `SESSION.md`.
The summary records host facts, the repository commit, UTC bounds, each step's
exit/result status and every check criterion with its evidence path. Exit codes
are 0 PASS, 1 FAIL, 2 BLOCKED and 3 orchestration/usage error.

The aggregate cannot establish that a physical calibration take is observable,
that a fixture represents current hardware, or that a full project gate passed.
Review raw evidence and the individual check READMEs; physical hardware behavior
is UNKNOWN until the session is actually run on the Pi.

Orchestrator tests use stub check executables and do not access hardware:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s lab/tests_session -v \
  > lab/tests_session/last-run.log 2>&1
```
