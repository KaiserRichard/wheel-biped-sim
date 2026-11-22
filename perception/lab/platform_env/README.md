# N0 Platform and software environment check

`check_platform.py` captures a read-only, camera-free inventory of the Pi software environment, compares required facts with `manifest.yaml`, writes `snapshot.json`, and records command output under `raw/`. The manifest is JSON syntax, which is also valid YAML, so this tool needs only the Python standard library.

The checked-in manifest is based on the **MEASURED October 13, 2026 4 GB Pi 5 stand-in**, not the later 8 GB target board. Model and RAM are INFO fields so the stand-in's board identity cannot fail the target check; re-pin them after reviewing a snapshot from the 8 GB board. Other environment pins remain independently enforced. The manifest also records the active D435i firmware expectation and currently known ORB build evidence. Review a candidate manifest after collecting on the intended board before adopting it as a new baseline. Unknown OpenCV, Eigen, and clocksource pins are INFO and do not affect the verdict.

## Run on the Pi

Run as the normal user. This tool does not use sudo, install packages, change services, modify the camera, or open a camera device. Source Jazzy so the collected `ROS_DISTRO` reflects the active shell.

```bash
cd ~/wbr-perception-phase1-runtime
source /opt/ros/jazzy/setup.bash
OUT="$HOME/wbr-evidence/platform-env-$(date -u +%Y%m%dT%H%M%SZ)"
python3 lab/platform_env/check_platform.py --outdir "$OUT"
```

To write a proposed manifest next to the run output, without replacing the checked-in manifest:

```bash
python3 lab/platform_env/check_platform.py --outdir "$OUT" --write-manifest-candidate
```

To re-evaluate saved command results on a Mac or another offline host:

```bash
python3 lab/platform_env/check_platform.py --analyze-only "$OUT"
```

The checker prints a field table and writes `result.json`. Exit codes are 0 PASS, 1 FAIL, 2 BLOCKED, and 3 usage/internal error.

## Criteria

Each manifest field has its expected value, comparison policy, severity, and evidence source in one place. REQUIRED exact fields must equal the pin; `min-version` requires the observed version to meet the declared minimum; `present` checks the expected serial appears in the D435i udev rule. A mismatch is FAIL. A missing required fact or unavailable required command is BLOCKED. INFO fields are captured but do not affect the overall status. `recording_free_bytes` is deliberately INFO because free space changes; this task does not invent a free-space gate threshold.

The checker compares the root filesystem mounted at the recording path `$HOME/wbr-evidence`, records free bytes, package versions, ORB artifact hashes, and the running shell's ROS distribution. Firmware is not queried: the check never opens the camera, and reports `not_probed_camera_access_prohibited`; the manifest's firmware value is the expected pin from saved enumeration evidence. `clocksource` is recorded as found or unknown. UTC collection labels use host `CLOCK_REALTIME`; command durations use `CLOCK_MONOTONIC`. The tool does not subtract timestamps from those domains.

## What this does not prove

A PASS only means the current read-only facts match the manifest. It does not qualify N0 as a whole: no power, thermal, USB, camera identity/firmware, storage write-rate, ROS streaming, or sustained-load gate is run. The D435i is not touched. A fixture PASS proves the checker logic only, not that any platform gate passed. Hash checks confirm file identity against the captured reference and do not prove the binary's source provenance by themselves.
