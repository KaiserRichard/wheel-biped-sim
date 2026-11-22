# phase-2 completion report — 2026-10-18

Implemented the task entirely under `lab/d435i_raw/`. Changes are uncommitted
for the maintainer. No other repository files or shared evidence were changed.

## Delivered

- `check.py`: new-directory capture/replay entry point, bounded SDK subprocess,
  exact command/config receipt, verbatim stdout/stderr, timeout/partial evidence,
  contract result JSON, table and exit codes 0/1/2/3.
- `capture.py`: native SDK profile inventory/selection, exact SDK identity,
  stereo IR 848x480 Y8/30 + gyro/200 + offered accel/100 or /200, emitter OFF
  readback/restoration, independent callbacks, bounded queue, integer host
  CLOCK_MONOTONIC/CLOCK_REALTIME receipts, actual SDK domains and available
  hardware/global/system stamps, full-precision camera IMU XYZ and spaced PGMs.
- `analyze.py`: all requested identity/profile/rate/drop/gap/timestamp/stereo/
  static-plausibility/image checks; callback-to-file accounting; unavailable
  evidence BLOCKED and observed violations FAIL; FAIL precedence. Latency is
  UNKNOWN without a qualified mapping, with same-clock receipt jitter reported.
- `legacy.py`: diagnostic replay of actual old-unit vendor CSV without inventing
  omitted timestamp domains or host monotonic epochs.
- `thresholds.yaml`: JSON-form YAML 1.2 with source/rationale per threshold;
  chosen acceptance limits are PROPOSED. Replay retains capture and analysis
  configurations. Default qualification duration stays 60 seconds.
- README: exact future Pi and portable analysis commands, criterion definitions,
  clock policy, dependencies, evidence handling and scope limits.

**SOURCE-SUPPORTED reuse:** imports existing
`lab/sensor_validation/sdk_analysis.py` for
`summarize()` and historical `read_capture()`. Its old thresholds/profile are
not applied to the current profile. The requested SDK Python/C++ capture and
metadata references, Allan recorder, and clockmap README were inspected.
`tools/clockmap` is not reimplemented: no qualified device-to-host map exists
in these captures, so absolute receipt latency is not calculated.

The old fixed-profile PGM reader cannot parse 848x480 capture images; this check
parses its own exact binary PGM format locally. Existing tools were not edited.
No existing-tool bug was established during this task.

## Verification

**MEASURED software-only:** 28 unittest tests passed in 14.293 s on the available
Mac Python **3.11.0rc2**. Full output: [tests/last-run.log](tests/last-run.log).

```bash
python3 -m unittest discover -s lab/d435i_raw/tests -v \
  > lab/d435i_raw/tests/last-run.log 2>&1
python3 -m py_compile lab/d435i_raw/*.py lab/d435i_raw/tests/test_check.py
git diff --check
```

Compilation, source/document whitespace and file-size checks passed. All
retained files are below 1 MB; there are no additions under `data/`. No shell
script was added, so no new `bash -n` check is applicable. `git diff --check`
has no tracked edits to inspect; the separate whitespace scan covered the new
Python, Markdown and configuration files.

Coverage includes result-schema fields/hashes and PASS/FAIL/BLOCKED precedence;
CLI exits including parse errors; replay immutability/no-overwrite and static
confirmation preservation; shortfall/frame gaps/nonincreasing frames/window
boundary stalls; duplicate/reversed/unknown/changing clock domains and host UTC
reversal; stereo missing pairs/offset/domain mismatch; static magnitude/noise;
black/white/malformed/missing images; malformed CSV/NaN/path traversal;
missing SDK/device/unsupported profile; SDK close/callback errors; actual queue
overflow; parent deadline with partial evidence; and simulated full callback
capture through analysis with native accel/200 fallback and emitter restoration.

**MEASURED historical fixture:** byte-identical `frames.csv` and `command.json`
from shared checkout experiment
`2026-09-13-d435i-post-reboot-20260913T023833Z/smoke/`, OLD SDK serial
**99003**, with original paths and SHA-256 in
[PROVENANCE.json](tests/fixtures/old-unit-smoke/PROVENANCE.json). Tests verify
those bytes and real-file CLI behavior. Its current-unit result is correctly
FAIL (wrong serial and IR profile), while absent explicit clocks/stereo/static
image evidence remain BLOCKED. Its column title `HW Timestamp` is not treated
as domain evidence. It cannot qualify current serial **99005**.

**ASSUMED / SYNTHETIC fixtures:** current-profile samples and fake SDK objects
exercise the software only. Generated files are removed from temporary
subdirectories under `tests/`. A fixture PASS is not a gate PASS.

## Not verified and remaining risks

**UNKNOWN / hardware not verified:** no SSH, Pi access, real camera/serial
device opening, package installation, GUI, network use or hardware mutation was
performed. The real pyrealsense2 API/ABI, actual Pi Python 3.12 execution, offered
profiles, metadata support/units on the installed SDK, actual domains/rates,
USB/power/thermal behavior, stationarity, optical content, callback scheduling
cost, sustained operation and cleanup/restoration on hardware remain untested.

All acceptance limits require lab review. Static mean/std checks are
plausibility, not bias/scale/extrinsic/noise-density calibration. Global time
is not host monotonic time; true receipt/exposure latency and camera-to-IMU
alignment remain UNKNOWN. Missing raw hardware counters stay explicitly
unavailable, and raw metadata counter wrap conservatively fails rather than
being guessed/unwrapped. SDK USB descriptor matching does not independently
prove physical bus speed or USB stability.

The capture retains sampled pixels and all per-sample metadata/IMU vectors,
not a full image dataset. It cannot qualify the measured 18.4 MB/s recording
requirement, power, cooling, kernel health or ROS record/replay. Forced SDK
termination may prevent emitter restoration; timeouts FAIL and retain evidence.
The 60-second raw check does not supersede separate historical soak/gate
requirements. Current-unit N1/N2/N3 gates remain OPEN/UNKNOWN.

## Proposed commit message

```text
feat(lab): add bounded D435i stereo and camera IMU qualification check

Add config-sourced raw SDK capture, clock-safe replay analysis and result
contract, with historical old-unit fixtures and hardware-free unittest coverage.
```
