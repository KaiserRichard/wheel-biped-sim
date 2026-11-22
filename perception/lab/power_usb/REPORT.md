# phase-1 completion report — 2026-10-18

**CALCULATED: task implementation and software verification complete.**
**UNKNOWN: physical Pi 5 + D435i qualification; no hardware gate PASS claimed.**
All changes are confined to `lab/power_usb/` and left uncommitted for the
maintainer. No commit, push, installation, network access, SSH, GUI, camera,
serial-device access, or change to the shared checkout was performed.

Built `check_power_usb.py`, `thresholds.yaml`, the operator/replay README,
56 unittest cases, small historical fixtures with provenance, and saved
verification logs. Python uses the standard library only. The `.yaml` file is
JSON-form YAML 1.2 so the Pi does not need PyYAML. Thresholds have sources or
rationales; chosen bench limits are explicitly PROPOSED. Runtime duration and
interval overrides are saved into the effective config.

The collector records before/during/after throttle/temperature/PMIC observations,
binary PSU current declarations, D435i sysfs identity/speed/topology, and
before/after kernel snapshots. Optional user commands run in a bounded process
group with saved stdout/stderr and exit/termination evidence. No supplied command
means an explicitly labelled idle check. Evidence is saved verbatim, checksummed
and replayable into a new directory. Results use the COMMON.md contract and
FAIL > BLOCKED > PASS precedence, with exits 0/1/2/3 as specified.

**Reuse:** imports the existing throttle decoder and CLOCK_MONOTONIC function
from `tools/runtime_observer/resource_sampler.py`. Loads the existing
`lab/sensor_validation/kernel_analysis.py`
classifier into an isolated module and supplies the task's configurable
patterns. Existing tool files were not edited or refactored. The review also
corrected the PMIC channel invocation, rejected malformed successful PSU
configuration output, and validated UTC sample timestamps. The general
run-harness and take-tools READMEs were inspected; camera recording, trajectory
analysis and clock mapping are outside this check.

**MEASURED software verification:** host Darwin 24.6.0 x86_64, available
Python 3.11.0rc2. The target Pi's Python 3.12 was not available or executed.

```sh
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s lab/power_usb/tests -v \
  > lab/power_usb/tests/last-run.log 2>&1
```

56 tests passed; complete output is in `tests/last-run.log`. Tests cover:

- Complete synthetic PASS and the required result fields, evidence references and SHA-256 values.
- Current/newly latched under-voltage FAIL, pre-existing history handling, all eight decoded bits, missing/malformed flags and history resets.
- Temperature policy checks 79.9 °C PASS and exactly 80.0 °C FAIL, matching Raspberry Pi's published Pi 5 throttling boundary.
- PMIC channel formats, low input voltage FAIL, exact boundary, and refusal to substitute core voltage for EXT5V.
- Hot firmware/sysfs temperatures and incomplete thermal evidence.
- Big-endian PSU declarations, 3 A FAIL, unavailable declarations, USB override semantics and optional 5 A policy.
- USB 2 FAIL, unrelated SuperSpeed devices, missing/re-enumerated cameras, multiple cameras and missing topology.
- Every configured kernel fault class, exclusion of baseline faults, denied/rotated/empty logs, boot changes and journal fallback.
- Missing/inconsistent capture clocks, incomplete windows and tampered/missing raw evidence.
- Load full-window success, early/failed exit, launch failure, planned termination, and a nonzero exit racing checker termination.
- Collector verbatim binary output, missing tools/timeouts, fake sysfs/procfs reads, actual bounded synthetic shell loads, and idle sampling order.
- CLI validation/output refusal, threshold justification validation, immutable replay, and the real historical partial capture.

The real fixture was assembled from authorized read-only October shared-checkout
files and September evidence already in this worktree. Each source file has a
whole-source hash and excerpt range in `tests/fixtures/real-partial/raw/provenance.json`.
The measured `throttled=0x50000` and 50.5 °C snapshot is preserved exactly;
missing observation clocks are null/UNKNOWN. Other sessions are context only,
not fabricated before/after samples. The October 14 addendum's under-voltage
observation is retained as a narrative excerpt, not invented numeric telemetry.

**MEASURED CLI verification:** invoked the actual script with `--analyze-only`
on the historical fixture, using a temporary output inside this directory.
It returned **2 / BLOCKED**, as required for missing complete-window, ADC, PSU,
USB and kernel-window evidence. Table output is saved in `tests/real-replay.log`.
The temporary replay output was removed after validating its result contract.
Both Python files compile; `--help` works. `git diff --check` reported no errors;
additional whitespace and scope checks covered new, untracked files. There are
no shell script files to run through `bash -n`.

**NOT VERIFIED / open risks:**

- No live Pi firmware commands, PMIC values, real negotiated-current property, kernel-access permissions or new-unit USB stability were tested. The 5 A property is a firmware declaration, not electrical proof. A USB-current override alone cannot pass it.
- PROPOSED voltage/sampling limits need bench-owner review before accepting production evidence. The 80 °C inclusive temperature boundary is SOURCE-SUPPORTED by Raspberry Pi's Pi 5 thermal article. The ADC's calibration/accuracy is UNKNOWN; values are sampled, not continuous rail measurements.
- A historical under-voltage bit already set at baseline cannot distinguish another historical-only event. The result explains this limitation. Current bits, ADC readings and kernel messages remain checked.
- Other throttling/capping/soft-temperature bits are recorded but do not independently fail phase-1. An idle or phase-1 PASS does not validate timing benchmarks under throttling.
- Sequential reads perturb timing. Missing samples or large gaps block; undetected events between samples remain possible. Shell-command lifetime is a polled upper-bound estimate of activity, not proof of useful load or camera frame delivery.
- Kernel suffix extraction requires an exact retained prefix and unchanged boot ID. Rotation without a complete fallback blocks. The interval conservatively includes setup/teardown; faults there may fail a run. Unreported kernel loss and faults outside the configured pattern vocabulary remain possible.
- Board-wide kernel matching may flag another USB device; saved matches require human attribution. USB descriptors/serials are recorded and are not equated with SDK serial numbers or calibration identity.
- The load must remain in its new process group for cleanup. Abrupt checker termination (SIGKILL, power loss, disk failure) cannot guarantee a finalized result or child cleanup; normal bounded completion and handled Ctrl-C perform teardown.
- Tests passing on fixtures establish software behavior only. They do not qualify N0/N1/N3, D435i streams, thermal endurance, PSU/cable suitability, camera IMU calibration, or STM32 body-IMU/balancing safety.

Proposed commit message:

```text
lab: add replayable Pi 5 power and D435i USB qualification check
```

## phase-21 audit corrections — 2026-10-18

The shared audit findings are resolved across this directory, Allan recording,
clockmap, and profile validity. The power check now uses the same inclusive
80 °C failure boundary as profile validity; Raspberry Pi's Pi 5 article says
throttling begins when temperature reaches 80 °C. Allan interval counters skip
timestamp subtraction across domain changes, and clockmap labels its NumPy
linear interpolation percentiles. Details and per-tool test results are in the
owned directory REPORT.md files; all four suites pass and logs are saved under
their respective `tests/last-run.log` paths. The real partial power fixture and
profile's saved N4A fixture remain included.

**NOT verified:** no Pi, camera, librealsense runtime or physical gate was
tested; fixture PASS results establish software behavior only.

Proposed commit message: `fix: resolve cross-tool audit findings`

## Independent review — 2026-10-18

The inclusive boundary is correct. Added regression coverage with exactly
80.0 °C in only vcgencmd or only sysfs, while the other source remains 79.9 °C;
either independently causes FAIL. The final 59-test run includes the historical
partial-evidence BLOCKED replay; exact output is `tests/last-run.log`.
The earlier statement that all four suites pass needs qualification after
review: profile now retains an expected-failure regression for an inherited
non-finite-temperature false PASS. Full findings and exact commands are in
[`calibration/allan/REVIEW.md`](../../calibration/allan/REVIEW.md). No hardware
or network was used, so the external thermal article was not independently
rechecked.
