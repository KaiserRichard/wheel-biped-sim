# Pi 5 power and USB qualification check

`check_power_usb.py` records read-only host diagnostics for the Raspberry Pi 5
and any D435i USB devices (8086:0b3a), then evaluates saved evidence. Python
3.11+ with the standard library only; Ubuntu 24.04 / Python 3.12 is the intended
Pi environment. No camera API, ROS publisher, serial device, network connection,
package installation, or system configuration change is performed by the checker.

**CALCULATED:** PASS means the configured criteria passed for this capture.
A tool passing on fixtures is **not a gate PASS**. An idle PASS cannot qualify
camera streaming, electrical safety, N0/N1/N3, or perception performance.
**UNKNOWN:** real hardware behavior until the operator runs this on the Pi.
STM32 body-IMU and balancing/safety qualification are outside this tool.

Run from the repository root on the Pi, using a new output directory each time:

```sh
python3 lab/power_usb/check_power_usb.py \
  --out lab/power_usb/runs/idle-20261018 --duration 60

# Optional example using an ALREADY installed load generator. No installation is performed.
python3 lab/power_usb/check_power_usb.py \
  --out lab/power_usb/runs/load-20261018 --duration 60 --interval 1 \
  --load-cmd 'exec stress-ng --cpu 4'
```

For a camera load, supply the bench's already reviewed streaming command as
`--load-cmd`. The checker itself never opens a camera. The command is trusted
bash code, inherits the environment and working directory, and receives closed
stdin. It runs in a new process group, gets SIGTERM at the end of the observation
window and SIGKILL if necessary. stdout, stderr, final exit code, whether the
checker stopped it, and observed lifetime are saved. The load should remain
active through the window; a command which launches a daemon and exits early
fails coverage. Natural nonzero exits fail; exits caused by the checker's
TERM/KILL are expected. Command lifetime is estimated by polling and cannot
prove useful work or full frame delivery. Descendants must stay in the owned
process group for cleanup.

Default operation is an **idle check**, explicitly labelled in the table and
`result.json`. Obtain a baseline sample before the load, sample during the
window, stop the load, then obtain an after sample. Diagnostic reads add overhead;
this is a best-effort 1 Hz observer, not a real-time sampler. Missed slots are
skipped; excessive sample gaps produce BLOCKED. A short or interrupted run is
not silently promoted to PASS.

`vcgencmd`, `lsusb`, Linux sysfs/procfs, and readable kernel logs must already
be available. `dmesg` and `journalctl -k -b` are captured before and after. The
PMIC read uses `vcgencmd pmic_read_adc EXT5V_V`, matching the repository's Pi
diagnostics.
The checker never escalates privileges. Missing tools, denied access or absent
sensor evidence produce BLOCKED unless another observed failure takes precedence.
Supply negotiation uses the device-tree property if present; firmware exposing
only a USB-current override cannot establish the 5 A criterion.

## Criteria and configuration

All numeric policy and timing defaults live in [thresholds.yaml](thresholds.yaml),
a JSON-form YAML 1.2 document read with `json` to avoid PyYAML installation.
Keep this format when editing. Each threshold has a `source` or `rationale`;
chosen bench limits are **PROPOSED**. `--thresholds PATH` selects another config.
`--duration` and `--interval` overrides are written into the effective config
with an operator-override rationale. Capture-time config is also preserved in
`raw/`. Device IDs and the kernel pattern list are in the same config.

| Criterion | Meaning |
|---|---|
| `raw_integrity` | Raw file bytes match the saved capture SHA-256 manifest; missing/changed bytes block qualification. Hashes do not authenticate who created the capture. |
| `under_voltage` | FAIL on bit 0 in any sample or a bit 16 transition from clear to set. Missing/malformed samples or cleared history block. All current bits 0–3 and historical bits 16–19 are decoded through the existing resource sampler. |
| `ext5v` | FAIL below the configured sampled EXT5V_V minimum (default 4.75 V, PROPOSED); missing PMIC readings in any sample block. Supports `EXT5V_V volt(24)=...V`, `voltage(24)` and plain `EXT5V_V=...V`. `measure_volts` is recorded as a core-rail observation and never substituted for the 5 V input. |
| `temperature` | FAIL at or above 80 °C. Raspberry Pi says Pi 5 begins throttling when temperature reaches 80 °C ([official Pi 5 article](https://www.raspberrypi.com/news/heating-and-cooling-raspberry-pi-5/)). Max combines `measure_temp` and every thermal-zone reading; both sources are required in each sample. Missing/malformed values block. |
| `window_coverage` | Requires a complete declared window, before/during/after samples in order, UTC labels, and no gap above the configured host-monotonic gap limit. Interruptions or missing clocks block. |
| `supply` | With `require_5a: true` (PROPOSED default), FAIL for a device-tree declared current below 5000 mA or an explicitly disabled USB-current flag. Missing negotiated-current evidence blocks even if the flag is 1. The property is interpreted as a big-endian u32 in mA; textual values are not accepted. Missing/unsupported `get_config` is tolerated when the device-tree declaration is available. With `require_5a: false`, observations remain in the result but this requirement is waived. |
| `usb_link` | Reads VID/PID, speed, serial, bus number, device number, port path and sysfs path before/after. FAIL if any matched D435i reports less than 5000 Mbit/s. Missing or ambiguous matching cameras, missing speed, unreadable identity, disappearance, or changed before/after enumeration blocks. Serial numbers are recorded, not assumed to identify the SDK serial. |
| `usb_topology` | Requires readable, nonempty `lsusb -t` before and after. Speed acceptance uses the matched sysfs device, not a root hub or another device in the tree. |
| `kernel_log` | FAIL on configured reset/disconnect, UVC/xHCI errors, under-voltage or lost-message patterns in the new kernel lines. Matches include text, line number and categories. Attribution is conservative across the whole board. Incomplete logs block. |
| `load_window` | Idle is explicitly declared; a requested load needs a successful launch, adequate observed lifetime and an acceptable exit. Early exit or natural nonzero exit fails; launch failure/missing exit evidence blocks. |

Overall precedence is **FAIL > BLOCKED > PASS**. Exit codes are **0 PASS,
1 FAIL, 2 BLOCKED, 3 usage/internal error**. Completed analyses always write
`result.json` and print a short criterion table. Invalid arguments, malformed
config/capture and output errors return 3; such errors may prevent a result.
Existing output directories are refused.

Bit definitions cite the [Raspberry Pi get_throttled documentation](https://www.raspberrypi.com/documentation/computers/os.html#get_throttled).
They are also cited in `tools/runtime_observer/README.md`. The 4.75 V bench
threshold is **PROPOSED**, not a claim about the firmware detector trip point.
ADC accuracy and the installed firmware's negotiation reporting are **UNKNOWN**.
The Pi's USB-current setting can be overridden in configuration, so it is never
used alone to infer a 5 A PSU. A device-tree declaration is software evidence,
not a measurement of the supply's actual capability or cable voltage drop.

Historical bit 16 already set at baseline is recorded with a note and does not
by itself fail this window. It cannot latch again, so another historical-only
event cannot be distinguished; sampled current bits, PMIC voltage and kernel
messages remain active checks. Frequency-capping, throttle and soft-temperature
bits are recorded but do not independently fail this task. A PASS cannot
claim the board never throttled. A stricter timing benchmark needs its own
acceptance policy.

## Clocks and kernel boundaries

Each periodic sample and each diagnostic command/read carries host
`clock_monotonic_ns` (**Linux CLOCK_MONOTONIC** on the Pi) and
`clock_realtime_utc` (**CLOCK_REALTIME**, rendered as UTC). Sample start stamps
bracket sequential, non-atomic diagnostic reads; command/read completion stamps
are retained. The load window and timing calculations use CLOCK_MONOTONIC only.
UTC may jump without changing durations. These fields are not D435i hardware or
global time, ROS `header.stamp`, or an MCU timer; no mapping to those clocks is
made or needed.

Kernel records retain the native kernel/journal boot-relative timestamps.
No kernel timestamp is subtracted from a host or UTC timestamp. Analysis chooses
a source whose after snapshot retains the entire before snapshot as an exact
ordered prefix and verifies equal boot IDs. Only the suffix is classified.
`journalctl` is a fallback if `dmesg` is unavailable or its prefix was lost.
Unrecognized timestamps, inaccessible logs, missing boot identity, or rotation/
truncation without a complete fallback produce BLOCKED. Existing boot faults
are excluded; the observed kernel interval conservatively encloses setup,
before sample, load/idle window, after sample and teardown. Therefore a fault
just outside the load itself can still fail the run. Log loss not reported by
the kernel cannot be independently ruled out.

## Evidence and replay on Mac or Pi

A new output directory contains:

- `capture.json`: original host, clocks, command return codes/errors, file references and raw hashes.
- `raw/`: verbatim stdout/stderr bytes, binary device-tree property, sysfs/procfs reads and capture config.
- `thresholds.yaml`: effective analysis config, including any operator overrides.
- `analysis.json`: decoded samples, USB/supply observations and coverage/integrity details.
- `kernel-window.log` and `kernel-matches.json`: derived suffix and categorized faults.
- `result.json`: the common check contract, criteria with threshold sources, raw-file hashes and notes.

Replay never runs diagnostic commands or the saved load, and does not change
its input. Specify a separate new output directory:

```sh
python3 lab/power_usb/check_power_usb.py \
  --analyze-only lab/power_usb/runs/load-20261018 \
  --out lab/power_usb/runs/load-20261018-reanalysis

# Portable historical partial evidence: expected exit 2 / BLOCKED.
python3 lab/power_usb/check_power_usb.py \
  --analyze-only lab/power_usb/tests/fixtures/real-partial \
  --out lab/power_usb/runs/historical-partial-reanalysis

PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s lab/power_usb/tests -v \
  > lab/power_usb/tests/last-run.log 2>&1
```

Replay defaults to the saved effective config. `--thresholds PATH` allows an
explicit policy comparison while retaining the capture-time config/raw evidence.
No sampling or load arguments are allowed in replay mode. Capture host/times
remain the original observations; `analysis_started_utc` identifies reanalysis.
Keep input capture and raw files together and immutable. Review raw logs and
matches before using this diagnostic result in a larger lab decision.
