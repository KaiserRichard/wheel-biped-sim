# D435i raw stereo + camera IMU qualification

Software-only implementation of phase-2. **UNKNOWN: physical qualification of current
SDK serial 99005. No hardware gate PASS is claimed.** A check PASS means
only its configured, **PROPOSED** raw-stream criteria passed on the supplied
evidence. A fixture PASS is not a gate PASS. This does not replace the project's
power, storage, kernel/USB monitoring, historical N2 soak, or ROS record/replay gates.

Python 3.11+ and the standard library suffice for analysis. Live capture on the
Pi also needs the **already-installed** `pyrealsense2` binding and librealsense;
missing bindings/device/profiles produce BLOCKED. There is no installer, build,
GUI, ROS dependency, SSH command, firmware write or calibration write here.

## Exact Pi commands

Run locally as the normal camera user from the repository root after the lab's
hardware prerequisites and camera availability have been established. Stop
other camera users first. Put the camera on a still, rigid surface for the full
measurement. `--static` records the operator's confirmation of that condition;
it is required for the static criteria, and does not establish stationarity by
itself. Choose a **new** output directory on the lab's qualified evidence volume:

```bash
python3 lab/d435i_raw/check.py --capture --static \
  --outdir <local workspace>/wbr-evidence/d435i-raw-20261018-01
```

Default capture: IR left/index 1 and right/index 2, **848x480 Y8 at 30 Hz**,
emitter **OFF**, gyro **200 Hz**, raw accel **100 Hz**, falling back to native
**200 Hz** only when 100 Hz is not offered. All enabled and offered profiles are
recorded. It enables exactly these four streams, without frame synchronization,
interpolation, decimation or a combined IMU topic. Direct sensor callbacks avoid
filtering motion delivery through a video frameset queue. This is the **D435i
camera IMU (`imu_camera`), never the STM32 body IMU**.

The default measurement window is **60 s**. SDK opening/closing is outside that
window. A parent process bounds the entire child to requested duration plus the
configured **20 s** grace. A timeout kills only that child, retains partial
metadata/raw logs and yields FAIL. The callback-to-writer queue is bounded;
overflow, callback exceptions, write errors and cleanup errors cannot pass.

```bash
# Longer capture, with the same criteria (maximum 600 s):
python3 lab/d435i_raw/check.py --capture --static --seconds 120 \
  --outdir <local workspace>/wbr-evidence/d435i-raw-20261018-02
# Custom reviewed thresholds/profile preference; no YAML package required:
python3 lab/d435i_raw/check.py --capture --static \
  --config lab/d435i_raw/thresholds.yaml \
  --outdir <local workspace>/wbr-evidence/d435i-raw-20261018-03
```

The check selects only the configured SDK serial. If it is absent, enumeration
serials are retained and no other camera is streamed. Firmware/USB mismatch is
FAIL before stream opening. Unsupported profiles are BLOCKED. It sets emitter
OFF before opening IR and records readback; normal cleanup restores its previous
value and verifies restoration. It leaves global-time and motion-correction
settings untouched. DDS discovery is explicitly disabled in the SDK context.
Forced termination cannot guarantee option restoration; such a run fails.

## Re-evaluate saved evidence on Pi or Mac

No camera or SDK import occurs in this mode. Input evidence is read only; output
must be a separate new, non-nested directory. Saved capture configuration is
used by default, including its threshold sources. An explicit `--config` applies
reviewed new thresholds and retains both configurations for comparison. Replay
cannot add a new static confirmation or change the captured duration.

```bash
python3 lab/d435i_raw/check.py --analyze-only \
  <local workspace>/wbr-evidence/d435i-raw-20261018-01 \
  --outdir <local workspace>/wbr-evidence/d435i-raw-20261018-01-analysis
# Portable historical fixture: expected exit 1 (FAIL), never a gate PASS:
python3 lab/d435i_raw/check.py --analyze-only \
  lab/d435i_raw/tests/fixtures/old-unit-smoke \
  --outdir /tmp/d435i-old-unit-analysis-20261018-01
```

All criteria and sources live in [thresholds.yaml](thresholds.yaml), encoded as
JSON, a valid YAML 1.2 subset, to avoid a PyYAML dependency. Every threshold has
a `source`; chosen acceptance limits explicitly say **PROPOSED**. Do not promote
those choices into approved lab acceptance limits without review.

## Results and criterion meanings

Every completed evaluation writes `result.json`, `metrics.json`, a short console
table and SHA-256 hashes of the copied raw inputs and configurations. `result.json`
contains check/version, UTC start/end, analysis host, per-criterion measured
values, units, thresholds, sources and local evidence paths. Its status is FAIL
if any criterion fails, otherwise BLOCKED if any criterion lacks evidence,
otherwise PASS. Exit codes: **0 PASS, 1 FAIL, 2 BLOCKED, 3 usage/internal error**.
Invalid CLI/configuration, an existing output directory and an unreadable output
location are usage/internal errors. Malformed saved samples/metadata fail their
evidence criterion; absent evidence blocks it.

| Criterion | Interpretation |
|---|---|
| Identity | Exact SDK serial and firmware, SDK USB descriptor matching 3.x. ASIC/USB descriptor serial is not substituted for SDK serial. The descriptor alone does not independently prove physical negotiated speed or absence of resets. |
| Profile, emitter, duration | Exactly the configured stereo Y8 + separate motion profiles, OFF option readback, and at least the configured qualification duration. A shorter diagnostic take fails the default 60 s qualification limit. |
| Sample accounting | Per-stream callback counts equal saved rows; the finalized capture has no overflow/errors and an orderly shutdown. |
| Rate | All received rows divided by the declared host CLOCK_MONOTONIC window, within 5% of the resolved nominal rate by default. No selected interior window can rescue a startup/trailing stall. |
| Drops | Sum of positive frame-number gaps and count of nonincreasing frame numbers, independently per stream. This cannot detect a frame lost before the first received frame or after the last without rate/boundary checks. |
| Receipt gaps/jitter | Maximum host CLOCK_MONOTONIC inter-arrival gap including both window edges; mean and population standard deviation of interior intervals. Default limits: IR 0.1 s, motion 0.05 s. These are callback arrivals, including SDK/Python scheduling. |
| Timestamp sanity | Count duplicates, reversals and domain transitions. Compare each stream's SDK stamps only within the same reported domain, and available hardware counters only within their named source. Host CLOCK_MONOTONIC and CLOCK_REALTIME are checked independently. Default: zero duplicates, reversals or domain transitions. No cross-clock subtraction. |
| Stereo | Pair left/right by frame number, require equal known SDK domains, compare timestamps within 0.01 ms by default, and require no unpaired frames. Duplicate frame numbers also fail stream continuity. No comparison is performed across unequal domains. |
| Static gyro | With saved operator confirmation: norm of the mean XYZ vector <=0.05 rad/s and maximum per-axis population std <=0.03 rad/s by default. |
| Static accel | With saved operator confirmation: mean of sample vector norms within 0.5 m/s² of configured local g (default standard gravity 9.80665 m/s²), maximum per-axis population std <=0.2 m/s². |
| IR content | Three raw PGM samples per side, spread across the window. Recompute Y8 mean/min/max and fraction equal to 255 from pixels. Default mean [1,254], saturated fraction <=0.95. Missing samples block/fail as recorded; malformed referenced pixels fail. |
| Latency policy | PASS means the check withheld unmapped cross-clock latency. The latency value itself is explicitly **UNKNOWN/null**, and is not credited as a measured latency PASS. |

Static statistics are **plausibility checks, not calibration**, Allan noise
densities, scale/axis/bias estimates, or a guarantee that the camera was still.
Image statistics exclude trivial black/white/mostly saturated samples; they do
not establish focus, stereo calibration, scene responsiveness, image integrity
between sampled frames or geometric correctness.

## Raw schema and clock policy

Live artifacts: `request.json` (exact argv/config hash), `capture.json` (identity,
profiles, timing window, static confirmation, counts and cleanup), `samples.csv`,
`sdk.log`, verbatim `capture.stdout` / `capture.stderr`, `capture_config.yaml`,
`analysis_config.yaml`, and sampled `ir_left-*.pgm` / `ir_right-*.pgm` files.
Timeouts additionally retain `timeout.json` and any `capture.partial.json`.
Pixels are unpadded Y8 in binary PGM. At defaults, six images total approximately
**CALCULATED 2.44 MB** plus CSV/logs; no full image stream or bag is stored.

CSV columns:

```text
stream,frame_number,sdk_timestamp_ms,sdk_timestamp_domain,hardware_timestamp_ms,hardware_timestamp_source,sdk_global_system_timestamp_ms,host_clock_monotonic_ns,host_clock_realtime_ns,x,y,z,image_file
```

- `sdk_timestamp_ms`: SDK `get_timestamp()` milliseconds, **only** in its
  reported `hardware_clock`, `global_time` or `system_time` domain.
- `hardware_timestamp_ms`: SDK hardware-domain timestamp when that is the
  reported domain; otherwise SDK FRAME_TIMESTAMP metadata in microseconds /1000
  when offered. The source says which. Metadata is a raw device counter, with no
  unwrap, global conversion or epoch equivalence assumed. Unsupported hardware
  timestamps are blank with an explicit UNKNOWN reason. A raw counter wrap or
  reversal conservatively fails timestamp sanity; no counter width is guessed.
- `sdk_global_system_timestamp_ms`: `get_timestamp()` only when the SDK reports
  global/system time, otherwise blank. Its exact domain is the SDK domain field.
- `host_clock_monotonic_ns`: Linux CLOCK_MONOTONIC at callback entry, before
  parsing, locking or file writing. `host_clock_realtime_ns` is a separate,
  subsequent host CLOCK_REALTIME read. Neither is a device timestamp or exposure
  time. They are never subtracted from a device epoch here.
- XYZ preserves SDK float values, in rad/s for gyro and m/s² for accel. IR rows
  leave XYZ blank. Frame numbers and host nanoseconds retain integer precision.

The local [clockmap documentation](../../tools/clockmap/README.md) explains that
SDK global time maps to OS system time, not host monotonic time. Arrival fits
absorb arrival delay and cannot establish absolute latency. This capture records
**no qualified device-to-host mapping** and accepts no fabricated mapping from
metadata, so every analysis reports latency UNKNOWN and only same-clock receipt
jitter. It does not reimplement `tools/clockmap`. Future mapped latency needs a
separately documented, session-bound mapping and uncertainty through that tool.
Unavailable hardware clocks remain unavailable; monotonicity is only proven for
observed domains. Forward clock steps hidden by monotonicity remain UNKNOWN.

## Reuse, tests and evidence limits

[Existing sdk_analysis.py](../../lab/sensor_validation/sdk_analysis.py)
is loaded by import: its `summarize()` supplies arrival/frame continuity counts,
and its strict `read_capture()` parses the historical vendor fixture. None of
its old profile constants or acceptance limits are used for new qualification.
The prior `run_sdk_capture.py`, `sdk_capture.py`, `analyze_sdk_csv.py`,
`camera_metadata.cpp`, `sample_ir_checked.cpp` and the
[Allan recorder](../../calibration/allan/d435i_imu_record.cpp) were inspected.
They informed profile verification, raw domains, callbacks and cleanup; their
fixed old-unit serial/profile/runtime preconditions are not used for this run.
The old PGM reader fixes dimensions to 640x480; this directory parses only its
own exact 848x480 capture format, without modifying existing tools.

Historical `frames.csv` + `command.json` can be analyzed diagnostically. The
included **MEASURED old-unit 99003** fixture is byte-identical, with
[provenance](tests/fixtures/old-unit-smoke/PROVENANCE.json). It lacks timestamp
domains, host monotonic stamps and right IR. Vendor-relative timing summaries
remain separately named; missing clocks are BLOCKED, never fabricated. Its
wrong serial and old IR profile FAIL current criteria. See
[fixture notes](tests/fixtures/README.md).

```bash
python3 -m unittest discover -s lab/d435i_raw/tests -v \
  > lab/d435i_raw/tests/last-run.log 2>&1
```

Tests exercise PASS/FAIL/BLOCKED, all CLI result codes, byte-preserving replay,
old-unit evidence, timing loss and boundaries, clock/domain faults, stereo,
static plausibility, image corruption, missing SDK/device/profiles, overflow,
SDK shutdown failure, parent timeout and an end-to-end **simulated** SDK capture.
No hardware is needed or accessed. Test-generated files stay under `tests/` and
are cleaned up. [REPORT.md](REPORT.md) records final verification and open risks.
