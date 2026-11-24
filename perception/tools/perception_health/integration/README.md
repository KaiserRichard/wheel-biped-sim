# VIO snapshot → perception health integration

This software prototype converts `tools/vio_adapter` snapshot JSONL to the
existing phase-7 sender's pose events, sends real v3 frames through an `os.openpty()`
pair, and feeds the existing supervisor model. Python 3.11+ is required (3.12
on the Pi); NumPy is needed only for the existing `tools/clockmap` implementation.
No package installation, camera access, UART hardware, or firmware is required
for the offline checks. All new files belong to this directory.

**MEASURED software acceptance: PASS.** The integration uses the reviewed phase-7
clock label, shared wire sequence and downgrade-only policy. Unavailable pose age
stays HOLD while reports and heartbeats continue; independent watchdogs can still
latch FAILSAFE. The phase-7 model's validity handling was repaired with regression
tests; see [REPORT.md](REPORT.md). Passing fixtures is **not a gate PASS**.

## Run and re-evaluate on the Mac or Pi

From the repository root:

```bash
python3 -B -m unittest discover -s tools/perception_health/integration/tests -v
# Equivalent unittest entry point:
python3 -B tools/perception_health/integration/e2e_pty_test.py -v

# Full captures stay temporary; these commands exercise PTYs only, also on Pi:
capture_root="$(mktemp -d /tmp/wbr-healthint.XXXXXX)"
python3 -B tools/perception_health/integration/e2e_pty_test.py \
  --outdir "$capture_root/full" --summary-outdir "$capture_root/summary"

# Read the FULL saved files again; no tty or sender subprocess is opened:
python3 -B tools/perception_health/integration/e2e_pty_test.py \
  --analyze-only "$capture_root/full"

# Inspect saved real associations without claiming real VIO estimates:
python3 -B tools/perception_health/integration/vio_to_pose_events.py \
  --snapshot-session saved-real-replay \
  --input tools/vio_adapter/real_replay/snapshots.jsonl
```

The check prints a short table and throughput comparison, and writes the common
`result.json` contract, `metrics.json`, input hashes, raw `uart.bin`, raw sender
stdout/stderr, read boundaries, converted events, snapshots and model traces.
Exit codes: 0 PASS, 1 FAIL, 2 BLOCKED (missing capture/dependency), 3 usage/internal
error. Existing output directories are refused. The complete fixture check exits
0 with all ten criteria PASS. The suites cover PASS, corrupted-raw FAIL and
missing-capture BLOCKED paths; no expected-failure markers remain.

Committed `evidence/` contains only `summary.json` and `sha256.json`, under the
200 kB task budget configured in `config.json`. The summary embeds the full
check result, metrics, input provenance and policy, plus state/reason counts;
the manifest records every original output's size and SHA-256, including all
445 real-replay events. Full outputs are regenerated in temporary directories
by tests. Summary-only archives cannot be passed to `--analyze-only`; that mode
requires the full captures made by the commands above. Input hashes identify
the saved fixtures; new PTY runs have different timing/output hashes.

The analyzer reconstructs frames from raw bytes and recorded read timestamps;
it does not trust a saved `trace.json`. Its checks cover:

| Criterion | Meaning |
|---|---|
| `<scenario>.transport` | Sender exits successfully; every event becomes a frame; frame counts, total bytes, CRC, parser completion and shared sequences agree. |
| `no_mapping` | Every synthetic OK report must yield HOLD when mapping is absent. Age flags remain unavailable; reason SOURCE_AGE_HOLD. |
| `mapped_ok` | Explicit synthetic exchanges, exact OK, known camera IMU/reset/features plus independent primary freshness produce FRESH. |
| `lost_latched` | Exact LOST produces FRESH → FAILSAFE → FAILSAFE, including a subsequent OK report. |
| `sender_stops` | EOF stops the sender; advancing the simulated MCU timer produces HOLD/RESULT_SILENCE then FAILSAFE/RESULT_TIMEOUT. |
| `real_replay` | All saved associations pass through the sender, with unknown mapped age and degraded flags, and never become FRESH. |

Policy scenarios explicitly assume `primary_fresh=True` from an independent
primary supervisor input; this assumption is recorded in traces and result notes.
`replay()` defaults to `False`, and a regression test confirms healthy health
frames alone cannot produce FRESH. Decoded shared wire sequences are supplied
to the model on both HEARTBEAT and health events. LOST uses the reviewed upstream
reason `SOURCE_AGE_TIMEOUT`.

The receipt-silence experiment uses diagnostic policy overrides from
`config.json` to postpone the competing pose-age and heartbeat deadlines. The
default phase-7 policy has coincident timeout values and a source-age HOLD before
receipt-silence HOLD. Default-policy traces are used for the other scenarios.
No elapsed hardware deadline is claimed: silence ticks use an independent
**simulated MCU-local timer**, derived from local host CLOCK_MONOTONIC observation
durations. Pi send epochs are never subtracted from those timer values.

## Conversion and unavailable fields

Input is the flat JSONL schema emitted by `vio_adapter/replay_fake.cpp`:
`source_ns`, `domain`, `sequence`, `state`, `valid`, `reasons`, `reset_counter`,
`epoch_id`, `map_id`, and optional pose/covariance fields. Source epochs remain
integer camera ns; `global_time` means D435i SDK global time, **not** Linux
monotonic time or ROS header.stamp. Poses/covariance are not transmitted by this
health message. Null source remains unavailable; its sender-required numeric
placeholder cannot participate in age arithmetic because the mapping is invalid.

The adapter loses exact ORB names in `orb_engine.cpp`. The converter therefore
supports a proposed optional `orb_tracking_state` extension containing the exact
ORB **name**, checked against the adapter state:

| Adapter state | Possible exact ORB names | Conservative output without extension |
|---|---|---|
| STARTING | SYSTEM_NOT_READY, NO_IMAGES_YET | SYSTEM_NOT_READY + degraded |
| INITIALIZING | NOT_INITIALIZED | NOT_INITIALIZED |
| TRACKING | OK, OK_KLT | OK + degraded |
| LOST | RECENTLY_LOST, LOST | LOST + degraded (conservative FAILSAFE) |
| FAULT/unknown | unavailable | SYSTEM_NOT_READY + degraded |

All seven exact names are preserved when supplied; these are looked up through
the existing phase-7 codec, rather than treating native ORB numeric enum values as phase-7
wire enum values. A missing exact name is explicitly marked `tracking_exact=false`;
recovering it from the current collapsed snapshot is impossible. LOST also names
an adapter state, so an exact LOST requires the extension to remove ambiguity.

`map_reset_counter` uses `reset_counter`, otherwise `epoch_id`, modulo the wire
u16 range. Adapter counters are unbounded within their object lifetime; a
regression is rejected. If only `map_id` exists, a local observed-change counter
starts at zero and increments on each known map-ID transition. It remains
degraded: hidden resets/merges and history before the baseline are UNKNOWN.
Without any of these fields the counter is omitted and flagged unavailable.
phase-7 serializes its default zero in that case; the degraded wire flag prevents
interpreting that placeholder as a known reset history.

Camera IMU initialization is copied only from an explicit boolean
`imu_initialized`; no tracking-based initialization is invented. It describes
the **D435i camera IMU**, never the STM32 body IMU. `tracked_features` is copied
only when reported, saturating to the u16 wire maximum. Its omission is recorded
as unavailable and degraded; phase-7's wire zero is a placeholder. Existing saved
snapshots omit both fields. Invalid poses, nonzero adapter reason bits and
unknown/ambiguous fields set `degraded`, which becomes phase-7's existing wire flag.
Extra `integration` metadata stays in JSONL and is ignored by the sender.

`--snapshot-session` must identify the producer/adapter lifetime and camera-clock
session. Source reversal, domain changes or lifetime-counter regressions stop
conversion with exit 3; start a new producer/converter session after recovery.
Repeated timestamps are preserved for adapter heartbeat/reset snapshots, so
repeated reports never renew source age. Live serializers must deliver reset
snapshots as well as fresh tracking results; the current fake replay driver
does not provide a live runtime.

## Explicit clockmap input

Only `--clockmap FILE` enables mapping. Event-embedded mappings in input snapshots
are ignored. The JSON file needs `valid: true`, `snapshot_domain` (exactly the
snapshot's domain string), `camera_domain` (the declared ClockMap source label),
`pi_domain: "CLOCK_MONOTONIC_RAW"`, `camera_session`, `pi_session`, and
`exchanges_ns` with ordered `[t1_camera,t2_pi_raw,t3_pi_raw,t4_camera]` integer-ns
rows. The supplied `camera_session` must match `--snapshot-session`. Domain and
session mismatches disable mapping for the event. A malformed file or unusable
fit is an error. No receipt time or wall clock is substituted for camera time.

`load_mapping()` reuses `tools.clockmap.ClockMap`, `Exchange` and `Stamp` to
validate exchanges with the same configured defaults used by phase-7. phase-7 then uses
its existing mapping function and adds its uncertainty to the conservative age
at the software send boundary. Without a file, events carry `mapping.valid=false`;
phase-7 clears age/mapping validity flags and writes UINT32_MAX as unavailable age.
No real D435i exchanges are available here. The generated synthetic files are
**ASSUMED**, explicitly labeled, and never applied to saved real epochs.

The file is a static, caller-qualified session-bound mapping, not a synchronization
service. The caller must establish the actual Pi raw-clock boot identity, camera
session, capture semantics, freshness interval and invalidation/restart policy.
Neither snapshots nor phase-7 independently authenticate those declarations. Replace
the map and restart the converter/sender after clock/session changes; static
exchanges cannot detect an unobserved later clock step. This remains unverified
for live operation.

## Exact future Pi pipeline

**Prerequisites, not implemented here:** a reviewed headless live runtime using
the optional real ORB binding writes the documented snapshot schema, including
reset notifications, into `/run/wbr/vio/snapshots.fifo`; it writes its current
camera/adapter session ID into `/run/wbr/vio/session-id`. A separately qualified
clockmap producer provides `/run/wbr/vio/camera-to-pi-raw.json` bound to that session
and the current Pi boot's CLOCK_MONOTONIC_RAW. Real ORB state and camera-IMU/feature
extensions are needed to avoid the deliberate degraded state. The repository
currently has no such live executable; `replay_fake` always uses a fake engine.

After receiver/firmware agreement and those prerequisites, run from the repo root
on the Pi (UART path/baud must match the agreed deployment):

```bash
set -euo pipefail
python3 -B tools/perception_health/integration/vio_to_pose_events.py \
  --input /run/wbr/vio/snapshots.fifo \
  --snapshot-session "$(cat /run/wbr/vio/session-id)" \
  --clockmap /run/wbr/vio/camera-to-pi-raw.json \
  | python3 -B tools/perception_health/py/sender.py \
      --device /dev/ttyAMA0 --baud 115200
```

Omitting `--clockmap` is the honest diagnostic mode when no qualified exchanges
exist; fresh OK reports remain HOLD/SOURCE_AGE_HOLD, with receipt/heartbeat
watchdogs still active. This command is documented only; no device was opened. Real ORB binding execution, Pi runtime
serialization, UART hardware, STM32 firmware parsing and supervisor integration
remain **UNKNOWN / unverified**. STM32 retains balance and safety ownership.

## Throughput interpretation

`metrics.json` reports **MEASURED on this Mac** frames/s and bytes/s actually
written by the sender and received intact on the PTY, including HEARTBEAT frames.
The observation window runs from process launch through EOF exit, including
imports/startup. The replay is an unpaced burst, not the original capture cadence.
Rates vary with host scheduling; a PTY does not throttle to its configured baud.
The raw read timing and counts are retained for recalculation.

**CALCULATED** ideal UART capacity for 8N1 is baud / 10: 11,520 bytes/s at
115200 and 92,160 bytes/s at 921600. Measured utilization is reported against
each capacity; this is a comparison, not a performance acceptance criterion.
Above-capacity bursts require pacing/backpressure in a real implementation.
Physical throughput, transit latency, power/thermal/storage behavior, exposure
timestamp semantics and all Pi/safety gates remain unverified.
