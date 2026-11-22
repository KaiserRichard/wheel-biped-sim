# Calibration recording check

Qualify **one saved D435i take** before expensive Kalibr or Allan processing.
`check_calib_take.py` uses Python 3.11+ standard library only (Pi: Python 3.12).
It never starts a recorder, opens a sensor, connects to a Pi, or runs Kalibr.
Keep this directory in its repository layout: it imports the existing
[take_tools](../../tools/take_tools/README.md) decoder/timing/stereo functions
and [take_diagnostics](../../calibration/lab/pi/take_diagnostics.py).
Those tools and everything under `calibration/` are unchanged.

**A fixture PASS is tooling verification, never a physical gate PASS.**
Camera–IMU results currently remain at least BLOCKED because the existing tools
provide no AprilGrid detection counts. A numeric blur proxy cannot replace target
detection. Allan can PASS these recording checks with complete saved receipts;
that does not establish
stationarity, noise parameters, power quality, temperature stability or calibration.
This tool concerns the D435i IMU sensor `imu_camera`, never the STM32 `imu_body`.

## Run on the Pi (saved files only)

Run from a checksummed repository-layout bundle already available on the Pi.
No installation or ROS/SDK Python bindings are required. The following commands
only analyze finalized recordings; physical acquisition remains governed by
[the existing recorder instructions](../../calibration/lab/README.md) and
[Allan instructions](../../calibration/allan/README.md).

```bash
set -euo pipefail
cd "$HOME/wbr-runtime/calib-check"  # actual repository-layout bundle root
python3 lab/calib_record/check_calib_take.py \
  --take-dir "$HOME/wbr-evidence/2026-10-15-calib-C1" \
  --kind camera-imu --outdir "$HOME/wbr-evidence/check-C1-01"
python3 lab/calib_record/check_calib_take.py \
  --take-dir "$HOME/wbr-evidence/allan-stationary-15h" \
  --kind allan --outdir "$HOME/wbr-evidence/check-allan-01"
```

Exit codes: **0 PASS, 1 FAIL, 2 BLOCKED, 3 usage/internal error**. Shell `set -e`
stops after a FAIL/BLOCKED, so run each example independently. A new output
directory outside the take is mandatory. Failed output directories are retained.
Symlinks anywhere inside a take are refused. The input is hashed before and
after analysis; changes cause FAIL. No source files are modified.

Mac or Pi reanalysis uses exactly the same saved raw take and policy:

```bash
python3 lab/calib_record/check_calib_take.py \
  --analyze-only /absolute/path/to/saved-calib-C1 \
  --kind camera-imu --outdir /absolute/path/to/new-check-C1-02
python3 lab/calib_record/check_calib_take.py \
  --analyze-only /absolute/path/to/saved-allan-take \
  --kind allan --outdir /absolute/path/to/new-check-allan-02 \
  --config lab/calib_record/criteria.json
```

`--take-dir` and `--analyze-only` are equivalent saved-file modes. Reanalysis
does not read the Mac's current free space and mistake it for Pi capture evidence.
The raw take must remain available; a result directory alone is not an archive
of the potentially gigabyte-scale recordings.

## Input contracts and supplemental receipts

Camera–IMU accepts the wrapper's session directory:

```text
SESSION/record.log                 actual SDK serial/FW/profile/emitter readback and stop receipt
SESSION/ram-sha256.txt             existing recorder payload hash manifest
SESSION/storage.json              supplemental saved free-space receipt (see below)
SESSION/take/stereo_imu.db3        finalized uncompressed SDK SQLite/CDR recording
SESSION/take/timestamps.csv        original callback audit, full precision
SESSION/take/sdk.log
SESSION/power-1hz.log              optional, preserved and given to existing diagnostics
```

A flat directory is also supported: the three recorder files, `record.log`,
`storage.json`, and `SHA256SUMS` at its root. The checksum manifest must cover
the three recorder files. Profile requests alone do not count as readback.
IR stream labels `Infrared` plus index or `Infrared 1` / `Infrared 2` are accepted.
An interrupted/failed wrapper receipt fails `recorder_stop`.

Allan accepts the existing recorder directory:

```text
TAKE/gyro.csv                     sdk_ts_ms,domain,frame_number,host_steady_ns,x,y,z
TAKE/accel.csv                    same exact original header; native rates, no interpolation
TAKE/meta.json                    existing recorder identity/resolved profiles/stop/counters
TAKE/sdk.log
TAKE/storage.json                supplemental saved free-space receipt
TAKE/emitter.json                supplemental actual emitter readback receipt
TAKE/SHA256SUMS                  supplemental finalized payload hash manifest
```

`meta.json` must identify the expected SDK serial/FW, `imu_camera`, no serial
bypass, and exactly gyro/accel MOTION_XYZ32F profiles. Callback counts, writer
counts and CSV counts must agree. Clean SIGINT/SIGTERM finalization is permitted
if the recorded duration is sufficient. Queue overflow or recorder errors fail.
The existing recorder enables only motion streams. That is not a physical
emitter readback. Allan's `emitter_state` remains BLOCKED unless saved actual
option readback is available in `emitter.json`, with `enabled_readback: 0`,
`sdk_device_serial`, `source`, `observed_utc`, and `clock: linux_clock_realtime`.
Preserve the raw readback in the session logs and identify it in `source`.
Do not invent this receipt from a profile request or enable an optical stream
to obtain it. This checker does not query options or alter the recorder.
Camera–IMU requires the existing recorder's actual OFF readback in `record.log`.

The old wrappers do not save post-capture free space, Allan emitter readback,
or an Allan payload manifest.
Missing receipts yield BLOCKED, never an invented PASS. On the **capture Pi**, after
normal finalization and before freezing/copying the session, save this supplemental
capacity receipt. Set `WBR_TAKE_DIR` to the outer camera session or Allan directory:

```bash
set -euo pipefail
export WBR_TAKE_DIR="$HOME/wbr-evidence/allan-stationary-15h"
python3 - "$WBR_TAKE_DIR" <<'PY'
from datetime import datetime, timezone
import json
from pathlib import Path
import platform
import shutil
import sys
take = Path(sys.argv[1]).resolve(strict=True)
receipt = {"free_bytes_after": shutil.disk_usage(take).free,
           "observed_utc": datetime.now(timezone.utc).isoformat(),
           "clock": "linux_clock_realtime", "host": platform.node(),
           "source": "Pi post-finalization shutil.disk_usage on capture directory"}
with (take / "storage.json").open("x") as f:
    json.dump(receipt, f, indent=2)
    f.write("\n")
PY
# Allan only; the camera wrapper already supplies ram-sha256.txt.
(
  set -euo pipefail
  set -o noclobber
  cd "$WBR_TAKE_DIR"
  sha256sum gyro.csv accel.csv meta.json sdk.log storage.json > SHA256SUMS
)
```

These commands create receipts without changing recorder data. Do not fabricate
a Pi receipt on a copied Mac directory. A manifest created later identifies the
files at that time; it cannot retrospectively attest capture-time integrity.
For an already frozen historical take, keep missing evidence BLOCKED. The tool's
new `input-manifest.json` covers all files but does not replace a missing original
copy-verification manifest. Existing manifests use standard `sha256sum` lines,
relative paths only; duplicate, traversing, missing or mismatched entries fail.

## Criteria and policy

Every numeric acceptance threshold, expected profile and unit identity lives in
[criteria.json](criteria.json), with a source/rationale and evidence label.
Custom policies require the same complete schema and per-value sources.
Reversals, duplicates, domain transitions and drops cannot be permitted.

| Criterion | Meaning |
|---|---|
| Completeness / checksums | Nonempty expected files; verify original hashes and required-file coverage; new full inventory; input unchanged throughout analysis. Hashes identify bytes, not authenticity. |
| Metadata / stop / counts | Actual identity, FW, resolved profiles; emitter OFF readback; normal finalization; Allan callback/writer/CSV count agreement. ASIC, firmware-update and USB serials are not substituted for SDK serial. |
| Duration | **ASSUMED project** camera interval 60–120 s, Allan provisional minimum 4 h. Every required stream's first-to-last SDK span must satisfy it, with no rounding or endpoint allowance. A requested 60 s take can have a slightly shorter sample span; prefer a take comfortably inside the interval, such as 90 s. |
| Rates / drops | Each complete stream within existing **ASSUMED ±5%** of 30/200/100 Hz; max SDK gap ≤1.5 periods; no frame-number skips or nonincreasing frame numbers. Full-take rate is `(count-1)/SDK span`, never a selected good segment. |
| Excitation | **CALCULATED** raw gyro RMS x/y/z ≥**PROPOSED 0.1 rad/s** each; accel population standard deviation x/y/z ≥**PROPOSED 0.5 m/s²** each. Report each axis. Gravity projection and bias can contribute; these numbers do not prove observability. Applies to camera–IMU only. |
| Motion blur | For each IR view, nearest same-domain gyro sample within **PROPOSED 10 ms**, gyro norm ≥**PROPOSED 0.1 rad/s** defines motion. At most **PROPOSED 10%** of moving frames may have Laplacian variance <**PROPOSED 50 gray-level²**. All frames are visited; centers use stride **PROPOSED 3**, neighbors are adjacent original Y8 pixels. No matched motion or unavailable/small images yields BLOCKED. Unmatched images prevent a PASS. |
| Target detection | BLOCKED with explicit reason: existing tools provide no AprilGrid detection counts. No detection percentage is fabricated or substituted with sharpness. Applies to camera–IMU only. |
| Timestamps / stereo | Strictly increasing SDK, callback host steady and callback host realtime stamps per stream. Bag decoder additionally checks header, writer and SDK host-system timelines. One common hardware/global SDK domain; unique matching IR frame numbers and ≤**ASSUMED 1 ms** skew. Bag/callback count, domain, frame number and SDK stamps agree within **ASSUMED 1 µs**. |
| Storage write headroom | **CALCULATED** all saved take bytes (including logs/receipts) divided by shortest valid required-stream SDK span, compared against historical **MEASURED 16.6 MB/s**, with **PROPOSED 20%** reserve. Insufficient headroom prints WARN and fails this criterion. It does not measure disk writes or copy throughput. A RAM-captured take can therefore fail the card comparison while retaining good transport evidence. Keep unrelated derivatives outside the take. |
| Storage capacity | Saved post-capture free bytes ≥**ASSUMED 2 GiB** OS reserve plus **PROPOSED two further payload copies**. The old wrapper's pre-capture value minus payload is reported as INFERRED; post-capture capacity remains BLOCKED. Low measured/estimated capacity is also warned. |

FAIL takes precedence over BLOCKED; BLOCKED takes precedence over PASS. Unknown
format (including ROS1) is BLOCKED at the decoder. Malformed supported files,
wrong identity, missing required files and failed acceptance checks FAIL.
Missing inputs leave dependent metrics explicitly BLOCKED. CLI/config/path errors
return 3 without a qualification result; missing repository dependencies produce
a BLOCKED `result.json`.

## Clocks and source guidance

SDK `hardware_clock` / `global_time`, bag writer stamp (physical clock UNKNOWN),
bag header stamp, SDK host-system metadata, callback C++ steady-clock arrival
and callback host realtime are separate timelines. Host stamps are arrivals,
not exposure times. No clock fit, epoch conversion, offset correction or
cross-clock latency subtraction is performed. Decimal SDK milliseconds are
subtracted within domain before conversion to relative binary-float nanoseconds;
integer host/frame counters retain precision beyond 2^53. Domain transitions
are rejected, and their intervals are never subtracted. Blur and stereo compare
only accepted common-domain SDK stamps from the single decoded device.

The numerical camera duration interval is project policy, not a numerical claim
from Kalibr. The [Kalibr camera–IMU wiki](https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration)
supports exciting **all rotational and translational axes**, keeping a fixed
target visible, and avoiding blur. The [Kalibr IMU Noise Model wiki](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model)
recommends approximately **15–24 h** for stationary Allan data. These are
**SOURCE-SUPPORTED through the existing calibration plan's reviewer-verified
citations**, not freshly fetched for this network-free task. Local policies and
their provenance are in [the plan](../../docs/D435I_CALIBRATION_PLAN.md),
[lab instructions](../../calibration/lab/README.md), and
[Allan instructions](../../calibration/allan/README.md).

The historical card figure comes from the 2026-10-14 session addendum: 1.66 GB
saved over 100 s, with observed recording gaps and an INFERRED storage bottleneck.
It is not a new sustained benchmark or qualification of the replacement card.

## Outputs, tests and limits

Each run writes `result.json` (common result contract), `summary.txt` (the printed
status table), `metrics.json`, `config.json`, `tool-provenance.json`,
`invocation.json`, `input-manifest.json`, and `input-manifest-after.json`. Small saved command outputs are
copied byte-for-byte under `raw/`; large recordings/CSVs remain referenced by
relative name, size and SHA-256 in the inventory. `raw_files` hashes every
result artifact except the self-referential `result.json`. Each criterion points
to evidence inside the output directory. `take_diagnostics.txt` is supplementary:
its inherited hardcoded assumptions are labeled diagnostic and do not override
the configured criteria or a BLOCKED target check.

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s lab/calib_record/tests -v \
  > lab/calib_record/tests/last-run.log 2>&1
```

Synthetic camera serialization is reused from `tools/take_tools/tests/fixture.py`;
known images, callback CSVs and metadata adapt that fixture. The small real
fixture's origin and SHA-256 are in [fixtures/README.md](tests/fixtures/README.md).
Tests cover PASS/FAIL/BLOCKED, exit codes and result hashes, camera/Allan layouts,
reanalysis, original hash errors, clock faults, drops, emitter readback absence,
stereo pairing, callback/bag disagreement, per-axis excitation, blur, missing
dependencies, free-space uncertainty and storage warnings.

**UNKNOWN / not verified:** Pi execution, hardware timing and throughput, real
SDK bag decoding in this new wrapper, long-run memory/CPU cost, optical emitter
state for Allan, physical all-axis observability, target visibility/scale,
power/thermal cleanliness, stationarity and temperature stability. Power/kernel
logs are preserved, not promoted to hardware acceptance by this tool. Review
them with the existing lab workflow. Blur depends on scene texture, contrast,
gain, exposure and target size, and misses pure translation if gyro is quiet.
Bag decoding retains camera pixels in memory (roughly 2.2 GB of stereo pixels
at 120 s before Python overhead); only short calibration takes are supported.
Allan keeps compact timestamp arrays and online vector moments, but upstream
timing percentile calculation allocates intervals; hours-long Pi memory/runtime
still need a pilot. The checker neither fits Allan noise nor runs calibration,
changes factory values, verifies ROS1 conversion, or qualifies N1/N2/N3/VIO.
