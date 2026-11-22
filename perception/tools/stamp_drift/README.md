# `stamp_drift`

This tool diagnoses the relationship between a D435i ROS message's
`header.stamp` and the rosbag2 recorder's host receive timestamp. It fits

```text
receive = a + (1 + d) * header_stamp
```

per topic, reports `d` in ppm, robust residual distributions, a lower-envelope
receive-minus-header proxy, reversals, possible residual steps, and pairwise
inter-topic drift differences. `IR` and `IMU` pairings are called out when the
topic names make those classes recognizable.

The JSON also has a diagnostic-only `diagnostic_status`: `PASS` means all
topics had usable fits without detected reversals/steps, `FAIL` means a fit had
a detected reversal or possible step, and `BLOCKED` means a topic lacked usable timestamp pairs or a fit. FAIL takes
precedence over BLOCKED, including a reversal with too few samples to fit. These statuses are not hardware or N3 gate verdicts.

The fit reuses the centered affine fitter from `tools/clockmap`; iterative MAD
rejection is applied around that fit. It does not modify `tools/clockmap`.
The JSON records the selected `header_stamp` interpretation and an independent
`global_time_enabled` readback declaration (`true`, `false`, or `null` when
unknown). A frame domain does not imply option readback. CSV receive-clock
identity is UNKNOWN unless separately established by the input producer.

## Input and exact commands

CSV input requires these columns (extra columns are allowed):

```csv
topic,header_stamp_ns,receive_ns
/camera/infra1/image_rect_raw,1700000000000000000,2200000000
```

For a rosbag2 SQLite bag, pass the `.db3` file. The reader uses the `topics`
and `messages` tables, validates CDR serialization, parses the first
`std_msgs/Header` for Image, Imu, CameraInfo and IMUInfo sensor types, and uses
`messages.timestamp` as the receive timestamp. It supports CDR little- and
big-endian encapsulation; no ROS Python packages are required. Little-endian
Header decoding reuses `lab/rosbag_qual/bag_core.py`; the aggregate bag API does
not return timestamp pairs, so SQLite iteration remains local. TF sequences,
unstamped context and unsupported types are listed in `input.skipped_topics`,
never interpreted as sensor stamps. Declared supported topics with no messages
are BLOCKED. NumPy and a full repository checkout are required. DB3 hashing and
payload reads are streamed; only parsed timestamp rows are retained.

On the Pi, after recording has stopped and the bag is finalized:

```sh
cd /path/to/wbr-perception-phase1-runtime
python3 tools/stamp_drift/analyze.py /qualified/path/bag_0.db3 \
  --header-clock D435I_GLOBAL_TIME --global-time-enabled true \
  --output-dir /qualified/path/stamp-drift
```

Use `D435I_GLOBAL_TIME` only when the run manifest/parameter readback records
that `realsense2_camera` `global_time_enabled` was enabled and effective. Use
`D435I_HARDWARE_TIME`, `SYSTEM_TIME`, or `UNKNOWN` otherwise; this option is
metadata and does not convert either clock.

On a Mac or another offline host:

```sh
python3 tools/stamp_drift/analyze.py timestamps.csv \
  --header-clock D435I_GLOBAL_TIME --global-time-enabled true \
  --output-dir saved-report-input
python3 tools/stamp_drift/analyze.py --analyze-only saved-report-input
```

Every run writes `report.json`, `report.md` and the COMMON result contract in
`result.json`, including UTC execution times, host, per-topic criteria and raw
SHA-256 records. It prints a short criterion table. Raw input is copied verbatim
as `input.csv` or `input.db3`; `config.json`, `run.json` (clock declarations) and
`diagnostic.txt` are saved alongside it. The tool runs no external commands.
Use a separate evidence directory with enough space for a bag copy.

`--analyze-only` re-evaluates that saved raw input with the saved configuration
and clock declarations. Missing/corrupt raw input or saved metadata is BLOCKED;
it never trusts a previous verdict. `--config`, `--header-clock` and
`--global-time-enabled` can explicitly override saved settings. Exactly one raw
CSV/DB3 must be present. Default output is `<input-stem>.stamp-drift/`, or the
saved directory for analyze-only. Optional `--json-out` / `--md-out` rename the
reports within the output directory; reserved evidence paths cannot be used.

Exit codes: **0 PASS, 1 FAIL, 2 BLOCKED, 3 usage/configuration/internal error**.
These are diagnostic results only; passing fixtures is not a hardware gate PASS.

Each topic has a fit criterion (every segment has at least the configured
minimum number of samples and a usable positive-slope affine fit), and a
continuity criterion (detected reversals plus steps do not exceed configured
`max_discontinuities`, currently zero). Missing/degenerate fits are BLOCKED;
measured discontinuities FAIL even if the fit is unavailable. Residual-step
sensitivity, sample windows and robust rejection settings all come from config.
At the segment cap, remaining detected steps are still reported as FAIL.

## Reading the report

`drift_ppm` is the fitted receive-clock rate minus the header-clock rate. The
residual percentiles are fit residuals and should be read as centered jitter.
`lower_envelope_proxy_ns` is the minimum observed `receive_ns - header_stamp_ns`.
It is approximately offset plus minimum transport delay only under additional
assumptions. A fit cannot separate absolute latency from clock offset without a
two-way exchange or an independently known clock mapping. It also cannot prove
exposure-to-arrival latency, camera–IMU physical timing, timestamp-domain
selection, ROS QoS behavior, or a hardware gate.

Backward/non-increasing header stamps split the fit and are reported as a
possible global-time reset. Receive-time reversals are reported separately as a
possible recorder ordering problem. A monotonic forward residual step is
reported as a possible clock/transport discontinuity; inspect raw rows,
`global_time_enabled`, driver logs, and bag metadata before calling it a reset.

All numerical detector thresholds are in [`config.json`](config.json), with a
`source` and `rationale` for each. They are **PROPOSED diagnostics**, not N3
acceptance limits.

## Evidence and limitations

The checked-in `tests/fixtures/real_saved_evidence.csv` is adapted from the
measured D435i timing-probe rows at
`experiments/2026-09-13-d435i-post-reboot-20260913T023833Z/pi/timing-probe/data/frames.jsonl`.
That source has paired SDK `Global Time` and host `CLOCK_MONOTONIC` observations,
not ROS `header.stamp` and DB3 receive rows. The fixture therefore validates the
CSV reader and real-scale integer handling, but is **not** ROS bag evidence.
The review also added `tests/fixtures/real_ros_headers.json`: 12 exact Header
prefixes and DB3 timestamp pairs from the saved 2026-10-14 converted
`stereo_imu.db3`, with its source path and SHA-256. Those rows were verified
against the original file read-only. The conversion uses equal Header and DB3
timestamps, so zero fitted drift is a conversion artifact, not measured device
synchronization or live ROS recorder arrival evidence. The original SDK fixture
now includes exact source JSONL rows and Decimal-to-ns provenance checks.
A future physical run must preserve the live ROS bag, parameter readback, and
effective timestamp-domain evidence. This analyzer does not use the STM32 body IMU.

## Tests

```sh
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tools/stamp_drift/tests -v
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tools/clockmap/tests -v
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s lab/rosbag_qual/tests -v
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover \
  -s experiments/2026-09-12-pi5-native-sensor-validation -p test_n3_metrics.py -v
```

Exact final commands/output are in `tests/last-run.log`; review findings and
remaining limits are in [REVIEW.md](REVIEW.md).

