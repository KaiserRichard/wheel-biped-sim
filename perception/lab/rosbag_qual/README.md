# ROS 2 rosbag record/replay qualification

This directory implements the N3 record/replay check as three Python 3.11+
CLIs. It is software-only on the Mac and uses the same explicit IR/IMU topic
and QoS conventions as the existing N3 preparation material. The default
profile records both IR cameras; the configuration must match the profile
actually launched on the Pi (including whether topics are `image_rect_raw` or
`image_raw`). All acceptance numbers are in [config.json](config.json), with a
source or rationale beside each threshold.

## Pi commands

Run from the repository root after sourcing ROS 2 Jazzy. Keep the camera in
ROS domain 42 for recording. The configured node must be `/camera/camera`.

```sh
source /opt/ros/jazzy/setup.bash
ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 python3 lab/rosbag_qual/record_check.py \
  --config lab/rosbag_qual/config.json \
  --output-dir /path/to/evidence/record
```

The recorder uses the configured SQLite or MCAP storage plugin, explicit
`qos_overrides.yaml`, configured topic names, and a bounded duration. It saves
the exact command, recorder log, `ros2 bag info`, camera `ros2 param dump`, and
`ros2 topic info -v` output for every configured topic. A stopped camera is
required before replay:

```sh
source /opt/ros/jazzy/setup.bash
ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 python3 lab/rosbag_qual/replay_check.py \
  --config lab/rosbag_qual/config.json \
  --bag /path/to/evidence/record/bag \
  --output-dir /path/to/evidence/replay \
  --live-domain 42 --replay-domain 43
```

`replay_check` checks the live domain with `ros2 node list` and BLOCKS if a
RealSense or configured camera node remains. It then creates lazy `rclpy`
subscribers in the separate replay domain, runs `ros2 bag play`, measures
received rates, and compares message counts, header-stamp sequence hashes and
arrival rates with recorded bag receive rates (scaled by the playback factor).
Header stamps are checked separately, including every transform in static TF;
a zero static TF stamp is valid. Subscribers exist before a configured playback
delay. Both live and replay domains must have no camera node, and unrelated
publishers on the configured replay topics block acceptance. If `rosbag2_py` is installed, MCAP can be
analyzed through its reader; SQLite `.db3` is always parsed offline with the
stdlib parser.

For a bag already on disk, with no ROS graph:

```sh
python3 lab/rosbag_qual/analyze_bag.py \
  --config lab/rosbag_qual/config.json \
  --bag /path/to/bag \
  --output-dir /path/to/evidence/analyze
python3 lab/rosbag_qual/analyze_bag.py \
  --config lab/rosbag_qual/config.json \
  --analyze-only /path/to/evidence/analyze
```

The second command re-evaluates saved `analysis.json`; it does not start ROS
or reopen a live graph. Each check writes `result.json` using the required
PASS/FAIL/BLOCKED contract and prints a short table. Exit codes are 0, 1 and 2
for PASS, FAIL and BLOCKED; usage/configuration errors exit 3.

## Criteria and limits

- Per topic: configured presence/count, both rate bands, header-stamp reversals
  and repeats, CDR header errors, and header gaps strictly greater than
  `drop_gap_factor / nominal_hz`. Rates use `(count - 1) / (last - first)`.
  A drop count means gap events, not a count of lost physical sensor samples.
  Both header and bag receive gap counts are reported. Per-topic receive-time
  boundary gaps also prevent a brief healthy segment hiding a partial stream.
  ROS `header.stamp` and rosbag2 `messages.timestamp` remain separate clocks;
  their mapping to D435i hardware/global time is UNKNOWN without run evidence.
  Neither represents the STM32 body IMU. Epochs from different clocks are never
  subtracted. Replay arrival timestamps use Python `time.monotonic_ns`.
- Recording: the requested process interval is bounded with SIGINT and a
  configured shutdown grace, followed by termination escalation if needed.
  Early exit, non-successful return codes, failed command receipts or inadequate
  bag receive-time span prevent PASS. The configured capture-span and elapsed
  fractions are PROPOSED allowances for process startup/shutdown.
- IR pairing: exact left/right header-stamp pairing fraction.
- IMU: gyro and accel are independent configured topics, so their rates and
  timestamp continuity are checked separately; a combined IMU is not inferred.
- Disk: total bag bytes divided by the receive-time duration, compared with the
  configured storage comparison ceiling. Only storage files and metadata
  contribute to bag size. The default ceiling is PROPOSED from the old card's
  achieved throughput of 16.6 MB/s; 18.4 MB/s was required stream demand, not
  storage capacity. Review this config against independently qualified storage
  before interpreting physical results. This ratio cannot measure write stalls,
  fsync latency, cache effects or remaining storage margin.
- Replay: CLOCK_MONOTONIC receive rate versus recorded receive rate, count
  fraction and exact ROS header-stamp sequence hash. Exact stamp preservation
  requires all stamped samples even if the count-fraction limit allows loss;
  that fraction alone is relevant to unstamped context. Single latched static
  messages are counted and stamp-checked without a dynamic rate requirement.

Passing a synthetic fixture proves only the analyzer/tooling behavior. It does
not prove Pi storage margin, USB stability, RealSense driver QoS compatibility,
power, thermal behavior, calibration, IMU alignment, or a physical N3 gate
PASS. A result with status `PASS` is still evidence for review under the
project’s gate rules.

## Tests

```sh
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover \
  -s lab/rosbag_qual/tests -v
```

Saved recording and replay receipts can also be re-evaluated on the Mac:

```sh
python3 lab/rosbag_qual/record_check.py --analyze-only /path/to/evidence/record
python3 lab/rosbag_qual/replay_check.py --analyze-only /path/to/evidence/replay
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover \
  -s experiments/2026-09-12-pi5-native-sensor-validation -p test_n3_metrics.py -v
```

All analyze-only commands use the saved config unless explicitly overridden.
Missing/corrupt observations, configs or required command logs block acceptance;
command and measurement criteria are recomputed rather than trusting old verdicts.
Copy the whole evidence directory for re-evaluation. The analyzer supports a
single SQLite storage file with stdlib; split bags and MCAP require
`rosbag2_py` and the appropriate storage plugin. Missing support is BLOCKED.
The reused historical `n3_metrics.read_topics` validates topic manifests, so
run from a full repository checkout on the Pi rather than copying these files
alone. Known defects in the external DB3 converter prevented reusing its CDR
reader; see REPORT.md.

The unit tests generate tiny SQLite bags with hand-written CDR `Image` and
`Imu` messages for PASS, dropped-gap, reversed-stamp and BLOCKED paths. When
present, the test also reads (without copying or modifying) the saved
2026-10-14 DB3 evidence as a real-input fixture. A portable 3.6 KB fixture also preserves exact
CDR header prefixes and separate receive/header timestamps from that real bag,
with its SHA-256 and source path; it runs even when the external bag is absent.
It omits payloads and is parser evidence only. Tests mock all recording/replay
commands and cover PASS, FAIL and BLOCKED for each CLI, round trips through
saved evidence, static TF, partial streams, absent ROS and error exits.
Test commands and complete output are recorded in `tests/last-run.log`.
See [REVIEW.md](REVIEW.md) for the independent review and fixes.

