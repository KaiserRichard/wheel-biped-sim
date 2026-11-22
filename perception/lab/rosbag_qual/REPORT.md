# phase-4 report — ROS 2 rosbag qualification

## Built and reviewed

The original implementation supplies bounded explicit-topic recording,
offline SQLite/optional rosbag2_py analysis, and isolated rclpy playback
observation under `lab/rosbag_qual/`. The independent review fixed the issues
ranked in [REVIEW.md](https://github.com/KaiserRichard/wbr-perception-phase1-runtime/blob/fde5721af0627eaf60c78d95296ab26924425e37/lab/rosbag_qual/REVIEW.md): duration versus splitting semantics, replay
saved-data reduction, incomplete evidence acceptance, rate-clock comparison,
static TF stamp preservation, partial captures, threshold provenance, CDR
alignment, raw-log preservation and configuration/usage errors.

Production acceptance settings are in `config.json`; synthetic fixture limits
are in `tests/synthetic_config.json`. Chosen settings are labeled PROPOSED.
`qual_common.py` imports the existing ROS-independent `n3_metrics.read_topics`
validator from the September sensor-validation experiment. Those tools were
not modified. The CDR reader extracts headers, including all static TF headers,
without claiming full Image/Imu/CameraInfo payload validation. SQLite image
payloads stream through the analyzer instead of remaining in memory; MCAP
image payloads are reduced to header prefixes as they are read.

## Verification

MEASURED software evidence: 41 qualification/regression tests and 11 tests in
the reused N3 metrics suite pass. Exact final commands and full stdout/stderr
are in `tests/last-run.log`. Initial five-test output is in
`tests/review-initial.log`; the first regression run reproduced 16 failures and
four errors in 26 tests (`tests/review-regressions-before.log`). These include
subtest failures for all three CLI usage exits.

The original full real-bag test reads the saved DB3 without modifying it. A
portable fixture, `tests/real_header_fixture.json`, preserves three exact CDR
header prefixes per stereo/gyro/accel stream, source path and SHA-256. It is
MEASURED parser input, with payloads omitted; synthetic tests serialize complete
small Image and Imu messages. A fixture PASS is not a physical N3 gate PASS.

## External source defect recorded, not fixed

SOURCE-SUPPORTED inspection of the shared checkout's
`experiments/2026-10-14-db3-to-euroc/db3_to_euroc.py:91` found that `decode_imu`
consumes nine doubles for the orientation quaternion instead of four. Its CDR
alignment also uses the absolute buffer offset rather than the encapsulation
payload origin (`:35`). Feeding it the standard local Imu fixture raises
`ValueError: truncated CDR field`; output is in `tests/review-source-check.log`.
This reader was therefore not suitable for reuse. Neither that file nor any
other historical/shared tool was changed.

## Not verified and open risks

- UNKNOWN: actual ROS 2 Jazzy recorder/player behavior, installed storage plugins,
  discovery timing and QoS compatibility on the Pi. ROS commands are mocked in
  tests. MCAP's optional native reader was not exercised against a real MCAP.
- UNKNOWN: physical camera, current sensor profile, hardware/global-to-ROS stamp
  mapping, Pi power/thermal/USB state, filesystem durability and storage capacity.
  No hardware, devices, SSH, package installation or network access was used.
- PROPOSED: the 16.6 MB/s storage comparison ceiling is derived from historical
  achieved throughput (SOURCE-SUPPORTED addendum section 3), not a qualified
  capacity of the current destination. 18.4 MB/s is historical demand. Review
  this setting against destination-specific sustained-write evidence.
- Exact IR stamp pairing may reject drivers that stamp stereo frames separately;
  a physical run must establish the actual stereo timestamp contract first.
- Header integrity and transport checks do not validate all payload fields,
  calibration, camera-to-body IMU alignment or VIO. The D435i IMU is distinct from
  the STM32 body IMU. Replay checks do not provide a continuous live-domain
  camera-process guard across every instant of playback.
- `--analyze-only` evaluates saved measurements and logs; it does not independently
  decode the original bag again. Re-run `analyze_bag --bag ...` to do so. Keep the
  checksummed bag for independent review. Old evidence missing the new measured
  fields is BLOCKED rather than silently accepted.

Proposed commit message: `Fix rosbag qualification capture bounds, replay evidence and clock checks`

Changes are uncommitted for the maintainer. No physical gate status was changed.
