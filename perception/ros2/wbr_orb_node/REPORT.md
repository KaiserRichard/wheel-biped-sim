# phase-14 completion report — 2026-10-18

**MEASURED software verification only. N4B/N4C remain OPEN.** All changes are
inside `ros2/wbr_orb_node/`. No git state-writing command, network operation,
package installation, Pi connection, GUI, camera or serial access was used.
Changes are intentionally uncommitted for the maintainer.

## Delivered

- An `ament_cmake` package with `rclcpp` stereo-image and combined D435i IMU
  subscriptions using sensor-data QoS, configurable raw/rectified topic names,
  a single tracking runner and a default fake-engine build. Optional
  `WITH_ORB_SLAM3` compiles the existing native ORB binding and constructs a
  headless `IMU_STEREO` System from explicitly supplied settings/vocabulary.
- A ROS-free C++17 core and Makefile target. Thin transport matching requires
  exact header equality; the existing adapter validates the atomic stereo pair,
  interpolates IMU and enforces coverage, monotonic source stamps, snapshot
  validity and reset semantics. Only one covered stereo pair enters that
  adapter at a time. No upstream source was copied or refactored.
- Bounded drop-oldest callback/image/IMU buffers, explicit loss accounting and
  nonwaiting callback queue access. Losing an IMU sample or detecting a source
  clock/value fault latches the input session. runner tracking and output I/O
  do not hold the ingress lock. Threshold defaults and rationales are centralized
  in `config/defaults.json`; a generated header prevents duplicated defaults.
- Diagnostic `nav_msgs/Odometry` in `odom -> imu_camera`, using the actual named
  optical-to-camera-IMU extrinsic and the original image header stamp. Quaternion
  extraction reuses `tools/ros2_bridge/src/mapping.hpp`; ROS clock checks reuse
  `clock_policy.hpp`. Invalid poses/missing geometry are withheld. After a reset
  or epoch change, Odometry requires a process restart. No TF is broadcast.
- Health JSONL compatible with the existing sender branch, including stock ORB
  state-name translation, mapping identity and conservative UNKNOWN metadata.
  A configured mapping retains the sender's exchange schema; preparation uses
  `tools.clockmap` rather than a second clock fit. Replay and fake results
  always invalidate mapping; missing evidence also invalidates it. Configured
  live mapping checks Linux boot identity and a finite source validity interval.
- `event_recorder.h` receipt/queue/tracking/output/drop events and queue depths,
  saved after runner join alongside application counts and a parameter manifest.
  Observer time is CLOCK_MONOTONIC; sender mapping targets CLOCK_MONOTONIC_RAW.
- Independent review fixed stereo-image rejection being misclassified as IMU loss
  and added a regression scenario; the root test runner now discovers this
  package's Makefile and Python suite.
- Offline-first Pi commands, calibration/frame/clock contracts, limits and
  evidence instructions in README. A host software checker writes the COMMON
  result contract, preserves raw command outputs and supports analyze-only.

## Verification

**MEASURED:** Apple clang 17.0.0, x86_64 Darwin 24.6.0; Python 3.11.0rc2.
The available host interpreter ran the tests; Python 3.12 on the Pi was not run.

`make -B -C ros2/wbr_orb_node test` rebuilt the C++17 fake-engine probe with
`-Wall -Wextra -Wpedantic` and passed **19 unittest tests**, no skips, in the
final saved run. Output: [tests/last-run.log](https://github.com/KaiserRichard/wbr-perception-phase1-runtime/blob/fde5721af0627eaf60c78d95296ab26924425e37/ros2/wbr_orb_node/tests/last-run.log).

Coverage includes exact stereo equality/unmatched drops; no processing before
IMU endpoint coverage; source ns precision at real epoch magnitude; a rotated
camera-IMU lever arm; all adapter initialization/tracking/loss/reset transitions;
queue drop-oldest order/counts; fault after IMU loss; timestamp reversal/gap and
wrong-domain rejection; replay clock seek; callback progress while an engine is
blocked; an engine exception before any source-bearing estimate; existing
runtime-observer aggregation; sender enum/schema compatibility; real saved
source epochs; clockmap identity and finite extrapolation policy; and software
result PASS, assertion FAIL, missing-executable BLOCKED and malformed-evidence
error paths. Analyze-only reevaluates the saved raw command receipts.

The real-evidence fixture preserves the first eight `source_ns` integers from
`tools/vio_adapter/real_replay/snapshots.jsonl`. Original input was the saved
2026-10-14 D435i RAM take. Its source SHA-256 and selection are in
`tests/fixtures/provenance.json`. **Only those epochs are MEASURED**; generated
images, IMU values, trajectories and health states are synthetic. No large raw
recording was copied. The sender's accepted tracking names were read with
`git show` from the existing health branch; its exact revision and minimal
schema are recorded in `tests/fixtures/sender_schema.json`.

Additional verification:

- `check_core.py --outdir ros2/wbr_orb_node/tests/evidence`, followed by
  `--analyze-only` on that directory: three software criteria PASS, exit 0.
  [tests/evidence/result.json](https://github.com/KaiserRichard/wbr-perception-phase1-runtime/blob/fde5721af0627eaf60c78d95296ab26924425e37/ros2/wbr_orb_node/tests/evidence/result.json) contains raw hashes,
  timestamps and host metadata. This is not a Pi gate result.
- C++ core compiled separately with `-g -O1 -fsanitize=address,undefined`; its
  `all` scenario passed without sanitizer diagnostics. Output:
  [tests/sanitizer.log](https://github.com/KaiserRichard/wbr-perception-phase1-runtime/blob/fde5721af0627eaf60c78d95296ab26924425e37/ros2/wbr_orb_node/tests/sanitizer.log).
- Python sources compile; existing test runner passes `bash -n`; no shell script
  was added. Whitespace review found no trailing whitespace.

## Test runner integration

**SOURCE-SUPPORTED:** `tools/run_all_tests.sh` has an explicit Makefile allowlist,
plus generic discovery for ROS package Makefiles and Python suites under
`ros2/*/`. The review added this package to both discovery paths. The package
target runs before the Python suite, so the Python tests reuse its host probe.

The root runner was executed after the change. `make:ros2/wbr_orb_node:test`
and `python:ros2/wbr_orb_node/tests` both reported PASS. The overall runner
reported FAIL only because the existing `tools/link_bench` host simulator cannot
open its pseudo-serial setup on this Mac; its implementation is outside phase-14.

## Unverified behavior and risks

**UNKNOWN / NOT VERIFIED:** colcon/ament build, ROS Jazzy subscription/publication
and rosbag playback, yaml-cpp integration, native ORB compile/link/ABI and System
shutdown, actual calibration/rectification, initialization/feature metadata,
physical source-clock binding and clockmap accuracy, trajectory accuracy,
latency, Pi scheduling/memory use, storage/thermal/power behavior and DDS loss.
Per the task, the ROS package and ORB branch were deliberately not built here.
No hardware gate is promoted.

The adapter does not expose initialized-IMU state or tracked-feature counts.
The JSONL therefore uses false/zero as explicitly UNKNOWN sentinels and always
sets `degraded=true`. Stock ORB reset/map observation also retains the existing
adapter's documented limitations. These outputs cannot qualify perception health
or controller eligibility without a separately reviewed metadata extension.

The requested `odom` view conflicts with the prior bridge's prohibition against
renaming corrected SLAM poses as continuous odometry. phase-14's explicit frame
requirement is implemented as a diagnostic topic only, with no TF, no invented
world alignment, and no valid twist/covariance (NaN). `imu_camera` follows the
existing tools/frames convention and is never `base_link` or the STM32 IMU.
Initial-map loop corrections can still jump; this is not REP-105 qualification.
Measured camera-IMU geometry plus provenance is required before publishing poses.
The health events continue even when geometry is absent or later epochs cannot
be published as Odometry.

Queue/drop accounting covers application-owned queues. DDS/driver losses remain
unknown. Recorder losses are independently visible; its fixed ring overwrites
old evidence on long runs. Heap/pixel-copy, mutex try-lock and OS scheduling
costs have no real-time guarantee. A blocked engine/output filesystem can delay
shutdown; bounded queues prevent backlog growth, not a hung dependency. Graceful
shutdown preserves evidence; a crash or kill can lose the in-memory trace.

A prepared clockmap file is trusted configuration, not a live synchronization
service. Future-source validity must be explicitly bounded and justified in that
file. Session/clock changes need a new mapping and process. Replay never borrows
a physical-run age mapping. The source field required by the sender is populated
with the image header ns, and is labeled with its actual configured domain;
without independently proving the header/D435i relationship its mapping remains
invalid. No cross-clock subtraction is introduced.

No bug in a shared tool was fixed or refactored as part of this task.

Proposed commit message:

```text
feat(ros2): add bounded stereo-inertial ORB node on vio_adapter
```
