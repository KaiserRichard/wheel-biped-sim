# Q5 — ROS 2 diagnostic output bridge

**INFERRED / PROPOSED design. UNBUILT ROS 2 Jazzy skeleton.** ROS 2 and
ORB-SLAM3 are not installed/built here. Only the ROS-free mapping core and
synthetic tests have been executed. Nothing here is an accepted architecture,
controller interface, navigation qualification, or gate PASS.

## Boundary and input

**MEASURED, inspected code:** use the actual
[`EstimateSnapshot`](../vio_adapter/adapter.h) from Q1, not a second invented
estimator message. It carries optional `source {ns, domain}`, `sequence`,
`reset_counter`, `epoch_id`, optional `map_id`, tracking `state`, `valid`,
`reasons`, optional `T_world_camera`, and covariance availability/data. The
adapter's [README](../vio_adapter/README.md) specifies object-lifetime counters,
strict source order, copied snapshots and heartbeat semantics. Its synchronous
owner must serialize feed/reset/snapshot access. `reset()` can change validity
and epoch while retaining sequence and source time; polling only sequence misses
that event. Stock ORB reset/map observation has known limitations documented there.

**INFERRED:** the bridge consumes copied immutable snapshots. It neither drives
the camera nor calls tracking/reset nor reads engine pointers. A serialized
adapter owner calls `SnapshotBridge::submit(snapshot, clock_evidence)`; the
unconnected executable currently reports `no_snapshot_ingress`. This is a C++
integration seam, not a DDS subscription or implemented IPC transport.
`adapter.h` is included directly; linking the adapter or ORB is unnecessary for
this skeleton. Set `WBR_VIO_INCLUDE_DIR` if packaging outside this source tree.

Bind a trusted, unique `adapter_session` before accepting estimates: counters
are not globally unique across process restart. Bind one `source_domain`, one
ROS time domain (`ros_live` or `ros_replay`), and one `clock_generation`. Changing
any identity requires a new bridge/Gate and explicit external re-establishment.
Parameters `max_age_ns` and `max_uncertainty_ns` default to -1, which suppresses
all pose output until configured. Their approved values remain **UNKNOWN**;
the tiny ns limits in tests are synthetic fixtures, not controller budgets.

The input mailbox has capacity one. A submit before the previous submit has
been inspected latches `ingress_overrun_restart_required`; it cannot silently
replace an unread reset/fault with a healthy estimate. The 20-ms wall timer is a
**PROPOSED** polling choice, not measured latency. Production may replace this
with a bounded event channel and latest-pose slot, provided boundaries/faults
are latched and queue drops counted. Feed/reset notifications must all reach the
bridge; the adapter's tracking observer alone does not notify owned reset calls.
No ROS publication belongs inside the tracking callback: reliable DDS output
can stall, so it must not block the estimator or any controller link task.

## Pose meaning and field mapping

**MEASURED, adapter contract:** `T_world_camera` is row-major SE(3), mapping
`camera_left_optical` into `slam_world`. Translation is in metres. The adapter
has already inverted engine camera-from-world pose; the bridge must not invert
it again. `slam_world` orientation, gravity alignment and geographic alignment
are **UNKNOWN**. Preserve these axes and publish diagnostic camera pose.

**SOURCE-SUPPORTED:** REP-103 defines SI units, right-handed body axes and the
optical convention; REP-105 distinguishes continuous `odom` from corrected
`map` [S1, S2]. **INFERRED:** an optical child label does not authorize rotating
the world or renaming the camera `base_link`. Q5 performs no mount conversion.
A future reviewed body view needs measured `T_base_camera` and
`T_world_base = T_world_camera × inverse(T_base_camera)`, using
[tools/frames](../frames/README.md). Moving mounts require time-matched dynamic
geometry. Camera IMU “body” is not the robot body.

**SOURCE-SUPPORTED schema; INFERRED mapping policy [S3–S5]:**

| PoseStamped field | Value |
|---|---|
| `header.frame_id` | Exactly `slam_world`; alternate parent rejected |
| `header.stamp.sec`, `.nanosec` | Trusted mapped **source** epoch, integer quotient/remainder by 10^9 |
| `pose.position.x/y/z` | Matrix entries 3, 7, 11; metres, no inversion |
| `pose.orientation.x/y/z/w` | Unit quaternion of the rotation block; explicit ROS xyzw order |
| Child frame | PoseStamped has no child field: fixed topic contract `camera_left_optical`, also in diagnostics |
| Sequence/reset/map/source domain | PoseStamped has no such fields: retain them in diagnostics and recording sidecar |
| Covariance/twist | Not present; no fields or values synthesized |

`mapping.py` is the ROS-free Python reference. It accepts the keys emitted by
Q1's `replay_fake.cpp` JSONL (`source_ns`, `domain`, and the snapshot fields),
with optional `covariance`. It assumes a typed, complete snapshot projection;
it is not an untrusted JSON/IPC parser. `src/mapping.hpp` supplies the C++
counterpart used by the node. Both check named frames, finite proper SE(3),
source order and eligibility. Quaternion extraction covers trace-positive and
largest-diagonal branches, including half turns; normalization only handles
floating-point extraction error after SE(3) validation. No transform is repaired.
Nonnegative stamps must fit signed int32 seconds and uint32 subsecond ns.

## Clocks and freshness

**INFERRED:** `ClockEvidence` is an external mapper's result, not an implicit
clock conversion. It contains the exact original `source_ns/source_domain`,
bound adapter `session`, target ROS domain, clock `generation`, mapping
`revision`, mapped `ros_ns`, conservative `age_upper_ns` and `uncertainty_ns`.
The mapper must validate source/session identity, its mapping interval and
uncertainty, and supply source age evaluated at `submit()` time. Missing or
invalid evidence means no pose. The skeleton implements no mapping service and
cannot establish the truth of supplied evidence (**UNKNOWN**).

For live input, characterize device→host steady mapping separately from
device→ROS/system mapping. In SDK `global_time`, do not assume host steady time:
[clockmap](../clockmap/README.md) explicitly documents that distinction. Age is
`host_steady_now - mapped_source_steady + uncertainty`, all in one bound steady
domain/session after conversion. Never subtract a device epoch from `now()`.
In replay, the mapper evaluates age using source and current replay time in the
same ROS generation, with mapped-stamp uncertainty. The node adds its elapsed
local steady residence to the supplied age bound. This conservative replay policy
can mark a paused snapshot stale; pause is not renewed perception.

Every repeated heartbeat needs age recomputed from its **original** source, not
from packet receipt. A heartbeat may not refresh a cached age bound. Evidence
must invalidate on host/system steps, changed device/adapter session, mapping
failure and replay seek. Mapping revision is provenance within a generation,
not permission to change source epoch. The gate consumes each new sequence
once, even if clock evidence is missing: no later republishing of that same
snapshot with a newly guessed stamp. Recover with a fresh estimate.

The ROS node additionally rejects a mapped pose stamp later than its ROS `now()`.
It detects backward ROS time at polling and changes to `use_sim_time`, latching
`clock_jump_restart_required`. Forward steps/seek events and jumps entirely
between polls require mapper/supervisor invalidation before submission; robust
ROS jump callbacks remain an integration item. Do not claim complete jump
detection from this skeleton. **SOURCE-SUPPORTED:** ROS time is uninitialized
at zero under simulation and can pause/jump; steady and ROS clocks serve
different purposes [S12].

## Stale, reset and lost behavior

**INFERRED policy, implemented in both ROS-free Gates:**

| Event | Pose behavior | Visibility / recovery |
|---|---|---|
| Valid fresh TRACKING, NONE reasons, correct frame/clock, within age/uncertainty limits | Publish once at mapped source time | Diagnostic retains source and identity |
| Same source/sequence heartbeat | No repeat pose | Reevaluate age; stale heartbeat remains stale |
| Stale source / unavailable or uncertain mapping | Withhold | Explicit reason; require later fresh estimate |
| STARTING / INITIALIZING / LOST / FAULT, invalid flag or any reason bit | Withhold, even with unchanged sequence | State/reasons remain visible |
| Reset counter, epoch or map metadata change | Suppress boundary snapshot, invalidate any future alignment | Later fresh TRACKING can resume diagnostic pose in new epoch |
| Regressing sequence/source or reset/epoch counters | Withhold and latch fault | New trusted session/Gate required |
| Unread mailbox overwrite | Withhold and latch ingress fault | Restart/rebind; never conceal dropped reset |
| Bridge death | No outputs | Consumers must expire their own cached pose/diagnostics |

Changing map metadata includes known↔unknown changes conservatively. The initial
snapshot establishes a boundary without assuming any previous map alignment.
An invalid event with unchanged sequence must still be processed. Source stamps
remain strictly increasing across owned resets within the same adapter session;
old-session or replay-loop data require a new binding.

PoseStamped cannot announce invalidity or associate an epoch atomically. Existing
RViz/subscriber cache entries cannot be retracted by silence. Consumers must
expire source age, clear trails at reset and never use this diagnostic topic for
control/fusion based only on asynchronous diagnostics. Record a snapshot sidecar
with source stamp/domain, session, sequence, reset/epoch/map, mapping generation
and revision for exact epoch association. An atomic stamped health envelope is a
future interface, not implemented here.

## Diagnostics

**SOURCE-SUPPORTED:** `diagnostic_updater` provides `Updater`, task callbacks,
hardware identity and `DiagnosticStatusWrapper` summary/key-values [S6–S8].
Its inspected 4.2.6 implementation publishes `/diagnostics` with depth one,
uses ROS time for its timer and stamps the array at publication. That stamp
describes the **diagnostic report**, not a fresh estimate [S7]. Verify the actual
installed Jazzy version before integration; 4.2.6 is an API reference, not an
installed dependency claim.

**INFERRED:** report process visibility separately from source validity. The
skeleton reports session, sequence, reset/epoch/map, source ns/domain, numeric
tracking enum (Q1 order), reason bits, mapping revision/uncertainty/source age,
ROS domain, camera child and covariance availability. Missing values say
`UNAVAILABLE`. Hardware ID `unbound_adapter:<session>` is a placeholder, not an
invented camera serial. Set real identity only from trusted provenance.

No ingress or zero replay clock is STALE; stale source is STALE; invalid/reset/
mapping faults are ERROR. Even a published pose remains WARN because the ROS
skeleton and frame/calibration qualification are unverified. OK in a future
qualified bridge still means diagnostic eligibility, not motor permission.
Use the standard level definitions [S9]. The updater's nominal period is one
second. A separate one-second wall check calls `force_update()` while simulated
time remains frozen, so pause-time staleness stays visible (after pause detection).
Both diagnostics and pose callbacks run on the default mutually exclusive group
with `rclcpp::spin`; only `submit()` may run on the external owner thread and
uses a mutex. DDS scheduling and callback latency remain **UNKNOWN**.

## TF ownership and odometry exclusion

**INFERRED, follows R1 §2 (archived phase-1 file, not published):**

| Transform | Sole owner / prerequisite |
|---|---|
| `odom → base_link` | Qualified continuous local wheel/body EKF; alternatively separately qualified continuous VIO |
| `map → odom` | One validated global correction bridge, after reviewed world alignment and same-epoch local pose |
| Robot mount/joint chain | URDF / robot_state_publisher; dynamic for moving links |
| Camera internal geometry | Camera driver, with no duplicate publisher |
| `slam_world` camera output in Q5 | PoseStamped only; no TF broadcaster |

Q5 publishes **no `/odom`, Odometry, `/tf`, or `/tf_static`**. Without a continuous
local source, publishing `odom` is prohibited. Loop-corrected SLAM cannot be
renamed `odom`, made continuous by restamping, or differentiated into navigation
twist. An identity `map→odom`/`map→slam_world` would assert an unestablished
alignment and must not be supplied.

Future correction uses `T_map_odom(t) = T_map_base(t) × inverse(T_odom_base(t))`
at exactly the same mapped source epoch, with time-bounded local interpolation,
reviewed body mount and world alignment. Never compose newest SLAM with latest
unrelated odom. Reset invalidates alignment and stops correction broadcasts
until explicitly re-established. REP-105 gives the parent chain and authority
semantics [S2]. Pose in `slam_world` is not a REP-105 `map` claim.

## Covariance policy

**MEASURED:** fake and current ORB adapters expose UNAVAILABLE covariance; the
snapshot contract describes optional 6×6 world-tangent covariance.
**SOURCE-SUPPORTED:** Odometry specifies parent-frame pose and child-frame twist
with covariance, while PoseWithCovariance specifies row-major x,y,z and fixed-axis
rotation order. Neither definition declares an unknown-covariance sentinel [S10].

**INFERRED:** PoseStamped carries no covariance. Preserve availability in
diagnostics; if marked AVAILABLE, require an actual finite 36-entry array before
publishing even the pose. Inconsistent availability/data suppresses the snapshot.
Do not fill zeros, identity, arbitrary large values, NaN/-1 sentinel rules from
other message families, or feature-count confidence. Finite arrays do not prove
an uncertainty model is calibrated, symmetric or positive semidefinite.

No PoseWithCovarianceStamped/Odometry publisher is supplied. A later consumer
requires qualified covariance units/convention/ordering, proper Jacobian
conversion into its pose tangent frame (including lever-arm cross terms), and a
consumer-approved unavailable policy. Velocity/twist need independently qualified
continuous semantics and their own covariance. This task provides neither.

## QoS by topic

**INFERRED choices; SOURCE-SUPPORTED mechanics [S11, S13, S14]:**

| Topic / owner | Reliability / durability / history | Status |
|---|---|---|
| `slam/pose` (root resolves `/slam/pose`), bridge | Reliable / volatile / keep-last 1 | Implemented skeleton; fresh updates only |
| `/diagnostics`, diagnostic_updater | Reliable / volatile / keep-last 1 | Inspected updater default; verify version/remap |
| `/clock`, sole replay owner; node time source listens | ClockQoS best effort / volatile / keep-last 1 | Via rclcpp time source, never bridge-published |
| `/tf`, future correction owner | DynamicBroadcasterQoS reliable / volatile / keep-last 100 | Not published by Q5 |
| `/tf_static`, URDF/camera owners | StaticBroadcasterQoS reliable / transient-local / keep-last 1 | Not published by Q5; listeners need transient-local |

Pose reliability is for compatible operator/state consumers, not completeness or
freshness proof. Small volatile queues avoid latched old poses; they cannot cancel
already queued DDS retries. Consumers enforce source age. Deadline, lifespan and
lease duration remain unspecified defaults; DDS liveness never renews estimate
validity. Inspect `ros2 topic info -v` and recorder/RViz QoS in the provisioned
environment; best-effort publisher cannot match reliable subscription. Use
explicit compatible rosbag QoS overrides if needed [S15]. Q5 has no image/IMU
subscriptions; future sensor input association is outside this output bridge.

## Replay /clock rules

**INFERRED:** stop live publishers and the robot link sender; use a separate
ROS_DOMAIN_ID/network and namespace, with `/diagnostics` remapped into the replay
namespace because updater uses an absolute name. Ensure replay has no path to an
enabled MCU receiver. Topic namespaces alone do not isolate `/clock` or TF.

Exactly one replay player owns `/clock`; every bridge/visualization/TF consumer
uses `use_sim_time=true` from startup. Native EuRoC replay needs a dedicated
supervisor/time source, which is absent here. For an already audited standard
ROS bag, `ros2 bag play <bag> --clock` is the Jazzy option [S16]; do not assume the
native SDK SQLite file is an ordinary VIO-ready ROS bag. Clock publication does
not rewrite a recorded header. Preserve source stamps; map explicitly into the
bag timeline only if that relation is established, and preserve the original
epoch/domain in provenance. Bag storage/arrival time is not automatically capture
time.

No pose before a nonzero replay clock and valid mapping. Pause stops poses and
never extrapolates `/clock`; wall supervision eventually marks frozen evidence
stale. Backward seek, looping, forward seek and clock-source changes require
queue clearing, fresh generation/session and explicit rebind; this skeleton uses
restart for recovery. Do not carry watermarks or TF alignment across replay loops.
After resume, only a later fresh source snapshot is eligible. Real-time/paced
source-age evidence and unpaced replay throughput are separate measurements.

## What this bridge must never do

**INFERRED requirements:**

- Restamp an old pose with `now()`, publish duplicates as fresh, or advance source
  sequence/time on heartbeat, reset or diagnostics.
- Invent a clock relation, world alignment, robot mount, covariance or twist.
- Emit odom without a continuous local source, duplicate another TF owner, or
  send loop/reset jumps into local balance state.
- Change MCU body estimation, balance/LQR/VMC/FSM/motor/jump authority; grant
  navigation enable/leases; send motion/torque commands; or route replay to MCU.
- Make STM32 fallback depend on ROS, DDS liveness, diagnostics cadence, or bridge
  survival. ROS bridge stalls/death must leave local controller authority intact.

## Build/test status and integration work

**MEASURED:** `make -C tools/ros2_bridge test` compiles only the ROS-free C++17
probe against the real Q1 header, executes core assertions, and runs stdlib
unittest with C++/Python field parity. Pure Python independently runs with:

```sh
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tools/ros2_bridge/tests -v
```

The parity case is skipped without `WBR_CPP_PROBE`; all reference mapping/policy
tests still run. See [REPORT.md](REPORT.md) for actual results. No installs needed.

**UNKNOWN / UNBUILT:** `package.xml`, `CMakeLists.txt` and `src/bridge_node.cpp`
are plausible Jazzy source, not a compiled ROS package. A future already
provisioned ROS workspace may build with `colcon build --packages-select
wbr_ros2_bridge`, using the Q1 include directory. This command was not executed.
The resulting unconnected node would only emit diagnostics. Snapshot ingress,
real clock mapper, complete jump invalidation, namespace/QoS discovery, ROS
serialization, paused diagnostics, mailbox/reset fault injection, DDS stalls,
native plus bridge Pi overhead and loss/death trials remain to be verified. Review
placeholder maintainer/license metadata before distribution.

## Inspected primary sources

**SOURCE-SUPPORTED:** URLs plus one-based source lines below were inspected for
this task. Jazzy branches and REP master are mutable; pin deployed revisions
before integration. No quoted code is copied from these ROS sources.

- **S1:** [REP-103, lines 39–75, 95–124](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst#L39-L124): SI/body/optical conventions.
- **S2:** [REP-105, lines 36–76, 124–195](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst#L36-L195): odom/map continuity, frame tree, authorities.
- **S3:** [Jazzy PoseStamped.msg, lines 1–4](https://github.com/ros2/common_interfaces/blob/jazzy/geometry_msgs/msg/PoseStamped.msg#L1-L4): header plus pose.
- **S4:** [Jazzy Pose.msg, lines 1–4](https://github.com/ros2/common_interfaces/blob/jazzy/geometry_msgs/msg/Pose.msg#L1-L4) and [Quaternion.msg, lines 1–6](https://github.com/ros2/common_interfaces/blob/jazzy/geometry_msgs/msg/Quaternion.msg#L1-L6): position, orientation, xyzw.
- **S5:** [Jazzy Node API, lines 167–190, 218–230](https://github.com/ros2/rclcpp/blob/jazzy/rclcpp/include/rclcpp/node.hpp#L167-L230) and [Jazzy C++ tutorial, lines 204–272, 337–342](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.rst#L204-L342): publisher, wall timer, package/CMake and spin.
- **S6:** [diagnostic_updater 4.2.6 header, lines 273–279, 348–365, 390–394, 408–409](https://github.com/ros/diagnostics/blob/4.2.6/diagnostic_updater/include/diagnostic_updater/diagnostic_updater.hpp#L273-L409): member task, construction, force_update, hardware ID.
- **S7:** [Updater 4.2.6 implementation, lines 38–65, 108–111, 165–175](https://github.com/ros/diagnostics/blob/4.2.6/diagnostic_updater/src/diagnostic_updater.cpp#L38-L175): depth-one diagnostics, ROS timer/report stamp.
- **S8:** [DiagnosticStatusWrapper 4.2.6, lines 85–89, 210–216](https://github.com/ros/diagnostics/blob/4.2.6/diagnostic_updater/include/diagnostic_updater/diagnostic_status_wrapper.hpp#L85-L216): summary and add.
- **S9:** [Jazzy DiagnosticStatus.msg, lines 1–18](https://github.com/ros2/common_interfaces/blob/jazzy/diagnostic_msgs/msg/DiagnosticStatus.msg#L1-L18): status levels, ID, keys.
- **S10:** [Jazzy nav_msgs/Odometry.msg, lines 1–14](https://github.com/ros2/common_interfaces/blob/jazzy/nav_msgs/msg/Odometry.msg#L1-L14) and [PoseWithCovariance.msg, lines 1–9](https://github.com/ros2/common_interfaces/blob/jazzy/geometry_msgs/msg/PoseWithCovariance.msg#L1-L9): frame/covariance definitions.
- **S11:** [Jazzy QoS docs, lines 30–61, 81–94, 109–165](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst#L30-L165): history, reliability, durability and compatibility.
- **S12:** [ROS clock design, lines 61–103, 107–112](https://github.com/ros2/design/blob/gh-pages/articles/130_ros_time.md#L61-L112): system/steady/ROS time, zero, pause and jumps.
- **S13:** [Jazzy rclcpp QoS API, lines 108–134, 167–190, 330–347](https://github.com/ros2/rclcpp/blob/jazzy/rclcpp/include/rclcpp/qos.hpp#L108-L347): KeepLast/reliable/volatile and ClockQoS.
- **S14:** [Jazzy TF QoS, lines 37–67](https://github.com/ros2/geometry2/blob/jazzy/tf2_ros/include/tf2_ros/qos.hpp#L37-L67): dynamic/static broadcaster defaults.
- **S15:** [Jazzy rosbag QoS overrides, lines 9–103](https://github.com/ros2/ros2_documentation/blob/jazzy/source/How-To-Guides/Overriding-QoS-Policies-For-Recording-And-Playback.rst#L9-L103): explicit recorder/player compatibility.
- **S16:** [Jazzy rosbag play CLI, lines 92–105](https://github.com/ros2/rosbag2/blob/jazzy/ros2bag/ros2bag/verb/play.py#L92-L105): --clock and exclusive clock modes.
