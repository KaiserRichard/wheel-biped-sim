# CANDIDATE Pi–MCU link specification

**CANDIDATE, not an accepted interface. STM32 firmware owner review is required
before any firmware use.** INFERRED: health-only prototype with optional reference
fixtures. **Every numeric layout value, identifier, rate, limit and arithmetic
result below is PROPOSED**, except explicitly attributed source facts and synthetic
test observations. This designation also applies to code constants and fixtures.

SOURCE-SUPPORTED inputs: [R2 integration §§1–2, lines 15–39](../../../wbr-perception-phase1-runtime/slides_roadmap/research/R2_integration.md)
and [J2 architecture §4, lines 44–57](../../research/sw-foundation/J2_architecture.md).
Exact widths and policy choices here are INFERRED. This is original framing/code,
not MAVLink wire compatibility. Borrowed reset/quality semantics are
SOURCE-SUPPORTED by [MAVLink common ODOMETRY](https://mavlink.io/en/messages/common.html#ODOMETRY)
(rendered source lines 2882–2886, inspected 2026-10-14): discontinuities increment
reset_counter; quality is signed, failed/unknown/graded. No probability/covariance
is inferred from feature counts. No covariance or pose is serialized; availability
reports the upstream estimator. Unknown uncertainty remains unavailable.

## Framing and representation (PROPOSED)

`COBS(header || payload || CRC-32C) || 00`. All multibyte integers and IEEE-754
binary32 reference fields are little endian. No C struct is serialized directly;
there is no ABI padding. Timestamps are integer nanoseconds; durations identify
units. Header offsets are absolute decoded offsets:

| Offset | Bytes | Field |
|---|---|---|
| 0 | 1 | version u8 = 1 |
| 1 | 1 | type u8: 1 PerceptionHealth, 2 BodyVelocityRef, 3 McuStatus, 4 TerrainAdvisory, 5 AdvisoryEcho |
| 2 | 2 | payload_length u16, exact type size |
| 4 | 8 | session u64: Pi boot ID for Pi records, MCU boot ID for status |
| 12 | 4 | packet sequence u32, separate sequence per type/session |
| 16 | payload_length | payload below |
| 16 + payload_length | 4 | CRC-32C u32 |

CRC covers header/payload only. Castagnoli reflected polynomial 0x82F63B78
(normal 0x1EDC6F41), reflected processing, init/xorout 0xFFFFFFFF. The
SOURCE-SUPPORTED check specified in the brief is `123456789` → `E3069283`.
Maximum decoded record including CRC is 192 B, encoded cap 193 B, wire cap
including delimiter 194 B. Unknown type/version, wrong length, nonzero reserved
fields, malformed COBS and CRC mismatch are rejected. Empty delimiters are
ignored. CRC detects corruption and provides no authentication.

COBS: a code byte followed by code−1 nonzero bytes; insert zero between blocks
unless code is 255. Encoder writes a final empty block after an exact 254-byte
nonzero run (255, run, 1); decoder also accepts terminal 255/run. Generic COBS
helpers support larger test inputs; stream/record functions enforce the link cap.
Zero return means failure except valid empty COBS decodes to zero bytes. The
link has no empty decoded records. Buffers must not overlap; callers provide
valid pointers/capacities (NULL with zero input length is allowed).

## PerceptionHealth payload (80 B, PROPOSED)

Offsets are relative to payload; add 16 for decoded wire offsets.

| Offset | Bytes/type | Field / meaning |
|---|---|---|
| 0 | 4/u32 | estimate_seq, advances only on a new estimator sample |
| 4 | 4/u32 | map_epoch, coordinate/discontinuity epoch |
| 8 | 4/u32 | transform_revision, reviewed body extrinsic/frame convention |
| 12 | 4/u32 | mapping_revision, reviewed source→Pi→current MCU boot mapping |
| 16 | 1/u8 | reset_counter, increment modulo 256 on discontinuities |
| 17 | 1/u8 | tracking: 0 STARTING, 1 INITIALIZING, 2 TRACKING, 3 LOST, 4 FAULT |
| 18 | 1/u8 | estimator_id, nonzero opaque identity (fixture uses 1) |
| 19 | 1/u8 | source_domain: 1 device capture, 2 Pi monotonic, 3 ROS replay |
| 20 | 2/u16 | flags, bits below |
| 22 | 2/u16 | frame_id = 2 continuous odom |
| 24 | 2/u16 | child_frame_id = 1 robot body |
| 26 | 1/i8 | quality: −1 failed, 0 unknown, 1–100 graded; 255 encodes −1 |
| 27 | 1/u8 | reserved zero |
| 28 | 4/u32 | upstream reason mask, diagnostic opaque bits |
| 32 | 8/u64 | source_ns, acquisition epoch in source_domain |
| 40 | 8/u64 | mapped_mcu_ns, same acquisition in current MCU boot monotonic |
| 48 | 8/u64 | pi_send_ns, Pi boot monotonic send epoch |
| 56 | 8/u64 | age_at_send_ns, upper source-age bound at send, duration |
| 64 | 4/u32 | uncertainty_ns, conservative clock mapping error bound |
| 68 | 12 | reserved zeros |

Flags by bit index: 0 initialized, 1 tracking, 2 finite upstream pose, 3 finite
upstream velocity, 4 reviewed frame, 5 reviewed calibration, 6 clock mapping
valid, 7 resource fault, 8 estimator uncertainty available. Other bits must be zero.
An eligible estimate requires bits 0–6 set, bit 7 clear, tracking TRACKING and
quality not −1. Quality 0 is allowed and never promoted to a grade. Bit 8 is
optional and does not establish fusion readiness. Ineligible health can report
process liveness without source-age arithmetic but cannot authorize references.

Body is right-handed forward/left/up; positive yaw is about body +z. Odom is
continuous local coordinates; global map jumps are not accepted as continuous
odom. A heartbeat copies estimate sequence, source domain/time and mapped
acquisition epoch exactly. Packet sequence/send time may advance. Frozen-estimate
uncertainty may increase, never decrease. Source domain cannot change after
an accepted estimate without a trusted bind. Pose/twist output is deferred;
no calibration, transform or covariance values are fabricated.

## Optional BodyVelocityRef payload (44 B, PROPOSED)

| Offset | Bytes/type | Field |
|---|---|---|
| 0 | 4/u32 | source estimate_seq, equal current accepted eligible estimate |
| 4 | 4/u32 | map_epoch, equal accepted health |
| 8 | 4/u32 | mapping_revision, equal locally bound revision |
| 12 | 2/u16 | frame_id = 1 body, inherits reviewed transform_revision from health |
| 14 | 1/u8 | reset_counter, equal accepted health |
| 15 | 1/u8 | reserved zero |
| 16 | 8/u64 | pi_creation_ns, reference creation in Pi boot monotonic |
| 24 | 8/u64 | mapped_mcu_creation_ns, same creation in current MCU boot monotonic |
| 32 | 4/u32 | lease_us, nonzero, at most 100000 microseconds |
| 36 | 4/binary32 | forward_m_s, signed body forward speed, m/s |
| 40 | 4/binary32 | yaw_rad_s, signed body +z reference, rad/s |

Both floats must be finite: reject NaN (including signaling) and both infinities.
Finite values are not actuator limits; controller owns speed/acceleration/yaw
limits. No torque, gain, jump permission or pose target exists. Optional references
remain disabled for health-only operation. Lease starts at mapped creation, not
receipt; mapping uncertainty is subtracted from lease. Heartbeats never extend
it. New accepted reference replaces the lease; duplicate/old references cannot.

## McuStatus payload (44 B, PROPOSED)

| Offset | Bytes/type | Field |
|---|---|---|
| 0 | 8/u64 | sample_mcu_ns, MCU boot monotonic sampling epoch |
| 8 | 4/u32 | last_health_seq, last accepted health packet |
| 12 | 4/u32 | last_estimate_seq, last accepted estimate sample |
| 16 | 4/u32 | last_ref_seq, last accepted reference |
| 20 | 4/u32 | local balance/safety fault mask, controller-owned meanings |
| 24 | 8/u64 | action_mcu_ns, MCU boot monotonic fallback action epoch |
| 32 | 2/u16 | battery_mv, measured millivolts only when battery_valid |
| 34 | 1/u8 | fsm, opaque controller state code |
| 35 | 1/u8 | armed, boolean 0/1 |
| 36 | 1/u8 | operator_enable, boolean 0/1 |
| 37 | 1/u8 | receiver state enum below |
| 38 | 1/u8 | rejection/fallback reason enum below |
| 39 | 1/u8 | battery_valid, boolean 0/1 |
| 40 | 4 | reserved zeros |

Status pack/unpack is supplied; Pi status acceptance policy is outside scope.
Controller populates actual observations; no battery value/action is synthesized.

## TerrainAdvisory (42 B, PROPOSED)

Type 4 is informational Pi→MCU data for the isolated link bench. Payload offsets:
Pi monotonic timestamp u64 at 0, source frame stamp u64 at 8, advisory u8 at 16
(0 NONE, 1 JUMP_CANDIDATE, 2 STOP_CANDIDATE, 3 UNCERTAIN), plane_valid u8 at
17, then six binary32 values at 18, 22, 26, 30, 34, and 38: confidence,
obstacle distance, obstacle height, obstacle width, plane pitch, and camera
height above plane. All six floats must be finite; advisory is at most 3 and
plane_valid is 0 or 1. This record never grants permission or commands motion.

## AdvisoryEcho (27 B, PROPOSED)

Type 5 is MCU→Pi bench diagnostics: sample_mcu_ns u64 at 0, echoed advisory
sequence u32 at 8, advisory u8 at 12, result u8 at 13 (0 accepted, otherwise
the existing `lp_reason` code), accepted count u32 at 14, rejected count u32 at
18, stale u8 at 22, and milliseconds since last accepted advisory u32 at 23.
Stale means no accepted advisory for 300 ms. This echo is not controller state.
Uninitialized sequence fields are zero, interpreted with state (zero is also a
valid wrapped sequence). Clock domain is implicit MCU monotonic, scoped by header
MCU boot ID. Pi cannot subtract these stamps from its own or source clock stamps.

## Clocks, sequences and trusted binding (PROPOSED)

Call `lp_receiver_update(now_mcu_us)` periodically and before accepting packets.
Now is solely current MCU boot monotonic microseconds truncated to u32. Mapped
nanoseconds are floored to microseconds, conservatively increasing age; uncertainty
is rounded up. Age is unsigned modular `now − mapped_epoch_us`. Eligibility:
age + uncertainty ≤150000 us, uncertainty ≤10000000 ns and age-at-send
≤150000000 ns. Mapped future epochs are rejected. Pi age is an additional check,
never the expiry clock. Source/send/creation metadata is preserved without
cross-domain subtraction. Stale/uncertain eligible health is rejected before it
can refresh timers. Only accepted health or reference refreshes link liveness.
Acceptance is distinct from quality and actual controller use.

New per-type packet and estimate sequence delta must be nonzero and <0x80000000.
Frozen estimates may repeat sequence only with identical source/mapped epochs.
New estimates need strictly increasing source_ns and mapped microsecond advance
in the same modular window. Invalid estimate flags preserve estimate sequence/time
high-water marks. Reset/map change starts a new epoch but clears references and
enable authority. A whole reset counter cycle hidden between messages cannot be
detected here; upstream must invalidate or change session in that situation.

Caller must keep timer observation gaps, sequence jumps and queued/mapped stamp
distances below the half-range horizon (2147483648 us, about 35.8 minutes for
timers). This excludes old packet aliases from full timer wraps. Invalidate/rebind
after prolonged suspension, MCU/source clock reset, clock-map invalidation, or
mapping/extrinsic revision change. The codec is not a mapping service. Trustworthy
mappings/uncertainties, unique boot IDs and replay isolation remain UNKNOWN until
separately qualified. The full original/mapped stamps remain ns, not float seconds.

`lp_receiver_bind` is a trusted local provisioning action binding Pi boot ID,
mapping revision and transform revision. Arriving packets cannot bind/switch
sessions. Mapping revision identifies the current Pi–MCU boot pair and must not be
reused across MCU reboot. Negotiation itself is outside the codec. Bind clears
history/lease and latches re-enable pending. Fresh eligible health plus a new
explicit local `lp_receiver_enable` action is required; enable never creates a
reference. Isolate replay traffic from enabled robot receivers. No authentication
or threat model is provided.

## Receiver state machine (PROPOSED)

States: 0 NO_LINK, 1 LINK_OK_NO_ESTIMATE, 2 ESTIMATE_VALID, 3 STALE,
4 REJECTED, 5 RESET_PENDING_REENABLE. Reasons: 0 NONE, 1 WIRE, 2 SESSION,
3 SEQUENCE, 4 FIELDS, 5 AGE, 6 RESET, 7 DISABLED, 8 ASSOCIATION,
9 SILENCE, 10 LEASE. Upstream reason mask is separate.

| Event | Transition / bookkeeping |
|---|---|
| Initialization, no accepted link, or silence ≥150000 us | Any → NO_LINK; withdraw reference, disable, latch re-enable; silence preserves high-water marks |
| Trusted bind or local disable | Any → RESET_PENDING_REENABLE; bind clears history, disable clears authority/lease |
| Packet rejection or stream error callback | Any → REJECTED; withdraw reference; no accepted timer/sequence advances. AGE rejection of already expired estimate retains STALE |
| Accepted reset_counter/map_epoch change | → RESET_PENDING_REENABLE; disable, clear ref/lease history, establish new estimate epoch |
| Accepted ineligible health, pending latch set | → RESET_PENDING_REENABLE; no eligible reference |
| Accepted ineligible health, previously enabled | → LINK_OK_NO_ESTIMATE; withdraw reference; preserve estimate history; advance link timer |
| Accepted eligible health, pending latch clear | → ESTIMATE_VALID; advance link; estimate advances only for new samples |
| Accepted eligible health, pending latch set | → RESET_PENDING_REENABLE; record current health but keep references disabled |
| Explicit local enable with accepted linked fresh eligible health, no rejection | → ESTIMATE_VALID; clear pending latch; no reference created |
| Stored eligible estimate age + uncertainty exceeds 150000 us | Linked → STALE; withdraw reference; heartbeats cannot renew source time |
| Recovery after silence or reset | Health alone cannot clear pending latch; fresh local enable required |
| Accepted reference, ESTIMATE_VALID/enabled, exact estimate/epoch association | Remain ESTIMATE_VALID; replace lease/ref; advance ref/link history |
| Lease deadline reached | Withdraw reference, reason LEASE; estimate/link state stays independent |
| Fresh new valid health after rejection/stale, no reset/silence latch | → ESTIMATE_VALID; reference stays withdrawn until a new reference is accepted |

Periodic update precedence is link silence, estimate staleness, lease expiry;
displayed state may change while pending latch stays set. REJECTED persists until
valid health, timeout or bind; references alone cannot recover it. Invalid packets
immediately withdraw references without refreshing deadlines. Stream error
callback must call `lp_receiver_reject(..., LP_REASON_WIRE)`; tests exercise it.
Incomplete packets have no acceptance effect. Callbacks run synchronously.

In NO_LINK, LINK_OK_NO_ESTIMATE, STALE, REJECTED and RESET_PENDING_REENABLE,
caller withdraws autonomous references and requests a **controller-owned bounded
ramp to zero forward/yaw reference**. ESTIMATE_VALID permits only fresh locally
enabled separately leased references; absent/expired lease still requests that
ramp. The library exposes `reference_active` and zeros its mailbox on withdrawal;
it does not implement physical ramp/stop. Local body estimator, balance/LQR/VMC,
FSM and motor protection remain untouched in every state. Local safety FSM may
override the navigation ramp. Ramp slope, physical stopping and balance-loop
jitter limits are UNKNOWN pending controller review. Host tests do not prove balance.

## Rates, sizes and bandwidth (PROPOSED arithmetic)

| Record | Payload | Decoded with header/CRC | Max wire | Rate |
|---|---:|---:|---:|---:|
| PerceptionHealth | 80 B | 100 B | 102 B | 20 Hz |
| BodyVelocityRef | 44 B | 64 B | 66 B | ≤20 Hz, valid/enabled only |
| McuStatus | 44 B | 64 B | 66 B | 20 Hz |

Estimate sequence follows actual samples, not health publication. UART candidate:
460800 baud, 8N1 (10 bits/byte). Health-only aggregate:
`(102+66)*20*10 = 33600 bit/s`, about 7.29%. With references:
`(102+66+66)*20*10 = 46800 bit/s`, about 10.16%. Full-duplex directions:
Pi→MCU ≤33600 bit/s; MCU→Pi 13200 bit/s. Cap-sized aggregate:
`194*3*20*10 = 116400 bit/s`, about 25.26%. Serialization is ≤2.214 ms for
health, ≤1.433 ms for ref/status, ≤4.211 ms for a cap record. These are arithmetic,
not measured latency or scheduling. Sender keeps only latest unsent per type,
never retransmits expired references, and exposes drops in separate diagnostics.

## Bounded implementation and verification

INFERRED: stream has fixed encoded 193 B and local decoded 192 B plus one fixed
message, no malloc/unbounded queue/recursion. Delimiter work is cap-sized
COBS/CRC/unpack, hence constant bounded work per incoming byte, including delimiter.
Oversize input drops until zero. Callbacks must be bounded; integrate with a
low-priority bounded mailbox, not the balance loop/interrupt. Arbitrary chunks do
not assume DMA/UART boundaries. Binary32 finite checks inspect exponent bits;
parser has no float arithmetic. Target needs 8-bit bytes, standard unsigned
widths, IEEE-754 binary32 and usual integer-compatible object byte order. Float
dimensions are compile-checked; MCU owner must confirm representation/golden bytes.

MEASURED (synthetic host only): all-type frozen C/Python vectors, CRC check,
COBS zero/run/capacity cases, every split/single-byte delivery, malformed/oversize/
truncated recovery, old/duplicate seq, timer/sequence/reset wrap, trusted session
and enable, map jump, bad frame/mapping, NaN/inf, frozen-estimate heartbeat, lease
expiry and silence. UNKNOWN: real files/estimates, STM32 build, link/mapping timing,
controller ramp and physical balance. Commands and limitations are in README.
