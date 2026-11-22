# UART Protocol v3 (v1/v2 decoding)

Status: software-tested (`make -C protocol test`). User-reported `hw_smoke1` validated
the previous full-duplex link (322 status frames, zero CRC errors). The new
STATE_EVENT and clock diagnostics are **UNVERIFIED ON HARDWARE**. This protocol
serves the communication and deadline experiments in `PROJECT.md` section 5.3.

## Frame

| Field | Size | Notes |
|---|---|---|
| SOF | 2 | `0xA5 0x5A`. Two bytes make false sync in payload data less likely than one. |
| version | 1 | Encoder emits `3`; decoder accepts `1`, `2` and `3`. Unknown versions are counted and dropped after CRC validation. |
| type | 1 | See below. |
| seq | 2 | Per-sender counter, little-endian. Gaps = lost frames. |
| len | 1 | Payload length, 0..64. Larger values are rejected before reading the payload. |
| payload | len | Little-endian fields. |
| crc | 2 | CRC-16/CCITT-FALSE over version..payload, little-endian. |

Overhead is unchanged at 9 bytes. Version 1 inference payload is 22 bytes (31-byte
frame); versions 2/3 are 26 bytes (35-byte frame). V2 nominal serialization is about
3.04 ms at 115200 baud and 0.38 ms at 921600 baud (8N1, 10 bits per byte; CALCULATED).
These times exclude software queues and are not validated transit bounds.

Why this shape: a SOF + length + CRC frame with a byte-at-a-time state machine is
small enough to read in full, runs unchanged in an ISR, a FreeRTOS task or a Linux
`read()` loop, and its failure counters (`crc_errors`, `len_errors`, `bytes_dropped`)
are measurements in their own right. COBS framing would give cleaner resync but adds an
encoding step that hides less of the OS-level behaviour we want to observe.

## Messages

| Type | Direction | Payload | Purpose |
|---|---|---|---|
| `0x01` HEARTBEAT | Pi to STM32 | `linux_send_ns` u64, **v3 only:** `session_id` u64 | Liveness for the supervisor watchdog. |
| `0x02` INFERENCE | Pi to STM32 | `linux_input_ns` u64, `linux_done_ns` u64, `input_seq` u32, `class_id` u8, `confidence_pct` u8, **v2/v3:** `age_at_send_us` u32 | Result, Pi timestamps and Pi-local input-to-send duration. Step 1 supervisor uses receipt silence only. |
| `0x03` ECHO_REQ | Pi to STM32 | `linux_t1_ns` u64 | Round-trip time and clock-offset estimation. |
| `0x04` NET_STATUS | Gateway to STM32 | 18 bytes: `linux_send_ns` u64, `result_age_ms` u32, `loss_per_mille` u16, `rtt_p95_ms` u32 | Optional v3 network health; sequence uses the existing shared u16 header counter. |
| `0x83` ECHO_RESP | STM32 to Pi | `linux_t1_ns` u64 (phase-1), `mcu_rx_us` u32 (phase-2), `mcu_tx_us` u32 (phase-3) | Linux adds phase-4 on receipt; offset and delay per RFC 4330 section 5 (see `docs/research/R8.md`). |
| `0x84` MCU_STATUS | STM32 to Pi | `mcu_us` u32, `state` u8, `missed_heartbeats` u16, `rx_crc_errors` u16 | Supervisor state (INIT / FRESH / HOLD / FAILSAFE), matching the supervisor state machine. |
| `0x86` INF_RX | STM32 to Pi | 23 bytes: accepted wire seq, supervisor age, extended receipt time, state, losses (layout below) | Optional v3 receipt timing for E3 real. |
| `0x85` STATE_EVENT | STM32 to Pi | 31 bytes: extended MCU time, old/new state, reason, last heartbeat/inference receipt times, losses (layout below) | Optional v3 event timing for E4. |

Pi timestamp fields and the age-at-send calculation must use the same Pi monotonic
clock (the planned sender uses `CLOCK_MONOTONIC_RAW`). `mcu_*_us` fields belong to the
STM32 timer; `age_at_send_us` is a Pi duration. Never subtract a Pi timestamp from
an MCU timestamp without an explicit mapping.

## Version 2 age-at-send contract

KNOWN: Append `age_at_send_us` at payload offset 22, uint32 little-endian. It covers
`t_send - t_input` on one Pi clock, including inference and outbound queue residence,
and must be computed at the actual software-send boundary rather than inference
completion. The library does not sample clocks. Hardware transit accounting is
separate and remains UNKNOWN.

`hea_age_at_send_us(input_ns, send_ns, &age)` converts the same-clock difference,
rounds up to whole microseconds (less than 1 us quantization), and saturates at
`UINT32_MAX` instead of wrapping. It rejects backward clocks or NULL output, leaving
the output unchanged. The sender must drop/count such a failed calculation. The
maximum value denotes saturation or an exact maximum age; future source-age policy
must treat it conservatively, never as an exact finite upper bound for older data.

Version 1 requires exactly 22 INFERENCE payload bytes; versions 2/3 require exactly
26. There is no silent version/length fallback. Legacy decoding sets the member to
zero for initialization, but **the age is absent**, not a measured zero. Callers
must inspect `frame.version` for presence and label legacy observations receipt
silence. ECHO_REQ, ECHO_RESP and the nine-byte base MCU_STATUS layouts are unchanged
across all three versions; the optional v3 status diagnostic extension is below. Old decoders reject newer versions; update both peers before physical-link tests.

Acceptance: v2 round-trip and endian field position; independent captured v1 golden
frame; both version/length mismatch directions; unknown version; zero/sub-us/exact
us conversion; regression; exact maximum, overflow and UINT64_MAX durations;
supervisor receipt-only behavior for both versions.

## Version 3 session and local generation contract

V3 HEARTBEAT appends `session_id`, uint64 little-endian at payload offset 8:
16 payload bytes, 25 frame bytes. V1/v2 HEARTBEAT remains exactly 8 payload bytes.
The decoder initializes absent legacy session IDs to zero; presence is determined
by version, not the value. Zero is a valid v3 ID. All version/length mismatches
are rejected, with no fallback. The packers and encoder emit v3; v3 INFERENCE
retains the v2 layout and age semantics.

The bridge must choose a new session ID on every process restart, keep it constant
for that incarnation, and send HEARTBEAT before results/echoes. IDs are opaque
identities, not sequence counters or timestamps. Avoid reuse across incarnations.
A first explicit session (including after legacy traffic), or a changed session,
clears both wire and input sequence history and all receipt/healthy history.
The establishing heartbeat is accepted even if its wire sequence restarts at zero.
The supervisor returns to safe INIT unless FAILSAFE is already latched, in which
case healthy observation and explicit local rearm are still required. Expiry is
checked first, so a restart heartbeat at a failsafe deadline cannot bypass the latch.
Same-session heartbeats obey the normal shared wire sequence checks. Once an
explicit session is known, legacy heartbeats are ignored to prevent downgrade;
legacy inference decoding remains supported. A legacy-only sender has no automatic
restart detection and its sequence history is preserved.

`generation` is a separate uint64 **local queue stamp**, never transmitted.
The supervisor initializes it to zero and increments it on successful rearm and
on session establishment/change, invalidating outstanding queue entries. The
accepted session heartbeat belongs to the closing generation; subsequent entries
must capture the new generation. Rearm preserves the session and both sequence
watermarks, clears receipt/health flags, and requires new traffic for FRESH.
Every queued event/frame must retain the generation and actual receipt time
captured at handoff; never restamp on dispatch. The single owner must serialize
capture, update and rearm. A mismatched generation is rejected before session,
sequence or timer renewal, even at the same tick; the saturating uint64
`rejected_generation_events` counter counts each rejected entry, including repeated
dispatches. Deadlines still run. Failed rearm does not increment generation.
Generation exhaustion refuses rearm/session reset; reinitialize the core and clear
the entire queue before exhaustion or any full core reinitialization.

These rules assume ordered delivery from one bridge. CRC is integrity checking,
not authentication. A historical session replayed on the wire with a newly captured
local generation is indistinguishable from another restart; arbitrary session
replay protection is not provided. Results/echoes have no session ID, so sender
ordering and preserved queue stamps are essential across a restart. Generation
rejects old local queue entries; session ID permits counter restart. Neither is a
source-age measurement.

Acceptance includes exact v1/v2/v3 heartbeat layouts, v1/v2/v3 inference decoding,
malformed lengths, legacy-to-session transition, same-session duplicates, restarted
wire/input counters, all supervisor states, and same-tick queue rejection/counting.

## Known limitation

After a CRC or length error the parser resumes SOF hunting at the next byte and does
not rescan the rejected bytes, so a valid frame that starts inside a corrupted one is
lost. One false candidate consumes at most a maximum-size frame (73 bytes); that can
overlap more than one shorter valid frame. CRC does not guarantee lossless resync.

## Reproducible noise test

KNOWN: Noise uses test-local xorshift32, seed 12345, with defined uint32 arithmetic.
The noise-only FNV-1a fingerprint is `8c9204cd`. All wire bytes are `uint8_t`.
The CRC known vector also uses an explicit uint8 array; no plain-char wire buffers
exist. Build both the protocol and supervisor tests with `-fsigned-char` and
`-funsigned-char`; CI exercises both modes.

MEASURED (software test on macOS and Ubuntu aarch64): the fixed noise stream recovers
20,000/20,000 frames, zero false accepts, zero CRC errors and one rejected oversize
noise header. These counts are asserted, not just a 95% recovery threshold.

The earlier `srand(12345)`/`rand()` stream was platform-dependent. Captured Pi losses:

| Real sequence | Noise immediately before frame (hex) | Parser behavior |
|---|---|---|
| 13355 | `bb fd a5 5a` | Noise SOF starts a false candidate; first five real-frame bytes `a5 5a 01 02 2b` become its header: len `0x2b` = 43; after the real frame, payload index is 26 |
| 13356 | `9c b4 b1 d4` | Continues false payload to index 30; consumes the start of this frame, fails CRC once, resumes SOF hunting |

Captured real frames, including their unchanged CRCs:

```
13355: a55a01022b3416887766554433221100ffeeddccbbaa99efbeadde075d70a8
13356: a55a01022c3416887766554433221100ffeeddccbbaa99efbeadde075d5d60
```

`test_false_sync_consumes_two_frames` reproduces these bytes without libc random
functions: both frames are lost, one false candidate fails CRC, and sequence 13357
decodes. This proves the previous two losses are resync behavior under a different
input stream, not a byte-signedness or platform-specific parser defect.

## Optional firmware diagnostics in v3 MCU_STATUS

**UNVERIFIED ON HARDWARE.** To observe ISR/queue contention and validate GPIO clock
capture, v3 status may contain 33, 38, 58 or 64 bytes instead of nine. The first nine
bytes retain the base layout. Six little-endian u32 fields follow:

| Offset | Field | Meaning |
|---:|---|---|
| 9 | last_capture_us | Last dequeued TIM2_CH1 rising-edge timestamp, raw us |
| 13 | capture_count | Saturating number of dequeued captures; zero means absent |
| 17 | rx_dropped | Saturating IRQ stream record losses; DMA discarded batches/possible stolen bytes |
| 21 | uart_errors | Saturating error-bearing SR samples (one sample may have multiple flags) |
| 25 | queue_dropped | Saturating frame/echo queue enqueue failures |
| 29 | capture_lost | Saturating capture queue loss plus hardware overcapture |

`hea_pack_mcu_status` still emits nine bytes. Firmware uses the explicit
`hea_pack_mcu_status_extended` API. The base decoder accepts nine bytes for v1/v2/v3
and 33/38/58/64 bytes for v3; the extended decoder accepts those four v3 extensions.
The current extended packer emits 64 bytes. Absent trailing
fields decode as zero, except `tick_rate_ppm` which is `INT32_MIN`; use the payload length to distinguish absent diagnostics. All other lengths and
versions are rejected. Existing strict nine-byte receivers must be updated before
using this firmware. This is an optional v3 diagnostic payload, not a change to
HEARTBEAT/INFERENCE/ECHO or their sequencing. Base counters in the firmware mean
heartbeat-timeout latch transitions and saturated parser CRC failures. Tests cover
wire round-trip, field byte order, every payload length, versions and wrong types.

The 38-byte status appends `clock_fallback` (u8, offset 33: 1 when HSE was
requested but failed its startup timeout, 0 for HSE success or deliberate HSI)
and `state_event_lost` (u32 LE, offset 34). The latter reports saturating event-ring
overflow even if no subsequent transition occurs. Older strict 33-byte status
decoders need updating; base nine-byte packing remains unchanged.

The 58-byte status additionally appends five little-endian u32 fields:

| Offset | Field | Meaning |
|---:|---|---|
| 38 | uart_ore | Saturating overrun flag observations |
| 42 | uart_fe | Saturating framing error flag observations |
| 46 | uart_ne | Saturating noise flag observations |
| 50 | uart_pe | Saturating parity error flag observations (8N1 normally has none) |
| 54 | max_irq_masked_cycles | DWT maximum observed masked-body duration since scheduler startup |

The total `uart_errors` remains at offset 21 for existing analysis. It counts
error-bearing SR samples, not the sum of the four flags. The Pi CSV appends the
five new columns, leaving them empty for 9/33/38-byte status (absence is not zero).
At 84 MHz divide cycles by 84 for microseconds. See the firmware README for
measurement coverage/overhead; this is runtime instrumentation, not a proven WCET.

The 64-byte status adds signed little-endian `tick_rate_ppm` (i32, offset 58)
and `tick_window_ms` (u16, offset 62). These append matching Pi CSV columns; both
are blank for older payloads. The ratio is
`1e6 * (elapsed_SysTick_ticks * 1000 / elapsed_TIM2_us - 1)`, rounded to the
nearest integer ppm. Positive values mean SysTick runs fast relative to TIM2.
The startup sample covers the first second after the scheduler starts; subsequent
samples cover ten-second windows selected by TIM2, not SysTick. Status repeats the
latest completed sample. `INT32_MIN` and a zero window mean no valid sample yet.
A one-tick endpoint uncertainty is about 1000 ppm at startup and 100 ppm over 10 s;
compare repeated windows rather than interpreting one-tick quantization as drift.
Both clocks share the PLL source, so this checks their **relative** rate and lost
ticks, not oscillator accuracy against the Pi. It uses the full existing 64-byte
payload limit and adds no message type or dependency. This supports the OS clock
and scheduler experiment while keeping the UART diagnostics bounded.

DMA firmware timestamps ECHO phase-2 and supervisor receipt when the RX task reads the
last CRC byte from RAM. IRQ fallback retains per-byte RXNE ISR timestamps. DMA
adds up to one 1 ms poll period plus scheduling/drain latency, or an earlier IDLE
wakeup; these timestamps must not be interpreted as hardware wire-arrival times.

## Optional v3 STATE_EVENT (0x85, MCU → Pi)

**UNVERIFIED ON HARDWARE.** This new optional type leaves protocol version 3 and
all existing control-message layouts unchanged. Older receivers may skip the
unknown type after normal framing/CRC validation. Payload is exactly 31 bytes
(40 bytes including framing), little-endian, without alignment padding:

| Offset | Type | Field |
|---:|---|---|
| 0 | u64 | mcu_us: extended TIM2 timestamp of state evaluation/change |
| 8 | u8 | old_state |
| 9 | u8 | new_state |
| 10 | u8 | reason |
| 11 | u64 | last_hb_mcu_us: last accepted heartbeat's receipt timestamp |
| 19 | u64 | last_inf_mcu_us: last accepted inference's receipt timestamp |
| 27 | u32 | lost_count: saturating cumulative event-ring losses at dequeue |

State values retain INIT=0, FRESH=1, HOLD=2, FAILSAFE=3. Reasons follow the
supervisor enum: NONE=0, READY=1, RESULT_SILENCE=2, HEARTBEAT_TIMEOUT=3,
RESULT_TIMEOUT=4, REARM=5, CLOCK_REGRESSION=6, SESSION_CHANGE=7,
INVALID_CONFIG=8, SOURCE_AGE_HOLD=9, SOURCE_AGE_TIMEOUT=10.
`UINT64_MAX` means no frame of that kind has ever been accepted since core
initialization; zero is a valid timestamp. These snapshots retain the most recent
accepted receipt even across session/rearm health resets, and rejected frames do
not change them. Every actual state change is recorded; initial setup is not a
transition. A single update can emit multiple events, even at the same timestamp.

The supervisor owner records transitions in a static 16-entry FIFO ring without
blocking. The supervisor notifies the sole status/TX task when events are pending; TX
drains them on wakeup (also polls every tick), ahead of echoes and periodic status. Events do not wait for the 100 ms status period and no ISR transmits them.
No finite queue guarantees losslessness under unlimited traffic: on overflow the
new event is dropped, existing event order is retained, and the loss counter
saturates. Frame `seq` uses the existing shared MCU TX sequence across all types;
a sequence gap alone does not indicate an event loss. TIM2 is extended across its
32-bit wraps, sampled every supervisor release. Timestamp precision is nominally
1 us, but actual decision latency still includes the supervisor's 1 ms release
period and execution time; this is not a physical GPIO edge timestamp.

The bridge logs `state_events.csv` (override `--state-events-csv PATH`):
`seq,t_rx_pi_ns,mcu_us,old,new,reason,last_hb_mcu_us,last_inf_mcu_us,lost_count,t_mcu_pi_est_ns`.
The last field is the affine inverse of the current echo clock fit (minimum-delay
offset until a drift fit is available), in Pi CLOCK_MONOTONIC_RAW nanoseconds.
It is blank before an estimator exists; RX time is logged separately. For reference
Pi time R, offset O and drift D, mapped time is
`1000 * (R + (mcu_us - R - O)/(1 + D))`. Extended event timestamps align the
raw echo clock's boot epoch, including when capture starts after a timer wrap;
observations must remain within half a timer wrap of the last echo sample. Mapping
is an estimate with UART/scheduling uncertainty, not a measured common clock.

## Optional v3 INF_RX (0x86, MCU → Pi)

This reports **accepted** inference frames, after the supervisor evaluates their
age/state. It supports the OS freshness comparison at the MCU boundary, rather
than a Pi PTY proxy. No scheduling, pinout, session or acceptance rules change.
The implementation is locally tested and **UNVERIFIED ON HARDWARE**.

Payload is exactly 23 bytes, little endian; v1/v2 and other lengths are rejected:

| Offset | Type | Field |
| --- | --- | --- |
| 0 | u16 | `wire_seq`: accepted INFERENCE header sequence (not INF_RX header sequence) |
| 2 | u64 | `age_mcu_us`: supervisor source-age estimate at this acceptance evaluation |
| 10 | u64 | `mcu_rx_us`: extended TIM2 inference receipt time |
| 18 | u8 | `state`: supervisor state after evaluating this accepted frame (0–3) |
| 19 | u32 | `lost_count`: saturating cumulative INF_RX ring overflows since MCU boot |

For v2/v3 inference the age is `age_at_send_us + (evaluation_us - receipt_us)
+ transit_bound_us`, using the supervisor's saturating u64 arithmetic. Thus it
includes supervisor queue delay and the configured **provisional** transit
allowance (default firmware 5 ms); it is not the mapped receipt AoI. Legacy v1
acceptances report the supervisor's receipt-silence fallback age, with no source
age or transit allowance; E3 real uses v3 exclusively. Rejected malformed,
duplicate, old-input, generation-mismatched or out-of-order frames create no
report. Accepted traffic still reports its age/state in latched FAILSAFE.

The supervisor records in a static 16-entry FIFO; overflow drops the new report
and increments `lost_count` up to UINT32_MAX. Session changes/rearm do not erase
queued reports or cumulative losses. The sole status TX task serializes ring
pops with the supervisor using scheduler suspension (IRQs stay enabled), drains
at most four reports per iteration, and samples the current cumulative loss
counter at pop. No TX happens in the supervisor or an ISR. A single report uses
32 wire bytes; at 10 Hz it adds 320 bytes/s, about 0.35% of 921600-baud 8N1
capacity. Batching is unnecessary at this rate.

`hea_bridge` writes `inf_rx.csv` (override `--inf-rx-csv PATH`):

```text
seq,frame_id,t_input_ns,mcu_rx_us,t_mcu_pi_est_ns,age_mcu_us,state,lost_count,aoi_mapped_us,t_rx_pi_ns
```

The bounded 2048-slot correlation history records each completed outbound
INFERENCE's wire sequence and original result identity. A consumed, absent or
replaced slot leaves identity fields empty, rather than inventing a match.
Full receipt timestamps align echo wrap epochs, as STATE_EVENT does. Mapped Pi
time uses the inverse affine clock-sync estimator and Pi CLOCK_MONOTONIC_RAW;
`aoi_mapped_us = (t_mcu_pi_est_ns - t_input_ns) / 1000`. Missing estimator leaves
mapped fields empty. `age_mcu_us` is logged independently of that estimator.
The report's Pi receive time is diagnostic and is never used as MCU receipt time.

The E3 real summary remaps receipt/state times using the nearest fitted echo
sample (including retrospective startup fits), at most two seconds away. This
retains early accepted frames before an online estimator existed. Fits must
have finite positive slope and RMS at most 200 us; empty identity, nonzero INF_RX
loss, nonzero STATE_EVENT loss or missing state/clock evidence invalidates the
run. Online mapping columns remain available for comparison with this replay.

## Optional v3 NET_STATUS (0x04, gateway → STM32)

NET_STATUS uses the existing framing, CRC, 64-byte payload limit and shared
per-gateway u16 header sequence. There is no payload sequence or version bump.
Payload is exactly 18 bytes (27 frame bytes), little-endian without padding:

| Payload offset | Type | Field | Meaning |
|---:|---|---|---|
| 0 | u64 | linux_send_ns | Gateway CLOCK_MONOTONIC_RAW timestamp at first UART write attempt |
| 8 | u32 | result_age_ms | Age of newest network inference input at send, rounded up to ms |
| 12 | u16 | loss_per_mille | Missing / expected results in the latest 64 sequence positions, rounded up ×1000 |
| 14 | u32 | rtt_p95_ms | Nearest-rank P95 of available gateway-measured result RTT samples in that window |

Age and RTT saturate at UINT32_MAX; this also denotes unknown. Empty-window loss
is zero (no observations, not proof of a working network). RTT is not inference
execution time or the UART ECHO round trip. Versions other than 3, wrong lengths
and loss values above 1000 are rejected. Unknown types may be skipped by older
v3 receivers. HEARTBEAT must establish the gateway session before NET_STATUS;
all outgoing types share one counter and sender. Session/generation rejection
rules apply to NET_STATUS as well.

The STM32 never subtracts linux_send_ns from its own clock. It estimates network
result age as result_age_ms × 1000 + local elapsed since receipt + the existing
provisional transit_bound_us, with saturating arithmetic. First valid NET_STATUS
enables an independent monitor. Result age or NET_STATUS silence at the existing
result_hold_ticks enters HOLD; at result_failsafe_ticks it latches FAILSAFE.
The local Linux HEARTBEAT and INFERENCE checks remain active. NET_STATUS does
not renew either timer. Loss and RTT are diagnostics; there are no new thresholds
or reason codes. Source-age and receipt-silence reasons are reused.

Monitoring remains required across local rearm and gateway session changes;
receipt history is cleared, and a new NET_STATUS is required for FRESH. HOLD
recovers when all monitors are healthy. FAILSAFE still requires uninterrupted
health for rearm_healthy_ticks, explicit local rearm, then new traffic. Firmware
currently exposes no rearm command: recovery from its latched FAILSAFE requires
board reset. No wire message grants rearm.

See [CN integration](../docs/CN_INTEGRATION.md) for the gateway IPC extension,
startup policy, reproducible host tests and hardware verification limits.
