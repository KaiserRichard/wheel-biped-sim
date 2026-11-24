# MCU PERCEPTION_HEALTH receiver candidate

This freestanding C99 candidate accepts only protocol v3 `PERCEPTION_HEALTH` (0x05) and `HEARTBEAT` (0x01) frames. It reuses `../perception_health.c` for health-frame decode/CRC. It is a firmware-owner integration candidate, not firmware, a safety implementation, or a gate PASS.

## Host parity check

From the repository root:

```sh
make -C tools/perception_health/c/mcu test
```

`generate_vectors.py` executes `py/supervisor_model.py` and writes the scenario outputs used by the C harness. Scenarios cover sequence gaps and duplicates, CRC rejection, source-age HOLD and FAILSAFE, valid saturated age, unavailable age, LOST, silence expiry, and u32 MCU timer wraparound. One scenario reuses tracking/reset labels from a saved input excerpt whose provenance labels the association MEASURED and fake estimates SYNTHETIC. The Makefile first builds with `cc -std=c99 -Wall -Wextra -Werror -pedantic`; it also runs address/undefined sanitizers when supported. Results on host fixtures verify software behavior only.

## Firmware call sites

1. Initialize one `struct ph_rx` and `struct ph_policy` during supervisor startup. Map values from `tools/perception_health/config.json` into `struct ph_policy_config`; that JSON remains the sole source of proposed timing values.
2. In the UART RX ISR, pass each received byte and its captured MCU timer sample once to `ph_rx_feed_at()`. It validates framing, version, fixed health payload length and CRC. Valid health and heartbeat messages enter a fixed SPSC queue with the final byte's receipt timer sample. Queue overflow increments `queue_drops`; a dropped report must not refresh watchdogs.
3. In one supervisor task, drain with `ph_rx_pop()` and pass `event.received_at_mcu_us` (not task dispatch time) to `ph_policy_update()` along with a current MCU timer sample. Call `ph_policy_update(..., NULL, ...)` periodically even when no frame arrives.
4. Combine the returned `NONE`, `HOLD`, or `FAILSAFE` request with the existing supervisor's own state. This module cannot request FRESH, make a primary input fresh, clear an upstream latch, or authorize movement. Only the supervisor's existing local rearm path can release its latch.

The parser stores at most one 73-byte protocol frame plus four event slots (three usable ring entries), with no heap allocation or floating point. The ISR/task handoff assumes a single producer and single consumer and atomic 8-bit index access on the target. Platform memory barriers or critical sections must be added if the MCU/compiler memory model requires them. Feed must not be called concurrently from multiple UART sources.

MCU timer elapsed calculations use modulo-u32 subtraction and require samples less than 2^31 microseconds apart. `linux_send_ns` and `pose_age_us` are Pi-owned values; the receiver never compares Pi timestamps to MCU timer values. `pose_age_us == UINT32_MAX` with validity flags set is conservatively timed out; with validity flags clear it is unavailable and requests HOLD. Existing upstream reason names are preserved where applicable: `SOURCE_AGE_HOLD`, `SOURCE_AGE_TIMEOUT`, `RESULT_SILENCE`, `RESULT_TIMEOUT`, `HEARTBEAT_TIMEOUT`, `CLOCK_REGRESSION`.

## UART airtime estimate

The existing `heartbeat_period_ms` in `config.json` is PROPOSED at 100 ms. For a matching proposed 10 health frames/s cadence, one health frame is 28 wire bytes. Using 10 serial bits per byte (start, 8 data, stop), frame airtime is CALCULATED as `28 * 10 / baud`:

| Baud | One 28-byte health frame | UART occupancy at 10 frames/s |
|---:|---:|---:|
| 115200 | 2.431 ms | 2.431% |
| 921600 | 0.304 ms | 0.304% |

These estimates exclude heartbeat frames, inter-frame gaps, driver buffering and scheduling. They are not measured link performance. The 10 Hz health cadence is a proposal aligned to the configured heartbeat interval, not a new validated threshold.

## Host-only memory measurement

`make test` compiles the receiver and policy objects and reports their host object sizes with `size`. Latest host build after review: `ph_rx.o` 1,940 bytes (`__TEXT` 1,684, `__DATA` 0, other object sections 256); `ph_policy.o` 2,873 bytes (`__TEXT` 2,585, `__DATA` 64, other object sections 224). Total reported object size is 4,813 bytes. **MEASURED on the development Mac host only**; this excludes target ABI/alignment effects, stack, caller-owned `struct ph_rx`/`struct ph_policy`, and final firmware linker layout. It does not predict STM32 flash or RAM use.

## Limits

Fixture parity and host sizes cannot prove target compiler behavior, interrupt latency, UART baud accuracy, physical transit bounds, queue sizing, clock capture accuracy, firmware integration, rearm behavior in the full supervisor, or safety. Timing values in `config.json` remain PROPOSED pending firmware-owner review and hardware evidence.
