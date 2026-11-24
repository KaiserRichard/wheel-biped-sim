# Proposed UART v3 PERCEPTION_HEALTH (0x05)

**Status: PROPOSED; software reference only.** Pi to STM32 optional health input. Uses protocol v3's existing shared u16 sender sequence across all message types. It does not alter the STM32 balance controller or grant motion authority. Any missing, degraded, stale, malformed, or lost perception report can only downgrade the external supervisor state.

## Framing and payload

Use `tools/perception_health/upstream_snapshot/PROTOCOL.md`: `A5 5A | version=3 | type=05 | seq:u16 LE | len=19 | payload | CRC:u16 LE`. CRC-16/CCITT-FALSE covers version through payload; CRC is little-endian. Maximum payload remains 64. There is no negotiation or alternate payload length. The message is optional to protocol v3 peers that do not implement this proposal.

| Offset | Field | Encoding and meaning |
|---:|---|---|
| 0 | `linux_send_ns` | u64 LE, Pi CLOCK_MONOTONIC_RAW at software send boundary |
| 8 | `pose_age_us` | u32 LE, rounded up `(t_send_ns - t_pose_source_ns)/1000`; saturates at UINT32_MAX |
| 12 | `tracking_state` | u8 enum below |
| 13 | `imu_initialized` | u8 boolean 0/1; camera IMU initialization status, never STM32 body IMU status |
| 14 | `map_reset_counter` | u16 LE wrapping increment on map reset, merge, relocalization discontinuity, or other pose-frame discontinuity; consumer detects change as reset, following MAVLink ODOMETRY reset-counter semantics |
| 16 | `tracked_features` | u16 LE raw count reported by estimator; no covariance or confidence fabricated |
| 18 | `flags` | u8 bitfield |

Tracking enum matches ORB-SLAM3 upstream `Tracking.h` at commit `4452a3c`: `NO_IMAGES_YET=0`, `NOT_INITIALIZED=1`, `OK=2`, `RECENTLY_LOST=3`, `LOST=4`, and `OK_KLT=5`. Upstream `SYSTEM_NOT_READY=-1` is represented as `255` (`0xFF`) in this u8 field. These values were locally source-verified at `<local workspace>/Desktop/Robotics Learning&Work space/wbr-perception-phase1/repos/ORB_SLAM3/include/Tracking.h`, lines 121–129, commit `4452a3c`. Any other u8 value is invalid and MUST downgrade to HOLD (`SOURCE_AGE_HOLD`); LOST downgrades to latched FAILSAFE (`SOURCE_AGE_TIMEOUT`).

| Bit | Name | Set/clear meaning |
|---:|---|---|
| 0 | `age_valid` | Set only when source time was mapped and age calculation did not regress |
| 1 | `mapping_valid` | Set only while the D435i-to-Pi monotonic mapping is valid for this camera/host session |
| 2 | `degraded` | Set for any estimator condition the producer wants the supervisor to treat as degraded |

Age validity and mapping validity are independently represented but both are required for FRESH. If either is absent, `pose_age_us` is UINT32_MAX and must be treated as unknown, never zero. Saturation at UINT32_MAX is conservative and is always beyond proposed FAILSAFE age. Reset counter wraps modulo 65536; only a change is meaningful, not its magnitude. Tracked feature count saturates at UINT16_MAX.

## Clock-domain contract

| Time field | Owner clock | Units / epoch | Production and allowed arithmetic |
|---|---|---|---|
| `linux_send_ns` | Pi Linux CLOCK_MONOTONIC_RAW | ns since unspecified boot epoch | Sample at actual software write boundary. Pi-only; compare only with Pi raw-monotonic timestamps from the same boot/session. |
| `t_pose_source_ns` (not transmitted) | D435i hardware/global timestamp domain | ns in device clock epoch | Camera exposure/source time supplied by pose event. It cannot be subtracted directly from a Linux timestamp. |
| mapped `t_pose_source_ns` (not transmitted) | Pi CLOCK_MONOTONIC_RAW | ns in Pi boot epoch | Explicit affine map `pi_ns = scale * d435i_ns + offset_ns`, with identified device and host sessions. Mapping must be refreshed/invalidated using qualified `tools/clockmap` evidence. Without valid mapping, age is UNAVAILABLE and validity flags clear. |
| `pose_age_us` | Duration on Pi CLOCK_MONOTONIC_RAW | ceil microseconds | `linux_send_ns - mapped_pose_source_ns` on same Pi clock only. MCU never subtracts this duration from MCU timer values. |
| UART `received_at_ticks` | STM32 extended local timer | us since MCU timer epoch | Captured at completed-frame receipt; model adds elapsed MCU-local silence to Pi duration with an explicit proposed transit allowance, as upstream source-age policy does. |

No age values are computed by comparing Pi and MCU epochs. `tools/clockmap` provides mapping primitives/evidence, but the sender accepts a supplied affine map and does not claim to validate its provenance or uncertainty. Mapping validity must expire on either clock session change, clock step/regression, or invalid fit.

## Supervisor recommendation

Thresholds live only in `config.json`; all are **PROPOSED**, not measured limits. A valid HEARTBEAT/session and health report can only clear this optional health gate; the existing primary supervisor input must independently be FRESH before the combined model can report FRESH. This message cannot grant motion eligibility. On each accepted report, compute conservatively `source_age = pose_age_us + (MCU_now - MCU_received_at) + transit_bound_us`, saturating u64. A valid age above `pose_age_hold_us` enters HOLD with `SOURCE_AGE_HOLD`; above `pose_age_timeout_us` enters the existing FAILSAFE path with `SOURCE_AGE_TIMEOUT`. Tracking LOST immediately enters latched FAILSAFE. Any other state besides OK/OK_KLT, degraded flag, absent age/mapping, or map reset change enters/remains HOLD. Receipt silence enters HOLD at its configured threshold and FAILSAFE at timeout; heartbeat timeout uses the existing heartbeat reason. FAILSAFE is latched until an explicit local rearm after the configured uninterrupted healthy observation interval. Session change clears freshness history but cannot release a latched FAILSAFE. Invalid or duplicate/out-of-order frames do not refresh timers.

The upstream supervisor implementation must add 0x05 as an event kind and preserve its existing sequencing, generation, capture receipt timestamps, expiry-before-acceptance, and rearm semantics. The Python model here describes intended behavior, not firmware integration. PERCEPTION_HEALTH can only DOWNGRADE; balance control, LQR, VMC, PID/PD, actuator commands and hard safety remain STM32-owned and do not depend on this message.

## Design choices

- Fixed 19-byte payload uses the full requested observables without inventing covariance.
- `pose_age_us` is u32 to mirror existing v2 `age_at_send_us`; saturation means “at least this old,” never wrapping into fresh.
- Mapping is explicit and flags unavailable ages instead of substituting receive time. The sender takes mapping parameters from each JSONL pose event so fixture events can carry provenance alongside the mapped values.
- `map_reset_counter` follows reset-counter semantics: increment on map reset, merge, relocalization discontinuity, or another pose-frame discontinuity, not every map update. Wrap is tolerated through inequality; loss of multiple increments remains detectable as a change.
