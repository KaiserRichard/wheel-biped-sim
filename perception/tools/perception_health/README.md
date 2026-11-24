# Perception health link prototype

This directory specifies and prototypes optional protocol v3 message `0x05 PERCEPTION_HEALTH` from the Pi to the STM32 supervisor. It reports Pi-local age of a D435i-derived pose, estimator state, camera-IMU initialization, map reset changes and raw tracked-feature count. It is a **software prototype**; nothing here is a firmware implementation or a safety/gate PASS.

The message follows the OS owner's `upstream_snapshot/PROTOCOL.md` UART framing by task direction: `A5 5A`, v3 header, shared little-endian u16 frame sequence, up to 64-byte payload, CRC-16/CCITT-FALSE. Existing `tools/link_proto/` is a separate candidate protocol with COBS framing and CRC-32C. This task does not modify it. Differences are framing, length delimitation (SOF/length versus COBS delimiter), CRC polynomial/width (CCITT-FALSE 16-bit versus CRC-32C 32-bit), and selected wire specification/owner (OS protocol versus candidate). Do not mix their frames.

## Files

- `SPEC.md`: fields, clock domains, proposed supervisor use and limits.
- `config.json`: sole location for all numeric timing thresholds, each explicitly PROPOSED.
- `py/perception_health.py`: frame/payload codec, CRC and byte-at-a-time parser.
- `py/supervisor_model.py`: MCU-policy reference model for this message only.
- `py/sender.py`: JSONL/stdin sender over serial; uses pyserial when installed, otherwise POSIX raw termios.
- `c/`: freestanding C99 codec.
- `tests/`: unittest fixtures, C test and frozen vectors. The upstream frames are saved in upstream PROTOCOL.md and repeated in `tests/vectors.txt`; the health-message vector is newly proposed.

## Pi commands

From the repo root, after the upstream receiver/firmware has been separately implemented and reviewed:

```sh
python3 tools/perception_health/py/sender.py --device /dev/ttyAMA0 --baud 115200 --input /path/to/pose_events.jsonl
# Or pipe JSONL pose events into stdin:
cat /path/to/pose_events.jsonl | python3 tools/perception_health/py/sender.py --device /dev/ttyAMA0 --baud 115200
```

Each event is one JSON object, for example:

```json
{"pose_source_d435i_ns": 123456789000, "mapping": {"valid": true, "camera_domain": "D435i_global_time", "pi_domain": "CLOCK_MONOTONIC_RAW", "camera_session": "camera-boot-id", "pi_session": "pi-boot-id", "exchanges_ns": [[1000, 2000, 2100, 1100], [1000001000, 1000002000, 1000002100, 1000001100], [2000001000, 2000002000, 2000002100, 2000001100]]}, "tracking_state": "OK", "map_reset_counter": 0, "imu_initialized": true, "tracked_features": 612}
```

`exchanges_ns` contains the existing `tools.clockmap.ClockMap` four-stamp tuples `[t1_camera,t2_pi,t3_pi,t4_camera]`; the Pi domain must be `CLOCK_MONOTONIC_RAW`, matching `linux_send_ns`, and domain/session identifiers must describe the actual bound clocks. The sender uses ClockMap conversion and adds its uncertainty to the conservative age. The example values are schematic, not calibration evidence. Without a valid mapping the sender flags age unavailable and sends the saturating sentinel; the supervisor model stays HOLD. Sender emits a v3 HEARTBEAT with a random process session ID before/alongside health and repeats it at the interval configured in `config.json`. Serial device, permissions and baud are deployment-specific and must be set only after protocol/firmware agreement.

## Criteria and limits

`pose_age_hold_us` triggers `SOURCE_AGE_HOLD`; `pose_age_timeout_us` triggers `SOURCE_AGE_TIMEOUT` and latched FAILSAFE. `receipt_silence_hold_us` / timeout independently cover missing health frames, `heartbeat_timeout_us` covers liveness, and `transit_bound_us` is the upstream-style conservative allowance added to Pi age and MCU receipt silence. Values are PROPOSED software exercise settings, not validated safety thresholds. In the reference model, a valid mapped age plus OK/OK_KLT, initialized camera IMU state, and clean flags can clear this optional health gate, but FRESH additionally requires an independently supplied `primary_fresh=True` from the existing supervisor path. Health frames alone cannot grant FRESH or motion eligibility; LOST is FAILSAFE. All other perception conditions can only reduce availability. The prototype cannot prove the mapping, exposure timestamp semantics, UART transit bound, firmware parser behavior, STM32 policy integration, physical link performance, safety, or Pi hardware behavior.

## Verification

```sh
python3 -m unittest discover -s tools/perception_health/tests -v
```

Fixtures cover CRC known vector, Python/C frozen vector parity, saved upstream frames, malformed length/CRC recovery, clock saturation, state policy, and a POSIX pty sender/receiver exchange. Passing on fixtures is **not a gate PASS** and does not verify Pi hardware or firmware.
