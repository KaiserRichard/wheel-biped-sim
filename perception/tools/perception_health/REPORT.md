# phase-7 implementation report

## Built

- `SPEC.md` defines proposed v3 message `0x05`, a fixed 19-byte LE payload, all clock domains, D435i-to-Pi mapping rules, saturation and supervisor downgrade behavior.
- `config.json` is the single source for proposed timing values and includes a rationale for each.
- Python codec/parser and C99 codec implement upstream `A5 5A` framing and CRC-16/CCITT-FALSE. The sender sends v3 HEARTBEAT with a per-process session before health messages and uses `tools.clockmap.ClockMap` for source-time mapping, including its uncertainty.
- The Python supervisor model exercises source age, receipt silence, heartbeat timeout, tracking state, map reset changes, session reset and explicit latched rearm.
- Tests include upstream saved protocol frames, an independent frozen proposal vector, byte-at-a-time and corrupted-stream parsing, pty sender/receiver traffic and sender termination.

## Verification (MEASURED software only)

Command: `python3 -m unittest discover -s tools/perception_health/tests -v`

Result after review fixes: 24 tests passed on the development Mac / Python 3.11; C encoder/decoder compiled with `cc -std=c99 -Wall -Wextra -Werror` and matched the Python frozen bytes. The imported ClockMap suite also passed all 37 tests. Exact commands and full stdout are saved in `tests/last-run.log`. These are fixture/software checks, not a gate PASS.

Covered paths: CRC `123456789 -> 0x29B1`; captured upstream v1 frames; new v3 frozen vector; CRC/length error counts and recovery; source-age freshness, HOLD and FAILSAFE; LOST latch/rearm qualification; heartbeat and receipt silence; missing mapping and clock-domain mismatch; map-reset downgrade; duplicate-sequence rejection; health-only non-promotion; invalid IMU boolean handling; pty wire round-trip; sender killed and silence; saturation and clock regression.

## Not verified / open risks

- **UNKNOWN / not verified on hardware:** Pi UART device/baud/permissions, physical D435i source timestamp semantics, valid online ClockMap production/session management, end-to-end send/receive latency and transit bound, STM32 parser/policy integration, actual ORB-SLAM3 feature/state extraction, camera IMU initialization reporting, safety effect, or hardware gate status.
- The authoritative upstream supervisor and OS protocol snapshot are read-only. The STM32 firmware does not yet receive `0x05`; receiver acceptance, shared sequence handling and mapping to existing reason codes still require a separate reviewed firmware change.
- PROPOSED timing values need review and measurement before any physical use. CRC is not authentication. The optional message can only downgrade, and the balance controller remains STM32-owned.
- The upstream ORB-SLAM3 `Tracking.h` was not present in this checkout; names follow the task-specified enum and should be confirmed against the selected upstream source revision before integration.
- The upstream parser contract intentionally does not rescan a failed candidate's bytes; tests preserve that behavior. C code provides fixed frame codec functions, not a streaming ISR parser.

Proposed commit message: `Add proposed UART perception health message prototype`
