# phase-22 implementation report

## Built

- Added a byte-fed protocol v3 receiver restricted to `PERCEPTION_HEALTH` and `HEARTBEAT`, reusing the existing C codec/CRC and using fixed buffers only. It captures final-byte MCU receipt time through `ph_rx_feed_at()` and hands validated events through a fixed SPSC ring.
- Added the candidate source-age policy with wrapping u32 MCU microsecond elapsed arithmetic, sequence duplicate/old rejection, session handling, heartbeat/receipt silence, pose age, LOST, unavailable/saturated age behavior and optional primary-fresh input. Its API returns only `NONE`, `HOLD`, or `FAILSAFE`; it has no API to grant FRESH or clear a latch. The existing supervisor retains rearm ownership.
- Added Python-generated stepwise parity scenarios, C receiver rejection/queue tests, a strict C99 Makefile target and firmware-owner integration guidance.

## Verification

Commands: `make -C tools/perception_health/c/mcu test` and `python3 -m unittest discover -s tools/perception_health/tests -v` from the repository root.

Result: **PASS** on the development Mac host. The strict `cc -std=c99 -Wall -Wextra -Werror -pedantic` build passed, scenario parity against `py/supervisor_model.py` passed, and address/undefined sanitizer run passed. The existing perception-health Python unittest suite also passed (28 tests). `size` measured the two host candidate objects at 4,813 bytes combined; detailed host-only values are in `README.md`. Full command output is saved in `tests/last-run.log`. These are software fixture checks, not a gate PASS.

Scenarios exercise good health, sequence gaps and duplicate rejection, invalid CRC, receipt silence HOLD/FAILSAFE, source age HOLD/FAILSAFE, unavailable age HOLD, valid saturated age FAILSAFE, LOST FAILSAFE, and MCU u32 wraparound. One additional scenario takes tracking/reset labels from the saved `integration/tests/fixtures/real_excerpt.jsonl`; its provenance states that the D435i input association is MEASURED while fake-engine estimates are SYNTHETIC, so it is not real estimator output. C receiver tests exercise valid health and heartbeat decoding, CRC rejection and queue overflow.

## Not verified / risks

- **UNKNOWN / not verified on hardware:** STM32 compiler/ABI, ISR timing, UART DMA/interrupt integration, atomic index and memory-order guarantees, actual queue sizing, timer capture semantics, baud accuracy, physical transit allowance, firmware integration, balance-system interaction or safety.
- Timing values remain PROPOSED in `tools/perception_health/config.json`. 10 health frames/s aligns with the existing proposed heartbeat interval; airtime is CALCULATED in `README.md`, not measured.
- The repo's `tools/run_all_tests.sh` invokes Make targets from a fixed list and does not currently list `tools/perception_health/c/mcu`; this task-owned-path change cannot add the suite to that out-of-scope runner. Run the documented Makefile command directly. The Python vector generator executes the model used as reference.
- C99 volatile SPSC indices require target-specific review for atomicity and memory barriers. The ring reserves one slot to distinguish empty/full, leaving three usable entries.
- This module is a requested-downgrade candidate only. Existing firmware must preserve its own sequencing, generation checks, expiry semantics and latch/rearm ownership.

Proposed commit message: `Add C MCU perception health receiver candidate`
