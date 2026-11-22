# CANDIDATE link_proto

CANDIDATE only; not an accepted interface. **STM32 firmware owner review is
required before any firmware use.** Layout values and budgets are PROPOSED.
Component C9 provides a bounded C99 COBS/CRC-32C codec, receiver policy and pure
Python Pi encoder for [SPEC.md](SPEC.md). Health-only comes first; optional
reference/status encodings are fixtures. No transport, firmware, motors, live
camera, invented covariance or clock mapping is supplied.

From repository root:

```sh
make -C tools/link_proto test
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tools/link_proto/tests -v
```

Makefile uses `cc -std=c99 -Wall -Wextra -Werror -pedantic` and runs C faults plus
Python unittest. Frozen [vectors.txt](tests/vectors.txt) holds hex COBS bytes
including zero delimiter for health/reference/status. Inputs are explicit in both
tests; tests compare frozen bytes and never regenerate them. No extra packages.

Use `lp_pack` to encode, `lp_stream_feed` to receive arbitrary chunks. Packet
callback calls `lp_receiver_accept` with local MCU monotonic time; error callback
calls `lp_receiver_reject(..., LP_REASON_WIRE)`. Locally bind session/mapping/frame
revisions; received sessions cannot bind themselves. Call `lp_receiver_update`
regularly during silence. New explicit local operator action calls
`lp_receiver_enable` after fresh eligible health. Never call enable periodically.
Read `reference_active` and bounded reference mailbox; caller owns withdrawal/ramp.
Its balance/safety loop remains independent. See SPEC for clock/wrap prerequisites.

Python: import `py/link_proto.py`, call `perception_health`, `body_velocity_ref`
or `mcu_status` with keyword fields in SPEC. Timestamps are integer ns, explicit
or message-defined domains; lease and expiry API use integer us. Encoders only
serialize supplied inputs; receiver decides acceptance. Pi sender must consume
reviewed snapshots/mapping, bound unsent records and isolate replay from robots.

MEASURED / verified: synthetic C/Python golden bytes, CRC/COBS, parser faults,
freshness/session/reset/lease transitions, counter/timer wrap. Host address and
undefined behavior sanitizer run passed. SOURCE-SUPPORTED inputs/borrowed
reset/quality semantics are cited in SPEC; policy is an INFERRED interpretation.
UNKNOWN / not verified: real files, Pi estimates, clocks/uncertainty, STM32
build/timing, UART/DMA/EMI, controller ramp and physical balance. No hardware
qualification or gate PASS is claimed. MCU owner must ratify/replace layout,
budgets, limits, session/mapping handshake and enable/fallback integration.

Files: `SPEC.md`, `README.md`, `link_proto.h`, `link_proto.c`, `Makefile`,
`py/link_proto.py`, `tests/test_link_proto.c`, `tests/test_python.py`,
`tests/vectors.txt`. `make -C tools/link_proto clean` removes host executables.
