Fixtures are software test inputs, never a hardware gate PASS.

`old-unit-smoke/frames.csv` and `command.json` are **MEASURED historical**
byte-identical copies of the 2026-09-13 smoke run from the shared checkout.
`PROVENANCE.json` records original absolute paths and SHA-256 values; tests
verify the retained bytes. SDK device serial **99003** is the OLD unit;
USB/ASIC serial **99004** is a different identifier. Current required
SDK serial **99005** does not inherit any acceptance evidence.

The old profile was IR1 640x480 Y8/15, gyro/200, accel/100. The vendor CSV
column labeled `HW Timestamp (ms)` does **not** serialize the SDK timestamp
domain. Its arrival timer is vendor-relative, not proven Linux
CLOCK_MONOTONIC. IMU vectors are rounded to 0.001. These limitations remain
UNKNOWN, and no timestamp domains or host monotonic epochs are fabricated.

`test_check.py` generates **ASSUMED / SYNTHETIC** current-profile samples and
PGMs in temporary directories beneath `tests/`, then removes them. Synthetic
one-second tests override the qualification duration with an explicit source;
the production threshold remains 60 seconds. SDK simulation uses in-memory
profiles, motion samples, byte buffers and threads; no SDK/device is loaded.
