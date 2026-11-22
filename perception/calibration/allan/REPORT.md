# phase-21 Allan recorder fix — 2026-10-18

**Built:** `Stats::add` counts SDK timestamp-domain changes without subtracting
stamps across the transition. The first sample in the new domain resets the
interval baseline; later same-domain samples again contribute reversal,
duplicate and max-gap diagnostics. `tests/test_stats.cpp` directly exercises
the recorder's private stats type with the local librealsense API stub.

**Verification:** `PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s
calibration/allan/tests -v`; exact output is in `tests/last-run.log`. The test
compiles and runs the synthetic C++ regression when a C++17 compiler exists.

**NOT verified / risks:** no actual librealsense headers/library, Pi capture,
or device behavior was tested. The stub tests recorder logic only and does not
establish SDK ABI compatibility or a hardware gate PASS.

Proposed commit message: `fix(allan): skip timestamp intervals across domain changes`

## Independent review — 2026-10-18

The C++ regression now also exercises Global Time, forward/equal-valued
cross-domain transitions, immediate baseline recovery, same-domain reversal
and frame counters. The recorder implementation needed no further change.
The transition sample establishes the baseline, as implemented by `Stats::add`.
Full phase-21 findings, exact commands and remaining scope restrictions are in
[`REVIEW.md`](https://github.com/KaiserRichard/wbr-perception-phase1-runtime/blob/fde5721af0627eaf60c78d95296ab26924425e37/calibration/allan/REVIEW.md). The final 13-test transcript is `tests/last-run.log`.

Proposed review commit message: `test: strengthen phase-21 regressions and document independent review`
