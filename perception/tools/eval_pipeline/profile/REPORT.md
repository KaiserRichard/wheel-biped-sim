# phase-6 Pi 5 runtime profiling report

Built in `tools/eval_pipeline/profile/`:

- `profile.py sampler`: stdlib fixed-period process sampler with paired UTC and
  CLOCK_MONOTONIC anchor, per-core and target CPU, optional per-thread CPU,
  RSS/PSS, CPU frequencies, thermal zones, fan/cooling state and lower-rate
  decoded VideoCore throttle bits. Missing sources are retained as UNKNOWN.
- `profile.py latency`: reuses runtime observer event parsing/aggregation and
  patch timing parsing. Tracking and end-to-end latency use host monotonic
  stamps when lifecycle events exist. It uses existing nearest-rank statistics,
  counts frames exceeding the single configured 33.3 ms budget, and documents
  realtime factor clock requirements.
- `profile.py validity`: analyze-only PASS/FAIL/BLOCKED gate. Any sampled
  current/historical voltage/throttle bit or temperature at/above 80 °C
  invalidates the latency run; missing telemetry blocks it. The inclusive
  boundary is source-supported by Raspberry Pi's Pi 5 thermal article.
- `profile.py report`: summary JSON/Markdown and optional matplotlib plot;
  explicitly marks latency VALID only on a PASS validity gate and INVALID on
  FAIL/BLOCKED; centralized threshold config; fixtures and unittest suite.

## Verification

Command:

```sh
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tools/eval_pipeline/profile/tests -v
```

The exact test output is saved in `tests/last-run.log`. Software fixtures cover
PASS, FAIL, BLOCKED, the 79.9/80.0 °C boundary, nearest-rank latency and
host-clock event timing. The N4A
fixture is derived from the shared checkout's saved report: `vcgencmd` reported
`0x50005` and max temperature 63 °C; the report records this value in 87/178
five-second samples. The fixture intentionally retains only a representative
sample and is labeled derived evidence rather than a copied full raw file.

## NOT verified / risks

UNKNOWN: sampler execution on Pi 5, procfs/sysfs permissions and field names on
the production image, `vcgencmd` behavior/rate, fan exposure, sampling overhead,
PSS cost, and actual performance under sustained ORB-SLAM3 load. No hardware,
camera, serial, SSH, package install or network was used. Tests on the Mac do
not establish any hardware gate PASS.

INFERRED limitation: the 80 °C boundary is based on Raspberry Pi's public Pi 5
description; actual sampled SoC temperatures and deployed firmware behavior
remain UNKNOWN until physical verification. `vcgencmd`
polling at 1 Hz can miss transient state if a firmware exposes non-sticky
current bits only; historical bits help but do not establish a full-window
physical truth. Procfs CPU snapshots and sampled resource values are
best-effort. The sampler currently expects Linux CPUFreq per-core sysfs files;
systems exposing only policy-level files will show frequency UNKNOWN.

Existing component bug reported, not modified: `tools/runtime_observer/resource_sampler.py`
includes `throttled_raw` in its `THROTTLE_FIELDS` slice, so its `throttling()`
zip maps bit 0 to that raw field and shifts the decoded bit columns. This task
decodes the documented eight named bits locally; the shared implementation was
left untouched per COMMON.md.

Proposed commit message: `Add Pi runtime profiling sampler and validity analysis`

## Independent phase-21 review — 2026-10-18

**OPEN inherited defect:** `profile.py:307-310` accepts non-finite temperatures.
For saved values `[55.0, NaN]`, `max` returns 55.0, the unknown-temperature flag
remains false, and validity returns PASS/0 instead of BLOCKED/2. The named
`test_known_defect_nonfinite_temperature_must_block` regression is explicitly
marked expected-failure until the owning tool task adds finite-value
validation. `tests/known-defect.log` records it run without that marker:
one assertion failure (`0 != 2`) and the erroneous PASS table. This was also
present before phase-21; `profile.py` is unchanged against origin/main.

COMMON.md requires reporting existing-tool bugs; phase-21 owns this directory's
config and tests, so production code was not edited. The final suite executes
14 tests: 13 pass and one expected failure. This is not a clean qualification
PASS. The 79.9/80.0 °C finite-value boundary itself passes its regressions.
Exact output is in `tests/last-run.log`; full commands and verdict are in
[`calibration/allan/REVIEW.md`](https://github.com/KaiserRichard/wbr-perception-phase1-runtime/blob/fde5721af0627eaf60c78d95296ab26924425e37/calibration/allan/REVIEW.md).

The older shared-decoder bug paragraph above is stale for this checkout:
`resource_sampler.py` currently uses `FIELDS[14:]`, correctly beginning at
`undervoltage_now`, and its tests pass. No decoder fix is needed here.

Proposed review commit message: `test: expose inherited profile temperature validation defect`
