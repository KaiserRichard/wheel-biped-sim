# phase-19 report — stamp drift analyzer

## Built

- Added `tools/stamp_drift/analyze.py` for CSV and rosbag2 SQLite `.db3` input.
- Added read-only stdlib SQLite iteration with the existing rosbag qualification
  Header decoder for little-endian data and a minimal big-endian fallback.
  Unsupported context types are recorded separately; serialization is checked.
- Added centered affine fitting through `tools.clockmap._fit_affine`, iterative
  MAD outlier rejection, per-topic drift/residual/proxy statistics, reversal and
  residual-step diagnostics, and inter-topic/IR-vs-IMU drift comparison.
- Added JSON and Markdown report generation, explicit clock-domain metadata, and
  the single threshold/rationale file `config.json`.
- Added diagnostic-only `PASS`/`FAIL`/`BLOCKED` statuses for clean fits,
  detected discontinuities, and insufficient samples; these are not hardware
  gate verdicts. The reviewed CLI writes the COMMON `result.json` contract,
  saves verbatim raw input/configuration/clock declarations, supports saved-input
  re-analysis, and returns 0/1/2/3 for PASS/FAIL/BLOCKED/usage or internal errors.
- Added synthetic PASS/FAIL-shaped diagnostics and a fixture adapted from real
  saved D435i timing evidence, plus a portable fixture of exact Header prefixes
  and separate DB3 timestamp fields from the real saved converted DB3. That
  conversion sets receive timestamps equal to Header stamps; it is parser
  evidence, not a live ROS clock relationship measurement.

## Verification

Run:

```sh
python3 -m unittest discover -s tools/stamp_drift/tests -v
```

The exact final commands and outputs for this suite and all imported suites
are saved in `tests/last-run.log` for this worktree. See REVIEW.md for counts. The tests
cover diagnostic `PASS`, `FAIL`, and `BLOCKED` paths; known offset/drift with
jitter and outliers; a forward residual step; a header-stamp reversal;
inter-topic comparison; malformed CSV; real-scale measured-evidence fixture
parsing; and a synthetic SQLite/CDR bag.

Evidence label: **MEASURED** for the adapted source rows and **CALCULATED** for
all analyzer output. Synthetic recovery is a tooling check, not a hardware gate
PASS.

## Not verified / open risks

- No Pi, camera, serial device, ROS daemon, or hardware was accessed.
- The saved 266 MB converted `.db3` was inspected read-only and all 5,372 sensor
  rows were analyzed (447 IR1, 447 IR2, 1,507 Accel, 2,971 Gyro). Header/DB3
  timestamp equality yields calculated zero drift; no live ROS recorder timing
  evidence was verified. The raw bag was not copied into this worktree.
- The analyzer cannot identify whether a ROS `header.stamp` is D435i global,
  hardware, or system time without run configuration/readback; the CLI records
  the operator's declared interpretation and does not infer it.
- Offset, transport delay, jitter, SDK global-time mapping error, and exposure
  timing are not separately identifiable from this one-way pair.
- A residual step is intentionally classified as possible clock/transport
  discontinuity; it is not automatically a global-time reset.
- The numeric detector settings are **PROPOSED** and are not N3/N7 acceptance
  thresholds.

## Independent review fixes

See REVIEW.md for ranked findings and regression coverage. Review fixes preserve
centered nanosecond residuals at UTC-scale receive epochs, enforce FAIL precedence,
handle absent/degenerate evidence without crashes, bound step splitting, validate
configuration, and separate effective timestamp-domain declarations from option
readback. One adapted SDK fixture stamp was corrected by 100 ns against the exact
saved decimal source row. Existing clockmap and lab tools were reused without
modification. All review edits remain under `tools/stamp_drift/`.

No external tool defects requiring changes outside this task were confirmed.
The pre-existing branch differences outside this owned directory were not edited.

Proposed commit message: `Fix and independently validate stamp drift diagnostics`
