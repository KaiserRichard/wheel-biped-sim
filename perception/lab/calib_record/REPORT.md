# phase-3 calibration recording check — completion report

Implemented within `lab/calib_record/` only. Changes are uncommitted for the
maintainer. Proposed commit message:

```text
feat(lab): qualify saved D435i calibration takes with explicit evidence gaps
```

`check_calib_take.py` qualifies one immutable camera–IMU or Allan recording,
using existing `tools/take_tools` decoder/timing/stereo functions and
`calibration/lab/pi/take_diagnostics.py` as supplementary diagnostics. No changes
were made to those tools, `calibration/`, project state, next gate, or slides.
No hardware, SSH, camera/serial device, network, GUI, package install, Git write,
commit or push was used.

Implemented criteria cover completeness and original SHA-256 verification;
SDK serial/FW/resolved profiles/emitter OFF; clean finalization and Allan
callback/writer/CSV counts; configured sample-span durations; whole-take stream
rates/gaps/frame drops; per-axis gyro RMS and accel population standard deviation;
same-domain angular-motion IR sharpness fractions; timestamp order and stereo
pairing; callback/bag agreement; and capture-volume capacity/write headroom.
Missing AprilGrid detection remains explicitly BLOCKED because the existing
tools do not expose detections. Malformed supported recordings fail; unsupported
formats block. FAIL > BLOCKED > PASS, with exit codes 1/2/0 and usage/internal 3.

All thresholds are in `criteria.json` with source/rationale labels. Excitation,
blur and headroom limits selected for this task are **PROPOSED**, not measured
acceptance limits. The camera 60–120 s interval and provisional four-hour Allan
minimum are inherited **ASSUMED project policy**. The Kalibr wiki citations are
inherited from the plan's reviewer verification: all-axis motion guidance and
roughly 15–24 h Allan recommendation. No fresh network verification was allowed.

Each completed check saves the common `result.json` contract, a printed/saved
status table, exact policy, detailed metrics, before/after input inventories,
tool hashes, invocation, and verbatim small original command outputs.
Large raw files remain at their original paths, hashed and referenced. Both
`--take-dir` and `--analyze-only` re-evaluate saved raw files; neither captures
hardware or uses the analysis machine's current disk free space as Pi evidence.

The existing camera wrapper records only pre-capture free space; its inferred
remaining capacity cannot PASS the post-capture criterion. The Allan wrapper
does not provide a payload checksum manifest, post-capture capacity or emitter
readback. Missing evidence is BLOCKED. The README gives exact saved-take Pi/Mac
commands and a Pi-local post-finalization capacity/hash receipt procedure.
Emitter OFF requires actual saved readback; an IMU-only profile does not establish
it. Historical frozen evidence should retain UNKNOWN/BLOCKED rather than receive
invented receipts. SHA-256 proves byte identity, not authenticity or original
capture-time integrity when a manifest was made later.

Validation: **26 unittest tests PASS**, output saved in
[`tests/last-run.log`](tests/last-run.log). Coverage includes full Allan PASS,
camera checks PASS with mandatory target BLOCKED, FAIL and BLOCKED precedence,
reanalysis consistency, all result-artifact hashes, checksum tampering/traversal,
original wrapper layout, unsupported/corrupt bag formats, missing dependencies,
wrong unit/FW/profile, interrupted/failed stops, missing/on emitter readback,
missing streams, domain changes, SDK/host reversals and duplicates, frame drops,
callback/bag disagreement, stereo missing/skew/mismatched pairs, weak excitation
on an individual axis, flat/sharp IR images, no-motion uncertainty, insufficient
capacity/bandwidth, pre-capture free-space uncertainty, and invalid CLI/policies.
No new shell scripts require `bash -n`.

Synthetic SQLite/CDR serialization is reused from the existing take_tools test
fixture. The calibration/lab fixtures contain post-fit YAML/results rather than
raw takes and therefore are not suitable recording-check inputs. One small
**verbatim real saved evidence** fixture was copied read-only from the shared
checkout's 2026-10-13 environment bundle (it contains September old-unit
transport observations). Its exact origin/hash are documented in
[`tests/fixtures/README.md`](tests/fixtures/README.md). Tests verify that its
measured IMU rates satisfy the diagnostic policy and its 15 Hz IR is rejected by
the 30 Hz calibration policy. Unknown timestamp domains and old-unit identity
prevent that evidence being used as new-unit acceptance.

**Not verified / open risks:** no Pi run, real recording through this new wrapper,
real camera/Allan take qualification, target detection, optical emitter state,
excitation observability, calibration accuracy, Allan noise estimates, real
conversion round trip, hardware power/storage/thermal performance, stationarity
or temperature stability. Camera decoding holds pixels in memory; long Allan
timing statistics also allocate intervals. Pi memory/runtime need a physical
pilot. Blur is a texture/exposure-dependent proxy and can miss pure translation.
RMS gyro can include bias; accel variation includes projected gravity. Strict
first-to-last spans can be slightly shorter than requested capture duration;
use an interior camera duration rather than rounding evidence. The historical
16.6 MB/s is achieved average throughput from a failed take, not a new card
benchmark. A RAM take can fail the comparison without disproving its transport
integrity. Power/kernel logs are retained and supplementary existing diagnostics
run where available; they do not qualify hardware gates here.

Pre-existing issues found by inspection, intentionally **not fixed**:

- `experiments/2026-10-13-n4b-d435i-prep/analyze_take.py` computes
  `host_minus_sdk_spread_ms` by subtracting C++ steady-clock arrivals from SDK
  stamps without a documented clock mapping. This task does not invoke that
  analyzer; it parses its original CSV and calls existing clock-aware
  `take_tools.timing` instead. No cross-clock latency/spread is manufactured.
- `calibration/allan/d435i_imu_record.cpp`, `Stats::add`, notes a domain change
  but still subtracts the new SDK stamp from the previous domain's stamp.
  This checker recomputes timing from raw CSV, excludes transition intervals
  through `take_tools.timing`, rejects domain changes and blocks duration when
  its timeline is invalid. Recorder counters do not override this recomputation.

Tooling completion does not close a physical gate. Hardware status remains OPEN.
