# T10b reconciliation report — 2026-10-18

**MEASURED software result: PASS.** Both unittest suites pass, with no expected
failures. The full PTY capture and independent saved-raw reanalysis pass all ten
criteria. Fixture success is **not a hardware, safety or project gate PASS**.
Changes are confined to `tools/perception_health/`; the only phase-7 code change is
the confirmed age-validity bug fix and its regression tests. Changes remain
uncommitted for the maintainer. No Git write, network, hardware, camera, physical
serial-device, installation, SSH or GUI operation was performed; transport tests
opened local PTYs only.

## Reviewed API and semantics

- The converter validates exactly `pi_domain: "CLOCK_MONOTONIC_RAW"`, matching
  the sender's actual send clock. The old `Linux_CLOCK_MONOTONIC_RAW` label is
  rejected; no alias or cross-clock subtraction hides a mismatch.
- Both HEARTBEAT and health events pass their decoded shared `wire_seq` to the
  existing model. Previously the default sequence zero caused every frame after
  the first heartbeat to be rejected as a duplicate.
- Replay defaults to `primary_fresh=False`. Acceptance scenarios explicitly
  assume independent primary freshness, recorded in result notes and traces.
  A regression test confirms valid health frames alone cannot grant FRESH.
- LOST assertions use upstream `SOURCE_AGE_TIMEOUT`; unmapped reports use
  `SOURCE_AGE_HOLD`. Decoded IMU boolean values are passed through without a
  truthiness coercion, preserving the reviewed decoder's 0/1 validation.
- Existing transit allowance, sequence checks, session/latch/rearm behavior,
  boolean validation, codecs and mapping implementation are reused.

## HOLD versus FAILSAFE decision

**SOURCE-SUPPORTED decision: unavailable pose age means HOLD/SOURCE_AGE_HOLD
while reports and heartbeats remain timely.** It must not become an immediate
source-age timeout merely because its wire placeholder is UINT32_MAX.

The binding sources distinguish availability from measured staleness:

1. `../SPEC.md`, Supervisor recommendation, explicitly places absent age/mapping
   in HOLD. The payload section requires validity bits to distinguish unknown
   age from valid age; a valid saturated age remains beyond FAILSAFE age.
2. The binding `T7_health.md` requires UNAVAILABLE when no camera-to-Pi clock
   mapping exists. Its source-age HOLD/timeout recommendation concerns a pose-age
   duration, not a fabricated duration derived from an unavailable field.
3. `../upstream_snapshot/supervisor.c`, `result_age()` and `evaluate()`, applies
   `SOURCE_AGE_HOLD`/`SOURCE_AGE_TIMEOUT` to source age including receipt elapsed
   time and transit allowance, and distinguishes receipt-silence reasons for an
   absent age (`receipt_silence_fallback`, set from `has_age_at_send`). This supports
   keeping presence distinct from magnitude. The snapshot does not implement
   PERCEPTION_HEALTH: its NET_STATUS unknown-age sentinel has no corresponding
   validity bit and is not the new health message's policy. The new message's
   explicit SPEC governs its unavailable-age behavior.

**MEASURED defect:** the reviewed phase-7 model correctly entered HOLD on the first
unmapped report, but saved UINT32_MAX and then unconditionally aged it before
checking validity on the next report, heartbeat or timer tick. That incorrectly
latched SOURCE_AGE_TIMEOUT. T10b explicitly authorizes this phase-7 fix.

`../py/supervisor_model.py` now remembers whether both age/mapping validity bits
were set. Deadline evaluation skips source-age comparisons for unavailable age;
unknown age is never substituted with zero. Receipt-silence and heartbeat
watchdogs still run, and LOST still immediately latches FAILSAFE even when age is
unavailable. Valid saturation, expiry before accepting replacement traffic,
transit allowance, downgrade-only authority and explicit local rearm are retained.
Validity history clears on a new session and rearm.

Four added phase-7 regression tests cover all absent-validity-bit combinations over
more than the source-age timeout, heartbeat and tick updates, independent receipt
and heartbeat failures, recovery with a valid mapped report, LOST with unknown
age, valid saturation and expiry before a replacement unknown report. The old
expected-failure test and defect-preserving assertion were replaced with ordinary
passing HOLD assertions. The single-report analyzer workaround was removed.

## Evidence retention and validation

Full captures are generated in temporary directories. Committed `evidence/`
contains only `summary.json` and `sha256.json`: **29618 bytes total**, below
the configured 200,000-byte task budget. The summary embeds the result, metrics,
source provenance and policies plus state/reason counts. SHA-256 and byte lengths
cover every original output, including the large real replay events and snapshots.
`--summary-outdir` produces this archive and refuses existing destinations.
The size limit lives in `config.json` with its source.

All 445 saved real-association snapshots are converted and transported in the
PTY tests. A four-line verbatim excerpt and source provenance remain in
`tests/fixtures/`. Its full source SHA-256 is `14c6a1caf12bb2b458e96b09f7cef14620ae4bc883bc6f6c3efcc80b4062c7ce`.
These are **MEASURED D435i input associations with synthetic fake-engine
estimates**, not real ORB-SLAM3 estimates. No real clock mapping was fabricated.

Exact commands and complete outputs:

| Check | Result | Saved output |
|---|---|---|
| `python3 -B -m unittest discover -s tools/perception_health/tests -v` | 28 tests PASS, exit 0 | `../tests/last-run.log` |
| `python3 -B -m unittest discover -s tools/perception_health/integration -p 'test*.py' -v` | 24 tests PASS, exit 0 | `tests/last-run.log` |
| PTY check `--outdir <temporary-full-dir>` | 10/10 criteria PASS, exit 0 | `tests/e2e-run.log` |
| `--analyze-only <temporary-full-dir> --summary-outdir <temporary-summary-dir>` | 10/10 criteria PASS, exit 0 | `tests/analyze-run.log` |

The phase-7 suite includes C99 codec compilation with `-Wall -Wextra -Werror`, frozen
vector parity, CRC, malformed input, transit, sequence and latch/rearm checks.
Integration tests cover converter availability, exact states, reset handling,
clock identity and boolean rejection, real fixtures, all five PTY scenarios,
non-promotion, corrupted-raw FAIL, missing-capture BLOCKED, missing-heartbeat
FAIL, invalid read-clock FAIL, missing-prerequisite BLOCKED and archive
size/hashes. Raw capture analysis requires both health and HEARTBEAT evidence
and rejects read timestamps that precede or regress from the capture's host
CLOCK_MONOTONIC start.
Python syntax checks, `bash -n` for every README bash block, and `git diff --check`
pass. Hardware commands in the README were syntax-checked only.

Host: macOS-15.7.9-x86_64-i386-64bit; 3.11.0rc2 (v3.11.0rc2:ed7c3ff156, Sep 11 2022, 16:22:23) [Clang 13.0.0 (clang-1300.0.29.30)].
The full capture and raw-analysis directory was temporary and removed after
archiving. To reanalyze, create a new full capture using the README commands;
summary-only archives deliberately omit the raw files required by analyze-only.
Fresh runs have new timing/output hashes; input hashes identify the fixture.

## Remaining limits and risks

- **SOURCE-SUPPORTED producer gaps:** current adapter snapshots collapse several
  exact ORB states and omit camera-IMU initialization and tracked feature counts.
  The converter preserves optional exact names when provided and degrades honest
  fallbacks. Collapsed LOST can conservatively latch even if the native state was
  RECENTLY_LOST. Local map-ID observations cannot prove complete reset history.
- **ASSUMED:** synthetic clock exchanges and independent primary freshness in
  policy exercises. All timing thresholds/transit bounds remain PROPOSED; none
  is qualified as a robot safety limit.
- **MEASURED local PTY throughput / CALCULATED UART comparison:** retained metrics
  include launch/import/EOF time. The unpaced real replay burst can exceed ideal
  115200-baud capacity. PTYs do not enforce baud; these rates do not prove real
  UART performance, live VIO cadence or latency.
- **UNKNOWN / not verified:** Pi execution/load, real ORB binding, live serializer,
  clock exchange collection/session authenticity, exposure timestamp semantics,
  clock-step handling, hardware UART/transit bound, STM32 firmware integration,
  power, thermal or storage qualification. Static caller-provided maps require
  qualified freshness/session invalidation. STM32 retains balance and safety.

Proposed commit message:

```text
fix(perception-health): reconcile integration with reviewed health policy

Pass shared sequences and exact raw-clock identity, model independent primary
freshness, retain unavailable-age validity across deadline checks, and replace
large replay evidence with compact summaries and hashes.
```
