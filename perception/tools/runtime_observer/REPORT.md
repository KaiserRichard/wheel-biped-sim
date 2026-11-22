# Q2 / C7 runtime observer report

Completed 2026-10-14. **INFERRED implementation; MEASURED synthetic software
checks; physical behavior UNKNOWN. Nothing here is a gate PASS.**

## Built

- `event_recorder.h`: header-only C++17 fixed-capacity ring, one mutex attempt per
  event, no hot-path allocation/I/O, `CLOCK_MONOTONIC` integer-ns host stamps,
  optional copied source epoch/domain/session, all seven requested event kinds,
  explicit drop reasons and queue depths. Normal-exit CSV dumping includes
  overwrite, contention, invalid-input and clock-error loss counters.
- `aggregate.py` and `stats.py`: nearest-rank P50/P95/P99/max for stage spans,
  host ages since receipt and source ages; queue quantiles/sample and weighted
  means/slope, drop counts, throughput, incomplete-frame/missing-pair diagnostics
  and input hashes. Cross-domain source ages require the existing
  `tools.clockmap.ClockMap`; point estimates, conservative upper bounds,
  uncertainties and revisions remain separate. Unmapped/invalid ages are
  UNAVAILABLE. Host sessions are never combined arithmetically.
- `resource_sampler.py`: psutil-free 1 Hz CSV from `/proc/stat`, process status
  and supplementary process stat (CPU/starttime), thermal zones, CPUFreq policies
  and optional `vcgencmd get_throttled`. Reads are timed, missed slots skipped,
  output flushed per row, and missing fields marked UNAVAILABLE. macOS retains
  host stamps and degrades Linux/Pi resource fields to UNAVAILABLE.
- `patch_timing.py`: the inspected patch's exact tracking CSV and stdout count
  summary parser, preserving printed image stamps and recording conversion
  semantics. It reuses measured tracking-call durations without inventing
  queue wait, absolute host epochs, source ages or drop reasons.
- `Makefile`, synthetic C++/Python tests, `.gitignore`, and `README.md` with
  integration/CLI/schema details and primary API citations with line ranges.

## Exact verification

From `<local workspace>/Desktop/Research/wbr-sw-foundation`:

```sh
make -C tools/runtime_observer test
```

**MEASURED result: exit 0.** C++ recorder checks passed; **20 Python unittest
cases passed**, including the C++ CSV→Python round trip, with no skips in the
Makefile run. Final Python test output: `Ran 20 tests in 0.429s`, `OK`.
C++ compiled with `-std=c++17 -Wall -Wextra -Werror -pedantic -pthread`.
Host: Darwin, Apple clang 17.0.0 (`clang-1700.0.13.5`), Python 3.11.0rc2.

**MEASURED / verified in synthetic fixtures:**

- Zero counted heap allocations during recorded/invalid events; ring wrap and
  retained ordering; copied large source epochs; deterministic contention loss;
  four-producer accounting and monotonic host stamps.
- Known receive/enqueue/track/publish delays and source ages, nearest-rank
  quantiles, explicit drops, 2-depth/second backlog growth, throughput denominator,
  zero-span availability and 1 ns durations at large epochs.
- Explicit ClockMap reconstruction, mapped age/uncertainty, no mapping, invalid
  mapping, unrelated clocks, wrong host session and future epoch rejection;
  session separation and incomplete/empty trace handling.
- Fake proc/sysfs CPU, RSS, threads, temperature and frequency units; guest
  counter exclusion; process CPU >100% for multiple cores; PID reuse and counter
  reset; missing/malformed resources; current/historical throttle flags and
  subprocess absence/timeout. Simulated 1 Hz scheduling and skipped missed slots.
- macOS degradation, CLI new-output protection, exact patch CSV/stdout parsing,
  malformed/nonfinite/negative/reversed values, count consistency, and analyzed
  input immutability.

Temporary fixtures stayed inside this component and were automatically removed.
The compiled test executable was removed afterward with:

```sh
make -C tools/runtime_observer clean
```

## Source scope and constraints

The proposal, component inventory, architecture note and requested existing
tool READMEs were read, along with read-only R1/R2 and the existing patch.
Upstream primary clock/resource/standard-library sources are cited in
[README.md](README.md); no installed Pi version is inferred from those sources.
Patch SHA-256:
`67264b563e67f1fb6826d846168cce84ca253df0bd8e4e1082d687ba877a51b9`.
Its actual timing format is SOURCE-SUPPORTED; tests use synthetic emitted rows.

All authored files are under `tools/runtime_observer/`. No SSH, Raspberry Pi,
camera, ORB-SLAM3, installs or network package downloads were used. Reference
checkout and real data remained read-only; no real data was copied or processed.

**Execution deviation:** before reading COMMON_QUEUE.md's prohibition, an initial
read-only `git status --short` was run. No later git commands, commits, branch
operations or git mutations were performed; the runner owns committing.

## NOT verified and open issues

**UNKNOWN / NOT verified:** Pi compilation/runtime, actual proc/sysfs/VideoCore
availability, real retained runtime traces, ORB/SDK/ROS hook placement, logger
on/off overhead, sustained-load tails, source capture semantics, physical clock
mapping accuracy, power/thermal behavior, firmware and any hardware gate.
No ORB-SLAM3 build or execution is required or claimed on this Mac.

**INFERRED integration work:** the runtime owner must add hooks and arrange CSV
dumping after joining producers, choose a capacity appropriate to the retained
trace window, provision honest clock/session identities and supply qualified
clock observations. One queue is summarized per file/session; multiple queues
need separate traces. Multi-hop clock maps and live synchronization are outside
this component. No warmup exclusion or live/replay mode is guessed; retain run
mode and selected analysis interval in the surrounding run manifest.

A bounded ring can lose history. Counts and quantiles are conditional on retained
events when losses occur; loss metadata and missing pairs expose that limitation.
Crash/SIGKILL loses the in-memory recorder, since dumping is an explicit normal
shutdown operation. Mutex/OS scheduling cost is not a hard real-time bound.
Offline aggregation loads the trace in memory, and procfs resource reads are
approximate, best-effort snapshots. `scaling_cur_freq` is a reported policy
frequency, not guaranteed instantaneous hardware frequency. No physical timing
or controller eligibility claim is made from these software checks.
