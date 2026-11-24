# runtime_observer — Q2 / C7

**INFERRED implementation:** a C++17 event recorder and Python 3.11 offline
diagnostics/resource sampler. No ORB-SLAM3, ROS, SDK, psutil, hardware access or
installation is needed for the tests. NumPy is used only by the existing
`tools.clockmap` library when a mapping is supplied (and by mapping tests).
**MEASURED:** software verification uses synthetic streams and fake Linux files.
**UNKNOWN:** physical latency, observer overhead and Pi behavior. No gate PASS.

From the repository root:

```sh
make -C tools/runtime_observer test
python3 tools/runtime_observer/aggregate.py /path/to/events.csv
python3 tools/runtime_observer/aggregate.py /path/to/events.csv --clockmap /path/to/map.json
python3 tools/runtime_observer/patch_timing.py /path/to/run_tracking.csv --stdout /path/to/run.log
python3 tools/runtime_observer/resource_sampler.py /path/to/NEW_resources.csv --pid 1234 --session pi-boot-run --samples 60
```

The three Python CLIs also work as `python3 -m tools.runtime_observer.aggregate`
(or `.patch_timing`, `.resource_sampler`). Offline analyzers print JSON to stdout
without modifying inputs, including input SHA-256 provenance. Exit 0 means the
command completed; exit 2 means invalid input or I/O failure. Sampling writes a
new CSV and refuses existing paths. Omit `--samples` to run until Ctrl-C.

## Event recorder integration

Include `event_recorder.h`, compile with C++17 and pthread support. Allocate the
recorder before producers start; choose capacity for the intended trace window.
This normal-exit example uses a caller-supplied source epoch, not a new clock map:

```cpp
#include "event_recorder.h"
#include <fstream>

// Before acquisition; session identifies this host/run and must be fresh after
// reboot/reset. Allocate a large recorder on the heap BEFORE entering hot paths.
wbr::observer::Recorder<4096> observer("pi-boot-run");
wbr::observer::SourceEpoch source{source_ns, "device_clock", "device-boot"};
using K = wbr::observer::Kind;
observer.record(K::frame_received, frame_id, &source);
observer.record(K::queued, frame_id);
observer.record(K::queue_depth, 0, nullptr, nullptr, queue_size);
observer.record(K::track_start, frame_id);
// The caller invokes its engine here.
observer.record(K::track_end, frame_id);
observer.record(K::snapshot_published, frame_id);
// For a different rejected frame:
observer.record(K::dropped, rejected_frame_id, nullptr, "queue_full");

// At normal exit AFTER joining all producers; check output failures in caller.
std::ofstream output("events.csv");
observer.dump_csv(output);
output.flush();
if (!output) { /* report the failed dump in caller */ }
```

`record()` stamps with `clock_gettime(CLOCK_MONOTONIC)` in integer ns, domain
`host_steady`. It copies optional source epoch/domain/session without conversion.
Source metadata can appear on any lifecycle event; aggregation propagates it
within that frame, rejecting changes to its identity/epoch. Use unique uint64
frame IDs in each session. Sessions identify the same host clock instance when
merging observer/resource files; labels alone cannot verify their provenance.

The inline ring uses `std::array` and fixed 48-byte token storage. Domain, session
and reason tokens must contain 1–47 ASCII letters/digits or `_`, `-`, `.`; invalid
inputs are counted/rejected, never truncated. Source pointers need only remain
valid during `record()`. It attempts the mutex once with `try_lock`, then performs
fixed-size copying and a clock read, with no heap allocation or I/O. Failed lock
attempts return false and increment contention loss. A full ring overwrites the
oldest retained event and counts that overwrite. `true` means retained at that
instant, not guaranteed retention until exit. uint64 loss atomics must be lock
free at compile time. OS scheduling and mutex/clock costs are **UNKNOWN**, not
a hard real-time guarantee.

The CSV starts with total recorder loss counters, then:

```text
host_ns,host_domain,host_session,frame_id,event,source_ns,source_domain,source_session,reason,depth
```

Optional fields are empty. `depth` is only on `queue_depth`, `reason` only on
`dropped`. Host/source epochs are int64, frame IDs uint64. Dumps preserve retained
chronological order. Dump only after producers stop; stream I/O holds the mutex.
Call it explicitly during normal shutdown, including a handled failure path.
The recorder does not use signal handlers/destructors to perform I/O; abrupt
termination loses the in-memory trace. Output-path ownership belongs to the caller.

## Aggregation and clock identity

Reports split by host session, rejecting reversed/out-of-order host events,
duplicate stages, conflicting source stamps, and drops after publication.
There is no sorting or subtraction across sessions. Stages are receive→queue,
queue→track start, tracking, track end→publish, and receive→publish. Host ages
since receipt are separate from source ages at each lifecycle stage. Every
distribution uses nearest-rank P50/P95/P99/max and sample counts. Missing stage
pairs and receipt events are counted; incomplete frames remain visible.

Same-domain ages require the same `host_steady` session. Cross-domain ages are
`UNAVAILABLE` unless an explicit `tools.clockmap.ClockMap` is supplied; missing,
invalid, unrelated-domain and wrong-session mappings retain unavailable reasons.
Mapped ages report the point estimate, conservative upper bound, mapping
uncertainty and revisions separately. A negative upper bound is unavailable;
a negative point with a nonnegative bound remains a clock diagnostic. Mapping
validity/uncertainty retains the existing clockmap library's assumptions and
session invalidation behavior. No source age is inferred from arrival timestamps.

Python callers may use `summarize(events, mapping=clock_map)`. The CLI reconstructs
one direct mapping from explicit exchange observations in JSON:

```json
{
  "domain_a": "device_clock", "domain_b": "host_steady",
  "session_a": "device-boot", "session_b": "pi-boot-run",
  "exchanges_ns": [[0, 1100, 1100, 200], [1000000, 1001100, 1001100, 1000200],
                   [2000000, 2001100, 2001100, 2000200]]
}
```

These are **synthetic** four-stamp exchanges `[t1_A,t2_B,t3_B,t4_A]`, not device
synchronization evidence. ClockMap requires three valid exchanges by default;
either mapping direction is supported. See [clockmap's assumptions](../clockmap/README.md).
Multi-hop mappings, automatic session rebinding and live clock synchronization
are outside this component.

Queue statistics include sample quantiles/mean, a time-weighted mean between the
first/last observed depths (previous depth held until the next observation), and
least-squares slope in depth/second. One-time observations have unavailable slope
and time-weighted mean. Epochs are centered as integers before slope calculation.
Queue snapshots describe one queue per file/session; split traces for other queues.
Throughput is each event count divided by the entire session's first-to-last
retained event span; zero span is unavailable. This is observed host throughput,
and cannot alone distinguish paced/live from unpaced replay. Retain run mode in
the enclosing manifest. Frame drops are counted by their explicit reason.

`trace_completeness` refers to recorder loss metadata: zero loss → COMPLETE,
nonzero → INCOMPLETE, absent → UNKNOWN. It does not certify input completeness
or lifecycle coverage. Lost/overwritten events make drop/throughput counts partial
and latency quantiles conditional on retained pairs; missing pairs are never
fabricated. Offline analysis holds the trace in memory; size long runs accordingly.

## Pi resource CSV

Sampling starts immediately, then targets 1 Hz using the same explicit host
clock. Missed slots are skipped; slow reads never cause catch-up bursts.
`sample_duration_ns` records read overhead. Rows name host clock/session/PID.
Missing/malformed/unreadable files yield `UNAVAILABLE` for affected fields.
On macOS all Linux/Pi resource fields are unavailable, while host stamps remain.
The sampler only reads these interfaces and never changes frequency/governor.

`/proc/stat` supplies whole-board CPU busy percentage (100% = the whole board);
idle and iowait are excluded, guest fields are not counted twice. First rows,
counter reversals and CPU-count transitions have unavailable utilization.
`/proc/<pid>/status` provides approximate RSS (bytes) and thread count.
Supplemental `/proc/<pid>/stat` supplies process CPU percentage (100% = one core,
possibly >100% for a multithreaded process) and starttime for PID-reuse detection.
`SC_CLK_TCK` scales its counters. Two starttime reads bracket status reading;
procfs snapshots are best effort, not atomic.

Thermal zone names/types/temperatures are retained in `thermal_c_json`;
`cpufreq_khz_json` retains per-policy `scaling_cur_freq` values. The latter is a
reported scaling frequency and may differ from instantaneous hardware frequency.
JSON maps keep individual sensor failures visible. If `vcgencmd` exists on Linux,
`get_throttled` is queried with a 0.2-second subprocess timeout. Raw hexadecimal
flags and all four current/four historical bits are separate columns. Missing
commands, failed exits/timeouts and malformed output remain unavailable.

## Existing patch timing

**SOURCE-SUPPORTED first-party format:** inspected read-only
`wbr-perception-phase1-runtime/experiments/2026-10-13-n4b-d435i-prep/stereo_inertial_d435i_file.patch`,
SHA-256 `67264b563e67f1fb6826d846168cce84ca253df0bd8e4e1082d687ba877a51b9`.
Its lines 128, 249–256 emit the tracking CSV; lines 263–265 emit stdout counts.
It prints no absolute host event stamps or queue-wait timing. `patch_timing.py`
parses these actual emitted formats, including scientific decimal notation,
without treating removed `-` lines in the patch as runtime output.

Tracking duration is summarized in ns; printed decimal seconds/ms convert using
half-even rounding and the printed image timestamp is retained in parsed records.
Original precision was already limited by the patch's C++ double representation.
Supply `--source-domain` and `--source-session` together to report processed rate
over source intervals; otherwise it is unavailable. This rate is separate from
host replay throughput. Ages, queue stats/wait and drop reasons remain unavailable.
Prefix/tail/pending counts are reported under their existing names, not relabeled
as runtime drop reasons. CSV/stdout count disagreement or multiple run summaries
are rejected. Patch and observer event traces are separate input contracts.

## Upstream API evidence

**SOURCE-SUPPORTED:** URLs below identify the depended-on clock/resource APIs.
For HTML references, ranges denote the inspected browser-extracted lines; raw
source numbers are one-based. These describe APIs, not the Pi's installed kernel.

| API | Primary reference and inspected lines |
|---|---|
| `clock_gettime`, `timespec`, monotonic clock and error return | [Linux man-pages](https://man7.org/linux/man-pages/man3/clock_gettime.3.html), HTML lines 14–18, 37–46, 91–105, 163–165 |
| Mutex attempt and `unique_lock(try_to_lock)` | [C++ working draft mutex requirements](https://eel.is/c++draft/thread.mutex.requirements), HTML lines 177–187; [unique-lock constructors](https://eel.is/c++draft/thread.lock.unique.cons), HTML lines 46–64 |
| Integer Python host clock | [Python 3.11 time API](https://docs.python.org/3.11/library/time.html#time.clock_gettime_ns), HTML lines 67–86 |
| System counter tick rate query | [Python 3.11 `os.sysconf`](https://docs.python.org/3.11/library/os.html#os.sysconf), HTML lines 3116–3124 |
| Board CPU counter order | [Linux v6.8 `fs/proc/stat.c`](https://github.com/torvalds/linux/blob/v6.8/fs/proc/stat.c#L119-L128), source lines 119–128 |
| Process CPU/starttime fields | [Linux v6.8 `fs/proc/array.c`](https://github.com/torvalds/linux/blob/v6.8/fs/proc/array.c), HTML lines 2827–2877 |
| RSS/status fields and asynchronous accounting | [Linux v6.8 proc documentation](https://raw.githubusercontent.com/torvalds/linux/v6.8/Documentation/filesystems/proc.rst), source lines 166–183, 210–213 |
| Temperature units | [Linux v6.8 thermal ABI](https://github.com/torvalds/linux/blob/v6.8/Documentation/ABI/testing/sysfs-class-thermal), HTML lines 724–732 |
| Policy path, frequency units and limitations | [Linux v6.8 CPUFreq documentation](https://docs.kernel.org/6.8/admin-guide/pm/cpufreq.html#policy-interface-in-sysfs), HTML lines 54–61, 107–112 |
| Throttle bit meanings | [Raspberry Pi documentation](https://www.raspberrypi.com/documentation/computers/os.html#get_throttled), HTML lines 338–350 |
| Subprocess timeout/captured output | [CPython v3.11.9 subprocess](https://github.com/python/cpython/blob/v3.11.9/Lib/subprocess.py#L463-L476), source lines 463–476 |

The recorder uses only standard C++17 containers/atomics/streams. Python parsing,
CLI, CSV and JSON use the standard library; no third-party tracing or resource
API is invoked. Clock mapping depends on the read-only local
[ClockMap implementation](../clockmap/clockmap.py) and its existing source citations.
See [REPORT.md](REPORT.md) for exact verification and remaining limits.
