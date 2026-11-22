# clockmap — B6 / C0

Python 3.11 + NumPy library for session-scoped clock-domain mapping and an
offline diagnostic analyzer for the D435i recorder's `timestamps.csv`.
**INFERRED / PROPOSED:** this is an implementation of the proposed clock method,
not an accepted firmware interface, synchronization service or hardware gate PASS.
All files and test-generated fixtures stay under `tools/clockmap/`.

## Sources and provenance

**SOURCE-SUPPORTED:** first-party requirements are
[R1 §3, lines 66–68](../../../wbr-perception-phase1-runtime/slides_roadmap/research/R1_algorithms.md),
[R2 §1, lines 25–27](../../../wbr-perception-phase1-runtime/slides_roadmap/research/R2_integration.md),
and [J1 C0/C1/C3](../../research/sw-foundation/J1_components.md).
R1 proposes affine bidirectional mapping with delay/residual uncertainty and
session/wrap checks. R2 defines upper source age as
`MCU_now - mapped_estimate_epoch + mapping_uncertainty`.
[link_proto/SPEC.md](../link_proto/SPEC.md) was read as a candidate integration
constraint; its codec/receiver remains separate from this library.

**SOURCE-SUPPORTED:** librealsense v2.58.4
[`src/global_timestamp_reader.cpp`, inspected raw-source lines 307–310 and 455–472](https://raw.githubusercontent.com/realsenseai/librealsense/v2.58.4/src/global_timestamp_reader.cpp)
pairs the device clock with OS **system time**, and converts hardware frame
epochs through its time-difference keeper. Its
[`get_frame_timestamp_domain`, lines 523–526](https://raw.githubusercontent.com/realsenseai/librealsense/v2.58.4/src/global_timestamp_reader.cpp)
reports GLOBAL_TIME only when enabled, ready, and converting a hardware clock;
otherwise it retains the original domain. Global time is therefore not a host
steady-clock stamp, and option readback alone does not prove frame mapping.
These are inspected upstream behavior, not a claim about the Pi's installed build.
The line numbers refer to the inspected raw source, not the web page wrapper.

**SOURCE-SUPPORTED:** the mimicked CSV header and stamp placement come from the
read-only first-party source
[`d435i_rs.h`, lines 73–92](../../../wbr-perception-phase1-runtime/experiments/2026-10-13-n4b-d435i-prep/d435i_rs.h),
called inside the recorder callback and mutex in
[`d435i_record.cpp`, lines 44–60](../../../wbr-perception-phase1-runtime/experiments/2026-10-13-n4b-d435i-prep/d435i_record.cpp).
Those reference files are untracked in the inspected reference checkout, so no
upstream commit URL is asserted for their first-party CSV format. SDK timestamp
units/domains are supported by upstream
[`rs_frame.h`, lines 16–22](https://github.com/realsenseai/librealsense/blob/v2.58.4/include/librealsense2/h/rs_frame.h#L16-L22)
and [`rs2_get_frame_timestamp`, lines 198–203](https://github.com/realsenseai/librealsense/blob/v2.58.4/include/librealsense2/h/rs_frame.h#L198-L203).
The analyzer accepts extra columns and does not interpret vectors/exposure.

## Library use

Import from the repository root; no install is required:

```python
from tools.clockmap import ClockMap, Exchange, Stamp, source_age, UNAVAILABLE

mapping = ClockMap("pi_steady", "mcu_steady", "pi-boot-id", "mcu-boot-id")
# Populate with at least three exchanges, in order, all in these sessions:
# mapping.add_exchange(Exchange(t1_pi_send, t2_mcu_recv, t3_mcu_send, t4_pi_recv))

estimate = Stamp(123_000_000, "pi_steady", "pi-boot-id")
now = Stamp(456_000_000, "mcu_steady", "mcu-boot-id")
age = source_age(now, estimate, mapping)
assert age.status == UNAVAILABLE  # No exchanges have been supplied in this example.
```

`Stamp(value_ns, domain, session)` is immutable and requires Python integer ns.
Same-domain, same-session subtraction returns an integer duration; adding or
subtracting an integer duration preserves the clock identity. Cross-domain or
cross-session arithmetic and ordering raise. Adding two epochs is undefined.
Only `ClockMap.convert(stamp)` converts a typed epoch between bound domains.
It returns `MappedStamp(stamp, uncertainty_ns, mapping_revision)`; uncertainty
uses the destination clock's ns. Conversion supports both directions.

**INFERRED:** the fit uses integer-truncated exchange midpoints
`x=(t1+t4)//2`, `y=(t2+t3)//2`, with the relation
`B = B_ref + slope*(A-A_ref) + intercept`.
Absolute epochs never pass through a float; only centered deltas do.
`offset_at_reference_ns` is **B minus A at A_ref**, and
`drift_ppm=(slope-1)*1e6`. The affine zero-epoch intercept is intentionally not
used as the reported offset because it extrapolates far outside the window.

The bounded window defaults to 32 exchanges and the minimum to three. Weighted
least squares prefers minimum-network-RTT observations with weights
`(min_RTT / sample_RTT)**2`, retaining temporal spread. RTT in B units is
approximately `slope*(t4-t1) - (t3-t2)`; bootstrap weights use unit slope.
Weighting improves the point estimate; every window observation contributes to
the conservative uncertainty, including slow exchanges.

**INFERRED uncertainty model:** assuming a single affine relation and
nonnegative forward/reverse path delays, each exchange constrains its offset:
`t3 - rate*t4 <= offset <= t2 - rate*t1`.
Combining all such inequalities bounds the true positive rate. At least two
nonoverlapping A exchange intervals are needed to obtain a finite upper rate;
an infeasible/unbounded window invalidates the mapping. Uncertainty takes the
maximum of `abs(midpoint residual) + half network RTT`, using the causal upper
rate for the RTT bound, plus midpoint/rounding allowance. Within the observed
midpoint span this bounds an affine clock's error. Outside the span it grows by
`2*window_bound*distance_outside/span`, rounded up. The inverse divides by a
conservative positive lower rate and refuses conversion if that rate is not
bounded. This is a conditional error bound, not a confidence probability.

Before admitting a new sample, a residual jump greater than the old mapping
uncertainty, the new exchange half-RTT bound, and `step_threshold_ns` invalidates
the map. The default threshold is **PROPOSED 1 ms**, configurable rather than
an approved robot budget. A long/asymmetric delay alone must not trigger a
detectable-step claim. Steps hidden inside delay/error bounds or occurring before
the first valid fit are **UNKNOWN**. Clock reversals/out-of-order observations
invalidate conservatively; an exact repeated A exchange is rejected without a
refit. Callers must serialize access and deliver exchanges in acquisition order.

A changed session on either domain invalidates the map. Invalidation clears
history and blocks further samples until trusted explicit
`mapping.bind(new_session_a, new_session_b)`. Rebinding the same IDs is also
allowed after a detected step. Refill the window before conversion resumes.
Revision increments on every successful refit, invalidation, and bind; retain it
with mapped epochs and never combine an old cached epoch with a new revision's
uncertainty. Revision IDs are local to one object lifetime: a link integrator
must persist/provision a globally fresh wire revision across object/process/MCU
restart, per the candidate SPEC, rather than blindly serialize this counter.

`source_age(now, estimate_epoch, mapping)` returns `AVAILABLE` with
`upper_bound_ns`, or `UNAVAILABLE` with no age if the map is missing/invalid,
sessions/domains disagree, inverse uncertainty is unbounded, or the source is
certainly in the future. The formula uses the estimate epoch, so waiting while
repeating a heartbeat still increases age. This helper does not decide navigation
eligibility or apply a maximum age/uncertainty budget. Device→Pi and Pi→MCU are
separate mappings; composing their drift-scaled uncertainties and revisions is
the integrator's responsibility.

For uint32 MCU microsecond ticks:

```python
from tools.clockmap import TickUnwrapper

timer = TickUnwrapper("mcu_steady", "mcu-boot-id")
before = timer.unwrap(2**32 - 2, "mcu-boot-id")
after = timer.unwrap(3, "mcu-boot-id")
assert after - before == 5_000  # ns, across the wrap
```

**INFERRED:** supply each domain's ordered acquisition/transport observations to
its own unwrapper. Equal ticks are allowed because µs resolution can coalesce
observations. Gaps must be strictly below `2**31` µs (about 35.8 minutes); unseen
whole wraps/reboots cannot be recovered. Reversal or a changed session blocks the
unwrapper until explicit `reset(session)`; also invalidate/rebind associated
ClockMaps. The first observation establishes the raw tick epoch, not an inferred
number of missed wraps. Integer representation does not remove timer quantization
or IRQ/DMA timestamping error; qualify/add these errors before controller use.

## Offline analyzer

```sh
python3 tools/clockmap/analyze.py /path/to/timestamps.csv
python3 -m tools.clockmap.analyze /path/to/timestamps.csv --utc-step-threshold-ns 1000000
```

JSON goes to stdout; exit 0 means diagnostics completed, exit 2 means malformed
input/read/fit failure. The tool writes no output files. Input path and SHA-256
are retained. SDK decimal milliseconds convert directly to integer ns with
half-even rounding; host integer ns retain their precision.

Reports are separated by `(stream,index)`. Only `Global Time`, `global_time`,
and `RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME` observations are fitted. Other domains are
counted and skipped. Domain changes, nonincreasing SDK/steady/frame epochs, or a
host UTC-versus-steady increment jump above the configurable threshold start a
new segment. A segment needs three observations for its diagnostic least-squares
fit. Per-stream latency distributions still retain all global-time observations.
Report fields include reference epochs, offset/drift, signed and absolute residual
min/p50/p90/p95/p99/max, latency distribution, and negative-latency counts.
Every percentile object and top-level report labels its method as
**numpy linear interpolation** (`numpy.percentile` default). This differs from
the **nearest-rank** method (`ceil(q*n)`, one-indexed) used by
`tools/runtime_observer` and `tools/eval_pipeline/profile`; finite-sample percentile values are not always
directly comparable across these methods.
The recorder has no boot/session column: the file hash supplies provenance only,
not trusted session identity. Hidden resets or forward SDK steps remain UNKNOWN.

**INFERRED:** fitting SDK global_time against callback `host_steady_ns` absorbs
mean arrival delay into the offset; time-varying arrival delay can bias drift.
Fit residuals are centered jitter, not absolute latency or conservative mapping
uncertainty, and these fits cannot be used as online `ClockMap`s.

The separately reported **arrival-latency proxy** explicitly bridges clocks using
each record's host UTC/steady pair:
`mapped_sdk_steady = host_steady + (sdk_global_ns - host_utc)`;
`arrival_latency_proxy = host_steady - mapped_sdk_steady`.
This assumes SDK global time and host UTC have comparable OS-system epochs.
**Arrival latency includes USB/driver delay and recorder callback/lock delay;
it is not exposure-to-arrival.** SDK synchronization error and the skew between
the recorder's sequential steady/UTC reads are unmeasured. Negative values are
retained as clock/skew diagnostics. `exposure_us` cannot establish exposure start
or remove those errors. Real latency is **UNKNOWN** until independently qualified.

## Verification and limits

```sh
python3 -m unittest discover -s tools/clockmap/tests -v
```

Result: **37 tests passed** on the available Python 3.11 / NumPy 1.26 environment.

**MEASURED (synthetic only):** offset/drift recovery; constant/time-varying path
asymmetry inside forward/inverse uncertainty; minimum-RTT preference; window and
revision behavior; stale frozen-source age; missing mappings; session/reboot and
step invalidation; refused cross-domain arithmetic; large-epoch ns preservation;
uint32 wrap, half-range ambiguity and a complete map fit across a wrap; per-stream
CSV fits/percentiles; domain/UTC-step segmentation; decimal ns precision; malformed
CSV rejection; both CLI invocation forms and input immutability.

**UNKNOWN / NOT verified:** real recorder files, actual Pi/librealsense build,
device/link clock drift/asymmetry, timer/transport precision, timestamp semantics
relative to exposure, camera–IMU physical timing, STM32 integration, physical
source-age/latency bounds or any hardware/safety gate. No calibration is fabricated.
The affine uncertainty requires the stated model: oscillator rate changes,
undetected steps and unknown capture errors require invalidation or independently
measured additional margins. No network install, hardware command or Git mutation
is needed or performed.

## First real-file run, 2026-10-14 (MEASURED proxy, data kept out of the repo)

Input: the recorder's `timestamps.csv` from the 15 s RAM take (sha256 `a15c8e65…7a2e`). The proxy is
host UTC at the recorder callback minus SDK global_time, so it is cross-domain and labelled as such.

| Stream | n | proxy p50 | p99 | max | negative values | fitted drift vs host_steady |
|---|---:|---:|---:|---:|---:|---:|
| Accel | 1506 | 0.094 ms | 0.357 ms | 1.768 ms | 225 | -0.29 ppm |
| Gyro | 2971 | 0.088 ms | 0.259 ms | 1.313 ms | 470 | -1.72 ppm |
| IR1 | 445 | 8.803 ms | 8.927 ms | 9.273 ms | 0 | -3.58 ppm |
| IR2 | 445 | 8.832 ms | 8.961 ms | 9.302 ms | 0 | -3.95 ppm |

INFERRED: the ~8.8 ms IR value is consistent with a frame stamp taken before readout and USB transfer of a
307 kB image; the ~0.09 ms IMU value, with negative samples, means the IMU global_time sits at or slightly
after host arrival. Whether that reflects librealsense's global-time mapping error or a different stamping
point for HID motion data is UNKNOWN. Do not treat this as a camera–IMU time offset: that offset must come
from a Kalibr camera–IMU calibration, and this 15 s take is far too short to judge drift.
