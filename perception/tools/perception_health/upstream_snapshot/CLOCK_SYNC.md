# Clock-sync math

`hea_clock_sync` is a pure C99, heap-free helper for host, Linux and firmware
callers. It does not change the UART wire format or align either clock to a wall
clock.

## Round-trip sample

The four timestamps use integer microseconds: phase-1/phase-4 are Pi timestamps and phase-2/phase-3
are MCU timestamps. The implementation uses RFC 4330 section 5:

```
offset = ((phase-2 - phase-1) + (phase-3 - phase-4)) / 2
delay  = (phase-4 - phase-1) - (phase-3 - phase-2)
```

Only same-domain elapsed times are unsigned-subtracted. Cross-domain differences
are checked before conversion, and a negative delay is rejected as unusable.
The reported reference time is the integer floor of the midpoint of phase-1 and phase-4;
the API uses `double` for offset and fit diagnostics.

## Filter and fit contract

`hea_clock_filter` stores estimates in caller-owned storage. Adding a sample
expires entries older than `window_us`; an entry exactly on the boundary remains.
The age comparison is against the newest retained reference, and a
reference-time regression is rejected without changing filter state. If storage
is full, the oldest reference-time entry is evicted. The minimum-delay query
returns the first/oldest entry on a delay tie. A single valid sample is enough
for the minimum-delay query, while `hea_clock_fit` requires at least two samples
with nonzero reference-time spread.

The fit is `offset(t) = offset_us + drift * (t - reference_pi_us)`. `drift` is
dimensionless microseconds-per-microsecond and `drift_ppm` is the same slope times
1,000,000. It also reports RMS and maximum residuals. Nonfinite results and
overflowed timestamp differences are rejected.

## STM32 timer wrap

`hea_clock_wrap_extend` extends a uint32 microsecond timer to uint64 using the
usual forward-distance rule. Exact half-range is ambiguous and is rejected, as
are regressions; rejected calls leave state and output untouched. Every gap must
be strictly below 2^31 microseconds. Initialization accepts an arbitrary raw
epoch and deliberately invents no relationship to the Pi epoch. For phase-2/phase-3, feed
each timestamp through one state in chronological order before constructing a
sample.

## Reproduction and tolerances

Run `make -C protocol test` for the normal ASan/UBSan build and
`make -C protocol test-char-modes` for `-fsigned-char` and `-funsigned-char`.
The deterministic test data contains 6 drift samples (4 retained by the
capacity/window), 4 jitter samples, 3 age/regression attempts (1 retained
after eviction), 2 large-reference samples, and 2 wrap-through-phase-2/phase-3 samples.
The test suite performs 50 checks. It uses 0.01 us offset tolerances, 0.001 ppm
for the ordinary synthetic drift check, 0.55 us deterministic-jitter RMS
tolerance, and 256 us for the deliberately huge reference-zero intercept
(whose `double` representation cannot retain low microsecond bits near 2^60).
These are test-model tolerances, not hardware accuracy claims.

The tests demonstrate the expected bias from asymmetric one-way delay rather than
discarding it: RFC round-trip math removes symmetric delay but cannot infer which
direction carried extra delay. Actual UART serialization, userspace scheduling,
oscillator drift, timestamp placement, Pi/STM32 hardware, and wiring remain
unvalidated here; hardware testing is required for an accuracy or deadline claim.
