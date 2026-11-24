# Pi 5 runtime profiling (phase-6)

This stdlib Python tool samples a target process and Pi resources, analyzes
runtime observer frame events or the existing ORB-SLAM3 patch timing CSV, and
gates latency validity on sampled power/throttle and temperature state. It
reuses `tools/runtime_observer/resource_sampler.py`, `aggregate.py`,
`patch_timing.py`, and `stats.py`. Fixture results validate software only; they
are not a Pi performance gate PASS.

Run from the repository root on the Pi (Python 3.11+):

```sh
python3 tools/eval_pipeline/profile/profile.py sampler --pid 1234 --out /tmp/wbr-profile-run --period-s 0.1 --throttle-period-s 1.0
python3 tools/eval_pipeline/profile/profile.py latency --events /tmp/run/events.csv --out /tmp/wbr-profile-run
# Add --clockmap /path/to/qualified-map.json for source stamps from another clock.
# Or parse the existing patch's run_tracking.csv:
python3 tools/eval_pipeline/profile/profile.py latency --tracking /tmp/run/run_tracking.csv --out /tmp/wbr-profile-run
python3 tools/eval_pipeline/profile/profile.py validity --analyze-only /tmp/wbr-profile-run
python3 tools/eval_pipeline/profile/profile.py report --run /tmp/wbr-profile-run
```

Add `--threads` for per-thread CPU deltas. `sampler` writes `samples.csv` and
`metadata.json` into a new output directory, stops after Ctrl-C when `--samples`
is omitted, and tolerates absent proc/sysfs/VideoCore fields as `UNKNOWN`.
The CSV's `throttle_sampled` column distinguishes 1 Hz poll rows from
intervening 10 Hz rows; failed polls remain `UNKNOWN` and block validity.
Metadata pairs a UTC anchor with `CLOCK_MONOTONIC`; UTC is descriptive and is
never subtracted from monotonic timestamps. RSS/PSS are bytes. CPU utilization
is sampled as per-core percentages; the target's process CPU is percent of one
core and can exceed 100%. Frequencies are `scaling_cur_freq` kHz. Thermal zones
and cooling-device `cur_state` are preserved by name. `vcgencmd get_throttled`
is polled at the configured lower rate.

`latency` reports nearest-rank P50/P95/P99/max (rank `ceil(q*n)`, one-indexed),
frame count, over-budget tracking frames, and the configured 33.3 ms frame
budget. Observer `track_start`→`track_end` and `frame_received`→
`snapshot_published` are both in `host_steady` / Linux `CLOCK_MONOTONIC`; the
second is end-to-end. The patch CSV reports only tracking-call duration; it
contains no host epoch, so source stamps are not combined with it and
end-to-end/realtime factor remain UNKNOWN. Event-file realtime factor is the
frame-received source timestamp span mapped to host steady time divided by the
observer host span. It is UNKNOWN unless source epochs are from the same host
session or a qualified existing ClockMap JSON is supplied with `--clockmap`.
That map must target the same observer `host_steady` domain and session before
mapped epochs are compared.

`validity` supports `--analyze-only <run-dir>` and writes `result.json` using
the shared PASS/FAIL/BLOCKED contract. Any current or historical
under-voltage/frequency-cap/throttle/soft-temperature bit fails the run. Missing
throttle or temperature observations block it. Raspberry Pi says Pi 5 begins
throttling when temperature reaches 80 °C ([official Pi 5 article](https://www.raspberrypi.com/news/heating-and-cooling-raspberry-pi-5/));
the maximum sampled SoC temperature must remain strictly below the 80 °C limit
in `config.json` (79.9 °C passes, 80.0 °C fails). Latency results from any
non-PASS run are INVALID. All numeric thresholds and defaults
are centralized in `config.json`, with source/rationale labels; selected limits
are marked PROPOSED.

`report` writes `summary.json` and `report.md`; a temperature time-series PNG is
created only if matplotlib is installed. No matplotlib installation is
required. Files under this directory's `tests/fixtures/` include one small
derived row from real saved N4A evidence (reported `0x50005`, 63 °C); it is not
a full raw resource trace.

## Criteria and limits

| Criterion | Meaning | Cannot prove |
|---|---|---|
| No throttle/undervoltage | No current or sticky historical invalid bit during observed samples | Conditions between polls, correctness of firmware reporting, or stable external supply |
| Temperature | Sampled max below configured soft limit | Junction hotspot accuracy or thermal behavior between samples |
| Tracking latency | Time spent between observer track events, or parsed patch tracking-call durations | Absolute camera-to-pose latency without full lifecycle events and trustworthy clock mapping |
| End-to-end latency | Observer receive to published pose on the host monotonic clock | Sensor timestamp age when clocks are unmapped; pose correctness or tracking quality |
| Realtime factor | Same-clock observed interval ratio when caller attests mapped source identity | Independent validation of that mapping, drops before receive, accuracy, or hardware gate acceptance |

The sampler is best-effort and adds observer load. `/proc`, sysfs and
`vcgencmd` snapshots can be missing or non-atomic. It does not capture
controller/STM32 timing, D435i camera IMU timing, image backlog, or guarantee
the Pi's camera clock is mapped to Linux monotonic time.
