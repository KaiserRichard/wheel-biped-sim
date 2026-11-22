# D435i IMU sensor Allan tooling — OPEN

task G-B is Mac-only, camera-free preparation. These tools concern **D435i
IMU sensor, frame imu_camera**, never the STM32 body IMU. All calibration, power,
timing, physical rates and deployment conclusions remain **OPEN**. No hardware
was accessed, installed software changed, Git command run, or network used.
The task's no-Git instruction takes precedence over the usual branch/status
workflow. All new files and synthetic artifacts belong under this directory.

## Local sources and provenance

Implementation follows [calibration plan §5](../../docs/D435I_CALIBRATION_PLAN.md),
its local calibration notes and readiness references, and the historical
[recorder](../../experiments/2026-10-13-n4b-d435i-prep/d435i_record.cpp),
[SDK helper](../../experiments/2026-10-13-n4b-d435i-prep/d435i_rs.h) and
[power pattern](../../experiments/2026-10-13-n4b-d435i-prep/smoke_ram.sh).
The [new-unit enumeration](../../experiments/2026-10-13-pi5-env-snapshot/rs_calibration_new_unit.txt)
lists accel 400/200/100 Hz and gyro 400/200 Hz; this utility requests only
gyro 200 and accel 100, MOTION_XYZ32F.

External facts are inherited from the local plan's reviewer verification,
**not freshly fetched**: Kalibr [IMU Noise Model (R3)](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model)
and [YAML formats (R1)](https://github.com/ethz-asl/kalibr/wiki/yaml-formats).
Bosch IMU sensor accel flyer 120 ug/sqrt(Hz) (R8, per local plan) converts using
g=9.80665 to **1.176798e-3 m/s²/sqrt(Hz)**, approximately 1.177e-3.
IMU sensor gyro datasheet density remains UNKNOWN.
Historical SDK calibration warnings and old-unit evidence are unchanged.

## Future Pi build — not executed on this Mac

Use an isolated checksummed runtime bundle copied from this canonical Mac
directory; preserve both existing checkouts. With already-installed headers and
library, run locally in that bundle:

```bash
g++ -std=c++17 -O2 -Wall -Wextra -pthread d435i_imu_record.cpp -o d435i_imu_record -lrealsense2
sha256sum d435i_imu_record.cpp d435i_imu_record record_allan.sh power_monitor.py > build-sha256.txt
```

This matches the historical C++17/O2/pthread flags and links only librealsense2
plus the C++ standard runtime. If the installed SDK uses a non-default prefix,
use its existing pkg-config cflags/libs with the same flags; preserve the exact
command and build log. No installation is implied. The SDK version in meta.json
is the compile-header RS2_API_VERSION_STR; preserve runtime package/library
version and hash in the bundle build receipt as well.

The recorder CLI is:

```bash
./d435i_imu_record NEW_OR_EMPTY_DIRECTORY 4
./d435i_imu_record NEW_OR_EMPTY_DIAGNOSTIC_DIRECTORY 0.01 --serial ANY
```

Duration must be finite, **0.01..30 hours**. Default SDK serial is
**99005**. Only the explicit ANY bypass is accepted; it is recorded and
must not be used for accepted unit-specific calibration. SDK, ASIC,
firmware-update and USB descriptor serials are separate fields. The last three
are null/UNKNOWN in the recorder, not copied from the SDK serial.

Recorder timing counters never subtract SDK timestamps across timestamp-domain
changes. A transition increments `domain_changes` and starts a new interval
baseline; subsequent same-domain samples resume gap/reversal accounting. A
synthetic C++ regression test exercises this behavior with the local
`tests/stub` librealsense API surface.

Only motion streams start. No images, bag, ROS, ORB-SLAM3, calibration/firmware
write or option change occurs. Global-time values are read before/after for
every sensor that supports the option, otherwise null. Global-time and motion
correction are preserved. The projector is not used because no depth/image
stream is enabled; optical/projector-off behavior still requires the Pi pilot.
SDK INFO log and raw stdout/stderr retain calibration warnings.

## Future Pi capture — prerequisites remain OPEN

Do not run these capture commands before the user explicitly says
**CAMERA AVAILABLE**, stable power/storage qualification and the project's
N1 → N2 → N3 procedure. This recorder does not qualify or replace those gates.
USB must remain >=5000M; the exact raw N2 profile/timing bands and mandatory
300-second N2 soak remain unchanged.

Place the camera rigidly on a vibration-free surface. Warm up **30 minutes**
in the same IMU-only mode before the accepted recording (a separate discarded
warm-up take is possible). Keep temperature stable, avoid sunlight/heat/fan
vibration, do not touch the camera, and strain-relieve the USB cable. Use the
qualified supply/cable, steady cooling and qualified storage. Minimum
**provisional 4 h**; the Kalibr wiki recommends roughly **15–24 h**. A shorter
recording may leave walk UNKNOWN even if white noise is visible.

Run locally on the Pi, with the prebuilt recorder beside the scripts:

```bash
bash ./record_allan.sh 4 stationary-4h
# Exact disconnect-resistant command, from the isolated runtime bundle:
nohup bash ./record_allan.sh 15 stationary-15h > "$HOME/allan-stationary-15h.launch.log" 2>&1 < /dev/null &
# Alternative:
tmux new-session -s allan 'bash ./record_allan.sh 15 stationary-15h'
```

SSH is control only; acquisition and disk writes stay on the Pi. The wrapper
builds nothing. ALLAN_RECORDER may point to another prebuilt bundle binary.
Its output is **~/wbr-evidence/allan-NAME/**, with gyro.csv, accel.csv,
meta.json, sdk.log, record.log, power-10s.jsonl, power_summary.json, capture
request and tool hashes. It refuses any pre-existing run path. The recorder
itself allows an empty directory but refuses any non-empty directory.
Sibling temporary control logs are moved into the run at clean stop.
Preserve failed and diagnostic recordings rather than rerunning over them.

CSV columns are exactly:

```text
sdk_ts_ms,domain,frame_number,host_steady_ns,x,y,z
```

SDK milliseconds and XYZ use %.17g. XYZ preserves the float32 SDK value, without
decimal truncation. Domain is the SDK string, including its spaces. Host steady
nanoseconds are callback arrivals, not device exposure/measurement times.
Raw accel remains 100 Hz, never interpolated for Allan.

CALCULATED conservative text budget is **300 rows/s × 192 B/row × 3600 =
207,360,000 bytes/hour** (207.36 MB/h), plus 1 MiB/h for small logs.
192 B rounds up a 163 B field-width bound (24 chars for SDK/XYZ, 20 for
domain/counters, delimiters/newline). At 4/15/24 h CSV projections are
0.82944/3.1104/4.97664 GB decimal. This deliberately exceeds the plan's
historical 160 B/sample assumption. The wrapper prints this calculation and
refuses if filesystem free space is below **3×** the projected total. Actual
pilot row sizes/log volume, sustained disk behavior and adequate storage
remain UNKNOWN; large runs require separately qualified storage.

The callback extracts samples into a bounded 65,536-entry queue (over 218 s at
300 Hz); a separate writer drains it without frame polling or a one-frame
queue. Queue overflow, write/flush failures and nonfinite samples produce a
nonzero exit with explicit metadata, never a silent successful drop. It logs
callback/written counts, queue high-water, per-stream frame-number gaps,
duplicate/backward stamps, domain changes and largest SDK gap. Buffered CSV
output is flushed at least every second during normal I/O. Flush is not fsync
or a power-loss guarantee. SIGINT/SIGTERM stop the pipeline then drain the
queue and finalize metadata; SIGKILL/power loss cannot finalize. One-line
counts, mean rates and last domains print every 60 s into record.log.

Power samples start **before** acquisition, repeat every 10 s, and include a
final sample after recorder stop: raw vcgencmd get_throttled,
pmic_read_adc EXT5V_V, measure_temp and Linux thermal-zone SoC temperature.
EXT5V is a Pi rail readout, not a calibrated measurement at the camera plug.
Temperature is **Pi SoC**, not IMU sensor/ASIC; camera IMU temperature is UNKNOWN.
Any live bit 0x1 or new sticky bit 0x10000 labels the run
**DIAGNOSTIC - power not clean**. A pre-existing sticky bit is disclosed, not
misattributed to this run. A new sticky-only event cannot be distinguished if
that bit was already set. Failed throttling reads yield UNKNOWN monitoring;
raw ADC/temperature read errors remain logged. The 10 s samples
cannot bound transient voltage, though a newly latched sticky bit preserves
some between-sample events. A clean bit report is not a hardware gate PASS.

## Mac analysis

Once immutable raw evidence has been copied to an authorized local location:

```bash
python3 calibration/allan/allan_analyze.py calibration/allan/evidence/allan-stationary-15h calibration/allan/results/stationary-15h
# Diagnostic override only:
python3 calibration/allan/allan_analyze.py calibration/allan/evidence/diagnostic calibration/allan/results/diagnostic --allow-system-time
# Plotting is optional:
python3 calibration/allan/allan_analyze.py RUN_DIR NEW_OUT_DIR --no-plots
```

Python 3 and already-installed numpy are required; matplotlib is optional.
Absent matplotlib skips plots with a warning. Its cache is confined to the
report directory. Output must be new/empty; input evidence is never changed.

Validation rejects nonfinite samples, any duplicate/backward SDK stamps,
unknown/changing domains, cross-sensor domain disagreement, and SYSTEM_TIME
unless explicitly overridden and labeled DIAGNOSTIC. It checks median dt
against the sensor's nominal period (ASSUMED ±5%). Gaps **>1.5 periods** and
short/bursty intervals **<0.5 periods** split the data. It uses the **longest
clean segment** (earliest on ties), reports all segment lengths/break indices,
dropped sample count, spans, dt percentiles and frame-number gaps. The chosen
segment must have each period within ASSUMED ±10%; otherwise analysis is
refused. There is **no resampling across gaps**. Inspect these timing thresholds
against the real SDK pilot; they are tooling assumptions, not weakened N2 bands.

For cluster size m, adjacent m-sample mean differences use every possible
overlap; variance is mean(difference²)/2. The log-spaced tau grid reports
**n-2m+1 overlapping terms** and **floor(n/m) independent clusters** separately.
Each tau has at least three independent clusters. These terms are correlated;
cluster counts are support indicators, not confidence intervals.

Per-axis fits require one decade of tau, at least five points, at least 30
independent clusters per point, free slope within ±0.1 of the model and
R² >=0.95. Among admissible ranges, lowest fixed-slope log residual wins,
then widest range on ties. R², measured slope, residual, fit range, support
counts and any extrapolation to the reference tau are reported.
These are ASSUMED quality criteria; automatic window selection requires review.

On the log-log plot, the left falling **-1/2** region represents white rate
measurement noise; its fitted line at **tau=1 s** gives N. The rising
**+1/2** region represents Brownian rate/bias walk; its fitted line at
**tau=3 s** gives K, following Kalibr (sigma_A=K sqrt(tau/3)).
Dashed colored lines mark fitted ranges. A low/flat minimum is reported
separately as the observed **bias-instability minimum**, without a flicker
correction; it is not K. Large-tau endpoints can be unstable because few
independent clusters remain. If no supported +1/2 region is observed,
**K=UNKNOWN**; extending the recording may help.

Outputs are allan.json, imu_allan_raw.yaml, imu_allan_x10.yaml, summary.md
and (if available) allan_gyro.png/allan_accel.png. JSON retains per-axis
curves, fits, cluster counts, spans, warnings and available acquisition/power
metadata. Raw scalars are maxima across axes only if **all three axes** have
supported fits; otherwise null/UNKNOWN. The YAML keeps Kalibr keys, /imu0
and update_rate 200.0, but **null means it cannot be used by Kalibr/ORB**.
No fallback ASSUMED scalar is silently inserted. The x10 set is an explicit
ASSUMED inflation hypothesis, not measured noise. Summary compares against
current ASSUMED ORB values and the Bosch accel flyer.

Missing power/acquisition metadata leaves qualification UNKNOWN. Recorder
errors, power contamination/incomplete monitoring and permitted SYSTEM_TIME
make the report DIAGNOSTIC. Hardware traces need independent inspection,
stationarity/temperature review and first/second-half estimates before choosing
parameters; half-run stability comparison is not automated here.

## Continuous-time mapping into ORB-SLAM3

| Kalibr YAML | ORB-SLAM3 YAML | Units |
|---|---|---|
| gyroscope_noise_density | IMU.NoiseGyro | rad/s/sqrt(Hz) |
| accelerometer_noise_density | IMU.NoiseAcc | m/s²/sqrt(Hz) |
| gyroscope_random_walk | IMU.GyroWalk | rad/s²/sqrt(Hz) |
| accelerometer_random_walk | IMU.AccWalk | m/s³/sqrt(Hz) |
| update_rate: 200.0 | IMU.Frequency: 200.0 | Hz, combined gyro grid |

Use the coefficients **continuous-time, with no pre-discretization**. Discrete
white sigma is N/sqrt(dt); walk increments have sigma K sqrt(dt). ORB performs
its own frequency scaling. Native 100 Hz accel does not become an independent
200 Hz sensor merely because the downstream combined IMU grid is 200 Hz.
Raw/x10/current control require later paired N4B evaluation; no existing YAML
is edited and no physical noise value is established by synthetic tests.

## Mac verification and remaining UNKNOWNs

Reproduction commands, from repository root:

```bash
python3 -m unittest discover -s calibration/allan/tests -p 'test_*.py' -v
bash -n calibration/allan/record_allan.sh
clang++ -std=c++17 -Wall -Wextra -Werror -pedantic -pthread -Icalibration/allan/tests/stub -fsyntax-only calibration/allan/d435i_imu_record.cpp
```

The local standard /usr/local and Homebrew librealsense include locations had
no headers. tests/stub/librealsense2/rs.hpp mirrors just the used API surface:
**syntax check only**, no real SDK build, ABI validation or capture. The stub
deliberately cannot resolve/start hardware. No camera-dependent command ran.

Synthetic recovery uses **3 hours at native 200 Hz, 2,160,000 samples/axis,
no decimation**, with N=0.03 and K=0.01. Generator uses white
sigma_d=N/sqrt(dt) and Brownian increments K sqrt(dt). Tests check all axes
within 15% for N / 30% for K, exact recorder CSV roundtrip including 17-digit
XYZ and integer hosts beyond 2^53, gap splitting, timestamp/domain rejection,
diagnostic override, overlapping formula versus brute force, UNKNOWN walk,
conservative maxima/x10 and power-bit logic. Local fake Pi commands also test
wrapper shutdown/no-overwrite/space refusal; plot tests exercise installed
matplotlib and the missing-module fallback. These fixtures prove tooling only.

Verified local checks and numerical receipts are recorded in
[verification.md](verification.md). UNKNOWN: real SDK compile, Pi wrapper
commands/storage/power cadence, throughput/drops, actual global-time/domain,
temperature, stationary physical N/K, sensor-unit identity/conditioning and
long-term slope support. Future manual validation starts with a separate
0.01 h Pi pilot after authorized camera/gates, checks native profiles,
17-digit rows, callback/written/frame counts, flushes, SIGINT/SIGTERM
finalization, SDK warnings, raw power output and actual bytes/hour. Then plan
the separately qualified hours-long capture. Everything remains OPEN.
