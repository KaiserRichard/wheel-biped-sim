# task G-B verification — OPEN

Executed on the canonical Mac, 2026-10-14. No SSH, network, installs,
Git commands, device enumeration, sensor capture or ORB process. Task files
were written only under calibration/allan/. Existing evidence was read only.
All physical calibration and gate conclusions remain OPEN.

## Verified tooling

| Check | Result | Scope |
|---|---|---|
| Python full synthetic suite | 12 tests passed, 32.444 s including cold matplotlib font cache | Tooling only, under 60 s |
| Native-rate recovery | Three synthetic hours at 200 Hz, no decimation, 2,160,000 samples/axis | Correct discrete white/Brownian generator; no measured IMU |
| C++17 clang syntax check | Exit 0 with Wall/Wextra/Werror/pedantic/pthread | Local API stub only; not librealsense build |
| Bash syntax | Exit 0 | No physical wrapper run |
| Plot output | Both sensor PNGs generated in temporary synthetic report | Installed matplotlib; temporary outputs cleaned |
| Missing matplotlib | Fits/YAML/summary produced; warning and plots skipped | Injected unavailable-module fixture |
| Wrapper | Empty directory handoff, no overwrite, free-space refusal, under-voltage label, SIGTERM drain/final power sample | Fake recorder, vcgencmd and df; no Pi hardware |

Full final Python receipt: [tests/last-run.log](https://github.com/KaiserRichard/wbr-perception-phase1-runtime/blob/fde5721af0627eaf60c78d95296ab26924425e37/calibration/allan/tests/last-run.log).
Python was 3.11.0rc2, numpy 1.26.4. Standard /usr/local and /opt/homebrew
librealsense include paths and Homebrew Cellar location were absent.
No SDK install or brew command was needed.

Synthetic inputs: N=0.03, K=0.01, dt=0.005 s, seed 20261014.
White standard deviation N/sqrt(dt); random-walk increment standard
deviation K sqrt(dt). Fits used the same analyzer functions as the CLI.
The exact CSV/CLI tests separately verify parsing, integer precision,
validation and output files.

| Axis | Fitted N | N error | Fitted K | K error |
|---|---:|---:|---:|---:|
| x | 0.0300216563 | +0.0722% | 0.00948705188 | -5.1295% |
| y | 0.0299789162 | -0.0703% | 0.00980982355 | -1.9018% |
| z | 0.0299920373 | -0.0265% | 0.00964130004 | -3.5870% |

All N errors <15%, all K errors <30%. Synthetic recovery itself took 6.171 s.
Measured fit slopes for N were -0.49963/-0.50016/-0.50001; K slopes were
+0.46365/+0.51261/+0.50292. R² exceeded 0.9982 for all fits.
An initial broadest-window fit exposed crossover contamination in synthetic
N recovery. Final selection minimizes fixed-model residual among supported
decades, with range width only breaking ties. The final receipt above reflects
the corrected algorithm; no fixture acceptance tolerance was relaxed.

## Reproduction

From repository root, offline:

```bash
python3 -m unittest discover -s calibration/allan/tests -p 'test_*.py' -v
clang++ -std=c++17 -Wall -Wextra -Werror -pedantic -pthread -Icalibration/allan/tests/stub -fsyntax-only calibration/allan/d435i_imu_record.cpp
bash -n calibration/allan/record_allan.sh
```

Temporary synthetic evidence and matplotlib cache are created below tests/
and removed by the fixtures. The stub throws on resolve/start and cannot
access hardware. It verifies C++ syntax against only the declared surface:
actual SDK overloads, ABI, device behavior and pipeline scheduling are UNKNOWN.

## UNKNOWN / future manual evidence

- Real librealsense compile/link and runtime version on the Pi.
- Physical D435i IMU sensor identity, profiles, timestamps, global-time setting,
  raw calibration warnings, sample/frame loss and throughput.
- Real SIGINT/SIGTERM shutdown and per-second disk flushing under Pi load.
- Actual serialized bytes/hour, storage qualification, rail stability and
  10-second power readout behavior; camera IMU temperature remains UNKNOWN.
- Physical stationary noise densities/walks, repeatability, half-run agreement,
  long-term slope support and suitability for Kalibr or ORB-SLAM3.

Future manual commands and capture conditions are in [README.md](README.md).
Start with a separately preserved 0.01 h pilot after CAMERA AVAILABLE and the
required hardware gates. Inspect receipts/counts/profile/timing/power/precision
and compare real file sizes with the conservative budget; then use the hours
protocol. A successful synthetic fixture does not make any hardware gate PASS.
UNKNOWN coefficients remain null in candidate YAML rather than fabricated.
