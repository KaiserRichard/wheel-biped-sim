# P3: robot log import and identification rehearsal

Python standard library only; no MATLAB invocation, licence, toolboxes or third-party Python packages required. Start with `LOG_SPEC.md`, `REHEARSAL.md`, `IDENTIFIABILITY.md`, and `EXPERIMENTS.md`.

## Run

From this directory, with Python 3.11 or newer. Set `PYTHONDONTWRITEBYTECODE=1` in the environment for every command so bytecode stays out of the publish tree:

```bash
export PYTHONDONTWRITEBYTECODE=1
python -m unittest discover -s tests -v
python rehearsal.py
python verify_package.py
```

`verify_package.py` runs both commands and checks full-rehearsal recovery thresholds. It streams test/rehearsal output and writes `REHEARSAL.md`, `rehearsal_results.json`, `SOURCE_MANIFEST.json`, and `verification.json` (exit codes, Python version, released-file hashes and hardware acceptance status). Expect roughly two minutes. Seeds are fixed; execution time and generation dates vary. Quick mode, `python rehearsal.py --quick`, writes only `out_quick/` and is also exercised by the tests.


## Offline importer API (invented filenames; no real logs bundled)

```text
python -m wbrlog capture.bin capture_long.csv
python -m wbrlog capture.txt aligned.csv --format wide --meta measured_meta.json --event studio_t0 --studio m13_loop3_estimation.csv --method linear --channels est.pitch est.body_speed est.L.L est.L.R
python -m wbrlog capture.bin wire.csv --format wire --event studio_t0
```

Each export gets `OUTPUT.json` with input SHA256, META, decode diagnostics, offset and optional comparison errors. `--meta` replaces the entire META object, so provide a complete measured configuration. The default time origin is the first record; `--event` or `--offset` specifies the robot time of Studio zero. `--studio` accepts the 10-column estimation CSV. Other Python APIs load 21-column packet wire and 24-column controller exports.

Long export retains samples at their native timestamps. Wide export uses the Studio time grid when supplied, otherwise `--dt` (default 1 ms). ZOH is causal and holds the last value indefinitely; inspect age and gaps before using it. Linear interpolation is for smooth quantities. `--method angle` interpolates a wrapped-angle-only channel selection; do not use it on mixed physical units. `studio.compare` automatically uses angular interpolation and circular residuals for wrapped encoder/yaw channels when its method is linear. No anti-alias filter or dropout imputation is applied.

The wire export rebins IMU to 1 ms and holds encoders. It loses sub-tick age and individual encoder acquisition stamps, and is an approximate packet-replay input. The untouched long export is the timing evidence. The Python task produces this file only; it does not run a Studio estimator or MATLAB.

## Layout and limitations

- `wbrlog/`: binary/text codec, DM frame decoding, metadata-based SI/frame conversion, Studio channel mapping, interpolation, alignment, exports and synthetic wire generator.
- `ident/`: overlapping Allan deviation/variance fit, lever and friction regression, lag/delay output-error fitting and timestamp/regression loop delay.
- `rehearsal.py`: seeded duration sweeps and independent sequential calibration; local M13 CSV round trip across timer wrap.
- `tests/`: format budgets/mapping, CRC/damaged-log recovery, unit/sign/frame conversion, timing, resampling, CLI and identification recovery tests.
- `SOURCE_MANIFEST.json`: relative released-file hashes and opaque original-reference hashes. Withheld inputs cannot be reconstructed from hashes. `m13_loop3_estimation.csv` is simulation output, not hardware data.

Decoder recovery is for complete files, not a streaming transport. Large files are loaded into memory; segment captures, especially hour-long stationary runs. Split at MCU reboots or META changes. Timestamp unwrapping assumes adjacent received records are separated by less than half the 32-bit wrap interval (about 35.8 min). Duplicate or out-of-order global sequence numbers are not a supported transport; bounded timestamp reordering is allowed.

Defaults are visibly marked placeholders. The importer reports missing META and can decode with defaults for inspection, but that output is not calibrated evidence. Unsupported CAN flags/DLC and special motor-mode commands are counted and excluded from torque channels. FD/extended identifiers are not supported by the v1 16-bit CAN-id/8-byte payload record. Do not treat logs with these counters as complete motor histories.

No real stationary log was available. The original measured-data gate requiring measured-data import and fitting is still open. This package completes the requested synthetic preparation; all hardware applicability remains UNKNOWN.

## Public scope

This is a standard-library codec/importer and seeded system-identification rehearsal. Run `python -m unittest discover -s tests -v` for the 45 software tests, or `python verify_package.py` for those tests plus the full synthetic rehearsal and recovery gates. Python 3.11+ is required. The bundled 10-column CSV is simulation output; its units and source hash are recorded in `SOURCE_MANIFEST.json`.

This does NOT demonstrate real-robot import, physical synchronization accuracy, measured sensor performance, motor calibration, a validated controller, or hardware acceptance. Hardware is UNKNOWN. Contact friction and battery resistance are UNASSESSED. P2 screening and its CAD-derived inputs are outside this release.
