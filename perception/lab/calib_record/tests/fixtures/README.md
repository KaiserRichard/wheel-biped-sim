# Fixture provenance

`real-sustained-analysis.json` is a **verbatim** small saved analysis from the
read-only shared checkout:

```text
<local workspace>/Desktop/Research/wbr-perception-phase1-runtime/
  experiments/2026-10-13-pi5-env-snapshot/bundles/
  wbr-runtime-20261013T044130Z/sustained-analysis.json
SHA-256: 363d657c570f48fc5367c7f98539ee0081fb1949e55ef2825d8e2bdee9cbf48b
```

Its own `source` points to the historical 2026-09-12 vendor CSV. It contains
**MEASURED old-unit** IR/15, gyro/200 and accel/~101 transport observations;
SDK clock domains were not serialized, and XYZ were vendor-rounded. Original
raw sample CSV is unavailable here. Tests use the saved counts, spans, gaps and
frame-number statistics to verify rate-policy decisions: the IMU observations
meet the diagnostic rate limits, while 15 Hz IR fails the 30 Hz calibration
policy. This fixture does not establish new-unit timing, excitation or calibration.

Other fixtures are generated in temporary directories at test time. They reuse
the independent SQLite/CDR serializer in `tools/take_tools/tests/fixture.py` and
the native Allan recorder's documented CSV/metadata schema. Synthetic thresholds
shorten durations and image dimensions with explicit test-policy sources;
production defaults are also exercised. No fabricated take is labeled MEASURED
hardware evidence. Temporary recordings are removed after every test.
