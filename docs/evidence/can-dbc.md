**Tóm tắt [COMPUTED].** Đã đối chiếu lại mã C, DBC và ba định dạng log. Chưa kết nối động cơ thật.

# CAN database evidence

[COMPUTED] The staged [toolkit](../../tools/can/README.md) reproduces byte agreement for 810 C vectors and 12,024 frames per format across candump, ASC and BLF. These counts come directly from the rerun [summary](../../tools/can/out/python_summary.json), not the draft. Nine DBC files are included.

[FROM FILE] The saved [MATLAB summary](../../tools/can/out/matlab_summary.json) records 1,890 command payload checks. That is prior campaign evidence; this staging pass did not start MATLAB.

## Figures and data

[FROM FILE] Numeric [bus-load tables](../../tools/can/out/bus_load.csv) and [codec steps](../../tools/can/out/codec_steps.csv) are retained. Draft figure links without staged, reviewed images were removed.

## What this teaches

[STANDARD PRACTICE] Wire-code agreement is stronger than approximately equal decoded values. The same physical input can produce different bytes if rounding differs from the source driver's quantization.

## What the real robot must confirm

[UNKNOWN] Offline parity does not establish drive configuration, wiring, physical torque or arbitration latency. Confirm those separately before commanding a motor.
