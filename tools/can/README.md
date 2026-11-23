**Tóm tắt [COMPUTED / FROM FILE].** Kiểm thử chạy lại khớp 810 vector C và 12.024 khung trên mỗi định dạng candump, ASC, BLF. Đây là kiểm thử ngoại tuyến.

# CAN databases and offline validation

[FROM FILE] Nine DBC files cover the study addressing, special commands and direction-specific firmware addressing. `wbr_dm.dbc` is the study profile; `dbcs/` contains the firmware profiles, including the wheel codec discrepancy. Firmware IDs overlap across independent buses and directions: do not combine these databases into a single arbitration domain without checking the motor map.

[COMPUTED] The staging rerun compiled `tests/dm_oracle.c`, regenerated the C oracle CSV and passed `python/run_tests.py`. [Python summary](out/python_summary.json): 810 C vectors and 12,024 frames in each of three formats agree. [Saved MATLAB summary](out/matlab_summary.json) records 1,890 command checks from the source campaign; MATLAB was not started during staging.

[FROM FILE] Synthetic candump and ASC inputs and decoded CSVs are retained. BLF is generated during `python/run_tests.py`; the opaque binary output is omitted from the public tree. The replay is derived from aggregate simulation channels, not measured six-axis motor telemetry. [Fixture provenance](reference/trace_provenance.json).

## Reproduce

[STANDARD PRACTICE] Use an existing Python environment with the versions in `python/requirements-lock.txt` and a C compiler:

```sh
gcc -std=c11 -O0 -ffp-contract=off tests/dm_oracle.c -o tests/dm_oracle
./tests/dm_oracle < tests/oracle_input.txt > out/c_oracle.csv
python python/run_tests.py
```

[FROM FILE] MATLAB entry points are in `matlab/`; the saved results can be read as CSV without MATLAB. The original reconstruction helper expects a separately supplied read-only driver source; it is unnecessary for the frozen tests.

## What this teaches

[STANDARD PRACTICE] Codec ranges define a wire representation, not safe motor ratings. Quantization must match the C implementation before floating-point physical values are encoded. Bus load depends on stuffing, replies and topology; matching three log formats checks decoding consistency, not actual bus timing.

## What the real robot must confirm

[UNKNOWN] Installed IDs, configured torque/velocity ranges, bitrate, bus termination, timestamps and physical actuation signs require hardware verification. DM-jump-derived oracle functions retain Tran Nguyen Binh (DM-jump) and dmBots credit and the [MIT notice](THIRD_PARTY.md).
