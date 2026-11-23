**Tóm tắt [FROM FILE / COMPUTED].** Đã đóng gói mã C, kiểm thử host và mô hình FIL. Bản công khai đạt 26/26 kiểm thử host ở Debug và Release. Kết quả FIL trước đó do chủ dự án xác nhận là 6/6; chưa chạy lại FIL khi đóng gói.

# Portable robot firmware

[FROM FILE] This is the slice 7 source snapshot. The nested `firmware/` directory contains portable C11 modules; it does not contain a target HAL, RTOS application or flashed robot image. [Verification receipts](evidence/README.md).

| Development slice | Change retained in this snapshot [FROM FILE] |
| --- | --- |
| 2 | USB framing, CAN codec/scheduling, safety state machine, estimator, balance control and MuJoCo FIL integration. |
| 3 | Standing position anti-windup and the trim-learning correction; nominal and displaced chassis models are distinguished. |
| 4 | Shared standing/driving/stopping position clamp, continuous handoff and brake-then-balance radio-loss policy. |
| 5 | Paired driving FIL comparison exposed a remaining speed-tracking failure, motivating the controller corrections in the next slice. |
| 6 | Predictor history stores feedback deviations separately from feedforward; wheel-loss adaptation and authority scaling corrections. |
| 7 | Integrates the reviewed S11 mailbox/policy and retains the controller corrections. A velocity quantization deadband prevents false frozen-encoder faults at standstill. |

[FROM FILE] The slice summaries are edited from the development reports; internal working reports and prebuilt binaries are omitted. The independent S11 suite lives in `s11_verified/`; the runtime implementation is `firmware/s11_packet_policy.c`. The runtime includes the later quantization correction, so it is not byte-identical to the archival round-two policy. See [packet review evidence](../docs/evidence/s11-packet-policy-fix.md).

## Build and test

[STANDARD PRACTICE] With a C11 compiler, CMake and threads support:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build
ctest --test-dir build --output-on-failure
```

[FROM FILE] A GCC-compatible compiler can also use `python run_host_tests.py --cc gcc`; add `--release` for optimization with mandatory test checks. On Windows MinGW only, this runner keeps float-conversion warnings visible without treating them as errors because its math classification macros emit them. The CMake source retains its original stricter flags. Test assertions remain active in both configurations.

[FROM FILE] FIL needs MuJoCo, NumPy and a built shared controller library. On Linux, after the CMake build, run `python tests/fil/run_fil.py`. The FIL model (`sim_model/`) is not published: it loads CAD meshes whose redistribution licence is not established (see [NOTICE](../NOTICE.md)). The host tests run without it. [Details](sim_model/README.md).

## What this teaches

[STANDARD PRACTICE] Transport tests, controller tests and plant simulation answer different questions. A quantized velocity channel can decode zero as a small nonzero value; integrating that bias can falsely declare a stationary angle stuck. Predictor history must preserve the deviation quantity used in the controller equations.

## What the real robot must confirm

[UNKNOWN] Target scheduling, interrupt/cache behavior, encoder conventions, drive limits, physical contact, radio-loss stopping and actual balance have not been established by these host checks. A passing FIL fixture is simulation evidence only.

[FROM FILE] DM-jump-derived PID, VMC and motor-driver code retains credit to Tran Nguyen Binh (DM-jump) and dmBots and the full [MIT notice](firmware/DM_JUMP_LICENSE.txt).
