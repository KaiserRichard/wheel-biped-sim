**Tóm tắt [FROM FILE / COMPUTED].** Đã đóng gói mã C, lớp phần cứng STM32H723, kiểm thử host và mô hình FIL. Bản FW6FIX có 46 kiểm thử host; các kết quả FIL được trích dẫn riêng và chỉ là mô phỏng. Lớp phần cứng mới chỉ được kiểm tra bằng kiểm thử host và mô phỏng; chưa chạy trên board thật.

# Portable robot firmware

[FROM FILE] This is the FW6FIX source snapshot, extending slice 7 and the STM32H723 hardware layer. The nested `firmware/` directory contains portable C11 modules; `firmware/target/` adapts them to the CTRL rev4 board. No flashed robot image is included, and the vendor STM32 HAL is not distributed. [Verification receipts](evidence/README.md).

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

[FROM FILE] CMake is the canonical test entry point; Release builds keep test assertions active.

[FROM FILE] FIL needs MuJoCo, NumPy and a built shared controller library. On Linux, after the CMake build, run `python tests/fil/run_fil.py`. The FIL model (`sim_model/`) is not published: it loads CAD meshes whose redistribution licence is not established (see [NOTICE](../NOTICE.md)). The host tests run without it. [Details](sim_model/README.md).

## Run-up and frozen-encoder handling

[FROM FILE: FW6FIX source and supplied development/review receipts] Run-up uses back-off, settle and charge phases for a declared single curb only (declared height at most 12.1 mm). Field declarations are refused and flagged. Stopping among repeated steps can trap a wheel in a gap and cause a fall; field run-ups remain disabled pending a reviewed stopping strategy and an owner decision. The earlier field-crossing result is withdrawn.

[FROM FILE: `firmware/s11_packet_policy.c`, `firmware/control/balance_ctrl.c`] A confirmed single frozen encoder requests `CONTROLLED_STOP`. A frozen wheel uses its partner's increment/rate; a frozen hip normally mirrors the same-role hip on the other leg, with held geometry as fallback. Wired stop, drive faults, unrecoverable tilt and insufficient remaining feedback retain torque-off escalation.

[FROM FILE: `firmware/safety_fsm.c`] An unsettled controlled stop times out after 20 s to torque-enabled `REST`, raises `stop_timeout_alarm` with `STOP_TIMEOUT`, and leaves `rest_confirmed=0`. This is an alarm, not proof of settling. Failed assists have the separate `STEP_ASSIST_ABORT` reason. The obsolete leg-pulse mode is removed.

[FROM FILE: supplied FW6FIX report, Gates; FW6 review, What I reran] Simulation-only development results: **CTest 46/46, FIL 6/6, 108 frozen-encoder runs with 0 falls** (72 detected stops, 36 undetected freezes). These are supplied evidence; public host tests can be rerun with the commands above. No hardware validation or new FIL campaign is claimed by this publication batch. See the [measurement checklist](../docs/sim-to-real/sim-to-real-checklist.md).

[FROM FILE] The standalone contract fixture includes its netlist and clock audit under `tests/contract/`. The healthy replay corpus contains 110 simulated runs (976,169 samples); it is test input, not a real-robot log.

## STM32H723 hardware layer

[FROM FILE] `firmware/target/` contains the board layer for the CTRL rev4 control board: clock and pin setup (`target_hw.h`), the 500 Hz control loop (`main.c`), FDCAN adapters (`target_fdcan.c`), radio receiver input (`target_crsf.c`), USB CDC link (`target_usb_cdc.c`), the independent watchdog (`target_iwdg.c`), wired-stop inputs and the permit output (`target_estop.c`), and the linker script.

- **Wired stop.** Both stop inputs (PB12 and PD7) trip on a rising edge. The trip test is `latched OR pin open OR pending EXTI flag` for each input, so a short pulse that ends before the interrupt runs still blocks the permit output and CAN transmit. Clearing the latch needs both inputs LOW and a local reset.
- **CAN mapping.** J5/FDCAN1 drives the right leg (axes 3–5), J6/FDCAN2 the left leg (axes 0–2), and J7/FDCAN3 is a spare with no motor traffic; see [REV4 CAN mapping](can_map/REV4_CAN_MAPPING.md). The emergency disable repeat samples its timestamp with interrupts masked.
- **Watchdog.** IWDG reload 52 at prescaler 32 gives a 50.476 ms worst-case timeout at the fastest LSI. A watchdog reset blocks precharge until a local reset.
- **Host tests.** `tests/target/test_target_hal.c` compiles the target sources against a local HAL stub (`tests/target/hal_stub.*`; constants regenerated by `make_hal_stub.py`, test values only). `tests/test_review_contract.py` checks the board connector roles in the vendored `tests/contract/ctrl_rev4.net` and runs the vendored clock audit `tests/contract/clock_check.py` (0 FAIL; one WARN: USART6 runs at 419,355 baud against a 420,000 target, within asynchronous UART tolerance), and checks the `-Wconversion -Werror` flags.

[STANDARD PRACTICE] To link `wbr_stm32h723.elf`, install `arm-none-eabi-gcc` and pass the location of an STM32CubeH7 `Drivers/` tree, which is vendor code and not included here:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DWBR_STM32_HAL_DIR=/path/to/STM32CubeH7
cmake --build build --target wbr_stm32h723
```

Without that directory, the host build and all host tests still run; only the linked image is skipped. Continuous integration compiles the portable modules for Cortex-M7 without the vendor HAL.

## What this teaches

[STANDARD PRACTICE] Transport tests, controller tests and plant simulation answer different questions. A quantized velocity channel can decode zero as a small nonzero value; integrating that bias can falsely declare a stationary angle stuck. Predictor history must preserve the deviation quantity used in the controller equations.

## What the real robot must confirm

[UNKNOWN] Target scheduling, interrupt/cache behavior, encoder conventions, drive limits, physical contact, radio-loss stopping and actual balance have not been established by these host checks. A passing FIL fixture is simulation evidence only.

[UNKNOWN] The hardware layer has not run on a board. Still to confirm: the J5/J6/J7 harness, drive CAN IDs, termination and loaded-bus timing; wired-stop trip levels, bounce and the external watchdog cutting the permit; crystal, HSI48 and LSI tolerances and the actual USART6 baud; USB enumeration (the descriptor/EP0 path is a stub); wheel-drive MIT register limits; and the IMU sensor SPI driver, whose single-slot mailbox must grow to a ring buffer when it is wired.

[FROM FILE] DM-jump-derived PID, VMC and motor-driver code retains credit to Tran Nguyen Binh (DM-jump) and dmBots and the full [MIT notice](firmware/DM_JUMP_LICENSE.txt).
