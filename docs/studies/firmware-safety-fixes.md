# Firmware safety fixes: disarm brake, fatal-halt stop frames, still startup

## Tóm tắt tiếng Việt

[COMPUTED: FW8 safety-fix run] Ba lỗi được xác nhận trong rà soát an toàn firmware đã được sửa: gạt công tắc arm xuống khi đang giữ thăng bằng giờ phanh có điều khiển (CONTROLLED_STOP) thay vì cắt mô-men; sau lỗi nghiêm trọng, firmware gửi lại khung tắt động cơ; và bộ ước lượng chỉ học bias con quay khi robot đứng yên. Bốn kiểm thử host mới: 4/4 lỗi trước khi sửa, 4/4 đạt sau khi sửa. Chỉ kiểm thử host và mô phỏng; chưa chạy trên board thật.

## What was reviewed

[FROM FILE: FW8 development report] A safety review of the published firmware listed four should-fix findings, F-04 to F-07. Each was checked against the source before any change. Three were confirmed and fixed; one was rejected. Findings F-01 to F-03 are bench items and were not changed (see the last section).

| ID | Verdict | What was wrong | Change |
|---|---|---|---|
| F-04 | Confirmed | [FROM FILE: `firmware/safety_fsm.c`, before] The arm switch going LOW while balancing latched a FAULT, which disables torque, so a balancing robot would fall. | [FROM FILE: `firmware/safety_fsm.c`] In BALANCING and STEP_ASSIST, arm LOW now enters CONTROLLED_STOP with reason `ARM_SW_DROPPED`; inside CONTROLLED_STOP the arm-LOW fault is removed, so braking continues to REST and the existing disarm. The radio e-stop switch, tilt, drive, power and S11 faults still cut torque at once. |
| F-05 | Confirmed, narrower than stated | [FROM FILE: `firmware/target/main.c`, before] The fatal handler masks interrupts and loops, so the repeated motor-disable burst never runs after a HardFault or `Error_Handler`; only the first burst goes out. | [FROM FILE: `firmware/target/target_estop.c`] The halt body moves into `target_estop_fatal_halt()`, which now polls the emergency CAN service (disable frames only, timed from the cycle counter, which runs with interrupts masked). The independent watchdog still resets the core. |
| F-06 | Confirmed, mechanism partly wrong | [FROM FILE: `firmware/control/s10_estimator.c`, before] The [FROM FILE] 101-sample startup window checked only the mean acceleration, so rotation during startup was learned as gyro bias. A static tilt was measured correctly, so the review's large attitude error was not confirmed. | [FROM FILE: `firmware/control/s10_estimator.c`] Every startup sample must be still: [ASSUMED] \|gyro\| ≤ 0.05 rad/s per axis and \|a\| within [ASSUMED] ±10 % of 1 g. A failing sample restarts the window instead of latching a failure. |
| F-07 | Rejected | [FROM FILE: `firmware/target/target_fdcan.c`] The suggested race cannot happen: every transmit-FIFO write already runs with interrupts masked, and the stop path already aborts all pending transmit buffers. | None; existing host tests already cover it. |

## Before and after

[COMPUTED: FW8 safety-fix run; repeated in the FW9 integration run] Four host tests were added. They were run first against the unfixed firmware and then against the fixed firmware.

| Test | Before the fix | After the fix |
|---|---|---|
| `fw8_arm_stop` (F-04) | [COMPUTED] FAIL: state FAULT, torque off | [COMPUTED] PASS: CONTROLLED_STOP with torque on, REST after [COMPUTED] 848 ms, then DISARMED; radio e-stop and tilt still cut torque |
| `target_fatal_halt_repeat` (F-05) | [COMPUTED] FAIL: [COMPUTED] 6 disable frames in [COMPUTED] 5,000 µs | [COMPUTED] PASS: [COMPUTED] 12 disable frames ([COMPUTED] 2 per motor), also with a full transmit FIFO |
| `fw8_startup_motion` (F-06) | [COMPUTED] FAIL: ready with a learned pitch bias of [COMPUTED] 0.3020 rad/s | [COMPUTED] PASS: not ready while rotating; ready with bias [COMPUTED] 0.0020 rad/s after a still window |
| `fw8_startup_gravity` (F-06) | [COMPUTED] FAIL: ready at [ASSUMED] 1.30 g | [COMPUTED] PASS: not ready |

[FROM FILE: FW8 development report] Two expectations in `tests/test_safety_fsm.c` encoded the old arm-LOW policy and were updated on purpose; the radio e-stop, tilt, drive-fault and frozen-encoder torque-off checks in the same tests are unchanged.

[COMPUTED: FW8 safety-fix run] Regression gates on the fixed firmware: build with [COMPUTED] 0 compiler warnings; firmware in the loop [COMPUTED] 6/6 passed with output byte-identical to the previous firmware; the frozen-encoder set of [COMPUTED] 108 runs had [COMPUTED] 0 falls with every record identical. [FROM FILE: FW8 development report] The simulations do not move because the harness never drops the arm switch while the radio link is up, F-05 is target-only, and every simulated startup sample was already still.

[COMPUTED: this publication's fresh build] The published firmware with these fixes builds in Release with [COMPUTED] 0 compiler warnings and passes CTest [COMPUTED] 52/52.

## What this teaches

[STANDARD PRACTICE] A disarm control and an emergency stop are different controls. Cutting torque on a balancing robot is itself a hazard, so the handset needs one control that brakes and one that cuts power. Code that runs after a fatal fault must not depend on interrupts. A startup calibration should check every sample, not only the average.

## What the real robot must confirm

[UNKNOWN] Pilots must be told that the arm switch now brakes and disarms; the immediate torque cut is the radio e-stop switch, and the channel layout is [ASSUMED]. The F-06 thresholds are [ASSUMED] to be wider than the IMU sensor's zero-rate offset and noise at rest; log raw gyro at boot on the bench. If the offset is larger, the estimator never becomes ready, which fails safe because the robot cannot arm. F-05 is host-tested only: on the board, trigger a deliberate fault in a debug build and confirm with a CAN analyser that a second disable burst follows about [ASSUMED] 1 ms after the first.

[FROM FILE: FW8 development report] Not changed here and still open: USB CDC enumeration (F-01), the safety-I/O stubs for reset pins, contactor mirror, gate enable and bus voltage (F-02), and the per-axis torque and encoder sign, which is checked only in MuJoCo (F-03). A low-torque tethered spin test on all six axes should come before balance mode.
