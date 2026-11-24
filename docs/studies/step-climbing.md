# Why a slow wheel climb fails at low friction

## Tóm tắt tiếng Việt

[COMPUTED: `../STEP3RC/STEP3RC_REPORT.md`, A2] Với μ=0.38, bánh xe không thể leo chậm bậc 5 mm hoặc 10 mm trong mô hình quasi-static vì điều kiện ma sát là μ ≥ tan α; cả hai bậc cần μ lớn hơn 0.38. [COMPUTED: report, A5] Run-up mode 1 đã qua 7/7 ô chậm trên trường bậc F3 trong mô phỏng; pitch đỉnh 7.5–11.3°. [FROM FILE: DOCS_B6 task brief] Tích hợp firmware FW6 đang tiến hành.

## Why slow climbing is impossible at μ=0.38

[COMPUTED: `../STEP3RC/STEP3RC_REPORT.md`, A2] In simulation, wheel radius is 60.3 mm. At the step edge, `cos α = (r − h)/r`; the friction cone condition for a quasi-static climb is μ ≥ tan α.

| Step height | Edge angle α | tan α | Result at μ=0.38 |
|---|---:|---:|---|
| 5 mm | 23.5° | 0.434 | [COMPUTED] Below threshold |
| 10 mm | 33.5° | 0.662 | [COMPUTED] Below threshold |
| 15 mm | 41.3° | 0.879 | [COMPUTED] Below threshold |

[STANDARD PRACTICE] When the wheel leaves the floor, the edge must support the robot. In a quasi-static climb, the edge force must be vertical and remain inside the friction cone. Leg extension, crouch, and wheel torque are internal forces, so they cannot change the net external force direction. The robot needs momentum and deceleration to create the required external force. At μ=0.38, the friction-cone half-angle is 20.8°, below the 5 mm and 10 mm edge angles.

[COMPUTED: STEP3RC report, A2] In the simulated nominal 10 mm, μ=0.38 case, edge friction use reached 1.0 and the edge slipped. Leg extension raised the center of mass by 11.8 mm, but wheel-center height changed by zero and forward wheel displacement was 0.04 mm. The leg pulse lifted the body without moving the wheel over the edge.

## Run-up simulation result

[COMPUTED: STEP3RC report, A5] The replacement assist backs off, settles, charges, then hands control back gradually. On the full F3 step field, all 7 of 7 slow simulation cases crossed and ended in BALANCING. Peak pitch was 7.5–11.3°. Peak wheel torque was at most 1.40 N·m of the 2 N·m modeled peak; peak hip torque was at most 2.14 N·m of 7 N·m.

[COMPUTED: STEP3RC report, A5] The assist is gated off above 0.15 m/s, and the reported grid-speed outcomes match the prior slice-8 grid. Host C tests on the modified firmware passed 27/27 with default `step_mode=0`. The report also records that 15 mm crossed on nominal but fell on `draw_001` and the assumed 6.2 kg model; 20 mm and 25 mm fell. The 12 mm gate is a simulation margin, not a hardware limit.

[UNKNOWN: STEP3RC report, A5] Real friction, tyre and edge compliance, motor torque-speed behavior, and stopping distance are unverified. The report calls for measuring friction and stopping distance before any robot step, followed by a tethered single-curb test at 5 mm.

## What this teaches

[COMPUTED: STEP3RC report, A2 and A5] A leg pulse can raise the chassis without moving the wheel when contact forces cannot overcome the edge geometry. Run-up adds momentum so the edge can provide a different external force while the wheel climbs.

[STANDARD PRACTICE] A simulated crossing is evidence for the tested model and declared surface only. Host declaration, stopping room, measured traction, and controlled hand-back are part of the system behavior, not incidental details.

## Firmware status

[FROM FILE: DOCS_B6 task brief] Firmware integration under FW6 is in progress. The run-up result remains simulation evidence; it is not a real-robot validation.

## What the real robot must confirm

[UNKNOWN] Measure surface friction, stopping distance at approach speed, wheel/edge compliance, motor torque-speed limits, actual clearance behind a stall, and pitch response. The STEP3RC report recommends a tethered 5 mm single-curb trial only after the bench measurements.
