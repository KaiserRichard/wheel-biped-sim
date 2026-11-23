# Simulation evidence

**FROM FILE** · These studies use a CAD-derived MuJoCo model and saved simulation results. Individual pages give the modeled conditions and the checks still needed on hardware. For the controller and contact methods, see [Batch 2 methods](../methods/batch-2.md).

| Study | Headline (COMPUTED, simulation only) | Evidence |
| --- | --- | --- |
| S5 · Delay-aware balance in C | A controller that remembers recent commands tolerated more gain at the tested delays. | [Read study](s5-firmware-predictor.md) |
| G2 · Paired mission Monte Carlo | The S5 controller stopped durably in 79 of 200 main trials, with a clear advantage on a matched subset. | [Read study](g2-monte-carlo.md) |
| S6 · Estimated feedback in the balance loop | Both simulated estimators balanced nominally, and both missed all nine cases under the stronger gyro drift. | [Read study](s6-estimator-in-loop.md) |
| S7 · Wheel-loss feedforward | Wheel-loss compensation and controller retuning cut speed error and eliminated falls in the shared simulated cohort. | [Read study](s7-wheel-loss-comp.md) |
| S8 · Durable rest and world-referenced rolling loss | Stop-position re-anchoring recovered 15 shared trials, and a wheel–ground model removed a spurious unstable region. | [Read study](s8-rest-and-rolling.md) |
| S9 · Learned balance point and smoother braking | All 260 paired S9 trials met the original rest criteria, and 197 of 200 shared trials reduced peak braking speed. | [Read study](s9-balance-point.md) |
| S10 · Adaptive gyro-bias estimation | Adaptive pitch gyro-bias correction recovered the injected drift cases, while acceleration bias still shifted pitch. | [Read study](s10-gyro-bias.md) |
| R5 · Corrected wheel–ground contact reruns | D1 passes rose from four to eight, both V1 trials passed, and both G1 arms still missed. | [Read study](r5-contact-rerun.md) |
| G3 · Full-stack Monte Carlo | Combining the revised controller, estimator and rolling model exposed a packet-age failure mode. | [Read study](g3-full-stack.md) |
| K1d · PPO policy against LQR | A PPO policy recovers from ±40° tilts with 13–33% less motor work than LQR, but drifts 10–37 cm. | [Read study](k1d-ppo.md) |
| S11 · Encoder packet tolerance and gyro-drift rejection | Bridging up to two missed encoder packets and revising the bias loop raised matched passes from 104/200 to 200/200 and 23/50 to 50/50. | [Read study](s11-packet-tolerance.md) |
| S12 · CAN, Wi-Fi and e-stop in the control loop | Three CAN buses reached 239/250 passes vs 231/250 on one bus; a 150 ms Wi-Fi timeout gave 0/250 normal nuisance stops and 25/25 durable rests. | [Read study](s12-comms-in-loop.md) |
| S13 · Battery sag, regenerative overvoltage and motor-power stop | An 18 V jump sagged logic input to 15.73 V, an open 25.2 V link crossed 32 V in 0.81 ms at 1 mF, and a 1.5 Ω chopper held the worst peak to 27.79 V. | [Read study](s13-power-safety.md) |
| S14 · Parameter sensitivity and radio-loss policy | Nine of 22 swept parameters crossed the 95% pass gate; radio-loss policy c gave 0/34 falls vs 34/34 for torque-off policies and is now the firmware default. | [Read study](s14-sensitivity-radio.md) |


| Added source evidence | Verification scope | Link |
| --- | --- | --- |
| AUX and CTRL revisions | KiCad rules and public schematic exports | [Hardware revision](hardware-control-board-revision.md) |
| S11 independent review | Six findings and supplied Debug/Release receipts | [Packet policy fixes](s11-packet-policy-fix.md) |
| CAN DBC | C-vector and multi-format offline parity | [CAN evidence](can-dbc.md) |
| Portable firmware | Host tests and supplied FIL verification | [Firmware](../../firmware/README.md) |
| Pi link | Python/C wire and transport tests | [Pi link](../../pi/link/README.md) |
