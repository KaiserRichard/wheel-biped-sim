# K1d: a PPO policy recovering the CAD robot from large tilts

Simulation only. MuJoCo, 5.1 kg CAD model, wheel–floor friction 0.7, 3 s per run. The PPO policy was trained in MATLAB (Reinforcement Learning Toolbox) on a two-angle model of the robot (chassis pitch and virtual-leg tilt), started from imitation of an LQR, and exported to C. The C actor matches MATLAB on 10,000 inputs (largest normalised action error about 2×10⁻⁶).

Video: [ppo-vs-lqr.mp4](../../media/ppo-vs-lqr.mp4) (51 s, slowed down 2×: the three robots on top, live pitch, wheel-axle position and wheel-speed plots below, and a results card after each scenario).

## Results

"Back in balance" is the time to enter the full balance gate. Motor work is the absolute mechanical work of all joints. Drift is the wheel-axle position at 3 s.

| Start tilt | Controller | Back in balance (s) | Motor work (J) | Drift (cm) | Fell |
|---|---|---:|---:|---:|---|
| +40° | R4 reference | 1.01 | 8.8 | 0.9 | no |
| +40° | LQR | 0.69 | 10.6 | 3.2 | no |
| +40° | PPO | 0.72 | 7.1 | 36.9 | no |
| −40° | R4 reference | 1.19 | 9.0 | −0.5 | no |
| −40° | LQR | 0.77 | 9.7 | −3.1 | no |
| −40° | PPO | 0.75 | 8.5 | 10.2 | no |
| +25° | LQR | 0.63 | 3.1 | 1.2 | no |
| +25° | PPO | 0.66 | 2.3 | 25.8 | no |

Across the full K1d grid, each PPO seed recovered 24/24 nominal starts (±10° to ±40°) and 36/36 sensitivity cases (lower bus voltage, hip torque limit, friction 0.3 and 1.2, soft contact, one weaker wheel). Starting PPO from random weights instead of LQR imitation never learned to balance (0/54 in the reduced-model check).

## What this shows

- PPO works on the full robot model and uses 13–33% less motor work than LQR.
- Its weakness is position drift (10–37 cm), which LQR and R4 avoid. The policy is also not symmetric: drift depends on the tilt direction.
- These runs have no sensor noise, delays, packet loss or real motors. The robot's main controller stays LQR; PPO is an optional helper (position-hold reward, or a bounded correction on top of LQR) to be tested in shadow mode before it drives any motor.
