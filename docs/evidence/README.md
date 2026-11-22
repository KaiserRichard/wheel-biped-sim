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
| R5 · Corrected wheel–ground contact reruns | The corrected rolling law recovered eight D1 stops and both V1 stops, while the G1 contact cohort still missed. | [Read study](r5-contact-rerun.md) |
