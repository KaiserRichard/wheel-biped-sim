# MATLAB and Simulink validation

The derivation, LQR schedule, linear cross-check, and export functions are provided with their generated symbolic functions and parameter inputs. `hip_mode_model.m` is the sampled differential-leg model. `margins.m` reports margins for an explicitly supplied linear loop; it omits actuator and sensor delay.

`comparison_results.csv` is a saved comparison output from a MATLAB R2026b analysis. `../docs/simscape_balance.csv` summarizes the prior balance failure from the archived acceptance output. These are prior results, not runs made during this refactor. The MATLAB and Simulink code was not run here. The exact Simscape model is not redistributed because its source license and source packaging have not been established.

The original Simscape assembly builder requires CAD mesh assets whose redistribution license could not be established. It is omitted; this tree does not claim to include a working Simscape multibody build.
