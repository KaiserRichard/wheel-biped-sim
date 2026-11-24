# Results and limits

All results here are simulation results. No motors have been powered. Geometry, inertias, mass distribution, friction, motor limits, sensor noise, and delays are assumptions unless a result says otherwise. The animations in the README are from simulation runs, not hardware recordings.

| Result | Evidence | Status |
| --- | --- | --- |
| Stand behavior | Pitch RMS `0.000034109°`; drift `0.000061911 m`; no fall | MEASURED by `scripts/stand.py`, saved in `results/stand.json` |
| Stand with a prescribed push | Saved reference trace reproduced with maximum absolute difference `0` across 5,000 samples and 21 columns | MEASURED by `check.py`, saved in `results/parity.json`; reference is `tests/traces/stand_integration.csv` |
| Obstacle run | Integration reference reproduced with maximum absolute difference `0` across 4,000 samples and 21 columns | MEASURED by `check.py`, saved in `results/parity.json`; reference is `tests/traces/obstacle_integration.csv` |
| CAD stand comparison | Largest per-column maximum absolute difference `1.785469515` in `Tp` torque (N·m) | MEASURED by `check.py`, saved in `results/parity.json`; the plant properties differ. |
| CAD obstacle comparison | Largest per-column maximum absolute difference `19.948454409` in rear hip torque `hrR` (N·m) | MEASURED by `check.py`, saved in `results/parity.json`; plant and controller setup differ. Exact cross-model parity is not established. |
| Obstacle with delay | Five of five trials passed at zero added delay; zero of five passed at assumed 2, 4, 6, and 8 ms delays | MEASURED by `scripts/recount_delay.py`, saved in [delay_summary.csv](delay_summary.csv). |
| Simscape balance, first CAD model (superseded) | Peak pitch `89.8903707414°`; the balance and firmware-equivalence criteria failed | MEASURED by the archived `check.py` output, summarized in [simscape_balance.csv](simscape_balance.csv). The Simscape run is not part of this CPU test suite. Superseded by the MATLAB studio model, which balances through the whole M14 mission (next row). |
| MATLAB studio M14: 16 s mission, friction 0.5–1.0 × added delay 0/2/5 ms | 777/777 gates; 11 of 11 fixtures to 16 s, no fall, peak pitch 2.98–3.13°; delay-aware controller, extra-delay margin 12 ms (was 1.4 ms) | MEASURED in MATLAB/Simscape, summarized in [docs/studio/m14-delay-robustness.md](studio/m14-delay-robustness.md) and `results/m14_summary.csv`. Studio source not yet released. |
| MATLAB studio M16: reproducibility of the first release | 14/14 gates; two clean runs of the M14 mission in separate processes are bit-identical (pitch, CoM, leg lengths, bus voltage, every native sample); all earlier milestone receipts still valid | MEASURED in MATLAB/Simscape, summarized in [docs/studio/m16-reproducibility.md](studio/m16-reproducibility.md) and `results/m16_summary.csv`. Studio source not yet released. |
| Motors and real robot | No powered-motor or physical robot result | UNKNOWN |

The exact reference traces are checked against the integration build at seed zero. The CAD stand and obstacle traces are also compared at seed zero; the largest per-column differences are reported by the parity script. These mismatches are not treated as parity. Robot physical masses remain ASSUMED. The simple model generator uses primitive geometry because the source of the CAD mesh export does not establish a redistribution license.
