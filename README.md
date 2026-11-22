# Wheel-biped simulation

A CPU MuJoCo study of a wheel-biped robot: two driven wheels support a body, and symmetric five-bar legs change body height and absorb impacts.

![CAD robot at an assumed 8.5 kg clearing an 8 cm obstacle in MuJoCo](media/preview-obstacle.gif)

*Simulation only: the CAD robot at an assumed 8.5 kg clearing an 8 cm obstacle in MuJoCo.*

This project builds on Trần Nguyên Bình's [DM-jump](https://github.com/nguyenbinh-shark/DM-jump) (MIT) and [dmBots/wheel-legged](https://github.com/dmBots/wheel-legged) (MIT). It is a personal refactor: the concept and scenarios come from that work, and the code here was rewritten and reorganized. CAD meshes are not included. AI tools were used heavily in building and refactoring it. See [NOTICE.md](NOTICE.md).

## Quick look

Everything here is simulation. Previews are shortened; full clips are listed under Videos.

### CAD robot in MuJoCo

The CAD model turning on the spot. This is a MuJoCo render of meshes exported from SolidWorks, not a SolidWorks screen recording.

<img src="media/preview/cad-model-turntable.gif" width="480" alt="CAD model turning on the spot">

| | | |
| --- | --- | --- |
| <img src="media/preview/cad-stand.gif" width="260" alt="Stand with a push"><br>Stand with a push | <img src="media/preview/cad-drive.gif" width="260" alt="Drive"><br>Drive | <img src="media/preview/cad-turn.gif" width="260" alt="Turn"><br>Turn |
| <img src="media/preview/cad-leg-height.gif" width="260" alt="Leg height"><br>Leg height | <img src="media/preview/cad-jump.gif" width="260" alt="Jump"><br>Jump | <img src="media/preview/cad-obstacle.gif" width="260" alt="8 cm obstacle"><br>8 cm obstacle |
| <img src="media/preview/mass-8.5kg-one-sided-ramp.gif" width="260" alt="8.5 kg, one-sided ramp"><br>8.5 kg, one-sided ramp | <img src="media/preview/mass-8.5kg-drop.gif" width="260" alt="8.5 kg, 0.245 m drop"><br>8.5 kg, 0.245 m drop | <img src="media/preview/cad-obstacle-2ms-falls.gif" width="260" alt="2 ms delay: falls"><br>2 ms delay: falls |

### MATLAB and Simscape

The MATLAB studio rebuilds the CAD robot in Simscape Multibody: chassis, five-bar legs and wheels from the CAD (3.62 kg CAD mass estimate), wheel-ground contact with friction, six drives on a battery bus, IMU and encoder models, and a delay-aware LQR balance controller. Its 16 s mission (stand up, drive about 1.5 m, brake, raise and lower the legs, push near 13 s) passes all 777 gates on 11 fixtures of ground friction 0.5-1.0 and added actuator delay 0-5 ms (M14). Two clean runs in separate MATLAB processes reproduce it bit for bit (M16).

[<img src="media/preview/m14-simscape-replay.gif" width="800" alt="Simscape Multibody replay of the full 16 s M14 mission, with body pitch, position, bus voltage, pack current and joint torques.">](media/m14-simscape-replay.mp4)

*Simscape Multibody replay of the full 16 s M14 mission (stand up, drive, brake, leg height, push), with body pitch, body position, bus voltage, pack current and joint torques; the dashed line marks the current time. Full-resolution video: [m14-simscape-replay.mp4](media/m14-simscape-replay.mp4). Simulation only; masses are the CAD estimate.*

```mermaid
flowchart LR
  M[Mission profile] --> C[Delay-aware LQR<br>and leg control]
  C --> D[Added delay 0/2/5 ms<br>+ 5 ms drive lag]
  D --> P[Six drives<br>on a battery bus]
  P --> B[Simscape Multibody:<br>chassis, five-bar legs, wheels]
  B <--> G[Wheel-ground contact<br>friction 0.5-1.0]
  B --> S[IMU and encoders<br>with noise]
  S --> C
```

<img src="media/figures/m14-pitch-all-fixtures.png" width="640" alt="Body pitch on all 11 M14 fixtures: peak 2.98-3.13 degrees, no fall.">

*Body pitch on all 11 M14 fixtures: peak 2.98-3.13°, no fall. Details in [docs/studio/m14-delay-robustness.md](docs/studio/m14-delay-robustness.md).*

<img src="media/figures/m14-nominal-mission.png" width="640" alt="Nominal M14 mission signals.">

*Nominal M14 mission.*

#### Earlier MATLAB models

These came before the studio. The controller was derived and cross-checked on dmBots' Simscape model. Our first Simscape model of the CAD robot did not balance (peak pitch 89.9°, see Results and limits); the studio model above replaces it.

<img src="media/figures/gains.png" width="640" alt="LQR gains against leg length, from the MATLAB derivation. Design model.">

*LQR gains against leg length, from the MATLAB derivation. Design model.*

<img src="media/figures/poles.png" width="640" alt="Closed-loop poles across the gain schedule. Linear model.">

*Closed-loop poles across the gain schedule. Linear model.*

<img src="media/figures/margins.png" width="640" alt="Delay and disk margins against leg length, with frozen contact and no estimator. Linear model.">

*Delay and disk margins against leg length, with frozen contact and no estimator. Linear model.*

<img src="media/figures/hip_boundary.png" width="640" alt="MATLAB hip-mode model: first unstable delay against sync damping. Gaps mean no instability found in the range searched.">

*MATLAB hip-mode model: first unstable delay against sync damping. Gaps mean no instability found in the range searched.*

<img src="media/figures/simscape.png" width="640" alt="dmBots' Simscape model: pitch with their gains, with gains from the rebuilt equations (the curves overlap), and with no control.">

*dmBots' Simscape model: pitch with their gains, with gains from the rebuilt equations (the curves overlap), and with no control.*

<img src="media/figures/simulink-dmbots-model.png" width="640" alt="Top level of dmBots' Simulink model (MIT), used for the cross-check above.">

*Top level of dmBots' Simulink model (MIT), used for the cross-check above.*

## Results and limits

All results here are simulation results. No motors have been powered. Geometry, inertias, mass distribution, friction, motor limits, sensor noise, and delays are assumptions unless a result says otherwise. The animation above is from a simulation run, not a hardware recording.

| Result | Evidence | Status |
| --- | --- | --- |
| Stand behavior | Pitch RMS `0.000034109°`; drift `0.000061911 m`; no fall | MEASURED by `scripts/stand.py`, saved in `results/stand.json` |
| Stand with a prescribed push | Saved reference trace reproduced with maximum absolute difference `0` across 5,000 samples and 21 columns | MEASURED by `check.py`, saved in `results/parity.json`; reference is `tests/traces/stand_integration.csv` |
| Obstacle run | Integration reference reproduced with maximum absolute difference `0` across 4,000 samples and 21 columns | MEASURED by `check.py`, saved in `results/parity.json`; reference is `tests/traces/obstacle_integration.csv` |
| CAD stand comparison | Largest per-column maximum absolute difference `1.785469515` in `Tp` torque (N·m) | MEASURED by `check.py`, saved in `results/parity.json`; the plant properties differ. |
| CAD obstacle comparison | Largest per-column maximum absolute difference `19.948454409` in rear hip torque `hrR` (N·m) | MEASURED by `check.py`, saved in `results/parity.json`; plant and controller setup differ. Exact cross-model parity is not established. |
| Obstacle with delay | Five of five trials passed at zero added delay; zero of five passed at assumed 2, 4, 6, and 8 ms delays | MEASURED by `scripts/recount_delay.py`, saved in `docs/delay_summary.csv`. |
| Simscape balance, first CAD model (superseded) | Peak pitch `89.8903707414°`; the balance and firmware-equivalence criteria failed | MEASURED by the archived `check.py` output, summarized in `docs/simscape_balance.csv`. The Simscape run is not part of this CPU test suite. Superseded by the MATLAB studio model, which balances through the whole M14 mission (next row). |
| MATLAB studio M14: 16 s mission, friction 0.5–1.0 × added delay 0/2/5 ms | 777/777 gates; 11 of 11 fixtures to 16 s, no fall, peak pitch 2.98–3.13°; delay-aware controller, extra-delay margin 12 ms (was 1.4 ms) | MEASURED in MATLAB/Simscape, summarized in [docs/studio/m14-delay-robustness.md](docs/studio/m14-delay-robustness.md) and `results/m14_summary.csv`. Studio source not yet released. |
| MATLAB studio M16: reproducibility of the first release | 14/14 gates; two clean runs of the M14 mission in separate processes are bit-identical (pitch, CoM, leg lengths, bus voltage, every native sample); all earlier milestone receipts still valid | MEASURED in MATLAB/Simscape, summarized in [docs/studio/m16-reproducibility.md](docs/studio/m16-reproducibility.md) and `results/m16_summary.csv`. Studio source not yet released. |
| Motors and real robot | No powered-motor or physical robot result | UNKNOWN |

The exact reference traces are checked against the integration build at seed zero. The CAD stand and obstacle traces are also compared at seed zero; the largest per-column differences are reported by the parity script. These mismatches are not treated as parity. Robot physical masses remain ASSUMED. The simple model generator uses primitive geometry because the source of the CAD mesh export does not establish a redistribution license.

## Videos

Everything below is simulation, played back from saved runs. Nothing here is footage of a real robot.

### MATLAB studio, Simscape Multibody, 3.62 kg CAD mass

| Clip | What it shows |
| --- | --- |
| [m14-simscape-replay.mp4](media/m14-simscape-replay.mp4) | M14 mission, 16 s: stand up, drive about 1.5 m, brake, leg height, push |

### CAD robot, 3.72 kg model

| Clip | What it shows |
| --- | --- |
| [cad-stand.mp4](media/cad-stand.mp4) | Stand with a push |
| [cad-drive.mp4](media/cad-drive.mp4) | Drive |
| [cad-turn.mp4](media/cad-turn.mp4) | Turn |
| [cad-leg-height.mp4](media/cad-leg-height.mp4) | Leg height |
| [cad-jump.mp4](media/cad-jump.mp4) | Jump |
| [cad-obstacle.mp4](media/cad-obstacle.mp4) | 8 cm obstacle |

### Realistic mass, 8.5 kg model (assumed masses)

| Clip | What it shows |
| --- | --- |
| [mass-8.5kg-obstacle.mp4](media/mass-8.5kg-obstacle.mp4) | 8 cm obstacle |
| [mass-8.5kg-one-sided-ramp.mp4](media/mass-8.5kg-one-sided-ramp.mp4) | One-sided ramp |
| [mass-8.5kg-drop.mp4](media/mass-8.5kg-drop.mp4) | 0.245 m drop |

### Delay

| Clip | What it shows |
| --- | --- |
| [cad-obstacle-2ms-falls.mp4](media/cad-obstacle-2ms-falls.mp4) | 2 ms added delay: falls. Clip stops at the fall |
| [cad-obstacle-2ms-lower-damping.mp4](media/cad-obstacle-2ms-lower-damping.mp4) | One run (seed 5) at 2 ms with lower jump damping: passes. Not yet shown to be a general fix |
| [design-obstacle-6ms-before-after.mp4](media/design-obstacle-6ms-before-after.mp4) | Design model at 6 ms, before and after the fix |
| [design-obstacle-6ms-vs-8ms.mp4](media/design-obstacle-6ms-vs-8ms.mp4) | The same fix at 6 ms and 8 ms: 8 ms still falls |

### Harder scenarios

| Clip | What it shows |
| --- | --- |
| [scenario-one-sided-ramp.mp4](media/scenario-one-sided-ramp.mp4) | One-sided ramp |
| [scenario-ramp-and-drop.mp4](media/scenario-ramp-and-drop.mp4) | Ramp and drop |
| [scenario-opposite-wheel-heights.mp4](media/scenario-opposite-wheel-heights.mp4) | Opposite-phase wheel heights |
| [scenario-yaw-sine.mp4](media/scenario-yaw-sine.mp4) | Yaw sine tracking |

### Simple design model

| Clip | What it shows |
| --- | --- |
| [design-obstacle-rule-before-after.mp4](media/design-obstacle-rule-before-after.mp4) | Obstacle at 1 m/s. Left: before the distance rule, falls. Right: after it, passes |
| [design-jump.mp4](media/design-jump.mp4) | Jump |

## Quick start

Use Python with NumPy, SciPy, MuJoCo 3.3.7, and pytest. These commands were run on CPU:

```bash
export PYTHONDONTWRITEBYTECODE=1
python check.py
python -m pytest -q -p no:cacheprovider
python models/generate.py
python scripts/stand.py --seed 0
```

These commands were run with Python 3.11, MuJoCo 3.3.7, NumPy, SciPy, and pytest available on CPU. The first command reruns the publish scan and stand and obstacle parity. The scenario command writes CSV and JSON beneath `results/`; it does not create compressed array dumps. To install the package and test runner in a clean environment, use `python -m pip install -e . pytest`.

## Synthetic log import and identification rehearsal

[analyses/logsysid](analyses/logsysid/README.md) contains a standard-library log codec/importer and seeded identification rehearsal. Use Python 3.11+; no MATLAB or additional Python packages are needed for this analysis:

```bash
export PYTHONDONTWRITEBYTECODE=1
(cd analyses/logsysid && python -m unittest discover -s tests -v)
(cd analyses/logsysid && python verify_package.py)
```

The bundled CSV and all recovery results are synthetic/simulation only. They do not show real-robot data import, measured sensor performance, physical motor calibration or hardware acceptance. Hardware applicability is UNKNOWN; contact friction and battery resistance are UNASSESSED. P2 reduced-model screening and its CAD-derived inputs are deferred pending clearance.

## Project layout

- `wbr/dynamics/`: five-bar kinematics, equations, and model support.
- `wbr/control/`: one controller with configuration options, gain schedule, and jump state machine.
- `wbr/sim/`: MuJoCo scenarios and simulation loop.
- `wbr/analysis/`: saved-trace parity analysis and SVG plotter (`pip install -e '.[plots]'`).
- `models/`: assumed parameter set and primitive MJCF generator. Meshes are intentionally omitted pending license verification.
- `matlab/`: derivation, LQR, margins, hip-mode, and Simulink cross-check sources with CSV results. No MATLAB command was run for this refactor. The original Simscape builder depended on the unlicensed CAD mesh set and is not included.
- `perception/`: dataset-free script entry points and notes. No photographs or datasets are included.
- `slam/`: live RGB-D SLAM launch kit and setup notes; hardware execution is unverified.
- `scripts/`: one entry point for each behavior, using a shared result writer.
- `tests/`: CPU parity test and text CSV reference traces.
- `media/`: simulation clips, listed under Videos.
- `docs/`: design notes and CSV summaries of delay and Simscape failures.

Summary CSVs under `docs/` are labeled MEASURED, ASSUMED, or UNKNOWN. Follow the same convention when adding results.
