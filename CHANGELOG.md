# Changelog

## 26 November 2026 — batch 9

- Firmware updated to FW11: bench bring-up images (BENCH / RUN / MOTOR_ID) with IMU sensor codec and USB enumeration, the standing-push window and grip-aware braking (73 host tests). Study pages for the [push window](docs/studies/push-window.md) and [grip-aware braking](docs/studies/grip-aware-braking.md), one figure each. Simulation and host tests only.

## 26 November 2026 — batch 8

- Firmware updated to FW9: heading/path tracking, lateral-push replay test, degraded-stop trim, three safety fixes, slip-gated estimator and traction governor (52 host tests). Study pages for [figure-8 tracking](docs/studies/figure-8-path-tracking.md), [lateral push](docs/studies/lateral-push-low-friction.md), [safety fixes](docs/studies/firmware-safety-fixes.md) and [wheel slip](docs/studies/slip-estimator-and-traction-governor.md) (2 of 16 slip cases still fall); two simulation clips. Simulation and host tests only.

## 25 November 2026 — batch 7

- [FROM FILE] Integrate FW6FIX curb-only run-up and single frozen-encoder controlled-stop handling, the 20 s stop alarm, dedicated assist-abort reason and vendored contract fixtures. Supplied FIL results remain simulation evidence.
- [COMPUTED] Correct repository-relative imports and demo fixture lookup in the log regression entry point.
- [FROM FILE] Add the V3 sim-to-real checklist and CSV with explicit unresolved source provenance.
- [FROM FILE] Update the motor power board schematic/library design study to rev 2 fix 4, with current ERC and open pre-fabrication gates; retain inherited PCB, BOM, PDF, DRC and renders with their limits documented.

## 23 November 2026

- Add simulation evidence pages, figures and summary CSVs for [S11](docs/evidence/s11-packet-tolerance.md), [S12](docs/evidence/s12-comms-in-loop.md), [S13](docs/evidence/s13-power-safety.md) and [S14](docs/evidence/s14-sensitivity-radio.md), plus a six-case S11/S12 replay video ([s11-s12-comms.mp4](media/s11-s12-comms.mp4)).
- S11/S12: document the graded encoder packet-loss policy and revised gyro-bias estimator (200/200 shared and 50/50 extended passes), one- versus three-bus CAN timing and closed-loop comparison (239/250 vs 231/250), 150 ms Wi-Fi watchdog (0/250 normal nuisance stops), and wired software e-stop latency (0.451–0.534 ms on three buses).
- S13: document 6S battery sag, isolated regenerative overvoltage and a 1.5 Ω / 26.5 V brake chopper screen (27.79 V worst corner peak).
- S14: rank 22 one-factor sensitivity parameters (9 crossed the 95% pass gate) and adopt policy c as the firmware default on radio loss (firmware not yet published here) (brake, then keep balancing in place: 0/34 falls vs 34/34 for torque-off policies; over-tilt, the wired e-stop and drive faults still cut torque at once).
- Hardware: add the design-only, not fabricated KiCad 9 motor power board (series contactors, 3.0 Ω precharge, 1.5 Ω brake chopper, dual-channel stop latch; 0 ERC errors/warnings and 0 DRC violations) to [hardware/README.md](hardware/README.md).

## 22 November 2026

- README rewritten as a showcase: MuJoCo, Simscape and KiCad 9 sections with evidence links; the full results table moved to docs/RESULTS.md; renders of the two learning boards added.
- Add the S5 delay-augmented gain design and C template with caller-supplied tables.
- Add the S10 adaptive estimator, S7 wheel-loss feedforward, and S8 stop-position re-anchor with host-compiled C checks.
- Add world-referenced rolling resistance with a primitive coast-down check and a seeded paired-trial scoring rehearsal.
- Document the [methods and simulation scope](docs/methods/batch-2.md).
- README: add the Simscape replay video of the M14 mission (MP4 plus GIF preview), replacing the single replay frame.
- README: show the MATLAB studio Simscape model (replay frame, model structure, M14 plots) and mark the earlier failed Simscape balance as superseded.
- Add the MATLAB studio M16 result: two clean runs of the accepted M14 mission in separate MATLAB processes are bit-identical and all 14 first-release gates pass. Adds a summary page and a gate CSV. Simulation only; studio source not yet released.

## 20 November 2026

- Add the MATLAB studio M14 result: a 16 s mission on 11 friction × actuator-delay fixtures passes all 777 gates with the delay-aware balance controller (R4). Adds a summary page, a per-fixture CSV and four figures with pinned hashes. Simulation only; studio source not yet released.

## Unreleased — 17 November 2026

- Harden the publication gate and add positive/negative regression cases for every audit probe, plus filename, metadata, token, receipt, cache and symlink checks.
- Use generic IMU sensor wording in simulation documentation.
- Add the P3 standard-library log importer and seeded synthetic identification rehearsal with relative provenance and regenerated receipts. No real-robot measurements or hardware acceptance are claimed; P2 and CAD-derived inputs remain deferred.
- Keep documented and CI test commands free of generated bytecode and pytest caches.
- Correct the gallery inventory and document the scan's limits. Existing upstream credit, MIT notices and AI-use disclosure are retained.


## Public hardware and firmware source batch

[FROM FILE / COMPUTED] Added AUX rev5 and CTRL rev4 sources/PDFs/checks, portable firmware through slice 7 with FIL fixtures, independently reviewed S11 policy evidence, CAN databases/tools and the corrected Pi USB package. Source receipts, privacy checks and local execution scope are recorded in CHECKS.md.
