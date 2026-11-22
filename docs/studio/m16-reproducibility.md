# MATLAB studio, milestone M16: reproducible first release

All results are simulation results (MATLAB/Simscape Multibody, R2026b). No motors have been powered. Hardware applicability is unknown. The robot mass is the CAD estimate of 3.62 kg.

## What M16 tests

M16 asks two things: can the accepted result be reproduced exactly, and is everything it depends on still valid? The accepted M14 nominal mission is run **twice, in two separate clean MATLAB processes, one after the other**. The mission lasts 16 s: stand up, drive, brake, change leg length, then a disturbance. Both runs start from a fresh build with the same fixed random seed, and neither may load an earlier trajectory. Then:

- each run must pass its own 74-gate mission check;
- the two runs are compared sample by sample on a 1 ms grid over 16 s (16,001 samples) for pitch, centre-of-mass position, both leg lengths and bus voltage. Every native simulation sample is compared too;
- the source and protected files are hashed before and after each run (about 23,000 files) and must not change;
- the receipts of all earlier milestones in the first-release profile must still be valid.

## Result

**14 / 14 gates pass** ([gate table](../../results/m16_summary.csv)).

| Check | Observed | Limit |
| --- | --- | --- |
| Each run: mission gates | 74/74 and 74/74, no fall, both to 16.0 s | all pass |
| Pitch difference between runs | 0 rad | 1e-6 rad |
| Centre-of-mass difference | 0 m | 1e-5 m |
| Leg-length difference | 0 m | 1e-5 m |
| Bus-voltage difference | 0 V | 1e-4 V |
| Native events (contact, falls, protection) | identical | identical |
| Source/protected files changed during the runs | 0 | 0 |
| Earlier milestone receipts in the profile (M00–M10, M13, M14, M15 batch replay) | all PASS, hashes unchanged | all PASS |

The two runs are **bit-identical**. All 40 logged columns match exactly, and so do the 8.5 million native samples in each run. Run 1 is also bit-identical to the accepted M14 nominal run, which confirms that the controller, parameters and seed did not change.

## Problems on the way

Three earlier attempts failed for computing reasons, not because of the robot model:

- running both processes in parallel ran out of memory;
- a crash of the controlling MATLAB session stopped the second attempt;
- the system drive filled up, because MATLAB's Simulation Data Inspector kept about 20 GB of temporary files per process.

The fix was to run the processes one after the other, give each its own temporary folder on a large drive, and check free space before each launch. No model, controller, parameter, seed, solver, scoring rule or threshold was changed. The failed attempts are kept in the private record.

Each run took about 92 minutes of wall time, against a 10-minute target. That target is advisory only.

## Limits

- Simulation only, at the CAD mass of 3.62 kg. The realistic whole-robot mass of about 5.1 kg is studied separately.
- The M15 interactive replay checks (live and detailed) are still open.
- The MATLAB studio source is not in this repository yet. It will be released in later batches after review.
