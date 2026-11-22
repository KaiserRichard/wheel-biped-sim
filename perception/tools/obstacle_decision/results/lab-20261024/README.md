# 2026-10-24 tabletop runs, re-evaluated

`python3 tools/obstacle_decision/eval_runs.py run_sheet_used.csv --out OUT` (stride 1, all frames;
bags in `~/wbr-evidence-archive/mobile-demo-20261024-01/`; superseded/invalid rows dropped).

Detector changes vs the lab-day numbers (MEASURED on these bags, median over frames):

| | before | after |
|---|---|---|
| height error (6 box runs ≤ 13 cm) | +0.3 to +0.85 cm | −0.2 to +0.4 cm |
| distance error | +1.3 to +4.3 cm | −1.6 to +0.3 cm |
| width error | +1.2 to +4.7 cm | −1.3 to +1.7 cm |
| 2.45 cm book advisory | JUMP_CANDIDATE (wrong, read 3.3 cm) | UNCERTAIN (correct, reads 2.6 cm) |
| W1 wall 1.00 m | 1.023 axial (FAIL) | 1.006 horizontal (PASS) |

- Distance: forward was optical Z; on a camera pitched 10° that adds (H−h)·sin(pitch) ≈ +3 cm.
  Now ground-frame horizontal distance from the fitted plane.
- Height: p95 of all points picked up the top-face noise tail. Now the median of the upper
  band of points whose local normal is within 25° of vertical (the top face); p95 only when
  the top face is not visible (B-H26: −0.9 cm, still STOP).
- Width: ptp included flying pixels at side edges. Now pixels > 3 % of range behind a
  left/right neighbour are dropped, and width is p0.5–p99.5.
- Percentiles and thresholds were chosen looking at all these runs (no held-out run).
  Synthetic sweep (`evaluate.py`, 80 cases) still within: height max 1.1 cm, distance
  max 4.4 cm, width max 1.8 cm.

Still failing / open:
- B-H05-D1p3: books touch the lamp base, one cluster → STOP. Nearest face is now right
  (1.321 m vs 1.30); STOP is the safe answer for a touching pole.
- S1: the lamp pole at 1.42 m is a real obstacle, so "false detection" 1.0 is expected.
- W2: 1.424 m vs 1.50 m tape (−5 %), same as the lab day. W1 on the same method is +0.6 cm,
  so this still points at the W2 setup, not the depth.

Speed (MEASURED, detector only, B-H08 frames, `process_frame` p50/p95): Mac ~27 ms;
Pi 5 (not throttled, 50–54 °C) 118/126 ms before → 67/76 ms after vectorising RANSAC
scoring (4000-point subset) and labelling clusters on the cell grid. Results above are
from the faster code.
