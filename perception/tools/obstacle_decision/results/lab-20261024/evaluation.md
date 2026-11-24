# RGB-D bag evaluation

Advisory thresholds: jump height 0.030–0.120 m; near range 1.50 m.
Relative bag paths are resolved from the run-sheet directory. All reported statistics use processed frames.

## Box runs

PASS requires height error ≤ 0.015 m and distance error ≤ 0.05 m when measured distance is ≤ 1.5 m; farther runs have no PASS/FAIL.

| run_id | height measured/estimated (m) | height error (m) | distance measured/estimated (m) | distance error (m) | width measured/estimated (m) | width error (m) | advisory expected/estimated | PASS |
|---|---:|---:|---:|---:|---:|---:|---|---|
| B-H02 | 0.025 / 0.025 | 0.001 | 0.975 / 0.978 | 0.003 | 0.237 / 0.237 | 0.000 | UNCERTAIN / UNCERTAIN | PASS |
| B-H05 | 0.046 / 0.044 | -0.002 | 1.000 / 0.987 | -0.013 | 0.248 / 0.252 | 0.004 | JUMP_CANDIDATE / JUMP_CANDIDATE | PASS |
| B-H08 | 0.085 / 0.086 | 0.001 | 1.000 / 0.989 | -0.011 | 0.237 / 0.250 | 0.013 | JUMP_CANDIDATE / JUMP_CANDIDATE | PASS |
| B-H08-rep | 0.085 / 0.086 | 0.001 | 1.000 / 0.988 | -0.012 | 0.237 / 0.250 | 0.013 | JUMP_CANDIDATE / JUMP_CANDIDATE | PASS |
| B-H26 | 0.264 / 0.255 | -0.009 | 1.000 / 0.984 | -0.016 | 0.222 / 0.216 | -0.006 | STOP_CANDIDATE / STOP_CANDIDATE | PASS |
| B-H13 | 0.133 / 0.131 | -0.002 | 1.000 / 0.989 | -0.011 | 0.222 / 0.238 | 0.017 | STOP_CANDIDATE / STOP_CANDIDATE | PASS |
| B-H05-D0p5 | 0.046 / 0.050 | 0.004 | 0.500 / 0.492 | -0.008 | 0.248 / 0.235 | -0.013 | JUMP_CANDIDATE / JUMP_CANDIDATE | PASS |
| B-H05-D1p3 | 0.046 / 0.570 | 0.524 | 1.300 / 1.321 | 0.021 | 0.248 / 0.494 | 0.246 | JUMP_CANDIDATE / STOP_CANDIDATE | FAIL |

## Empty-floor runs

PASS requires plane height std ≤ 5 mm and plane pitch std ≤ 0.3°.

| run_id | false-detection rate | camera-height error (m) | plane height std (m) | plane pitch std (°) | expected/estimated advisory | PASS |
|---|---:|---:|---:|---:|---|---|
| S1 | 1.000 | 0.004 | 0.002 | 0.155 | NONE / STOP_CANDIDATE | PASS |

## Wall-target runs

Wall forward = horizontal ground-frame distance (mode of corridor points ≥ 10 cm above the plane). The central-ROI axial depth is shown for reference only. PASS requires absolute relative error ≤ 2%.

| run_id | tape distance (m) | wall forward (m) | error (m) | relative error | central ROI axial (m) | PASS |
|---|---:|---:|---:|---:|---:|---|
| W1 | 1.000 | 1.006 | 0.006 | 0.006 | 1.023 | PASS |
| W2 | 1.500 | 1.424 | -0.076 | -0.050 | 1.437 | FAIL |
