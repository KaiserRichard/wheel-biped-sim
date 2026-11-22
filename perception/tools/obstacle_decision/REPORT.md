# Obstacle decision evaluation report

## Run and actual output

Command:

```sh
python3 -m unittest discover -s tools/obstacle_decision/tests -v
```

Output: **5 tests passed** (box geometry at 0°/3°, tall-box STOP, empty/outside
NONE, one-frame blip rejection, codec round trip).

Command:

```sh
MPLCONFIGDIR=/private/tmp/wbr-mpl-cache MPLBACKEND=Agg python3 tools/obstacle_decision/evaluate.py
```

Actual successful run: 80 box cases completed; mean measured host processing time was
63.90 ms/frame after the supervisor rerun (59.04 ms in the runner run; including plane estimation and Python clustering). This is not
a Pi performance result.

The initial evaluator invocation wrote `metrics.json` but no plots; rerunning
with a writable Matplotlib cache and no GUI backend first exited 134 while
building the font cache. With `MPLBACKEND=Agg` and `MPLCONFIGDIR=/private/tmp/wbr-mpl-cache`,
the exact evaluation completed and wrote both plots. This environment-specific
failure is retained here; all requested results are from the successful run.

| Metric | Count | Mean absolute error | Maximum absolute error |
|---|---:|---:|---:|
| Height | 80 | 0.00150 m | 0.00555 m |
| Front-face distance | 80 | 0.01275 m | 0.03360 m |
| Lateral width | 80 | 0.00169 m | 0.00452 m |

### Advisory confusion matrix

Rows are expected labels, columns are observed labels. The 0.12 m maximum jump
height is included in expected JUMP_CANDIDATE; noise caused 14 samples at that
boundary to estimate slightly above the configured maximum and conservatively
produce STOP_CANDIDATE.

| Expected \ Observed | NONE | JUMP_CANDIDATE | STOP_CANDIDATE | UNCERTAIN |
|---|---:|---:|---:|---:|
| JUMP_CANDIDATE | 0 | 50 | 14 | 0 |
| STOP_CANDIDATE | 0 | 0 | 16 | 0 |

Controls: empty floor → NONE, no detection; box outside corridor → NONE, no
detection.

Plots: [height error](https://github.com/KaiserRichard/wbr-perception-phase1-runtime/blob/fde5721af0627eaf60c78d95296ab26924425e37/tools/obstacle_decision/results/height_error.png),
[advisory confusion](https://github.com/KaiserRichard/wbr-perception-phase1-runtime/blob/fde5721af0627eaf60c78d95296ab26924425e37/tools/obstacle_decision/results/confusion.png).

## Supervisor correction (2026-10-23)

The front-face distance was the minimum forward coordinate of the cluster, so
depth noise biased it short as range grew (mean error -0.058 m, max 0.082 m at
2.0 m). It is now the 5th percentile (`detect.py`). Rerun: mean error by range
+0.004 / -0.002 / -0.012 / -0.028 m at 0.5 / 1.0 / 1.5 / 2.0 m, max 0.034 m.
Height errors and the confusion matrix are unchanged. The 14 STOP results for
0.12 m boxes sit exactly on the configured 0.12 m limit: with a p95 height
estimate that reads about +1.5 mm high, that boundary is a coin-flip by design,
not a detection failure. The residual short bias at 2 m is conservative (the
obstacle reads closer).

## Limitations

Synthetic renderer only, ASSUMED 0.30 m camera height and decision thresholds,
two seeds per geometry, and no real-box validation, ROS integration, Pi timing,
or transport. The estimated box height is a plane-relative point-cloud p95,
not a certified physical dimension. The advisory is PROPOSED and informational
only; firmware owner review is required. STM32 owns all balance, jump, and
safety decisions. This does not provide jump permission.
