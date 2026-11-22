# Local elevation map: synthetic evaluation

## Run

Command:

```sh
MPLBACKEND=Agg MPLCONFIGDIR=/private/tmp/wbr-mpl-cache \
  python3 tools/local_map/evaluate.py
```

The completed run used 16 rendered frames, 1.5 m forward travel, 0.30 m camera
height, ±3° pitch, stride 4 and 4 m / 5 cm cells. The first-frame floor fit was
valid with 2,299 inliers, estimated camera height 0.30004 m and pitch 3.0033°.
Mean points processed were 2,170.3 per frame. Host runtime was 297.29 ms total,
or 18.58 ms/frame; this is a single Mac-side synthetic run, not Pi timing.

## Results

| Pose/map case | Height RMSE (m) | Obstacle precision | Obstacle recall | TP / FP / FN |
|---|---:|---:|---:|---:|
| Gravity-aligned, no injected drift | 0.00425 | 1.000 | 0.964 | 27 / 0 / 1 |
| 1 cm accumulated translation drift | 0.00391 | 1.000 | 1.000 | 27 / 0 / 0 |
| 3 cm accumulated translation drift | 0.00468 | 0.514 | 0.900 | 18 / 17 / 2 |
| 1° accumulated rotation drift | 0.00530 | 0.735 | 0.893 | 25 / 9 / 3 |
| Nominal optical axes, no gravity alignment | 0.11559 | 0.011 | 1.000 | 27 / 2,385 / 0 |

The small 1 cm case scoring slightly better than zero drift reflects cell
quantization and sparse synthetic sampling; it is not evidence that pose error
improves the map. Rotation drift was applied about the camera's local y axis.
The unaligned case treats the initial optical frame as nominally level and
leaves the actual initial 3° pitch in the map. It consequently tilts the floor
relative to the height axis, producing widespread floor false positives.

The three generated artifacts are `results/height_map.png`,
`results/error_map.png`, and `results/obstacle_classes.png`; numeric outputs are
in `results/metrics.json`.

## Test run

Command:

```sh
MPLBACKEND=Agg MPLCONFIGDIR=/private/tmp/wbr-mpl-cache \
  python3 \
  -m unittest discover -s tools/local_map/tests -v
```

Result: 4 tests passed. They cover flat floor height, a noiseless 5 cm box
sample, preservation through scrolling, and reduced variance after repeated
observations.

## Limitations

Synthetic renderer and supplied camera trajectories only; no robot depth/pose
sequence was processed. ORB-SLAM3 pose quality on our data has not been
validated for this map. The grid does not model ray-based free-space clearing,
pose covariance, rolling shutter, dynamic-object rejection, or non-planar
terrain. Timing is machine- and run-dependent and says nothing about Pi
real-time performance.
