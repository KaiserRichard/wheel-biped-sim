# Replay-04 sparse map export report

## Reproduction

```bash
MPLCONFIGDIR=/private/tmp/map-export-mpl python3 \
  tools/map_export/export_map.py \
  <local workspace>/wbr-evidence-archive/orb-map-points-20261021-01/replay-04/map-bag \
  --output <local workspace>/Desktop/Research/wbr-data/derived/map-export-20261023/replay-04
```

The input bag was read in place. Artefacts are in `<local workspace>/Desktop/Research/wbr-data/derived/map-export-20261023/replay-04/`.

## Recorded result

Replay-04 contains 20 cloud snapshots, 294 poses, and 294 status messages. The newest replacement snapshot is sequence 20 with 965 finite points, map ID 0, `tracked=true`, and `truncated=false`. At the default 2 cm voxel width, the PLY contains 695 points. Its axis-aligned extents are x=3.293 m, y=2.369 m, z=2.782 m; bounds are x=[-1.149, 2.144], y=[-1.236, 1.133], z=[0.225, 3.006] m. Bounding-box volume is 21.701 m³ and exported-point density is 32.03 points/m³. The trajectory has 294 poses. All 294 recorded status JSON messages report `tracking_state=2`, which is `OK` in ORB_SLAM3 `include/Tracking.h` (`NOT_INITIALIZED=1, OK=2`). Supervisor correction 2026-10-23: the first version used the off-by-one table from `tools/perception_health/SPEC.md` and wrongly reported `NOT_INITIALIZED`; `summary.json` now gives `status_counts: {OK: 294}`.

## Limitations

- These are sparse ORB map landmarks, not a dense surface or a sensor depth reconstruction. Voxel filtering changes the exported count and keeps one observed point per occupied voxel.
- There is no metric ground truth or surveyed alignment in this export. Stereo scale is set by the configured camera baseline; these figures do not validate that scale.
- `slam_world` follows the initial optical axes (x right, y down, z forward), not gravity or robot-body axes. Pose and cloud messages can reflect asynchronous local optimization; matching headers do not make them an atomic state.
- The exporter selects one latest snapshot. This is intentional because messages are replacements and map resets/merges can change map identity. Separate recordings must remain separate.
- Bounding-box density is only a compact descriptor of the sparse point set; it does not measure completeness, accuracy, drift, or navigability.
