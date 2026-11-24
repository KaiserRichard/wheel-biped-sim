# Box-ahead obstacle estimate and jump advisory

Offline NumPy prototype for estimating box geometry from a depth frame and
filtering it into an informational STM32 advisory. **The Pi does not grant jump
permission.** Balance, jumping, actuation, and safety remain entirely owned by
the STM32. There is no transport, ROS node, camera access, or hardware action.

`detect.process_frame(depth_m, K, params, state)` fits the floor with the
existing `tools/ground_plane/plane.py`, constrained to a camera-facing normal
within 30°; it reuses the last plane for up to three frames when a fit fails.
Optical-frame `(x right, y down, z forward)` maps to ground coordinates
`(forward=z, lateral=−x, up=n·p+d)`. A 2 cm grid and connected components
produce front-face distance, 95th-percentile height, lateral width, depth
extent, and point count. ASSUMED defaults include 0.25 m corridor half-width,
0.02 m minimum obstacle height, and 0.3–3.0 m range.

`decide.update` applies a 3-of-5 temporal confirmation, EMA geometry, and a
two-frame clearing hysteresis. Jump height limits are ASSUMED at 0.03–0.12 m;
taller confirmed objects produce STOP_CANDIDATE. Invalid plane near a tracked
obstacle also produces STOP_CANDIDATE; otherwise plane failure is UNCERTAIN.
Optional forward speed yields distance/speed time-to-contact.

`advisory_codec.py` is a PROPOSED type-4 message using the existing COBS and
CRC-32C framing. See [ADVISORY_SPEC.md](ADVISORY_SPEC.md). Firmware owner review
is required before any firmware use.

## Run

From the repository root:

```sh
python3 -m unittest discover -s tools/obstacle_decision/tests -v
MPLCONFIGDIR=/private/tmp/wbr-mpl-cache MPLBACKEND=Agg python3 tools/obstacle_decision/evaluate.py
```

Evaluation writes `results/metrics.json`, `results/height_error.png`, and
`results/confusion.png`. The sweep uses 80 deterministic synthetic boxes:
heights 0.03/0.05/0.08/0.12/0.20 m, front distances 0.5/1.0/1.5/2.0 m,
camera pitch 0°/3°, disparity noise sigma 0.1 px and two seeds. Camera height
0.30 m is ASSUMED. Empty-floor and out-of-corridor controls are included.

## Limits

Synthetic renderer only; no real-box validation, no RealSense camera validation,
no ROS integration, no Pi timing or transport. Corridor and decision thresholds
are assumptions, not robot clearance/jump capabilities. Synthetic geometry
does not model reflective surfaces, multipath, motion distortion, camera
extrinsic error, floor irregularity, or real depth holes. Advisory names are
candidates only and must never be interpreted as permission to jump.

## Replay recorded RGB-D bags and evaluate mobile-demo runs

`replay_bag.py` uses `rosbags.highlevel.AnyReader` with the ROS 2 Jazzy typestore.
It accepts a rosbag2 directory containing `metadata.yaml` and `.db3` or `.mcap`
storage, or a single `.db3`/`.mcap` file. Topic auto-resolution prefers aligned
color-depth with color camera calibration, otherwise depth with depth camera
info. `16UC1` is converted from millimetres to metres; `32FC1` is treated as
metres. `--stride` processes every Nth depth frame. Output contains per-frame
CSV and summary JSON, including a configurable central square ROI depth (used
for wall runs). It treats all replayed frames as the evaluation interval; trim
bags to the intended stationary segment if they include setup or motion.

```sh
python3 tools/obstacle_decision/replay_bag.py capture/ --wall-roi-size 80 --out results/capture
python3 tools/obstacle_decision/replay_bag.py capture.db3 --depth-topic /camera/camera/depth/image_rect_raw --info-topic /camera/camera/depth/camera_info --stride 2 --out results/depth
```

`eval_runs.py` accepts the mobile-demo run sheet. Only `run_id` and `bag_path`
are mandatory; numeric cells can be blank and any additional input columns are
preserved in `evaluation.json`. A relative `bag_path` is resolved from the
run-sheet file's directory. The evaluator classifies rows as:

```csv
run_id,bag_path,capture_type,mode,box_height_m,box_front_distance_m,box_width_m,lateral_offset_m,camera_height_m,camera_tilt_deg,wall_distance_m,floor_type,lighting,usb_disconnects,notes
```

- A wall row has `wall_distance_m` set. It reports the median valid depth in a
  centered 80×80 px ROI per frame and the median across frames, plus tape error
  and relative error. `--wall-roi-size N` changes the square ROI width/height;
  PASS is absolute relative error ≤ 2%.
- An empty-floor row has empty `box_height_m` and `wall_distance_m`. It reports
  false-detection rate (frames containing a detector obstacle candidate divided
  by processed frames), camera-height error when a measured camera height is
  supplied, and plane-height/pitch standard deviations. Stability PASS limits
  are 5 mm and 0.3°.
- Other rows are box runs. PASS is defined only when measured box height and
  front distance are supplied and distance is ≤ 1.5 m; it requires absolute
  height error ≤ 0.015 m and distance error ≤ 0.05 m. Beyond 1.5 m the estimates
  remain reported without PASS/FAIL. Advisory correctness is reported when
  measured box height is supplied, using assumed 0.03–0.12 m jump thresholds.

Evaluation writes separate box, empty-floor, and wall tables in `evaluation.md`,
full run-sheet fields and metrics in `evaluation.json`, a replay subdirectory
per run, and box-only plots for height and distance error.

```sh
MPLCONFIGDIR=/private/tmp/wbr-mpl-cache MPLBACKEND=Agg \
  python3 tools/obstacle_decision/eval_runs.py \
  lab/mobile_demo/run_sheet.csv --out results/mobile_demo
```

Use the exact run sheet header above. All geometric distances and dimensions
are metres; `camera_tilt_deg` is degrees. Wall ROI size is in pixels.
