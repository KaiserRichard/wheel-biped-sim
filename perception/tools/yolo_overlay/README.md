# P1: YOLO detections with aligned D435i depth

## Status

The Mac pipeline and Pi benchmark are prepared, but this experiment has **no detector results yet**. The Mac has no `rosbags`, `onnxruntime`, or `opencv` in its available Python environments, and package installation could not resolve PyPI inside the sandbox. The ONNX model therefore could not be exported or run. SSH to `pi-host` is blocked by the sandbox, so Pi timing is **BLOCKED**. No FPS values, detection counts, overlays, or GIF are claimed as measured.

## Data and tape references

The reader uses [`rosbags.rosbag2.Reader`](https://ternaris.gitlab.io/rosbags/api/rosbags.rosbag2.html) (aliased in code as `Rosbag2Reader`) for the SQLite rosbag2 folders at `~/wbr-evidence-archive/mobile-demo-20261024-01/`. It pairs color frames from `/camera/camera/color/image_raw` with `/camera/camera/aligned_depth_to_color/image_raw` by ROS header timestamp, with a 30 ms maximum difference. Depth is read as `16UC1` using an explicit **assumption** of 0.001 m per unit; `32FC1` is treated as meters. The scale is configurable because the bag parameters do not record a depth-units value, so the assumed scale must be checked before interpreting ranges.

| Run | Tape or lab-log reference | Comparison limit |
|---|---|---|
| B-H13 | Two 13.3 cm-high box stacks, front edge at 1.00 m | COCO has no cardboard-box class, so a model output cannot be reliably matched to this target by class. |
| B-H05 | Books 4.50 cm high, front edge at 1.00 m | `book` detections will be compared with 1.00 m in `detections.csv`; this is a front-edge tape distance while box-median depth covers the full 2D detection box. |
| S1-empty | Empty tabletop; nearest logged obstacle was the lamp pole at about 1.42 m | No specific COCO object target or tape range was recorded. |
| W1 | Wall at 1.00 m | COCO has no wall class. |

The tape/run descriptions come from `lab-day-20261024/LAB_LOG.md` on `claude2/20261023-runner` (read with `git show`). The separate B-H13 and B-H05 targets should not be treated as equivalent: boxes are not a COCO class, while books are.

## Install and export on the Mac

Run from this directory. Python 3.12 is preferred over the machine's default Python 3.11.0rc2.

```sh
UV_CACHE_DIR=/private/tmp/p1-uv-cache uv venv --python 3.12 .venv312
UV_CACHE_DIR=/private/tmp/p1-uv-cache uv pip install --python .venv312/bin/python -r requirements.txt
UV_CACHE_DIR=/private/tmp/p1-uv-cache uv pip install --python .venv312/bin/python ultralytics
.venv312/bin/yolo export model=yolo11n.pt format=onnx imgsz=640 opset=12
mkdir -p models
mv yolo11n.onnx models/yolo11n.onnx
```

Ultralytics downloads the COCO-pretrained `yolo11n.pt` weights during export if they are not already cached. The export is an ONNX model; inference below uses ONNX Runtime with its CPU execution provider. References: [YOLO11 model](https://docs.ultralytics.com/models/yolo11/), [COCO class list](https://github.com/ultralytics/ultralytics/blob/main/ultralytics/cfg/datasets/coco.yaml), and [Ultralytics ONNX export](https://docs.ultralytics.com/modes/export).

Run bag inference and create the outputs:

```sh
.venv312/bin/python run_p1.py --model models/yolo11n.onnx
```

The script writes `detections.csv`, `results.json`, eight sampled overlay PNGs, and `p1-detections.gif` under `out/`. It also saves 50 preprocessed RGB frames as CHW `uint8` NumPy arrays under `out/pi-bench/frames/` for the matching Pi benchmark. The default depth conversion is 0.001 m per `16UC1` unit; change it with `--depth-scale` if the recorded camera scale differs.

The detailed CSV contains one row per detection, median valid aligned-depth pixel within the box, and—only for a `book` detection in B-H05—the difference from its 1.00 m front-edge tape distance. A median over the whole 2D box may include background and is not a surface-point measurement. Empty detections still appear in the frame count in `results.json`, not as fabricated CSV rows.

## FPS protocol

`run_p1.py` reports Mac model-only FPS across all paired bag frames and a second Mac pipeline FPS including image preprocessing, ONNX inference, NMS, and box-depth median. Bag decoding and writing PNG/GIF files are excluded from that pipeline timer.

For a matched Mac/Pi CPU comparison, `bench_frames.py` times ONNX Runtime on the same 50 preprocessed frames. It reports inference-only FPS and input-conversion-plus-inference FPS. Input loading and the five warm-up calls are excluded. The Pi runs use ten passes over the same 50-frame set and report 500 timed inferences per thread count.

| Platform / ONNX Runtime threads | Result | Status |
|---|---:|---|
| Mac, bag pipeline, default threads | Not measured | **BLOCKED**: runtime dependencies and ONNX weights unavailable in the sandbox. |
| Mac, 50-frame benchmark, default threads | Not measured | **BLOCKED**: same dependency/model blocker. |
| Pi 5, 1 thread, 50-frame benchmark | Not measured | **BLOCKED**: `ssh pi-host` returned `Operation not permitted`. |
| Pi 5, 4 threads, 50-frame benchmark | Not measured | **BLOCKED**: `ssh pi-host` returned `Operation not permitted`. |

Observed setup attempts on this Mac:

- Initial `uv` setup failed while opening the protected global cache at `~/.cache/uv/sdists-v9/.git` (`Operation not permitted`). Redirecting it to `/private/tmp/p1-uv-cache` created the local venv, but `uv pip install --python research/explore/p1/.venv/bin/python rosbags onnxruntime opencv-python-headless` then failed after three retries because DNS lookup for `pypi.org` failed.
- `ssh -o BatchMode=yes -o ConnectTimeout=8 pi-host 'uname -m && python3 --version'` returned `connect to host pi-host port 22: Operation not permitted`.
- The available Python 3.11, 3.10, 3.12, 3.14, and Command Line Tools 3.9 environments did not contain `rosbags` or `onnxruntime`. No ONNX model was present in the local cache.
- `/usr/local/bin/python3.12 -m py_compile research/explore/p1/run_p1.py research/explore/p1/bench_frames.py` passed; this only confirms Python syntax, not runtime behavior.

Once dependencies and the ONNX model are available, copy the benchmark inputs to the Pi and run both thread settings:

```sh
ssh pi-host 'mkdir -p ~/p1-bench/frames'
scp models/yolo11n.onnx bench_frames.py pi-host:~/p1-bench/
scp out/pi-bench/frames/frame_*.npy pi-host:~/p1-bench/frames/
ssh pi-host 'source ~/course-stage/cn-venv/bin/activate && cd ~/p1-bench && nice -n 10 python bench_frames.py --model yolo11n.onnx --frames-dir frames --threads 1 --passes 10'
ssh pi-host 'source ~/course-stage/cn-venv/bin/activate && cd ~/p1-bench && nice -n 10 python bench_frames.py --model yolo11n.onnx --frames-dir frames --threads 4 --passes 10'
```

The benchmark uses the existing `~/course-stage/cn-venv` ONNX Runtime environment and does not access `/dev/ttyAMA0`, the Nucleo, or motor controls.

## What this COCO model can identify

YOLO11n detection is trained on the 80 COCO object categories. It includes `book`, but not `box`, `floor`, `step`, `curb`, or `wall`. A box on the floor may be missed or assigned a misleading known label; a step edge is not a class this model can report. Depth can give a distance for a detected box, but it does not make an absent semantic class detectable.

Fine-tuning needs an explicitly defined robot-hazard label set (for example `box_obstacle`, `step_up`, and `step_down`), representative positive and hard-negative images across the robot's camera height, tilt, surfaces, lighting, and approach distances, and consistent boxes or masks for the chosen task. Keep validation recordings separated by capture session and include empty-floor, texture, shadow, and ordinary-object negatives. If step geometry matters, segmentation or an explicit depth/plane edge algorithm may be a better target than a COCO-style bounding box alone. The four bags in this brief are a small prototype set, not enough for a generalization claim.
