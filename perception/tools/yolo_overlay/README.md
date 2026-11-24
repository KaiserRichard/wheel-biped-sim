# YOLO11n bench

**FROM FILE.** The offline bench reads four lab rosbag2 recordings. It pairs colour and aligned depth within 30 ms using ROS header timestamps, runs the COCO model through ONNX Runtime on CPU, and reports median valid depth within each detection box. For `16UC1`, verify the depth scale for the recording before interpreting distances. [Recorded results and overlay](../../docs/evidence/yolo-overlay.md)

## Replay the recordings

**FROM FILE.** Use the dependencies in `requirements.txt` and a YOLO11n COCO ONNX export at 640 × 640. From this directory, pass the folder containing the four bag directories and the local model file.

```sh
python run_p1.py --bag-root ./recordings --model ./models/yolo11n.onnx --out-dir ./out
```

**FROM FILE.** The output contains per-detection ranges, timing from the host, sampled images and an animation. It also contains a fixed set of preprocessed frames for a matched timing run on a second host. Copy the same ONNX file and frame set to that host and run:

```sh
python bench_frames.py --model ./models/yolo11n.onnx --frames-dir ./out/pi-bench/frames --threads 4
```

**FROM FILE.** The host benchmark reports inference-only timing and timing with input conversion. The separate `live_cam.py` bench uses a Pi camera through `rpicam-vid` and reports capture and inference timing. [Original benchmark and run conditions](https://github.com/KaiserRichard/wbr-perception-phase1-runtime/blob/fde5721af0627eaf60c78d95296ab26924425e37/research/explore/p1/RESULTS.md)

**FROM FILE.** B-H05 contains books and B-H13 contains cardboard boxes. COCO has a `book` class. Use the [depth geometry workflow](../obstacle_decision) to evaluate box and floor geometry.
