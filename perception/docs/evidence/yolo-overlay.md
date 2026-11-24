# YOLO11n with aligned depth

**FROM FILE.** The recorded B-H05 colour/depth pair was processed with YOLO11n COCO at 640×640 and ONNX Runtime on CPU. The overlay shows class, confidence and estimated range in metres. There were zero `book` detections on the B-H05 recordings, so a comparison with tape distance for the books was unavailable. The four-thread Pi 5 frame bench reported 3.43 FPS for inference alone. [Original bench report](https://github.com/KaiserRichard/wbr-perception-phase1-runtime/blob/fde5721af0627eaf60c78d95296ab26924425e37/research/explore/p1/RESULTS.md)

**FROM FILE.** The [geometry path](../../tools/obstacle_decision) uses a floor model and depth deviations to evaluate the box and book fixtures independently of COCO semantics.
