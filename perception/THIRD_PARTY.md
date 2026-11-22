# Third-party sources

**FROM FILE.** The Phase 1 checkout recorded the following dependency pins in `.gitmodules` and its gitlinks. Check out upstream sources separately when needed.

| Project | Revision | Terms and credit |
| --- | --- | --- |
| [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) | `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4` | GPLv3. UZ-SLAMLab, Carlos Campos and coauthors. [Upstream licence](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/LICENSE) |
| [ORB_SLAM3_ROS2](https://github.com/gjcliff/ORB_SLAM3_ROS2) | `efd7acd9ea31bfdbcd843a4c90eeaa85f7e80dd8` | BSD-3-Clause, Graham Clifford. The underlying ORB-SLAM3 code retains its own GPL terms. [Upstream licence](https://github.com/gjcliff/ORB_SLAM3_ROS2/blob/efd7acd9ea31bfdbcd843a4c90eeaa85f7e80dd8/LICENSE) |
| [realsense-ros](https://github.com/realsenseai/realsense-ros) | `9a11121700cb4780e273e34141f6402fe184321d` | Apache-2.0. Intel/RealSense contributors. [Upstream licence](https://github.com/realsenseai/realsense-ros/blob/9a11121700cb4780e273e34141f6402fe184321d/LICENSE) |

**FROM FILE.** The ORB wrapper and marked native/offline integration sources retain their GPL notices. The native example patches also retain GPL terms. See the [GPL v3 text](licenses/GPL-3.0.txt) and the [historical startup patch](https://github.com/KaiserRichard/wbr-perception-phase1-runtime/tree/fde5721af0627eaf60c78d95296ab26924425e37/patches/orbslam3). [Adapter source notes](tools/vio_adapter/SOURCES.md)

**FROM FILE.** The old checkout also recorded the [micro-ROS STM32CubeMX utilities](https://github.com/micro-ROS/micro_ros_stm32cubemx_utils) under Apache-2.0 with [additional notices](https://github.com/KaiserRichard/wbr-perception-phase1-runtime/tree/fde5721af0627eaf60c78d95296ab26924425e37/tools/microros_f446/vendor). The link-health tests retain the [snapshot attribution](tools/perception_health/upstream_snapshot/SOURCE.txt) for their external firmware-interface reference.

**FROM FILE.** [EuRoC](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) is credited to the dataset authors and ETH Zürich. The benchmark used the MH_01_easy sequence. The detector overlay used Ultralytics YOLO11n COCO via ONNX Runtime. Datasets, model weights, libraries and container images are external dependencies.
