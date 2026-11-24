# Host tests

**FROM FILE.** From the main repository root, use the main suite and the perception runner. The runner changes into `perception/` so existing `tools`, `sim`, `lab` and `ros2` imports resolve. Each suite runs separately so a failure in one suite does not hide the others.

```bash
python -m pytest
bash perception/tools/run_all_tests.sh --core
bash perception/tools/run_all_tests.sh
python publish_check.py
```

**FROM FILE.** `--core` runs CPU unittest fixtures for calibration math, clocks, frames, ground-plane and obstacle decisions, the local map, the ROS planner and bridge mapper, scene replay, scenario scoring, trajectory alignment and pose geometry. Use a Python environment with NumPy, SciPy, PyYAML and MuJoCo. Set `PYTHON` to a specific interpreter when needed.

**FROM FILE.** The full runner keeps the host Makefile targets and adds the imported unittest suites. Use Linux with Bash, GNU Make, C/C++ and the relevant Python dependencies, including `opencv-python`, `rosbags`, `mcap`, and `mcap-ros2-support`. Some fixtures use POSIX serial or PTY APIs, signals and process groups. The ORB adapter default test uses a fake engine. Pi, live ROS topics and captured raw sessions are separate from the host tests.

**FROM FILE.** Run a single suite from `perception/` with `python3 tools/test_support/run_unittest.py tools/ground_plane/tests`. [Historical Phase 1 tests and logs](https://github.com/KaiserRichard/wbr-perception-phase1-runtime/tree/fde5721af0627eaf60c78d95296ab26924425e37)
