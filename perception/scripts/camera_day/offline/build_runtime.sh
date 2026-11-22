#!/usr/bin/env bash
# Inside an isolated Ubuntu22.04 container; paths are explicit arguments.
set -euo pipefail
if [[ $# != 2 ]]; then echo 'Usage: build_runtime.sh WORK_ROOT SOURCE_REPO' >&2; exit 2; fi
work="$1"
repo="$2"
cmake -S "$work/Pangolin" -B "$work/Pangolin/build" -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_PANGOLIN_PYTHON=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF -DBUILD_TESTS=OFF
cmake --build "$work/Pangolin/build" --parallel 2
cmake --install "$work/Pangolin/build"
ldconfig
python3 - "$work/ORB_SLAM3" <<'PY'
from pathlib import Path
import sys
root=Path(sys.argv[1])
for p in [root/'CMakeLists.txt', *root.glob('Thirdparty/*/CMakeLists.txt')]:
    s=p.read_text().replace('-std=c++11','-std=c++17').replace('-std=c++14','-std=c++17').replace('-march=native','')
    p.write_text(s)
# GCC11/C++17 rejects upstream's increment of this boolean generation counter.
p = root/'include/LoopClosing.h'
s = p.read_text()
if 'bool mnFullBAIdx;' in s:
    p.write_text(s.replace('bool mnFullBAIdx;', 'int mnFullBAIdx;'))
# Rectified input has no separately allocated camera2 object. Upstream's
# diagnostic printer dereferences it; skip only that redundant print block.
p = root/'src/Settings.cc'
s = p.read_text()
prefix, printer = s.split('ostream &operator<<', 1)
old = 'if(settings.sensor_ == System::STEREO || settings.sensor_ == System::IMU_STEREO){'
new = 'if((settings.sensor_ == System::STEREO || settings.sensor_ == System::IMU_STEREO) && settings.cameraType_ != Settings::Rectified){'
if new not in printer:
    if old not in printer:
        raise RuntimeError('unexpected upstream camera settings printer')
    p.write_text(prefix + 'ostream &operator<<' + printer.replace(old, new, 1))
PY
cmake -S "$work/ORB_SLAM3/Thirdparty/DBoW2" -B "$work/ORB_SLAM3/Thirdparty/DBoW2/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$work/ORB_SLAM3/Thirdparty/DBoW2/build" --parallel 2
cmake -S "$work/ORB_SLAM3" -B "$work/ORB_SLAM3/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$work/ORB_SLAM3/build" --target g2o --parallel 1
cmake --build "$work/ORB_SLAM3/build" --target ORB_SLAM3 --parallel 1
tar -xf "$work/ORB_SLAM3/Vocabulary/ORBvoc.txt.tar.gz" -C "$work/ORB_SLAM3/Vocabulary"
cmake -S "$repo/scripts/camera_day/offline" -B "$work/runner-build" -DORB_ROOT="$work/ORB_SLAM3" -DCMAKE_BUILD_TYPE=Release
cmake --build "$work/runner-build" --parallel 1
