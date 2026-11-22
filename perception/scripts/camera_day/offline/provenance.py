#!/usr/bin/env python3
"""Archive runtime versions and hashes without changing source or recorded input."""
import argparse
import json
from pathlib import Path
import platform
import subprocess

from extract_stereo import sha


def command(args):
    return subprocess.check_output(args, text=True).strip()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('work', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    work = args.work
    orb = work / 'ORB_SLAM3'
    files = [orb / 'lib/libORB_SLAM3.so', orb / 'Vocabulary/ORBvoc.txt',
             work / 'runner-build/replay_stereo', work / 'clean-pairs-03/pairs.csv',
             work / 'clean-pairs-03/extraction.json']
    files += sorted((work / 'tools/scripts/camera_day/offline').glob('*'))
    files += sorted((orb / 'src').rglob('*'))
    files += sorted((orb / 'include').rglob('*'))
    files += sorted((orb / 'Thirdparty').glob('*/lib/*.so*'))
    files += [work / 'tools/scripts/camera_day/replay_metrics.py',
              work / 'tools/scripts/camera_day/native/orb_access.hpp']
    # Hash the libraries actually loaded, including installed Pangolin/OpenCV.
    loaded = command(['ldd', str(work / 'runner-build/replay_stereo')])
    libraries = sorted({Path(line.split('=>', 1)[1].split()[0]).resolve()
                        for line in loaded.splitlines()
                        if '=>' in line and line.split('=>', 1)[1].strip().startswith('/')})
    result = {'platform': platform.platform(), 'machine': platform.machine(),
              'orb_commit': command(['git', '-C', str(orb), 'rev-parse', 'HEAD']),
              'pangolin_commit': command(['git', '-C', str(work / 'Pangolin'), 'rev-parse', 'HEAD']),
              'compiler': command(['c++', '--version']), 'ldd_runner': loaded,
              'dpkg': command(['dpkg-query', '-W', 'libopencv-dev', 'libeigen3-dev', 'libboost-serialization-dev']),
              'python_packages': command(['python3', '-m', 'pip', 'freeze']),
              'hashes': {str(p.relative_to(work)): sha(p) for p in files if p.is_file()},
              'loaded_library_sha256': {str(p): sha(p) for p in libraries},
              'orb_compatibility_diff': command(['git', '-C', str(orb), 'diff'])}
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2)
        stream.write('\n')


if __name__ == '__main__':
    main()
