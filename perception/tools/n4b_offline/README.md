# N4B offline bag → EuRoC → ORB-SLAM3 tools

Status: **OPEN**. This directory provides an offline D435i rosbag2 SQLite `.db3` exporter, a saved-sequence qualifier, an ORB Settings generator, and a Pi runner. It does not itself pass N4B or verify a patched Pi binary.

## Export and qualify on the Mac

Use a finalized rosbag2 SQLite bag with the four topics below, a fresh output directory, and Python 3.11+ (standard library only for export/qualification):

```bash
python3 tools/n4b_offline/db3_to_euroc.py /path/to/stereo_imu.db3 /path/to/euroc-export
python3 tools/n4b_offline/qualify_euroc.py /path/to/euroc-export --outdir /path/to/qualify-run
```

Expected topics are IR1 `/device_0/sensor_0/Infrared_1/image/data`, IR2 `/device_0/sensor_0/Infrared_2/image/data`, gyro `/device_0/sensor_2/Gyro_0/imu/data`, and accel `/device_0/sensor_2/Accel_0/imu/data`. Images must be 640×480 mono8/Y8. Export writes `mav0/cam0`, `cam1`, `imu0`, `timestamps.txt`, `times_ns.txt`, and `metadata.json`. An existing nonempty export directory is refused.

The export clock is ROS `header.stamp` for each source message. It is interpreted as D435i device/global time as configured by `realsense2_camera`. SQLite `messages.timestamp` (bag receive time) and librealsense metadata timestamp are ignored. Header stamps from different streams are not shifted or mapped. No camera–IMU offset or frame transform is applied. PNGs are encoded with Python `struct` and `zlib`, without OpenCV.

The exporter examines a combined RealSense IMU topic when present and compares its values/stamps against the raw gyro and accel streams. It records detected `driver_method` as `copy` or `linear_interpolation` only when sample data supports that classification. When the bag only contains separate gyro/accel topics, `driver_method` is `UNKNOWN`: those topics do not reveal the driver's unite method. `export_method` records that this tool linearly interpolates accel onto the gyro `header.stamp` grid and discards gyro rows outside accel brackets. Units are copied as ROS values (gyro rad/s, accel m/s²); axes are untouched.

Qualification can be repeated from exported files alone:

```bash
python3 tools/n4b_offline/qualify_euroc.py --analyze-only /path/to/euroc-export --outdir /path/to/qualify-run-2
```

Each run writes `result.json` and `qualification.txt`; exit 0=PASS, 1=FAIL, 2=BLOCKED, 3=usage/internal. Criteria cover equal camera counts, referenced PNGs, exact stereo stamps, strictly increasing IMU stamps, IMU image-span coverage, mean image/IMU rates, maximum gaps, and `timestamps.txt`/declared clock consistency. Numeric limits live only in `qualify_config.json`; each has a rationale and is marked PROPOSED. They are screening limits, not a gate PASS or proof of sensor quality. Missing required camera/data files are BLOCKED; present but invalid/misaligned evidence is FAIL.

## Generate ORB-SLAM3 settings

```bash
python3 tools/n4b_offline/make_orb_yaml.py --output-dir /path/to/orb-settings --fps 30
```

This reuses `tools/calib_tools/calib_convert.py`, combining the factory camera YAML and provisional IMU YAML with the documented factory SDK IR1→gyro transform. Outputs include `orbslam3.yaml` and `provenance.json`; the latter maps every setting to factory readback, provisional noise, operating assumption, or source-supported schema. Zero camera–IMU time shifts are explicit ASSUMED placeholders because the source files contain no measured offsets. The converter's SI input attestation is not calibration evidence. Factory rectified pinhole projections have zero distortion in the saved readback; ORB performs rectification internally from raw images matching those projections. Review calibration and timestamp association before a real run.

## Pi run (not run by this task)

Build/patch the pinned ORB-SLAM3 tree separately and record the verified patched executable SHA-256 in `tools/n4b_offline/patched_build.json`. The repository currently has no completed patch-build checksum: experiment evidence says the Pi rebuild was interrupted. The runner therefore refuses to execute by default until a recorded checksum is available. Explicit diagnostic override:

```bash
bash tools/n4b_offline/run_n4b.sh --allow-unpatched \
  "$HOME/orbslam3/ORB_SLAM3" \
  "$HOME/wbr-evidence/2026-10-14-d435i-ramtest15s/euroc-export" \
  "$HOME/wbr-evidence/2026-10-14-d435i-ramtest15s/euroc-export/times_ns.txt" \
  "$HOME/wbr-runtime/n4b-prep-20261013/D435i_serial-alias-6_stereo_inertial.yaml" \
  "$HOME/wbr-evidence/n4b-offline-run"
```

The exported path in the command must contain `mav0` layout accepted by the selected binary. This existing example YAML is an experiment artifact; generate/review a current YAML before use. The script hashes the executable, linked ORB library when present, dataset time list, settings, vocabulary, runner and build manifest; it saves `ldd`, a shell-quoted command receipt, `/usr/bin/time -v`, command output and exit code, and optionally starts `tools/eval_pipeline/profile` if that tool exists on the Pi branch. It never opens a camera. No physical Pi run is claimed here.

## Evidence and limitations

`tests/fixtures/real_saved_stream_summary.json` records MEASURED stream counts/rates from the saved real-file check in `tools/take_tools/README.md`. The shared checkout held no `.db3`, so a real bag fixture could not be included; the round-trip DB3 in tests is synthetic. Tests passing on synthetic fixtures prove tooling behavior only, not N4B. Remaining unknowns include actual D435i serialization variants, unite method when combined data is absent, Pi binary/patch identity, ROS/ORB runtime compatibility, time offset and calibration accuracy, resource use on long takes, and estimator tracking/initialization.
