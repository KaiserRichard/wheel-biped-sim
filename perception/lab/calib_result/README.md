# Kalibr result acceptance gate

`check_kalibr_result.py` evaluates the camera–IMU fit and stereo validation
outputs before they are considered as ORB-SLAM3 calibration candidates. It
reuses the repository parser and transform checks in
`calibration/lab/mac/summarize_kalibr.py`. The gate writes `result.json`, a
factory comparison in `diff.txt`, captured table output in `check.log`, and
hashed input snapshots in each output directory.

The fixture PASS means only that the software accepted a controlled fixture.
It does not pass a project gate or establish physical calibration accuracy.

## Capture and run commands

The Pi records takes; Kalibr and this checker run on the Mac with the existing
local Kalibr environment. These checks do not require Pi access or hardware.
After N1/N2/N3 and the capture prerequisites in `calibration/lab/README.md` are
reviewed, the Pi capture commands are:

```bash
export WBR_CAMERA_AVAILABLE='CAMERA AVAILABLE'
bash ~/wbr-runtime/calib-lab-20261015/pi/record_calib_take.sh C1 60
bash ~/wbr-runtime/calib-lab-20261015/pi/record_calib_take.sh C2 60
bash ~/wbr-runtime/calib-lab-20261015/pi/record_calib_take.sh C3 60
bash ~/wbr-runtime/calib-lab-20261015/pi/record_calib_take.sh S1 60
```

Copy each take to the Mac using the separately reviewed evidence-transfer
procedure. From the repository root on the Mac, create the ROS1 bags and fit
the camera–IMU and stereo results:

```bash
bash calibration/lab/mac/take_to_kalibr_bag.sh /path/to/C1/take /path/to/derived/C1-bag
bash calibration/lab/mac/run_kalibr_camimu.sh /path/to/derived/C1-bag/take.bag /path/to/measured-aprilgrid.yaml /path/to/derived/C1-imucam
bash calibration/lab/mac/take_to_kalibr_bag.sh /path/to/S1/take /path/to/derived/S1-bag
bash calibration/lab/mac/run_kalibr_stereo_check.sh /path/to/derived/S1-bag/take.bag /path/to/measured-aprilgrid.yaml /path/to/derived/S1-stereo
```

Run the acceptance gate on the Mac from the repository root. Repeat for each
camera–IMU take by supplying the matching result directory; use the dedicated
stereo result directory for the stereo check:

```bash
python3 lab/calib_result/check_kalibr_result.py \
  --camimu-chain /path/to/derived/C1-imucam/take-camchain-imucam.yaml \
  --camimu-results /path/to/derived/C1-imucam/take-results-imucam.txt \
  --stereo-chain /path/to/derived/S1-stereo/take-camchain.yaml \
  --stereo-results /path/to/derived/S1-stereo/take-results-cam.txt \
  --run-imu /path/to/derived/C1-imucam/work/imu.yaml \
  --outdir /path/to/derived/C1-acceptance
```

Re-evaluate the exact saved inputs and threshold policy without the source
result directories:

```bash
python3 lab/calib_result/check_kalibr_result.py \
  --analyze-only /path/to/derived/C1-acceptance \
  --outdir /path/to/derived/C1-acceptance-reanalysis
```

Exit codes are 0 PASS, 1 FAIL, 2 BLOCKED (missing or unreadable required input),
and 3 usage/internal error. Every run writes `result.json`. Thresholds and
their source/rationale are centralized in `thresholds.yaml`; chosen limits are
marked **PROPOSED**. Use a fresh output directory for each run.

## Criteria

| Criterion | What is evaluated |
|---|---|
| Reprojection | Camera–IMU results mean and standard deviation per camera; stereo results X/Y mean and standard deviation per camera, in pixels. Normalized residuals are not substituted. |
| IMU residuals | Absolute gyro and accelerometer residual mean, plus standard deviation, in the report's stated SI units. Missing residual summaries BLOCK. |
| Time shift | Absolute `timeshift_cam_imu` for each camera; sign convention is `t_imu = t_cam + shift`. |
| Camera–IMU transform | Inverse of Kalibr `T_cam_imu` (camera to IMU) compared with the repository factory/CAD reference translation and identity rotation. Translation and angle limits are the documented assumed 5 mm / 2°. |
| Stereo baseline | Fitted translation norm compared with the exact factory camchain value (49.912553280592 mm), not a nominal 50 mm. |
| Intrinsics | Absolute `fx`, `fy`, `cx`, and `cy` drift from the factory readback for both cameras. |
| Distortion | Requires pinhole-radtan, bounds each coefficient's drift from factory and its absolute magnitude. |
| IMU provenance | Run's topic, update rate, noise densities and random walks must equal `imu_d435i_provisional.yaml` exactly. This verifies which provisional values were used; it does not validate them physically. |

`diff.txt` gives fitted/factory intrinsics, distortion, and stereo baseline.
Transform criteria and values are also included in `result.json`.

## Limits

All reported acceptance is policy-based. The fixture test suite has one parser
fixture copied from the real TUM-VI Kalibr smoke run; it tests parsing only and
is not D435i evidence. Other fixtures are synthetic. The gate cannot prove
target metrology, image coverage, systematic residual structure, physical
camera/IMU alignment, time-offset repeatability across takes, validity of the
provisional noise model, correct sensor clock mapping, ORB-SLAM3 runtime
behavior, or Pi throughput. Review Kalibr PDFs and source receipts separately.
The factory camchain YAML carries the stereo factory geometry; the IMU
translation/rotation comparison uses the rounded factory/CAD reference already
held by the existing summary tool.
