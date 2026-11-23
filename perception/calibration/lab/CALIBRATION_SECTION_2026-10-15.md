# Calibration section — lab day 2026-10-15 (D435i 99005)

Status: everything below is **OPEN / diagnostic**. Power is still 5 V / 3 A, so no take qualifies a gate.
Detail lives in the repo: `docs/D435I_CALIBRATION_PLAN.md`, `calibration/lab/README.md`, `calibration/allan/README.md`
(an earlier private branch).

## What is ready (prepared 2026-10-14 on the Mac)

| Item | State | Evidence |
|---|---|---|
| Kalibr | Docker image `kalibr:noetic-1f60227` built on the Mac (Intel x86, stock Noetic Dockerfile, `catkin build -j4`) | `python3/logs/docker_build.log` |
| Kalibr smoke test | **Works (MEASURED).** Camera–IMU calibration on the public TUM-VI `dataset-calib-imu4_512_16` bag, using the reference intrinsics from ORB-SLAM3's `TUM-VI.yaml`: reprojection 0.09 px, timeshift 0.03 ms, and camera→IMU transform within **1.0 mm / 0.02°** of the ORB-SLAM3 reference. Kalibr's own stereo calibration found the AprilGrid in 427/427 images; on these 195° fisheye lenses its final optimization diverged, which is a lens-model issue not expected for the D435i's rectified pinhole IR | `python3/smoke/` |
| Take → Kalibr bag | Worked on the real 15 s D435i RAM take from 2026-10-14: 447 stereo pairs at 29.99 Hz, IMU 200.16 Hz (combined), all four streams `global_time`, 0 gaps (MEASURED) | `python3/realtake/ramtest15s-bag/` |
| Target | A3 6×4 AprilGrid, 45 mm tags (A4 30 mm fallback), printed tonight | `calibration/aprilgrid/README.md` |
| Scripts | Pi: `calibration/lab/pi/record_calib_take.sh`. Mac: `take_to_kalibr_bag.sh`, `run_kalibr_camimu.sh`, `run_kalibr_stereo_check.sh`. 20 synthetic tests pass | `calibration/lab/tests/verification.log` |
| Allan | IMU-only recorder (C++, build on the Pi) + `allan_analyze.py`. 12 synthetic tests pass; N recovered within 0.1 %, K within 5.2 % | `calibration/allan/verification.md` |

## Tonight at home (optional, only if the Pi and camera are with you)

The IMU-only Allan recording draws little power, so it can start tonight on the 5 V / 3 A supply. Its power log decides
whether the data counts: any under-voltage bit during the run labels it **DIAGNOSTIC - power not clean** (then repeat it
once the 5 A supply is in use). Put the camera on a solid table, cable strain-relieved, and don't touch it until morning.

On the Mac (copy the tools; adjust the host alias if needed):

```bash
ssh pi-host-wifi 'mkdir -p ~/wbr-runtime/allan-20261014'
scp calibration/allan/d435i_imu_record.cpp calibration/allan/record_allan.sh calibration/allan/power_monitor.py pi-host-wifi:~/wbr-runtime/allan-20261014/
```

On the Pi:

```bash
cd ~/wbr-runtime/allan-20261014
g++ -std=c++17 -O2 -Wall -Wextra -pthread d435i_imu_record.cpp -o d435i_imu_record -lrealsense2
./d435i_imu_record /tmp/allan-pilot 0.01          # 36 s pilot: check rates and domain, then delete /tmp/allan-pilot
nohup bash ./record_allan.sh 10 home-20261014 > ~/allan-home-20261014.launch.log 2>&1 < /dev/null &
```

Ten hours ≈ 2.1 GB of CSV (CALCULATED 207 MB/h); the script refuses to start without 3× that free. Stop early with
`pkill -INT d435i_imu_record` (clean stop). Results go to `~/wbr-evidence/allan-home-20261014/`. Allow 30 min of warm-up:
the first 30 min can be cut in the analysis. Kalibr suggests 15–24 h; 4 h is the plan's provisional minimum.

Next morning, on the Mac: copy the folder back, then
`python3 calibration/allan/allan_analyze.py RUN_DIR OUT_DIR` → `imu_allan_raw.yaml`, `imu_allan_x10.yaml`, plots, `summary.md`.

## At the lab — camera–IMU takes (after N1 on the new unit and the ORB demo, if time allows)

Mount the target flat on the wall or a table, with daylight or incandescent light. Check that both IR views show crisp tags
with the emitter off. Then, on the Pi:

```bash
export WBR_CAMERA_AVAILABLE='CAMERA AVAILABLE'
bash ~/wbr-runtime/calib-lab-20261015/pi/record_calib_take.sh C1 60     # repeat as C2, C3; S1 for the stereo check
```

(copy `calibration/lab/pi/record_calib_take.sh` and `take_diagnostics.py` to `~/wbr-runtime/calib-lab-20261015/pi/` first.)
60 s takes need ≈2.0 GB free RAM, which is fine on the 8 GB Pi. Each take ends with `take usable for Kalibr: yes/no (diagnostic)`.

**How to move (C takes, 60 s):** 5–10 s completely still, target in view → slow rotations about each camera axis
(roll, pitch, yaw, ±20–30°) → slow translations left-right, up-down, toward-away (±15–20 cm) → mixed smooth motion, keeping
the whole grid in view and filling different parts of the image → 5–10 s still. Distance 0.4–0.8 m (A3) / 0.3–0.5 m (A4).
No jerks or taps (they cause blur and IMU spikes), and don't let the cable tug the camera. **S take (stereo check):** hold still
for ~10 s at 0.4, 0.6 and 0.8 m, then slow sweeps so the grid visits all four image corners.

## Back on the Mac — Kalibr

```bash
bash calibration/lab/mac/take_to_kalibr_bag.sh TAKE_DIR/take DERIVED/C1-bag
bash calibration/lab/mac/run_kalibr_camimu.sh DERIVED/C1-bag/take.bag MEASURED_TARGET.yaml DERIVED/C1-imucam [imu_allan_x10.yaml]
bash calibration/lab/mac/run_kalibr_stereo_check.sh DERIVED/S1-bag/take.bag MEASURED_TARGET.yaml DERIVED/S1-stereo
```

`MEASURED_TARGET.yaml` = a copy of the target YAML with the measured `tagSize`. Without Allan results the cam-IMU run uses the
ASSUMED provisional noise file (fine for a first look; repeat later with the Allan values). Cam-IMU takes a few minutes per take.

## Reading a Kalibr report

Two real example reports from the smoke test are in the project files: `kalibr_example_GOOD_tumvi_report.pdf` (correct intrinsics) and `kalibr_example_BAD_wrong_intrinsics_report.pdf`. The bad one used intrinsics from an unfinished camera calibration: the extrinsic still came out within ~2 mm, but the reprojection error was **8.6 px instead of 0.09 px**. That's the lesson: a plausible transform doesn't prove a good calibration, so always check the reprojection error first.

Good (ASSUMED review thresholds from the plan, not gate criteria):
- reprojection error RMS ≤ 0.5 px with errors spread evenly (no pattern toward the image edges);
- gyro and accel error plots look like noise around zero, mostly inside the 3σ bounds;
- T_cam_imu within ~5 mm and ~2° of the factory CAD transform; the three C takes agree within ~3 mm / 1°;
- `timeshift_cam_imu` of a few ms or less, repeatable within ~1 ms across C1–C3.

Bad, check the data before trusting anything:
- many frames without detected corners (blur, dark IR, inkjet print, target too far away);
- reprojection error ≥ 1 px, or a structured pattern;
- IMU residuals far outside 3σ, or spikes (taps or jerks during the take);
- translation differing from CAD by centimetres, or a rotation of tens of degrees (frames or axes mixed up);
- time shift hitting the ±30 ms search limit, or jumping between takes.

`run_kalibr_camimu.sh` prints these numbers with "within / outside ASSUMED threshold" and writes an ORB-SLAM3 candidate
YAML under `orbslam3/`. It doesn't replace the operating YAML. Note that ORB-SLAM3 has no time-offset key (plan §4).
