# N4A — upstream ORB-SLAM3 stereo-inertial on EuRoC MH01 (Pi 5, 4 GB) — 2026-10-13

**Status: OPEN** (accuracy reproduces the published value; timing is invalid as a measurement because of live under-voltage).

## Setup (MEASURED unless noted)
- Platform: Raspberry Pi 5 **4 GB** (temporary stand-in for the 8 GB target), Ubuntu 24.04.4, kernel 6.8.0-1064-raspi, 32 GB microSD, 5 V/3 A supply. One 4 GB temporary swapfile active (≈42 MB used).
- ORB-SLAM3 upstream 4452a3c built natively in `~/orbslam3/ORB_SLAM3` with three compatibility patches (C++11→C++17 + `-march=native` removed; Sophus tests off; `mnFullBAIdx++` on a `bool` → `= true`, identical C++11 semantics). Pangolin v0.9.1. Patches/logs: `../2026-10-13-orbslam3-pi-build/` and `~/wbr-evidence/2026-10-13-orbslam3-build/` on the Pi. Hashes in `provenance.txt`.
- Dataset: EuRoC MH_01_easy, ASL format, extracted from the official ETH Research Collection "Machine Hall Datasets" zip (12,683,729,426 B); inner `MH_01_easy.zip` sha256 `5f4ecbc5…a4bfc6`. 3,682 cam0 frames, 184 s.
- Command (upstream example, viewer already disabled upstream; upstream `EuRoC.yaml` and `EuRoC_TimeStamps/MH01.txt` unchanged):
  `stereo_inertial_euroc Vocabulary/ORBvoc.txt Examples/Stereo-Inertial/EuRoC.yaml ~/datasets/euroc/MH01 Examples/Stereo-Inertial/EuRoC_TimeStamps/MH01.txt mh01_stereoi`
- Evaluation: evo 1.x on the Mac, `evo_ape tum gt.tum est.tum -a --t_max_diff 0.02` (SE(3) alignment, translation ATE). Estimate = `f_mh01_stereoi.txt` (body/IMU frame, ns→s); GT = `state_groundtruth_estimate0/data.csv` (body frame) via `evo_traj euroc --save_as_tum`.

## Results
| Run | ATE RMSE SE(3) [m] | ATE max [m] | Sim(3) RMSE [m] | Keyframes | Maps | Wall time | Peak RSS |
|---|---|---|---|---|---|---|---|
| 1 | 0.0415 | 0.081 | 0.0189 | 126 | 1 | 5:01 | 823 MiB |
| 2 | 0.0368 | 0.076 | 0.0238 | 126 | 1 | 4:42 | 828 MiB |
| 3 | 0.0416 | 0.088 | 0.0223 | 129 | 1 | 4:44 | 828 MiB |

- Mean SE(3) ATE RMSE **0.040 m** (spread 0.037–0.042). All runs: one map, no resets, IMU initialized (VIBA 1 and 2), exit 0, 3,678 poses.
- **SOURCE-SUPPORTED (verified 2026-10-14):** Campos et al., ORB-SLAM3, IEEE T-RO 2021, Table II (EuRoC, RMS ATE in m, SE(3) alignment for stereo-inertial): stereo-inertial MH01 = **0.036 m** (MH02 0.033, MH03 0.035, MH04 0.051, MH05 0.082). Source: arXiv 2007.11898v2 HTML. Our 0.037–0.042 m is in the same range. INFERRED: the native ARM64 build and the C++17 patches did not degrade accuracy on this sequence.
- **Timing (not a valid measurement):** wall time 282–301 s for a 184 s sequence including vocabulary load. INFERRED ≈60–70 ms per stereo frame, i.e. slower than the 20 Hz camera rate on this 4 GB Pi. During the runs `vcgencmd get_throttled` showed live under-voltage/throttling (`0x50005`) in **87 of 178** five-second samples (49 %), max temp 63 °C. CPU frequency was therefore not stable. Timing must be re-measured on the 8 GB Pi with the 5 V/5 A supply.

## Clock note
All UTC labels written on the Pi (`runs.log`, power log) come from the Pi clock, which was **unsynchronized and ≈80 min behind** true UTC at the time (MEASURED 2026-10-13: Mac 09:28:25Z vs Pi 08:08:01Z; Wi-Fi down after the ~07:46Z reboot, so no NTP). Durations within a run are unaffected unless the clock was stepped mid-run (no step observed). Do not compare these labels with Mac-side times without the offset.

## Not verified / limits
- The vocabulary-load time was not separated from tracking time (upstream example prints no per-frame timing unless built with REGISTER_TIMES).
- Single sequence only (MH01). No RPE, no other sequences.
