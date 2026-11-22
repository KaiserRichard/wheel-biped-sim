# G-A AprilGrid take → Kalibr utilities

**Status: OPEN throughout.** Prepared on the canonical Mac on 2026-10-14 for the
2026-10-15 lab session. These files do not qualify a gate or establish calibration
accuracy. No camera, Pi, SSH, network, install, Git or Docker command was executed
during preparation. Existing evidence and external source trees were read only.
No build is required for these scripts; the reviewer supplies the Docker image.

Follow [the calibration plan](https://github.com/KaiserRichard/wbr-perception-phase1-runtime/blob/fde5721af0627eaf60c78d95296ab26924425e37/docs/D435I_CALIBRATION_PLAN.md), including
N1 → N2 → N3 and independent attestation of the 30 Hz recording profile. N2's
15 Hz profile and mandatory 300-second soak are unchanged. USB must be ≥5000M.
All physical capture remains OPEN until the owner explicitly says
`CAMERA AVAILABLE`, stable power/cooling and storage are qualified, and the
prerequisites are reviewed. This script cannot attest those prerequisites.

## Inputs and evidence labels — OPEN

- **MEASURED factory SDK readback:** SDK serial **99005**; IR1/IR2
  pinhole stream projections at 640×480, four radtan zeros, identity IR1→IR2
  rotation and translation `[-0.049912553280592,0,0]` m. These are archived SDK
  values, not an independent optical calibration. Configuration provenance is
  commented in `config/camchain_factory_serial-alias-6.yaml`.
- **MEASURED readback / INFERRED CAD:** camera→D435i IMU translation
  `[-0.00552,+0.00510,+0.01174]` m, identity rotation. This rounded reference is
  used only for comparison. D435i `imu_camera` is distinct from STM32 `imu_body`.
- **ASSUMED:** provisional continuous-time noise densities/walks in
  `config/imu_d435i_provisional.yaml`: gyro `1e-3`, accel `1e-2`, gyro walk
  `1e-6`, accel walk `1e-4`. Combined gyro-grid update rate is 200 Hz; the raw
  accel stream is 100 Hz. Allan variance remains OPEN.
- **ASSUMED review thresholds:** CAD translation difference ≤5 mm, rotation
  angle ≤2°, radial reprojection RMS ≤0.5 px, stereo baseline difference ≤1 mm.
  Reports say **within / outside ASSUMED threshold**. No gate PASS is produced.
  Time-shift spread ≤1 ms requires at least three independent takes and remains
  UNKNOWN in a single-take summary. Intrinsic/distortion delta thresholds are
  unspecified; the tool reports values without inventing acceptance criteria.
- **SOURCE-SUPPORTED:** local PR #3 tool interfaces and Kalibr source interfaces,
  supplied source pin `1f60227`. No Git operation verified the pin. Runtime
  receipts hash the actual sources and record local image ID/OS/architecture/
  repository digests; a tag alone is not proof of source identity.
- **UNKNOWN:** physical camera–IMU alignment/time offset/noise, board scale,
  optical accuracy, Docker build/runtime compatibility, real ROS1 round trip and
  recorder resource behavior on tomorrow's hardware. All remain OPEN.

SDK serial, ASIC serial, firmware-update ID and USB descriptor serial must be
recorded as four separate fields. Preserve new SDK calibration warnings in raw
logs; do not infer that old-unit warnings were observed on this unit.

## Order of use — OPEN

### 1. Prepare and measure the target

Preferred target is `calibration/aprilgrid/aprilgrid_6x4_45mm_A3.yaml`; fallback
is `aprilgrid_6x4_30mm_A4.yaml`. Both have **6 columns × 4 rows**, spacing ratio
0.30. Use the actual measured black-edge size and gap ratio in a separate
session copy of the YAML; retain the nominal original. Record scale uncertainty,
board flatness, illumination and decoded-tag visibility in both IR views.
The plan's older print paragraph mentioning six rows conflicts with its revised
target definition; these utilities follow the actual 6×4 target files.

The recorder has no exposure/gain control added by this task. Suitable manual
exposure/gain and metadata completeness remain OPEN. Its raw `record.log`,
`sdk.log` and `timestamps.csv` must be reviewed for actual profile, emitter,
clock domains, SDK warnings and available readbacks. Do not claim an unlogged
automatic exposure choice is qualified.

### 2. Later: record locally on the Pi

Copy `pi/record_calib_take.sh` **and** `pi/take_diagnostics.py` as checksummed
copies into a fresh isolated runtime bundle. Preserve both checkouts and the
existing recorder bundle. No transport/deployment is performed by this task.
On the Pi, after explicit camera authorization and prerequisite review:

```bash
# Replace the bundle path with the actual verified deployment location.
export WBR_CAMERA_AVAILABLE='CAMERA AVAILABLE'
bash "$HOME/wbr-runtime/calib-lab-20261015/pi/record_calib_take.sh" C1 60
bash "$HOME/wbr-runtime/calib-lab-20261015/pi/record_calib_take.sh" C2 60
bash "$HOME/wbr-runtime/calib-lab-20261015/pi/record_calib_take.sh" C3 60
```

The script accepts finite seconds in [1,120]; NAME is a safe 1–64 character
identifier. The existing recorder is fixed at
`~/wbr-runtime/n4b-20261013T080913Z/bin/d435i_record`; analysis uses that bundle's
`bin/analyze_take.py`. No binary or SDK is rebuilt. The recorder selects IR1+IR2
Y8 640×480@30, gyro 200, accel 100, emitter OFF and SDK 99005.

Capture goes into `/dev/shm/wbr-calib-NAME`, then is copied to
`~/wbr-evidence/2026-10-15-calib-NAME/take`. Existing paths are refused.
RAM free bytes must be at least `18,400,000 * seconds * 1.2 * 1.5`:
60 s requires **1,987,200,000 bytes**, 120 s **3,974,400,000 bytes**. A 4 GB Pi
with a smaller tmpfs may refuse even 60 s. Actual usable RAM is UNKNOWN;
shorter takes are tooling diagnostics and do not silently satisfy the plan's
60–120 s C protocol. Do not relax the reserve or silently remount tmpfs.

Before capture, destination capacity is checked against twice the
overhead-adjusted take budget plus an **ASSUMED 2 GiB OS reserve**. Capacity is
not storage qualification. The plan's explicit 128 GB microSD exception still
requires separate sustained-write qualification; no new storage exception is
created here. Full-rate data must not go to an unqualified OS volume.

Outputs beside `take/` are `record.log`, `power-1hz.log`, `kernel.log`,
`analysis.json`, `analysis.stderr.log`, `ram-sha256.txt`, `copy-verify.log`,
`tools-sha256.txt`, `sizes.txt`, and `summary.txt`. Power is sampled before,
approximately once per second during, and after recording: raw throttled bits,
EXT5V ADC, temperature and ARM clock. Live/latched bits are preserved and decoded
as their OR over the sampled window. Sampling/command overhead is UNKNOWN.
Kernel log permission errors stay visible; no permissions are broadened.

Only a successful SHA-256 verification permits RAM deletion. Failed recorder
takes are also copied if present; copy/hash failures preserve RAM. Interruption
signals request normal recorder shutdown rather than a forced kill. Partial
directories remain for review and must never be reused for a retry.

The exact summary line is `take usable for Kalibr: yes/no (diagnostic)`.
`yes` requires recorder/analysis success, verified copy, complete logging, zero
sampled throttled masks, all streams with increasing stamps, no skipped IR
frame numbers, one common global-time or hardware-clock domain, rates within
**ASSUMED ±5%**, and max gaps ≤**ASSUMED 1.5 nominal periods**. These conservative
checks do not replace N2/N3, export audit, target/excitation inspection or
exposure qualification. Exit 0 means usable by this diagnostic, 1 means no,
2 means setup/error; a copy/setup failure may exit earlier with evidence retained.

After capture, copy the session evidence to the Mac through the separately
authorized workflow and verify its hashes before deleting any card evidence.
Retain at least three independent C takes, still intervals and six-DOF motion.
Use a separate S take with static views plus slow coverage for stereo validation.

### 3. On the Mac: export and create a ROS1 bag

From the repository root, with a finalized local take and **fresh** output:

```bash
bash calibration/lab/mac/take_to_kalibr_bag.sh \
  /path/to/2026-10-15-calib-C1/take /path/to/derived/C1-bag
```

The fixed PR #3 exporter path is
`<local workspace>/Desktop/Research/wbr-sw-foundation/tools/take_tools/take_tools.py`.
The input filename is `TAKE_DIR/stereo_imu.db3`. `export-euroc` writes
`OUT_DIR/euroc`, which is always fresh. Normal use never passes
`--allow-diagnostic`. Explicit override syntax is:

```bash
bash calibration/lab/mac/take_to_kalibr_bag.sh \
  /path/to/failed-take /path/to/derived/C1-diagnostic --diagnostic
```

That prints a loud warning and records the override. It may include rejected
data; do not use it to manufacture acceptable evidence. Read `euroc/audit.json`,
`audit.md`, `manifest.json`, raw IMU CSVs and discarded-boundary/interpolation
counts before fitting. The exporter holds decoded frames in memory; long-take
Mac RAM consumption remains UNKNOWN.

The helper validates the exact EuRoC SI header, strictly increasing canonical
integer nanoseconds, finite IMU values and image filenames. It creates
`OUT_DIR/bagcreater/cam0/<ns>.png`, `cam1/<ns>.png` as **hard links inside the
mount**, and `imu0.csv`. The pinned positional bagcreater skips one header, then
reads `ns,omega_x,omega_y,omega_z,alpha_x,alpha_y,alpha_z` in rad/s and m/s².
It creates `/cam0/image_raw`, `/cam1/image_raw`, `/imu0`, confirmed from source.
Combined IMU is the exporter's gyro grid with linearly interpolated accel;
this helper adds no interpolation, shift, axis rotation or unit scaling.
The pinned source slices timestamps into seconds plus the final nine digits;
stamps below 1e9 ns are refused rather than changing their epoch.
Hard-linked PNGs must be treated as immutable: editing either path edits both.

Outputs: `take.bag`, `euroc/`, `bagcreater/`, `export.log`, `kalibr.log`,
`docker-image.txt`, and `README.txt` with exact shell-quoted commands, UTC times,
input/tool/config/output hashes and diagnostic mode. Failures leave all partial
outputs. Select a new OUT_DIR for every retry; no automatic cleanup is performed.

### 4. On the Mac: fixed-factory camera–IMU calibration

```bash
bash calibration/lab/mac/run_kalibr_camimu.sh \
  /path/to/derived/C1-bag/take.bag \
  /path/to/session/measured-aprilgrid.yaml /path/to/derived/C1-imucam
# Optional final argument: an independently measured flat Kalibr IMU noise YAML.
```

Repeat independently for C2 and C3. Defaults use the ASSUMED provisional IMU
YAML; custom values must carry their own noise/unit provenance. The command uses
`--cams` (plural), `--imu-models calibrated`, `--target`, `--bag`, and
`--dont-show-report`. No `--no-time-calibration` or camera-chain recomputation
flag is supplied: time estimation is ON and factory camera geometry stays fixed.

Each OUT_DIR stages immutable copies at `work/take.bag`, `target.yaml`,
`factory-camchain.yaml`, `imu.yaml`. Kalibr writes beside the staged bag; copies
of `take-report-imucam.pdf`, `take-results-imucam.txt`,
`take-camchain-imucam.yaml` and `take-imu.yaml` are collected at OUT_DIR root.
`kalibr.log`, `docker-image.txt`, `README.txt` and `summary.txt` preserve receipts.
Failed fits retain their logs/partial results for inspection.

PR #3 `tools/calib_tools/calib_convert.py` is invoked with `--target orb --fps 30
--units-si`. Outputs go into `orbslam3/`, including `orbslam3.yaml` and
`provenance.json`; conversion log is `convert.log`. These are **OPEN candidates**
for review. The converter emits PinHole settings while the current operating
candidate is Rectified. No operating YAML is replaced or ORB process executed.
Review the rectification choice before using this candidate on rectified Y8.
The converter assumes the caller's SI/continuous-density attestation; numeric
magnitudes alone cannot prove units. ORB YAML has no time-offset field.

The summary reads transforms/time shifts from the fitted YAML and residual
statistics from results text. `T_cam_imu` maps IMU→camera. ORB
`IMU.T_b_c1 = inverse(cam0.T_cam_imu)`, with translation `-R^T t`, maps
camera→D435i IMU. Compare **that inverse translation** against factory CAD.
`t_imu = t_cam + timeshift_cam_imu`; apply the shift once in a separately reviewed
association layer, never silently in this exporter. Both cameras' shifts are
reported in ms. Missing/unrecognized pixel statistics are UNKNOWN, not zero.

The IMU-camera report prints normalized and pixel residuals; only `[px]` lines
are parsed. CALCULATED radial RMS is `sqrt(mean_norm² + std_norm²)` from Kalibr's
population statistics. It is compared to ≤0.5 px, not the mean alone.

### 5. On the Mac: stereo validation

```bash
bash calibration/lab/mac/run_kalibr_stereo_check.sh \
  /path/to/derived/S-bag/take.bag \
  /path/to/session/measured-aprilgrid.yaml /path/to/derived/S-stereo
```

This uses `--models pinhole-radtan pinhole-radtan --topics /cam0/image_raw
/cam1/image_raw --dont-show-report`. It collects `take-report-cam.pdf`,
`take-results-cam.txt`, `take-camchain.yaml`, logs, staged inputs and receipts.
`summary.txt` compares both fitted cameras' fx/fy/cx/cy, distortion coefficients,
and positive baseline norm against factory. Fitted camera values are
**validation only** and never replace factory. Stereo results contain X/Y mean
and population std; radial RMS is `sqrt(mu_x²+mu_y²+sigma_x²+sigma_y²)`.
Inspect PDF residual patterns, detection failures, coverage and outlier removal
as required by the plan; scalar summaries are insufficient for acceptance.

## Exact Docker invocation — OPEN, not executed during preparation

The locally read `Dockerfile_ros1_20_04` has shell-form `ENTRYPOINT ... /bin/bash`
and its workspace-source line is **commented out**. Appending a quoted rosrun
string to that entrypoint is unreliable. The scripts explicitly override it:

```bash
docker run --rm --pull=never --network none \
  -v "${output_dir}:/data" -e MPLBACKEND=Agg \
  --entrypoint /bin/bash kalibr:noetic-1f60227 -c \
  'export KALIBR_MANUAL_FOCAL_LENGTH_INIT=1; source /catkin_ws/devel/setup.bash && cd /data && rosrun kalibr kalibr_bagcreater --folder /data/bagcreater --output-bag /data/take.bag'
```

Host paths are passed as argv, not injected into the shell string; staged
container paths are fixed. Spaces are supported, but colons, commas and newlines
in mount paths are refused. The image must already exist locally. No script
builds, pulls, installs, uses SSH or changes network configuration.

Runtime command strings for fits are:

```bash
rosrun kalibr kalibr_calibrate_imu_camera --bag /data/work/take.bag --cams /data/work/factory-camchain.yaml --imu /data/work/imu.yaml --target /data/work/target.yaml --imu-models calibrated --dont-show-report
rosrun kalibr kalibr_calibrate_cameras --bag /data/work/take.bag --target /data/work/target.yaml --models pinhole-radtan pinhole-radtan --topics /cam0/image_raw /cam1/image_raw --dont-show-report
```

Manual bag sanity command, later with the reviewer-built image:

```bash
docker run --rm --pull=never --network none \
  -v "${bag_dir}:/data":ro -e MPLBACKEND=Agg \
  --entrypoint /bin/bash kalibr:noetic-1f60227 -c \
  'source /catkin_ws/devel/setup.bash && rosbag info --yaml /data/take.bag'
```

Verify exactly the three topics, expected sensor_msgs types and counts against
the export manifest/CSV rows. Before physical use, independently compare all
bag message/header stamps to original SDK metadata/export integer ns (ASSUMED
tolerance ≤1 µs), stream domains, selected Y8 pixels and IMU units/values.
Bagcreater copies stamps from filenames/CSV by source inspection, but no actual
ROS1 bag was created or decoded here. That round trip remains **OPEN/UNKNOWN**.

## Local verification — tooling only, OPEN physical validation

```bash
python3 -c 'import yaml,numpy'
python3 -m unittest discover -s calibration/lab/tests -v
for script in calibration/lab/mac/*.sh calibration/lab/pi/*.sh; do
  bash -n "$script" || exit
done
python3 calibration/lab/mac/summarize_kalibr.py camimu \
  --chain calibration/lab/tests/fixtures/synthetic-camchain-imucam.yaml \
  --results calibration/lab/tests/fixtures/synthetic-results-imucam.txt
```

Verified on this Mac: NumPy/PyYAML imports; **20 synthetic/offline tests**;
syntax of all **5 shell scripts**. Tests cover nonidentity inverse composition,
origins/basis vectors, factory translation/baseline signs, positive/negative
integer-ns time shifts, pixel versus normalized statistics, both RMS formulas,
missing/invalid results, transform/chain errors, fresh-output protection,
CSV order/SI header/ns precision, hard links, RAM reserve and fault diagnostics.
A synthetic chain was also converted through the actual local PR #3 converter;
its ORB matrix payload was independently read with PyYAML to check inverse signs
and fps. OpenCV/ORB engine loading remains UNKNOWN. Layout tests use clearly
labelled synthetic byte payloads; they do not claim PNG decoding validation.
Test temporary outputs stay inside `calibration/lab/tests/` and are removed.
Receipt helpers were exercised under the Mac's Bash 3.2 in normal/diagnostic
mode; missing-argument checks were exercised without starting external work.

No Pi capture path, real take export, Docker/image inspection, Kalibr optimization,
camera process, calibration/firmware change, sensor capture or ORB execution was
performed. Pi telemetry access, real failure cleanup, Docker entrypoint/runtime,
real reports and physical calibration remain **OPEN**. No commit or push.
