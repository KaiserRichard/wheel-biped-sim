# Isolated native ORB-SLAM3 camera examples

This runner prepares and builds only. **The maintainer exclusively starts camera
streams/native examples.** No live run, tracking success, depth-unit value, IMU
initialization, runtime shutdown or trajectory accuracy is claimed by this build.

`../native_headless.py` copies only the three existing native example sources,
applies these patches, copies helper headers, and reuses the installed
`flags.make`/`link.txt`. All compiler and linker subprocesses run sequentially
(one compiler, equivalent to `-j1`). It never rebuilds or writes the ORB library,
upstream source, ROS wrapper or calibration. Original copyright/license notices
remain in each private source; helpers/patches are GPL-3.0-or-later.

## Prepared Pi artifacts

- ORB: `<local workspace>/orbslam3/ORB_SLAM3`
- Private build: `<local workspace>/wbr-camera-native-20261021` (exclusively created)
- Tool copy: `<local workspace>/wbr-camera-native-tools-20261021`
- Executables: `bin/stereo_realsense_D435i`, `bin/rgbd_realsense_D435i`,
  `bin/stereo_inertial_realsense_D435i`
- Settings: `settings/{stereo,rgbd,stereo_inertial}-30fps-1250features.yaml`
  and `settings/{stereo,rgbd,stereo_inertial}-15fps-800features.yaml`
- Logs: `logs/patch-*.log`, timestamped `logs/build-*.log` and build receipts.
  Failed logs are retained. `manifest.json` records original source/library and
  evidence SHA256; receipts record final binaries, sources, headers and settings.

Reproduce from a checkout on the Pi, using a **new** destination:

```sh
python3 scripts/camera_day/native_headless.py prepare --destination <local workspace>/wbr-camera-native-reproduction
python3 scripts/camera_day/native_headless.py build --destination <local workspace>/wbr-camera-native-reproduction
```

`prepare` refuses any existing destination. `build` can rebuild private outputs,
always retaining separate logs; its lock prevents two instances of this builder
compiling concurrently. The maintainer must keep other Pi compilers idle.

## maintainer-only execution commands

These are instructions, **not commands run by this runner**. Run each mode as a
separate maintainer-owned session, in order stereo, RGB-D, then stereo-inertial.
Each command binds SDK serial **99005** through its generated YAML.
Runtime output directories must be new; their parent must already exist.
No display server, Xvfb or viewer is used (`System(..., false, ...)` in all three).

```sh
# On the Pi; setup does not launch a camera.
cd <local workspace>/wbr-camera-native-20261021
mkdir -p runs
set -o pipefail

# Stereo session.
run_tag="stereo-$(date -u +%Y%m%dT%H%M%S)"
WBR_OUTPUT="$PWD/runs/$run_tag" WBR_DURATION_S=60 \
  ./bin/stereo_realsense_D435i \
  <local workspace>/orbslam3/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  settings/stereo-30fps-1250features.yaml 2>&1 | tee "logs/$run_tag.log"
```

```sh
# RGB-D session, after the stereo process has exited.
cd <local workspace>/wbr-camera-native-20261021
set -o pipefail
run_tag="rgbd-$(date -u +%Y%m%dT%H%M%S)"
WBR_OUTPUT="$PWD/runs/$run_tag" WBR_DURATION_S=60 \
  ./bin/rgbd_realsense_D435i \
  <local workspace>/orbslam3/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  settings/rgbd-30fps-1250features.yaml 2>&1 | tee "logs/$run_tag.log"
```

```sh
# Stereo-inertial session, after the RGB-D process has exited.
cd <local workspace>/wbr-camera-native-20261021
set -o pipefail
run_tag="stereo-inertial-$(date -u +%Y%m%dT%H%M%S)"
WBR_OUTPUT="$PWD/runs/$run_tag" WBR_DURATION_S=60 \
  ./bin/stereo_inertial_realsense_D435i \
  <local workspace>/orbslam3/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  settings/stereo_inertial-30fps-1250features.yaml 2>&1 | tee "logs/$run_tag.log"
```

For the authorized lower-load variant, substitute that mode's
`15fps-800features.yaml`; source stream FPS and requested ORB features both follow
YAML. Default settings remain 30 FPS/1250 features. The runtime verifies active
stream FPS, and stdout records each active profile. IMU requests are explicitly
200 Hz for both gyro and accelerometer, including in the 15 FPS image variant.
Pure RGB-D requests neither IMU stream.

## Lifecycle and image ownership

SIGINT/SIGTERM handlers only set `sig_atomic_t`. Image waits wake every 50 ms and
check the signal/duration. ORB initialization precedes capture, and cancellation
during initialization prevents starting capture afterward. Runtime defaults are
`WBR_STARTUP_LIMIT_S=120`, `WBR_DURATION_S=60`, `WBR_SHUTDOWN_GRACE_S=30`.

The active duration starts immediately before `pipe.start`, after ORB vocabulary
initialization. SDK startup/profile queries, image waits and processing are in
this interval. When duration/signal stops the loop, capture stops first, then
`System::Shutdown()` is called. The helper waits until LocalMapping and
LoopClosing finish and global bundle adjustment is no longer running; only then
does it export `CameraTrajectory.txt` and `KeyFrameTrajectory.txt` in TUM format.
An empty atlas yields empty files instead of triggering the installed exporter's
`keyframes[0]` access. Export rows are read back for eight finite columns, with a
nonempty keyframe export required when keyframes exist. This is a format/output
check, not trajectory accuracy validation.

This installed ORB library comments out its shutdown wait. `orb_access.hpp` uses
three typed member pointers through explicit template instantiation to inspect
runner completion and the atlas. It does not change ORB headers, class layout or
the library. Its build receipt pins the source/header/library hashes; this is a
compatibility shim for that build, not a portable public ORB API. Upstream
automatic atlas saving is rejected because it runs before runner completion.

An active Track call cannot be safely interrupted. A watchdog enforces a startup
bound or active-duration-plus-grace bound. On overrun it writes
`INCOMPLETE-watchdog.txt` and exits **124**, without pretending to export a
trajectory. A thrown SDK stop writes `INCOMPLETE-stop.txt`, requests ORB shutdown
and exits **1** without unwinding live callback captures. Setup/processing
exceptions otherwise take the stop/shutdown/export path and return **1**.
Only normal duration/signal completion returns **0**. A signal during a stuck
Track is subject to the existing hard deadline; it cannot instantly cancel Track.
Treat missing summary or an INCOMPLETE marker as incomplete, even if CSV exists.

Stereo callbacks clone each incoming image while SDK frames are alive, using
actual frame dimensions/stride. RGB-D keeps owning framesets through alignment,
clones color and converts native Z16 depth using **that frame's read-only SDK
`get_units()`** into float metres (`RGBD.DepthMapFactor: 1.0`). No assumed 0.001
scale is used. The summary records the last observed depth-unit value. This
runner has not queried live units. All example sensor-option writes are removed.

IMU startup waits for real accelerometer brackets. Native SDK XYZ axes are
retained; acceleration is linearly interpolated onto gyro timestamps without
latest-value padding or extrapolation. Pending samples survive image processing,
and frames wait for unified IMU coverage through their image time. The original
unlocked post-start vector clear is removed. See the follow-up below for the
runtime checks and the measured single-run evidence.

## Metrics definitions

`attempts.csv` is flushed before entering every Track call. `frames.csv` contains
one row for every returning or throwing call; a hung/crashed call has an attempt
receipt but no fabricated end/duration/state. Both files use 1-based call indices;
`processed_frame_index` is assigned only to calls that return normally.

| Field/summary | Definition |
| --- | --- |
| source timestamp | Left IR timestamp for stereo/inertial; color timestamp for RGB-D; SDK milliseconds converted to seconds. SDK timestamp domain is retained as its enum value. |
| Track start/end/duration | Steady-clock elapsed seconds from Runtime construction; duration surrounds Track only, excluding image copy/resize/alignment and metric receipt flush. |
| tracking state / tracked | Raw `GetTrackingState()` after Track; tracked means **exactly state 2 (OK)**. No pose-existence proxy. -1/0/1/3/4/5 mean not-ready/no-images/not-initialized/recently-lost/lost/OK_KLT in this build; state 5 is not counted by this declared OK-only metric. |
| IMU initialized | Exact atlas `isImuInitialized()` after Track in inertial mode; `-1` means not applicable/unknown. It can reset with the atlas. |
| processed FPS | Normally returned Track calls divided by the full active interval, including SDK startup and waits, excluding ORB initialization/shutdown. |
| successfully tracked FPS | State-2 returned calls divided by the **same** interval. Neither FPS is inferred from source timestamps. |
| startup frames | Non-OK returned calls before the first state-2 call. |
| untracked after established | Non-OK returned calls after first state 2, including reinitialization. |
| loss transitions | State 2 -> non-OK transitions after tracking has been established; sustained loss is one transition, recovery then loss adds another. |
| lost-state frames | Calls specifically reporting state 3 or 4; separate from startup/reset frames. |
| source delta / frame-number delta | Difference between source timestamps/frame numbers of processed inputs. These can reflect skipped application inputs or source discontinuities; they are **not camera-drop counts**. First deltas are blank. |
| processed-start delta | Steady-clock start-to-start cadence; separate from source delta and Track duration. |
| callback overwrites | Unique received images replaced in the single latest-image slot before this input was consumed. This measures application-buffer overwrites only. |
| requested FPS / features | Values actually read from YAML, not measured throughput or observed feature detections. |
| failed Track calls | Throws recorded with `track_completed=0`, unknown state/IMU `-1`, excluded from processed/tracked counts. |

The camera TUM exporter rebases its camera poses to the first keyframe;
the keyframe TUM exporter retains the native map frame. The files therefore need
not have the same origin. Multi-map/reset behavior is the installed ORB exporter
behavior; these files are not robot-body poses and no world-frame continuity is
claimed across resets.

## Device-specific inertial settings

Input: read-only
`<local workspace>/wbr-evidence/camera-day-20261021-01/rs-enumerate.txt`, SHA256
`567c08f57953aef175577550314cfe37ff0b704c54a0a641c6eabdc8dd57baef`, and the
same evidence directory's current `stereo.yaml`/`rgbd-ros.yaml`.

SDK IR1 -> gyro/accelerometer extrinsics (readback lines 619–633) give:

```text
p_b = T_b_c1 * p_c1
T_b_c1 = [ 1 0 0 -0.00552000012248755
           0 1 0  0.00510000018402934
           0 0 1  0.011739999987185
           0 0 0  1 ]
```

Translation is metres. This is **camera to IMU**, as confirmed by the example's
`cam_left.get_extrinsics_to(imu_stream)`, Settings.cc loading `IMU.T_b_c1` as
Tbc, ImuTypes.cc inverting it to Tcb and Frame.cc composing `Tcw=Tcb*Tbw`.
The upstream YAML's opposite-direction comment is misleading. SDK printed rows
are parsed as rows; SDK's in-memory rotation array is column-major. Gyro and
accelerometer extrinsics agree; raw SDK motion XYZ is not permuted.

NoiseGyro `1e-3`, NoiseAcc `1e-2`, GyroWalk `1e-6`, AccWalk `1e-4` are copied
from the upstream example **as explicitly labeled assumptions**, not measured
device noise. SDK readback reports zero noise/bias variances and does not supply
a useful device noise calibration. No additional live query is required for the
transform; no other device's factory transform is copied.

## Targeted offline tests

```sh
python3 -m unittest discover -s scripts/camera_day/native -p 'test_*.py' -v
# Include patch application to saved, unmodified source snapshots:
WBR_ORIGINAL_EXAMPLES=/private/tmp/wbr-camera-native-readback-20261021 \
  python3 -m unittest discover -s scripts/camera_day/native -p 'test_*.py' -v
```

Tests cover metric counts/loss/recovery, transform direction with nonidentity
rotation, source provenance, exclusive output, compiler/linker output isolation,
and applying all patches while preserving notices. One tiny host C++ metric test
uses only the standard library. No test starts ORB or RealSense.

## Stereo-inertial interpolation follow-up

`prepare` now applies `stereo_inertial_unified.patch` after the headless patch.
The upstream example's interpolation result was overwritten by the current
acceleration, and image callbacks padded unpaired gyro samples with the latest
acceleration. The follow-up waits for two bracketing accelerometer samples and
linearly interpolates acceleration at each gyro timestamp; pending gyro samples
survive image processing. This is native SDK unification, not a ROS
`unite_imu_method` parameter. Frames wait for unified IMU coverage through the
image time. No noise assumption or ORB initialization threshold is changed.

The native run verifies configured `IMU.T_b_c1` against SDK IR1→Gyro extrinsics
(body-from-camera, metres) and requires `IMU.Frequency=200` alongside active
200 Hz gyro/accelerometer profiles. Raw `imu_raw.csv` stores SDK gyro rad/s and
accelerometer m/s² with source timestamps for measured-rate checks afterward.
Successful checks are configuration evidence; they do not establish sustained
inertial tracking. Keep the mount fixed during the initial 10–15 seconds of
owner-performed excitation and record which axes were physically possible.

The upstream IMU-only `offset` (milliseconds, default0) is applied before
unification. Raw CSV timestamps therefore include this offset; the recorded
accepted run used0. Timestamp faults in the SDK callback set an atomic flag and
wake the main loop; the main thread handles the error, stops the SDK and exits
nonzero with `stop_reason=processing_error`. The flag is checked again after
SDK stop, so a fault during the final callback cannot be reported as success.
