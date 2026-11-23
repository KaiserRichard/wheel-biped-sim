# calib_tools

Convert a two-camera Kalibr spatial/temporal calibration plus IMU noise file into
ORB-SLAM3 settings and OpenVINS calibration files, with hashes and explicit frame,
time and unit provenance. Python 3.11, NumPy and PyYAML; no installs or frameworks.

**SOURCE-SUPPORTED:** the exporter targets ORB-SLAM3 commit
`4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4` and OpenVINS `v2.6.3`.
The ORB checkout and the development settings named in B2 were read only.
**INFERRED:** the validation bounds and operating choices below are policies for
this converter, not evidence that a sensor is calibrated.
**UNKNOWN:** real calibration accuracy, engine loading/execution, image delivery
rates, clock mappings and replay performance. Only synthetic tests were run.

From the repository root:

```sh
python3 tools/calib_tools/calib_convert.py \
  --camchain tools/calib_tools/tests/fixtures/camchain-imucam.yaml \
  --imu tools/calib_tools/tests/fixtures/imu.yaml \
  --output-dir tools/calib_tools/example-output \
  --target both --fps 30 --units-si

python3 -m unittest discover -s tools/calib_tools/tests -v
```

Use your calibration paths for real work. The example inputs are hand-made
synthetic values, never a calibration for a D435i. `--target orb`, `openvins` or
`both` selects outputs; the default is `both`. `--fps` is required for ORB because
Kalibr does not supply the delivered image rate. `--units-si` is a required caller
attestation: translations in metres, timeshifts in seconds, focal lengths and
principal points in pixels, rates in Hz, gyro in rad/s, accel in m/s², and noise
as continuous densities, not per-sample standard deviations or variances.

Outputs are `orbslam3.yaml`, `kalibr_imucam_chain.yaml`,
`kalibr_imu_chain.yaml` as selected, plus `provenance.json`. Existing output
files and input files are never overwritten. All calibration validation and path
checks precede output creation. I/O failure while writing can leave partial
output; use a fresh output directory when retrying. Exit status is 0 on success,
2 on invalid inputs/arguments or I/O errors. Warnings go to stderr.

The sidecar records input/output SHA-256 hashes, converter hash, UTC generation
time, frame conventions, original per-camera timeshifts, integer-ns offsets,
units, bounds, operating choices, source links and warnings. It labels the stamp
clock domain UNKNOWN. It does not perform timestamp arithmetic or establish a
clock mapping. Original offset seconds are preserved; sub-nanosecond offsets
are rounded half-even and their rounding error is reported.

**SOURCE-SUPPORTED — transforms and time:** Kalibr's
[YAML formats wiki](https://github.com/ethz-asl/kalibr/wiki/yaml-formats#camera-system-calibration-camchainyaml)
defines `T_cam_imu` as IMU→camera, and `cam1.T_cn_cnm1` as cam0→cam1.
Here `T_A_B` maps points in B into A. Therefore ORB camera1 corresponds to
Kalibr cam0 and camera2 to cam1:

```text
IMU.T_b_c1     = inverse(cam0.T_cam_imu)       # camera1 -> IMU
Stereo.T_c1_c2 = inverse(cam1.T_cn_cnm1)       # camera2 -> camera1
inverse([R,t]) = [Rᵀ, -Rᵀt]
t_imu = t_cam + timeshift_cam_imu              # Kalibr seconds
```

The wiki's IMU configuration section identifies continuous noise and an Hz
update rate. The [noise model](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model)
gives sampled white-noise std `density / sqrt(dt)` and bias-increment std
`random_walk * sqrt(dt)`. Export densities unchanged. The unit labels are
gyro noise `rad/s/sqrt(Hz)`, accel noise `m/s^2/sqrt(Hz)`, gyro walk
`rad/s^2/sqrt(Hz)` and accel walk `m/s^3/sqrt(Hz)`.

**SOURCE-SUPPORTED — ORB schema (pinned file + lines):**

| Emitted fields / behavior | Inspected upstream source |
|---|---|
| String `File.version: "1.0"` selects Settings | [System.cc L77–79](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/System.cc#L77-L79) |
| `Camera.type: "PinHole"`; `Camera1.fx/fy/cx/cy/k1/k2/p1/p2` | [Settings.cc L184–225](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Settings.cc#L184-L225) |
| `Camera2.fx/fy/cx/cy/k1/k2/p1/p2`; stereo transform, positive norm baseline, `Stereo.ThDepth` | [Settings.cc L276–351](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Settings.cc#L276-L351) |
| Integer `Camera.width/height/fps/RGB` | [Settings.cc L356–411](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Settings.cc#L356-L411) |
| `IMU.T_b_c1`, `IMU.NoiseGyro/NoiseAcc/GyroWalk/AccWalk/Frequency`, optional `IMU.InsertKFsWhenLost` | [Settings.cc L414–431](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Settings.cc#L414-L431) |
| Required ORBextractor and Viewer fields | [Settings.cc L443–469](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Settings.cc#L443-L469) |
| Noise `Ng*sqrt(f), Na*sqrt(f)` and walks `Ngw/sqrt(f), Naw/sqrt(f)` | [Tracking.cc L603–614](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Tracking.cc#L603-L614) |
| Invert stored `Tbc` to `Tcb`; camera pose uses `Tcb*Tbw` | [ImuTypes.cc L395–406](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/ImuTypes.cc#L395-L406), [Frame.cc L465](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Frame.cc#L465) |

**SOURCE-SUPPORTED:** ORB inverts `T_c1_c2` before OpenCV stereoRectify and
adjusts the camera→IMU pose for the rectified camera in
[Settings.cc L485–525](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Settings.cc#L485-L525).
Feed raw images matching Kalibr's intrinsics/distortion; ORB performs
rectification internally. Do not feed previously rectified images with these
raw settings. For a conventional right camera, Kalibr translation is about
`[-b,0,0]`; the exported right-camera center in left coordinates is about
`[+b,0,0]`. General rotations require the full inverse. PinHole does not consume
`Stereo.b`; ORB derives the positive magnitude from the transform.

**SOURCE-SUPPORTED:** the inspected Settings parser has no camera–IMU offset
field. Every ORB conversion prints an association warning and retains both
offsets in the sidecar. Apply the positive/negative offset once to each camera's
integer-ns stamp on a common/mapped IMU timebase, then recheck stereo pairing
and IMU coverage. `both` requires equal offsets for OpenVINS; `orb` preserves
unequal offsets for an external association layer to handle.

**INFERRED operating choices:** `Camera.RGB`, `Stereo.ThDepth`, keyframe-loss
policy, ORBextractor and Viewer values reproduce the inspected development file
`D435i_serial-alias-6_stereo_inertial.yaml` L21–23, L35, L48–62 (archived phase-1 file, not published).
They are separately identified in provenance and are not Kalibr measurements.
Intrinsics/extrinsics/noise are never borrowed from that file. The candidate
itself uses Rectified; this exporter uses the requested PinHole/Camera2 schema.
Matrices are row-major `!!opencv-matrix`, `dt: f`; real-valued settings emit
real scalars and integer settings emit integers, matching the strict parser.

**SOURCE-SUPPORTED — OpenVINS:**
[VioManagerOptions.h L142–152, L205–249](https://github.com/rpng/open_vins/blob/v2.6.3/ov_msckf/src/core/VioManagerOptions.h#L142-L249)
reads Kalibr noise, camera vectors, resolution, cam0 timeshift and extrinsics.
[opencv_yaml_parse.h L448–490](https://github.com/rpng/open_vins/blob/v2.6.3/ov_core/src/utils/opencv_yaml_parse.h#L448-L490)
accepts `T_cam_imu` and inverts it when `T_imu_cam` is requested.
The [external loader L532–560](https://github.com/rpng/open_vins/blob/v2.6.3/ov_core/src/utils/opencv_yaml_parse.h#L532-L560)
uses OpenCV FileStorage and sensor nesting. Outputs preserve input text/values
and comments, add `%YAML:1.0`, remove document markers and wrap a flat IMU file
under `imu0`. Existing `imu0` remains nested. The
[IMU example L1–15](https://github.com/rpng/open_vins/blob/v2.6.3/config/euroc_mav/kalibr_imu_chain.yaml#L1-L15)
confirms that layout. No calibration values are invented.

Provide a separate OpenVINS estimator configuration with
`relative_config_imucam: kalibr_imucam_chain.yaml`,
`relative_config_imu: kalibr_imu_chain.yaml`, and two cameras. Its native offset
handling means images must not also be externally pre-shifted. Unequal camera
offsets are rejected because the loader uses only cam0. Pinhole with radtan or
equidistant is accepted for OpenVINS.
**UNKNOWN:** compatibility with other versions; current master requires more IMU
intrinsic fields ([VioManagerOptions.h L283–290](https://github.com/rpng/open_vins/blob/master/ov_msckf/src/core/VioManagerOptions.h#L283-L290)).
These outputs target v2.6.3 calibration loading, not a complete estimator config.

**INFERRED validation policy:** require cam0/cam1, all requested calibration
fields and four distortion coefficients; accept flat IMU or one nested imu0.
Reject duplicate keys, unsupported models, nonfinite/invalid values, nonpositive
noise/rates/focal lengths, noninteger resolutions, out-of-image principal
points, zero baselines, conflicting inverse extrinsics, inconsistent stereo
chains and unequal ORB image sizes. Rotations require `RᵀR≈I`, `det(R)≈+1`
within absolute `1e-6`; homogeneous bottom row within `1e-9`. Chain closure
requires `T_cam1_imu ≈ T_cam1_cam0 @ T_cam0_imu` within absolute `1e-5`
(rotation dimensionless, translation metres), accommodating rounded YAML.
Matrices are never silently repaired. Equidistant→ORB is an explicit error;
no KannalaBrandt8 mapping is claimed.

Translation norm defaults to a 1 m upper bound on every input transform.
`--max-translation-m` explicitly changes this policy for a larger rig. This
catches common mm-valued stereo translations supplied as metres; it cannot
detect every plausible but mislabelled number. Optional `units` mappings at
document/sensor level must agree with the canonical labels in `UNITS` in the
script. For example `units: {translation: mm}`, `timeshift: ms`, gyro `deg/s`,
accel `g` or per-sample noise units are rejected. Numeric unit suffixes and quoted
numeric strings are rejected; plain scientific notation is accepted.
**UNKNOWN:** unlabelled numerical unit errors or prior discretization
cannot be detected from magnitudes alone; the SI/continuous-density attestation
is mandatory and its limitation is printed and recorded.

**Verified with synthetic tests only:** hand-made `Rz(90°) Rx(0.1 rad)` IMU→cam0,
small `Ry(0.04 rad) Rz(-0.03 rad)` stereo rotation, 50 mm baseline plus transverse
offsets, millimetre-scale IMU translations, and +12.345678 ms time shift.
Tests reparse emitted ORB YAML with an independent minimal OpenCV-format reader
(including float32 matrix storage), check inverse products and transformed
points, right-camera center sign, density preservation across frequency changes,
required scalar types, OpenVINS value preservation, provenance hashes, negative
and sub-ns shifts, CLI behavior and rejection paths. **NOT verified:** any real
Kalibr file, OpenCV's actual parser, ORB/OpenVINS runtime or hardware calibration.

Files: `calib_convert.py`, `README.md`, `tests/test_calib_convert.py`,
`tests/fixtures/camchain-imucam.yaml`, `tests/fixtures/imu.yaml`.
