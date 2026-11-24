# take_tools — B1 (C0 + C1), B4 replay times

Inspect, audit and export the four D435i recorder streams with Python 3.11 and
stdlib only. Tool version: **0.1.0**. All implementation files are in this directory.

**SOURCE-SUPPORTED:** the supported format profile is **librealsense v2.58.1**,
using the pinned tag links below, not mutable master. This does **not** identify
the Pi's installed build. v2.56.5's [ROS 1 writer, lines 22–29](https://github.com/realsenseai/librealsense/blob/v2.56.5/src/media/ros/ros_writer.cpp#L22-L29)
opens a ROS 1 bag. Format is detected from magic regardless of filename.

**UNKNOWN:** compatibility with real recordings, the actual Pi writer/version,
compression, device clocks, camera–IMU offset, calibration, and estimator replay.
No real take was available or tested. An audit PASS means these implemented
structural checks pass; it is not a hardware qualification or VIO gate PASS.

## Usage

Run from the repository root:

```sh
python3 tools/take_tools/take_tools.py inspect stereo_imu.db3
python3 tools/take_tools/take_tools.py audit stereo_imu.db3
python3 tools/take_tools/take_tools.py audit stereo_imu.db3 --json audit.json --markdown audit.md
python3 tools/take_tools/take_tools.py export-euroc stereo_imu.db3 NEW_OUTPUT_DIRECTORY
python3 tools/take_tools/take_tools.py export-euroc stereo_imu.db3 NEW_DIAGNOSTIC_DIRECTORY --allow-diagnostic
python3 -m unittest discover -s tools/take_tools/tests -v
```

`inspect` prints JSON with table definitions, every topic's name/type/CDR/QoS,
counts and min/max writer timestamps. SQLite detection alone does not establish
a supported schema. `audit` prints JSON to stdout and a short Markdown summary
to stderr, or writes the two explicit destinations. Audit destinations cannot
overwrite the input or each other. Export requires a new output directory.

Exit codes: **0** successful inspection/audit/export, **1** completed failed audit,
**2** malformed/unsupported selected message or refused export/output,
**3** ROS 1 detected-but-unsupported or unknown magic/unsupported inspected schema.
`--allow-diagnostic` permits exports after audit rejection and labels the
manifest **DIAGNOSTIC**; it does not bypass decoder/type/encoding errors.

Use a finalized, immutable single SQLite file. Reading is immutable/read-only;
a nonempty WAL is refused because hashing only the main file would omit data.
No ROS installation, network install, external image library or SDK is needed.

## Pinned upstream evidence and layout

All line numbers below are GitHub's one-based numbers. These are the
**SOURCE-SUPPORTED** references for the synthetic schema and CDR layout:

| Source | Relevant evidence |
|---|---|
| [ros2_writer.cpp, 32–71](https://github.com/realsenseai/librealsense/blob/v2.58.1/src/media/ros2/ros2_writer.cpp#L32-L71) | SQLite storage, topic registration and CDR serialization format. |
| [ros2_writer.h, 57–77](https://github.com/realsenseai/librealsense/blob/v2.58.1/src/media/ros2/ros2_writer.h#L57-L77) | DDS CDR encapsulation; bag stamp uses writer timestamp argument. |
| [ros2_writer.cpp, 124–195](https://github.com/realsenseai/librealsense/blob/v2.58.1/src/media/ros2/ros2_writer.cpp#L124-L195) | Image and partial Imu; header uses writer stamp; gyro and accel fill separate vectors. |
| [ros2_writer.cpp, 240–270](https://github.com/realsenseai/librealsense/blob/v2.58.1/src/media/ros2/ros2_writer.cpp#L240-L270) | Per-frame String metadata: frame number, SDK timestamp/domain, system time; additional metadata retained. |
| [ros2_file_format.h, 125–146, 175–244](https://github.com/realsenseai/librealsense/blob/v2.58.1/src/media/ros2/ros2_file_format.h#L125-L146) | Data/metadata topics; `/device_N/sensor_N/Stream_index/{image,imu}/{data,metadata}`. |
| [ros2_file_format.h, 247–254](https://github.com/realsenseai/librealsense/blob/v2.58.1/src/media/ros2/ros2_file_format.h#L247-L254) | String CDR length includes NUL. |
| [ros_common.h, 34–47, 120–126](https://github.com/realsenseai/librealsense/blob/v2.58.1/src/media/ros_common.h#L34-L47) | Y8 becomes `8UC1`; RAW8 becomes `mono8`; exact metadata keys. |
| [sqlite_storage.cpp, 197–213](https://github.com/realsenseai/librealsense/blob/v2.58.1/third-party/realsense-file/rosbag2/rosbag2_storage_default_plugins/src/rosbag2_storage_default_plugins/sqlite/sqlite_storage.cpp#L197-L213) | `topics(id,name,type,serialization_format,offered_qos_profiles)`; `messages(id,topic_id,timestamp,data)` and timestamp index. |
| [Image.cpp, 143–200](https://github.com/realsenseai/librealsense/blob/v2.58.1/src/media/ros2/ros2-msg-types/sensor_msgs/msg/Image.cpp#L143-L200) | Header, height, width, encoding, endian byte, step, byte sequence; four-byte scalar/length alignment. |
| [Imu.cpp, 137–194](https://github.com/realsenseai/librealsense/blob/v2.58.1/src/media/ros2/ros2-msg-types/sensor_msgs/msg/Imu.cpp#L137-L194) | Quaternion, covariance[9], angular vector, covariance[9], accel vector, covariance[9]; eight-byte double alignment. |
| [Header.cpp, 98–125](https://github.com/realsenseai/librealsense/blob/v2.58.1/src/media/ros2/ros2-msg-types/std_msgs/msg/Header.cpp#L98-L125), [Time.cpp, 100–131](https://github.com/realsenseai/librealsense/blob/v2.58.1/src/media/ros2/ros2-msg-types/builtin_interfaces/msg/Time.cpp#L100-L131) | Signed seconds, unsigned nanoseconds, frame ID string. |
| [rs_frame.h, 16–22, 189–203](https://github.com/realsenseai/librealsense/blob/v2.58.1/include/librealsense2/h/rs_frame.h#L189-L203) | SDK timestamp is milliseconds; hardware/system/global domains have different meanings. |
| [to-string.cpp, 9–13, 839–851](https://github.com/realsenseai/librealsense/blob/v2.58.1/src/to-string.cpp#L839-L851), [make-less-screamy.h, 9–20](https://github.com/realsenseai/librealsense/blob/v2.58.1/third-party/rsutils/include/rsutils/string/make-less-screamy.h#L9-L20) | Domain spelling: Hardware Clock, System Time, Global Time. |

The decoder implements plain little- and big-endian XCDR1 (`0001`/`0000`),
aligned relative to the payload after the four-byte encapsulation. It checks
bounds, string terminators, exact message consumption, finite IMU fields and
image stride/size. It accepts only `mono8` and `8UC1` for images. PNG preserves
all pixel bytes; row padding is not an image pixel and is omitted.

Only Infrared_1/Infrared_2 and Gyro_0/Accel_0 data/metadata topics are decoded.
Static/nonframe topics are listed with ignored counts, not interpreted as
calibration. Other frame topics, competing stream topics, multiple devices,
wrong selected types/serialization, compression, and unknown encodings fail
loudly. The decoder does not implement general ROS message introspection.

## Clocks, audit and rejection policy

**INFERRED policy:** export the recorded SDK metadata `timestamp`, converted
from decimal milliseconds directly to integer nanoseconds. Reject fractional
nanoseconds instead of rounding. This preserves the metadata value exactly;
it cannot recover precision already lost by the writer's `std::to_string` of
an SDK double. No SDK/header/system conversion or fitted clock mapping occurs.

Metadata joins use the exact writer stamp and stream. A missing or ambiguous
join is an audit rejection. Messages retain insertion (`messages.id`) order;
input is never sorted to hide reversals. Duplicate/backward stamps are checked
for SDK, header and writer timelines. SDK statistics include domain change
events; intervals at transitions are excluded, and a rate is unavailable when
the domain changes. p99 is the nearest-rank positive interval. Rates use
`(count-1)/duration`; gaps exceed 1.5 nominal periods at 30/200/100 Hz.

Every stamp is separately identified:

- `sdk_ns`: per-frame metadata domain, normalized to `hardware_clock`,
  `system_time` or `global_time`; unrecognized strings stay UNKNOWN.
- `bag_ns` and `header_ns`: writer argument, physical clock UNKNOWN. Their
  equality is checked; they are never treated as device stamps.
- `system_ns`: SDK frame system time, **INFERRED** host system clock. Its
  numeric value is retained separately, never used for pairing/interpolation.

Export requires one common known SDK domain across all four streams. One
device is required so two hardware clocks cannot be mistaken for one clock.
Stereo pairs require unique matching metadata frame numbers and <=1 ms SDK
skew within the same known domain. There is no nearest-neighbor repair or
invented right-camera stamp. A camera's own stamp names its PNG.

Audit failures refuse export before creating the destination. Diagnostic
exports retain independently decoded camera images and all raw records;
missing/duplicate SDK filenames are explicitly rejected/countable, and
`stamps.csv` retains their record identities. Skew/unpaired images remain
visible as diagnostic artifacts. Cross-domain stereo subtraction is refused.
Combined IMU is suppressed if any stream makes the common domain unknown or
mixed, or if IMU metadata/order fails. Diagnostic mode never permits
cross-domain interpolation.

## Export artifacts and derivative

```text
OUT/manifest.json
OUT/audit.json
OUT/audit.md
OUT/stamps.csv
OUT/times_ns.txt
OUT/mav0/cam0/data.csv
OUT/mav0/cam0/data/<SDK_integer_ns>.png
OUT/mav0/cam1/data.csv
OUT/mav0/cam1/data/<SDK_integer_ns>.png
OUT/mav0/imu0/raw_gyro.csv
OUT/mav0/imu0/raw_accel.csv
OUT/mav0/imu0/data.csv
```

**INFERRED derivative policy:** `imu0/data.csv` combines untouched gyro vectors
with accel linearly interpolated on the gyro SDK grid. Adjacent accel brackets
must be strictly increasing and <=15 ms (1.5 nominal accel periods); endpoints
are included, extrapolation is forbidden. No interpolation bridges an accel
gap. Prefix, tail, invalid-bracket and clock/order exclusions are counted
separately. Raw CSVs preserve every decoded gyro/accel vector and SDK/writer/
header/system stamps, frame number, domain and message ID. Axes are unchanged;
no calibration or transform is fabricated or applied.

The manifest records input SHA-256, tool version/source hash, domain and stamp
choice, stream/image/raw/derivative counts, rejection reasons, ignored topic
counts and output SHA-256 hashes. All outputs except the self-referential
manifest are hashed. Input hashing is repeated before writing to detect a
changed main file. PNG uses only `struct`/`zlib` with filter zero and CRCs.
EuRoC-style folders do not establish that an estimator can consume this data
with valid calibration; no `sensor.yaml` values are fabricated.

## ORB-SLAM3 stereo-inertial replay (B4)

**SOURCE-SUPPORTED:** inspected upstream commit
`4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4` (`4452a3c`), file
`Examples/Stereo-Inertial/stereo_inertial_euroc.cc`:

- [Lines 85–96](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/Examples/Stereo-Inertial/stereo_inertial_euroc.cc#L85-L96)
  take a sequence root and a separate times file, loading `mav0` cameras and IMU.
- [Lines 243–264](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/Examples/Stereo-Inertial/stereo_inertial_euroc.cc#L243-L264)
  append each times-file line verbatim to both camera paths plus `.png`, and
  convert its numeric nanoseconds to seconds. Headers or CSV rows are unsuitable.
- [Lines 109–113](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/Examples/Stereo-Inertial/stereo_inertial_euroc.cc#L109-L113)
  advance the IMU cursor while its stamp is <= the first image, then decrement
  it. An image earlier than the first IMU leaves cursor -1.
- [Lines 169–176](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/Examples/Stereo-Inertial/stereo_inertial_euroc.cc#L169-L176)
  advance without an upper-bound check; an image at/after the final IMU can
  exhaust the vector.

**INFERRED export policy:** `times_ns.txt` contains one original integer-ns
stamp per line, with no header, sorted by the already-validated IR1 input order.
Only unique metadata frame-number pairs with identical left/right SDK stamps
and `first_combined_imu_ns < image_ns < last_combined_imu_ns` are included.
Bounds come from emitted `mav0/imu0/data.csv`, not the raw gyro/accel ranges.
Equality at either bound is excluded. A <=1 ms stereo skew can still pass the
existing audit, but any nonzero skew excludes that pair from the times file
because its PNG names differ. Original camera PNGs/CSVs, raw streams, audit
policy and combined IMU derivative are preserved.

`manifest.json` adds `times_file`: path, method, SDK domain label, first/last
combined-IMU stamps, counts and exclusion reasons. Counts partition the IR1
input records into emitted, dropped head, dropped tail, unpaired/mismatched,
clock/order failure, or no combined IMU. The times file is included in output
hashes. An absent combined IMU or failed camera clock/order check yields an
empty list, including in diagnostic mode; a single IMU stamp also has no
strict interior. Export status/exit behavior is unchanged. An empty list
cannot be replayed; inspect the manifest before running the example.

Run the already-built upstream executable with settings appropriate to the
recorded camera/IMU geometry, noise and image size:

```sh
ORB_DIR=/absolute/path/to/ORB_SLAM3
TAKE_DIR=/absolute/path/to/NEW_OUTPUT_DIRECTORY
SETTINGS_FILE=/absolute/path/to/recording_settings.yaml
"$ORB_DIR/Examples/Stereo-Inertial/stereo_inertial_euroc" \
  "$ORB_DIR/Vocabulary/ORBvoc.txt" "$SETTINGS_FILE" \
  "$TAKE_DIR" "$TAKE_DIR/times_ns.txt"
```

**UNKNOWN / NOT verified for B4:** actual ORB execution on exported data and
real-recording replay coverage. Synthetic tests establish file selection and
the cursor bounds, not calibrated VIO or replay accuracy. The earlier real-file
check below predates this times-file addition.

## Verification and limits

**Verified with synthetic tests only:** 19 stdlib unittest cases pass. The
independent fixture serializer creates the cited SQLite schema, 31 pairs of
8x6 images (including row padding), 201 gyro and 99 accel records spanning
about one second, standard CDR and writer metadata. Full round-trip compares
PNG pixels, integer camera/IMU stamps, raw vectors, interpolated values,
manifest counts and every output hash. The valid fixture emits 197 combined
IMU records, dropping two prefix and two tail gyro records.
Its B4 times file emits 29 stereo stamps and excludes one image at each end.
B4 tests cover exact IMU endpoints and one-ns interior stamps, matching PNG
names, the upstream cursor traversal, all images outside coverage, zero/one
combined IMU sample, nonzero stereo skew, missing/mismatched frame-number pairs,
and suppressed lists on invalid clocks/order. Existing round-trip checks also
verify the new file's hash.

Variants cover missing frame/metadata, backward and duplicate stamps, domain
change, mixed/unknown domains, odd IR2 skew, unknown encoding, truncation,
unsupported encapsulation, accel gap, big-endian CDR, mono8, ROS 1 magic,
unknown magic, schema absence, competing topics/devices, WAL, output protection
and the actual CLI entry point. No external packages or network installs were
used. Tests create temporary files and remove them automatically.

**NOT verified / UNKNOWN:** real librealsense CDR round-trip, actual device
metadata, recorder compression/version, real frame-number correspondence,
long-take memory/performance and any physical calibration or replay accuracy.
The current tool holds decoded frames in memory; validate resource use before
processing long/high-resolution takes. The next validation is one finalized
retained take plus the recorder's separate `timestamps.csv`, compared for
counts, frame numbers, domains, stamp precision and selected pixels.

## Real-file check, 2026-10-14 (MEASURED, data kept out of the repo)

Input: 15 s RAM-test take `stereo_imu.db3`, sha256 `d9adff8ae17b4e9f8b89bf6a8a798ca1abfd19bf06575efdd7ea1befe05ea1fe`,
written by librealsense **2.58.4** on the Pi (SQLite rosbag2, CDR; per the Pi thread's direct inspection).

- `inspect`, `audit` and `export-euroc` all exit 0. Audit: IR1/IR2 447 frames each at 29.992 Hz, gyro 2971 at
  200.159 Hz, accel 1507 at 100.978 Hz; one domain (`global_time`), no gaps > 1.5 periods, no backward or
  duplicate stamps; 447 stereo pairs by frame number with 0 ns skew.
- Independent cross-check against the recorder's own `timestamps.csv` (callback log): every CSV stamp
  (445 IR per camera, 2971 gyro, 1506 accel) is present in the export within 1 us. The bag has 2 extra
  leading IR frames per camera and 1 extra trailing accel sample (INFERRED: the file writer starts before
  and stops after the CSV callback).
- Pixels: 9 of 9 sampled exported PNGs decode to byte-identical 640x480 Y8 data found in the bag.

Still NOT verified: other librealsense versions, compressed recordings, longer takes, and anything about
clock mapping, camera–IMU offset or calibration.
