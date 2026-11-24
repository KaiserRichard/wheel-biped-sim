# Native VIO adapter — Q1 / C4

C++17 engine interface, timestamp/stereo/IMU association, value snapshots,
scripted fake engine, optional ORB-SLAM3 binding and a stdlib Python EuRoC replay
driver. All implementation and generated evidence stay in this directory.
This is a proposed diagnostic boundary; nothing here is a gate PASS.

```sh
make -C tools/vio_adapter test
python3 tools/vio_adapter/euroc_replay.py \
  <local workspace>/Desktop/Research/wbr-data/2026-10-14-d435i-ramtest15s/euroc \
  tools/vio_adapter/NEW_REPLAY_OUTPUT
```

Replay requires a **new** output directory outside the source export. It reads
`manifest.json`, `stamps.csv`, `times_ns.txt`, camera `data.csv` and grayscale
PNG files, and **raw** accel/gyro CSVs. It validates each source's order and domain
before merging streams; it does not consume the export's already interpolated
`imu0/data.csv`. All selected pixels are decoded and supplied as Y8 to C++.
PNG CRCs and filters 0–4 are supported; interlaced/color/non-Y8 files are refused.
Outputs are `snapshots.jsonl`, `adapter_counts.json` and `summary.json`, with input
hashes and a decoded-pixel hash. Fake translations/states/maps are synthetic,
including when inputs come from a real recording. There is no trajectory accuracy
or latency claim. Raw data are never copied into this repository.

MEASURED: `real_replay/` retains one completed association run. See [REPORT.md](REPORT.md)
for exact counts, commands and limitations; [SOURCES.md](SOURCES.md) for pinned API
citations and attribution.

## Library contract

Include `adapter.h`; link `build/libvio_adapter.a`. Construct `Adapter(engine,
clock_domain)` with a caller-owned engine. One owner serializes all feed, reset,
heartbeat and snapshot access. This synchronous library creates no runner threads;
a future runtime can wrap it with one serialized tracking runner.

`feed_stereo` validates an exact epoch/domain pair before either image is enqueued.
Alternatively `feed_left`/`feed_right` perform bounded exact-epoch association;
when an earlier unmatched image cannot meet the other ordered stream it is
counted and dropped. Images own contiguous `width*height` Y8 bytes, max 8192 per
axis. Feed `Motion` via `feed_accel`/`feed_gyro`; SI values are caller-supplied
(accel m/s², gyro rad/s). Return false means input rejected; inspect counters.

All stamps carry integer ns and a domain. The adapter accepts nonnegative stamps
in its single configured domain and strictly increasing stamps per stream. No
cross-domain subtraction occurs. A different clock/session needs a new Adapter;
`reset()` never rebinds clocks or clears timestamp watermarks. Default bounds are
4096 samples per IMU stream, 8 images per side and a 50 ms per-stream IMU gap
**development guard**, not qualified N2 limits. Queues reject overflow; ordering,
nonfinite vectors, gaps, bad images and domain errors have separate counters.
A rejected gap does not advance the stream watermark: resuming after a clock/gap
fault needs a new input session or deliberate caller recovery; do not conceal it
by sorting or restamping. Rejection counters do not themselves qualify health.

Accel interpolates onto gyro epochs only inside strictly ordered brackets.
No extrapolation occurs. Processing waits for the first gyro at/after the image
and its trailing accel bracket. Gyro before the first accel is a counted prefix;
initial images before first gyro are counted prefix pairs. Each used gyro endpoint
is sent once. `finish()` classifies pending matching pairs as missing coverage,
unmatched images separately, and retained IMU tails. Accel tail includes retained
interpolation anchors; it is not a disjoint count of all unused accel observations.
Further feed/reset after finish raises. Reset discards pending queues with counts.

`EstimateSnapshot` is returned by value, including source epoch/domain, sequence,
tracking enum, reset counter, local epoch ID, optional engine map ID, validity and
reason bitmask, named pose and covariance availability. Local sequence/reset/epoch
IDs belong to this object lifetime; an integrator must add a process/session ID.
`T_world_camera` maps `camera_left_optical` into `slam_world`, with row-major SE(3)
(rotation proper/orthonormal; translation metres). Names are enforced exactly.
World axes/alignment, robot mount/body pose and continuous `odom` are not inferred.
Rectification conventions must match the selected ORB settings.

Accepted engine results must echo the current input epoch/domain exactly and
increase the last published source epoch. Stale/mismatched engine output changes
health to FAULT and invalidates pose without advancing source or sequence. A
fresh engine result advances sequence even during initialization/loss; these
invalid snapshots contain no pose. Frame/pose errors invalidate the fresh result.
A reported discontinuity or changed known map ID increments reset and epoch
counters once and invalidates that result; a later fresh tracking result can
be valid in the new epoch. `reset()` immediately increments counters and clears
pose validity, preserves source/sequence, and requests engine reset. Reset or
tracking exceptions expose FAULT/ENGINE_FAULT. `heartbeat()` returns the same
snapshot; it never renews source time or sequence. Callers enforce age separately.

Covariance defaults to UNAVAILABLE. Fake and ORB supply none. An engine can supply
an explicit finite 6×6 covariance in the documented world tangent convention;
there is no synthesized confidence or zero covariance default. Finite covariance
alone is not a qualification of its model. The optional synchronous `on_snapshot`
observer receives each processed result; callbacks must not reenter the adapter.

## Optional ORB binding (not compiled on this Mac)

```sh
make -C tools/vio_adapter WBR_WITH_ORBSLAM3=1 \
  ORB_INCLUDE_FLAGS='-I/your/ORB/include -I/your/ORB/include/CameraModels -I/your/Eigen -I/your/Sophus -I/your/OpenCV' \
  ORB_LIB_FLAGS='-L/your/ORB/lib -lORB_SLAM3 OTHER_REQUIRED_LIBRARIES'
```

Replace paths and transitive link flags with those of an existing qualified build;
this task installs/builds no ORB dependency. Make uses `-DWBR_WITH_ORBSLAM3` and
requires both include and library flags; optional objects use `build/orb/` to avoid
mixing with the default fake-only objects. `orb_engine.cpp` is excluded otherwise,
and refuses compilation without the macro. The replay executable always uses the
fake engine, even in the optional configuration.

`OrbEngine` borrows an IMU_STEREO `System`. Owner controls construction/settings,
all access and Shutdown. The small binding calls TrackStereo, inverts camera-from-
world to world-from-camera, maps tracking enums and reports MapChanged boundaries.
It uses a common local time origin for camera and IMU before ns→seconds conversion;
source epochs remain exact integer ns. No camera–IMU timeshift is guessed/applied.

Stock System keeps Atlas private. Map ID therefore remains unavailable unless a
reviewed **copied, locked** `OrbMapObserver` extension supplies current map ID,
`GetLastBigChangeIdx` and reset generation. With that extension, metadata changes
supply boundaries independently of stock MapChanged's shared static counter.
Without it, use a single System/sole MapChanged consumer; loop/global BA
notifications and owned resets are observed. A transition from TRACKING/LOST to
STARTING/INITIALIZING is an **INFERRED conservative** additional boundary.
Internal map switches/resets that preserve state or lower the big-change index
may remain invisible. Reset is requested now and serviced by the next TrackStereo;
completion is not independently acknowledged by stock System. A locked extension
and real loss/reset/map trials remain necessary before production use.
