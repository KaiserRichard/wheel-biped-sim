# Q1 / C4 report — 2026-10-14

MEASURED: completed the Mac-testable native adapter, scripted fake engine,
optional unbuilt ORB binding and real-export association replay. All authored
files and derived logs are under `tools/vio_adapter/`. Nothing is a gate PASS.

## Built

- `adapter.h/.cpp`: C++17 library with integer-ns/domain stamps, owned Y8 images,
  exact stereo pairing, finite/order/gap/queue rejection counters, accel-on-gyro
  interpolation, prefix/tail accounting and trailing IMU coverage. It adapts the
  first-party `d435i_input.h` algorithm with attribution in [SOURCES.md](SOURCES.md).
- Engine interface and value `EstimateSnapshot`: source/sequence, five states,
  reset counter, local epoch, optional map ID, named SE(3), validity/reasons and
  explicit covariance availability. Heartbeats preserve epoch/sequence; stale
  engine results cannot replace them. Reset and reported map changes invalidate.
- `fake_engine.h`: scripted initialization → tracking → loss → reset/new map →
  tracking. Its translations, map IDs and states are deliberately synthetic.
- `orb_engine.h/.cpp`: small optional binding, included only with
  `WBR_WITH_ORBSLAM3=1` / `-DWBR_WITH_ORBSLAM3` and explicit include/lib flags.
  Inverts returned Tcw, maps states, observes map changes and owned reset requests.
  Optional copied/locked map metadata hook supports ID/big-change/reset-generation
  observation when an engine extension provides it. This binding was not compiled.
- `euroc_replay.py` + `replay_fake.cpp`: stdlib PNG decode and binary event driver
  into the actual C++ adapter, with per-event domains and decoded pixels. Reads
  raw accel/gyro, not the already combined IMU derivative. Emits snapshots and
  count/hash provenance. Makefile builds `libvio_adapter.a` and fake replay.

SOURCE-SUPPORTED: pinned ORB API references, direction/endpoint semantics and the
private-Atlas limitation are linked with lines in [SOURCES.md](SOURCES.md).
INFERRED: boundary invalidation, fixed frame names and the reinitialization
fallback are implementation policies, not accepted navigation semantics.

## Exact checks and outcomes

From the repository root:

```sh
make -C tools/vio_adapter test
make -C tools/vio_adapter all
clang++ -std=c++17 -O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer -Itools/vio_adapter tools/vio_adapter/adapter.cpp tools/vio_adapter/tests/test_adapter.cpp -o tools/vio_adapter/build/test_adapter_sanitized
./tools/vio_adapter/build/test_adapter_sanitized
python3 tools/vio_adapter/euroc_replay.py <local workspace>/Desktop/Research/wbr-data/2026-10-14-d435i-ramtest15s/euroc tools/vio_adapter/real_replay
```

MEASURED: `make test` exit 0: **11 C++ synthetic fixture groups and 4 Python
unittest cases passed**. Build uses Apple clang C++17 with warnings enabled; no
warnings remained. `make all` exit 0. Sanitized C++ fixtures exit 0 with no ASan or
UBSan diagnostics. Real replay exit 0. Output directory must be new for replay;
use another name to reproduce without overwriting the retained evidence.

MEASURED: fixtures cover the reference interpolation/prefix/endpoint/bracket
intent; invalid vectors/order/domains/development gaps; bounded IMU/image queues;
exact stereo pairing and atomic pair validation; initialization/loss/reset/new
map; stale engine epoch/domain; heartbeat freshness invariants; named frames;
nonidentity pose inverse; malformed transform; engine exception; covariance
availability; large integer epochs; PNG filter/CRC checks; binary truncation;
source order/domain refusal and synthetic export immutability.

MEASURED: `make -C tools/vio_adapter -n WBR_WITH_ORBSLAM3=1` exit 2 as expected,
refusing absent `ORB_INCLUDE_FLAGS`. This tests the configuration guard only,
not ORB compilation. Source selection excludes `orb_engine.cpp` in default builds.

## Real export association

MEASURED: input export was read only at
`<local workspace>/Desktop/Research/wbr-data/2026-10-14-d435i-ramtest15s/euroc`.
The manifest and all camera metadata declare `global_time`; no clock conversion
or camera–IMU timeshift was applied. Input hashes are retained in
[summary.json](real_replay/summary.json) and were rechecked after replay.
Selected images were decoded as grayscale Y8 and sent to C++; raw camera/IMU
files were not copied into the repository.

| Quantity | Measured count |
|---|---:|
| Exported images per camera | 447 |
| Selected by times_ns per camera | 445 |
| Images per camera excluded by times file | 2 |
| Raw accel accepted | 1,507 |
| Raw gyro accepted | 2,971 |
| Stereo pairs processed / snapshots | 445 / 445 |
| Interpolated gyro epochs supplied to engine | 2,967 |
| Gyro remaining after final selected image | 4 |
| Accel retained at EOF, including interpolation anchors | 4 |
| Domain/order/nonfinite/gap/overflow/image rejection | 0 each |
| Unmatched/prefix gyro/prefix pair/missing coverage | 0 each |
| Engine-result rejection | 0 |

MEASURED: all 445 source epochs and sequences increase strictly. Source range:
`1791950284553748047` through `1791950299357908936` ns, `global_time`.
The real export's first two camera epochs precede gyro coverage and were excluded
by the supplied times file; they are selections excluded before adapter ingestion,
not unreported adapter rejects. Four trailing gyro samples remain explicit EOF
tail exclusions. No sample was counted as a pose merely to conceal missing coverage.

MEASURED: [snapshots.jsonl](real_replay/snapshots.jsonl) contains 445 adapter
snapshots. **SYNTHETIC**: 442 contain fake valid poses; the initial, lost and
reset/map-boundary snapshots are invalid. The script changes map 1→2 once,
increments reset/epoch counters once, and never supplies covariance. Those
states/poses/maps provide no evidence about the physical camera or ORB.

## Verified versus not verified; open issues

MEASURED / verified: native C++ build, synthetic snapshot/association semantics,
real-file decoding and exact association/counts on this one short export, immutable
input metadata/CSV hashes, and retained monotonic fake snapshot output.

UNKNOWN / NOT verified: ORB compilation/linking/execution, D435i VIO accuracy,
calibration, inertial initialization, loop closure/relocalization behavior,
actual reset completion, physical camera–IMU offset, live association/thread
scheduling, real-time performance, hardware clock mapping/source age, Pi resource
use, ROS/STM32 integration, robot body pose and any hardware/navigation gate.

Open issues:

1. Stock System keeps Atlas private; map ID is unavailable without the reviewed
   locked metadata extension. MapChanged is a big-change notification using a
   shared static index, not a complete reset log. Sole-consumer/single-System
   use is required without the extension; internal map switches/resets that
   preserve state or lower the index can remain invisible. The reinitialization
   fallback is conservative and INFERRED. Production reset observation remains
   unresolved until the extension and retained real trials exist.
2. Owned reset invalidates immediately; stock ResetActiveMap is serviced at the
   next TrackStereo. Independent reset completion acknowledgement is unavailable.
3. Library calls are synchronous and must be serialized by the caller; no live
   runner/IPC/output bridge or qualified health/age policy is provided. The 50 ms
   gap guard is inherited development behavior, not an approved timing budget.
4. Exact pair epochs are intentional; shifted/skewed stereo and other datasets
   need explicit reviewed preprocessing. Raw units/calibration and rectification
   remain caller responsibilities. The fixed camera frame is not base_link/odom.

No SSH, Pi/camera access, installations, package downloads, Git commands, writes
to the reference checkout/data, or edits outside the assigned directory occurred.
