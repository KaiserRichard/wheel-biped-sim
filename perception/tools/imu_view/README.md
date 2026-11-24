# IMU-driven 3D WBR view

Tilt the IMU sensor (IMU sensor) by hand and a 3D wheel-bipedal robot on the Mac tilts the same way.
It is the "teapot" demo, but with our own complementary filter and the WBR shape.
**Visualisation only:** no balance, no dynamics, no control claims.

- Solid robot: complementary filter.
- Blue ghost (left): accelerometer only.
- Orange ghost (right): gyro integration only.
- Overlay: pitch/roll of all three, plus a Madgwick filter (numbers only) and the gyro bias.

Theory: [docs/imu_view/LESSON.md](https://github.com/KaiserRichard/wbr-perception-phase1-runtime/blob/fde5721af0627eaf60c78d95296ab26924425e37/docs/imu_view/LESSON.md).

## Run without hardware

The sim venv is `./.venv` (has `mjpython`, MuJoCo 3.3.7, Pillow).
Run from the repo root.

```bash
./.venv/bin/mjpython -m tools.imu_view.view --fake
```

```bash
./.venv/bin/mjpython -m tools.imu_view.view --replay run.csv
```

```bash
./.venv/bin/python -m tools.imu_view.view --fake --record out/ --seconds 10 --calib-s 0
```

- macOS needs `mjpython` for the window; `--record` is headless and uses plain `python`.
- `--fake` is SIMULATED: pitch ±20° at 0.25 Hz, roll ±10° at 0.1 Hz, plus the gyro bias MEASURED on
  29 Oct (−3.25, +0.02, −0.18) °/s.
- Tests: `python3 tools/test_support/run_unittest.py tools/imu_view` (19 tests, no MuJoCo needed).

Evidence (SIMULATED, fake mode, `--calib-s 0`): `results/2026-10-30-fake-nocalib/`.
It holds a 10 s GIF, frames at t = 1, 5 and 9.5 s, and a side view at t = 1 s (pitch +20°, nose down). The orange gyro-only ghost rolls away
(−34° roll at 9.5 s, from the −3.25 °/s x bias). The solid robot stays within ~2° of the truth, offset by bias × τ ≈ 1.6°.

## Options that matter

| Flag | Default | Meaning |
|---|---|---|
| `--tau` | 0.5 s | Complementary crossover; α = τ/(τ+dt) = 0.980 at 100 Hz |
| `--calib-s` | 2.0 s | Hold still at start; the mean gyro is subtracted as bias. `0` = off (shows drift) |
| `--axes` | `x,y,z` | Robot x,y,z expressed in board axes; e.g. `x,-y,-z` for a board mounted upside down |
| `--save run.csv` | off | Record raw samples for `--replay` |
| `--camera` | `demo` | `side` looks along +y (ghosts hidden): nose-down tilts the right-hand end down |
| `--no-ghosts` | off | Solid robot only, for a clean report video |

## Live: what the owner does on the bench

The bench firmware `imu_can_bench` (PR #155) prints `I,ms,fresh,ax,ay,az,temp,gx,gy,gz` at 100 Hz
(±4 g, ±500 dps) on **USART1 PA9/PA10 → Pi `/dev/ttyAMA0`**, 115200 8N1. It does **not**
print on the ST-LINK USB port (that is USART2), so a USB cable to the Mac would show nothing.

**Transport choice: the Pi forwards the UART to the Mac over SSH.**
It needs no firmware change, no rewiring and no new server: `ssh … cat /dev/ttyAMA0` piped into
the viewer. The Pi load is one `cat` process (~1 kB/s). Making the Mac read USB directly would
need a second image that prints on USART2 plus a new flash, and the bench wiring would differ from the
documented one. Rejected for now.

Prerequisites:

- The LED end-to-end replay (`~/e2e_led_replay_run.log` on the Pi) is finished.
- Nobody else (course-projects session) is using the Pi UART.

Steps:

1. **Flash imu_can_bench** (only the owner can do the physical part). On the Pi, in `~/imu-can-bench/`
   (layout as in `tools/imu_can_bench/README.md`):
   - `pkill -f "micro_ros_agent serial"`
   - Fit the **BOOT0 jumper first**, then press **RESET**.
   - `./flash.sh imu_can_bench.bin <sha256 from SHA256SUMS> ~/wbr-evidence/<dir>/flash`
   - Remove BOOT0, press RESET again. Green LED D4 = IMU ok.
2. **Board pose:** lay the IMU sensor flat, chip side up, with its printed **X arrow pointing to the robot's
   front** (away from you). Then the default `--axes x,y,z` applies (Y arrow = left, Z = up).
3. **Start the viewer on the Mac** (repo root, command below), and keep the board **still for 2 s**
   (bias calibration).

```bash
ssh pi-host 'stty -F /dev/ttyAMA0 115200 raw -echo cs8 -parenb -cstopb && cat /dev/ttyAMA0' | ./.venv/bin/mjpython -m tools.imu_view.view --stdin --save ~/imu_view_run.csv
```

`cs8 -parenb -cstopb` forces 8N1. The flash tools leave the port at 8E1 (the bootloader's parity),
and plain `raw` keeps that, which garbles the text. Garbled lines are skipped and counted in the overlay
("UART: N bad lines").

4. **Axis check** (do this before filming). Each line is one physical move, and the expected result:

   | You do | Model must do | Overlay |
   |---|---|---|
   | Hold flat | Upright, parallel to the grey post | pitch ≈ 0, roll ≈ 0 (±2°) |
   | Tip the **X-arrow end down** (nose down) | Nose (red ball) goes down | pitch **positive** |
   | Tip the **left edge (Y arrow) up** | Left wheel lifts | roll **positive** |
   | Hold tilted still 5 s | Solid robot stays put; orange ghost creeps | — |

   What wrong signs look like:

   - **Nose down → model nose up:** pitch sign flipped. Typically the board is turned 180° about Z,
     so X points backwards. Rotate the board, or run `--axes -x,-y,z`.
   - **Nose down → model rolls sideways:** X and Y are swapped (board rotated 90°). Use `--axes y,-x,z`
     or `-y,x,z`, whichever makes the pitch row correct.
   - **Model flips upside down when held flat (pitch ≈ ±180° or roll ≈ ±180°):** chip side down.
     Use `--axes x,-y,-z`.
   - **Ghosts agree but the solid robot lags or overshoots:** that is not a sign error. Check `--tau`.
   - **Static angle right, but fast motion goes the wrong way first, then creeps back:** gyro and
     accelerometer disagree in sign. A real board cannot cause this; a mirrored axis map can. The viewer
     therefore refuses an `--axes` with determinant −1 (e.g. `-x,y,z`). If it happens anyway, report it.

5. **Film it** with macOS screen recording (Cmd+Shift+5, select the viewer window) while tilting.
   The `--save` CSV lets you re-render the same motion later with `--replay`.
6. **Restore the AGV image** afterwards (`./restore_canfix01.sh`, BOOT0 + RESET) if the robot
   firmware is needed again.

## Model parameters

`sim/models/wbr_3d_simple.xml` is **generated** by `sim/models/make_wbr_3d_simple.py`; edit the script.
Each side is a closed five-bar like report Fig. 2.1: two hip motors O, D on the body, upper links
OA, DC, lower links AB, CB meeting at the wheel axle B. The knees splay outwards. The legs are drawn in
one fixed pose (virtual leg l0 = 0.12 m, θ0 = 0). Each robot is a mocap body pivoting about the wheel axle.
It has no joints, so the masses only document intent; nothing is simulated. The test
`tests/test_model_geometry.py` checks that every link length closes and that the committed XML
matches the generator.

Sources:

- [R] = team report *Báo cáo tổng kết* (Fig. 2.1, Tables 4.1, 4.3, 5.1).
- [C] = `nguyenbinh-shark/DM-jump` `User/Algorithm/VMC/VMC_calc.c` `VMC_init`.

| Parameter | Value | Status |
|---|---|---|
| Hip motor axis spacing d (l5 in code) | 0.08 m | SOURCE-SUPPORTED [R][C] |
| Upper links l1 = l4 | 0.075 m | SOURCE-SUPPORTED [R][C] |
| Lower links l2 = l3 | 0.14 m | SOURCE-SUPPORTED [R][C] |
| Wheel radius | 0.0603 m (report rounds to 0.06) | SOURCE-SUPPORTED [C observer constant][R] |
| Virtual leg range | 0.06–0.20 m; drawn at 0.12 m | range SOURCE-SUPPORTED [R]; 0.12 ASSUMED |
| Masses: body / leg / wheel | 3.5 / 0.2 / 0.7 kg | SOURCE-SUPPORTED [R Table 5.1] (not used: no dynamics) |
| Hip motor DM-J4310 | Ø57 mm | SOURCE-SUPPORTED [R Table 4.1] |
| Wheel motor DM-H6215 (hub) | Ø68 mm | SOURCE-SUPPORTED [R Table 4.3] |
| Body box | 0.30 × 0.20 × 0.12 m, hip axes 3 cm below the top | x, z INFERRED from Fig. 2.1b scale; y ASSUMED |
| Lateral stack (upper / lower link / wheel centre) | y = ±0.112 / ±0.126 / ±0.162 m | ASSUMED |
| Tyre width | 36 mm | ASSUMED (from the photo) |
| IMU location | top centre of the body | ASSUMED. The real robot's IMU is a IMU sensor on the DM MC02 board [R]; mounting UNKNOWN |

To change the drawn height: `python -m sim.models.make_wbr_3d_simple --l0 0.16`. Then update the test's
expected value only if you want 0.16 to become the committed default.

### When the team's CAD arrives

1. Export each rigid part as **STL** (or STEP → STL), in metres, with the part frame at its joint axis:
   body, the four links leg_OA/leg_AB/leg_BC/leg_DC (names as in report §5.1), and the wheel.
2. Add them as `<mesh file="…"/>` assets. Replace each primitive geom in the generator with `type="mesh"`.
3. Take the mass, CoM and inertia tensor of each part from CAD and put them in `<inertial>`.
   Mark them SOURCE-SUPPORTED (CAD), not MEASURED.
4. Put the IMU site at the real IMU sensor location and orientation. Set `--axes` from the CAD mounting.
5. Only then is a 3D dynamics model worth building. Revolute joints, the five-bar loop closure
   (an MJCF `<equality connect>`), actuators and contact are a separate task. This file stays a
   visualisation.

## Limits

- **Yaw is not shown:** without a magnetometer, gyro z integration only drifts.
- The accelerometer angle is wrong while the board accelerates; the filter only hides short accelerations.
- The bias is calibrated once at startup; temperature drift after that is not tracked.
- Live mode is not hardware-tested yet (BLOCKED on flashing `imu_can_bench`, which needs BOOT0 + RESET).
  Measured so far: fake/replay rendering, plus bench-format lines piped through `--stdin`. Both are SIMULATED inputs.
