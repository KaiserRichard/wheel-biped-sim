# LOG_SPEC: WBR real-robot log format v1

Author: project owner, 17 Nov 2026. Synthetic preparation package; see SOURCE_MANIFEST.json for lineage limits.
Target: the senior's wheel-biped (Jan to Feb 2027) on an STM32H723-class board, with DM motors over FDCAN and an onboard IMU sensor.
Tags: **SOURCE** (read in a file), **INFERRED**, **ASSUMED**, **UNKNOWN**.

The code is the reference. Record layouts, rates and budgets live in `wbrlog/spec.py`. The mapping to Studio signals lives in `wbrlog/importer.py` (`CHANNELS`). `tests/test_format.py` checks the numbers quoted in this file against the code.

## 1. Design rules

1. **Log raw, convert on the PC.** IMU samples are logged as int16 counts and CAN frames as the 8 bytes on the wire. All scaling, signs, offsets and frame changes are done in the importer from the META record. A wrong sign found on robot day then costs one edit to the META JSON, not a new firmware build or a lost session.
2. **One clock.** Every record carries `t_us`, a 32-bit microsecond count from the same MCU timer, taken when the data was produced (IMU data-ready, CAN reception, start of the control step). Host receive times are never used to align data.
3. **1:1 with the Studio by name.** After import, each Studio signal has exactly one channel with the same name, unit and frame (section 5). Channels that exist on only one side are labelled as such.
4. **Never block control.** The logger is a lowest-priority consumer of a ring buffer. On overflow it drops whole records and counts them (`TIMING.log_drop_count`, plus the frame sequence number).
5. **Additive firmware change.** The logger only copies variables that DM-jump already computes (SOURCE `chassisR_task.c`, `VMC_calc.c`, `observe_task.c`). It does not change the control law. Whether the senior allows a logging build is UNKNOWN (unbundled hardware planning reference, question 9).

## 2. Binary framing

Little-endian, packed, no padding.

| Offset | Size | Field | Notes |
|---|---|---|---|
| 0 | 2 | magic `A5 5A` | resync marker |
| 2 | 1 | `type` | record id, section 3 |
| 3 | 2 | `len` | payload length, at most 4096 |
| 5 | 2 | `seq` | global frame counter mod 65536; a gap means frames were lost |
| 7 | 4 | `t_us` | MCU time, µs, wraps every 71.6 min |
| 11 | len | payload | section 3 |
| 11+len | 2 | `crc` | CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over bytes 2 to 10+len; check value 0x29B1 for "123456789" |

Each frame adds 13 bytes. On a bad CRC or magic, the decoder skips one byte and searches for the next magic (tested: garbage, a flipped bit and a truncated tail).

## 3. Records

| Type | Name | Payload (bytes) | Fields (units on the wire) | Time stamp `t_us` means |
|---|---|---|---|---|
| 0x01 | META | JSON (about 2000) | board, firmware commit, clock, IMU ranges and `R_BS`, per-motor bus, CAN id, sign, offset, PMAX/VMAX/TMAX, clamp, sign table for the controller (section 5.3) | write time; sent first and again on request |
| 0x02 | SYNC | 13 | `u32 sync_count`, `u8 source` (0 GPIO pulse, 1 host ping, 2 power-on), `u64 host_time_ns` | the GPIO edge, or receipt of the host ping |
| 0x03 | EVENT | 16 | `u16 code`, `u16 arg`, `char[12] label` | the event: experiment start `studio_t0`, step start, e-stop, mode change |
| 0x10 | IMU | 14 | `i16 gx, gy, gz, ax, ay, az` (raw counts, sensor frame S), `u16 imu_seq` | IMU sensor data-ready interrupt (or SPI read start; say which in META `imu.stamp`) |
| 0x11 | IMU_TEMP | 2 | `i16 temp_raw` (°C = raw × 0.125 + 23, IMU sensor accel temperature register) | read time |
| 0x20 | CAN_TX | 13 | `u8 bus`, `u8 flags` (bit0 FD, bit1 extended id), `u16 can_id`, `u8 dlc`, `u8 data[8]` | frame placed in the FDCAN TX FIFO |
| 0x21 | CAN_RX | 13 | same as CAN_TX | FDCAN RX time stamp extended to 32 bit, or the RX interrupt time |
| 0x30 | CTRL | 140 | `u32 ctrl_tick`, `u32 imu_t_us_used`, `u16 imu_seq_used`, `u16 flags`, then 32 × `f32` native DM-jump variables (3.1) | start of the control step |
| 0x31 | EST | 48 | 12 × `f32`: `ready, pitch, roll, yaw, body_speed, L_L, L_R, gbias_x, gbias_y, gbias_z, accel_bias, accel_norm` (NaN when the firmware has no such quantity) | same step as CTRL |
| 0x40 | TIMING | 8 | `u16 loop_period_us`, `u16 exec_us`, `u16 log_drop_count`, `u16 can_tx_err` | end of the control step |

`imu_t_us_used = 0` means no IMU sample has been used yet. DM-jump waits for `INS.ins_flag` before its first control step (SOURCE `chassisR_task.c` line 62).

### 3.1 CTRL native variables (f32, in this order)

| # | Name | Unit | DM-jump source (SOURCE) |
|---|---|---|---|
| 0, 1 | `theta_R`, `theta_L` | rad | `vmc->theta = pi/2 - Pitch - phi0` (`VMC_calc.c` 83) |
| 2, 3 | `d_theta_R`, `d_theta_L` | rad/s | `vmc->d_theta` |
| 4, 5 | `L0_R`, `L0_L` | m | `vmc->L0` |
| 6, 7 | `d_L0_R`, `d_L0_L` | m/s | `vmc->d_L0` (finite difference) |
| 8, 9 | `alpha_R`, `alpha_L` | rad | `vmc->alpha = pi/2 - phi0` (`VMC_calc.c` 72) |
| 10, 11 | `d_alpha_R`, `d_alpha_L` | rad/s | `vmc->d_alpha` |
| 12, 13 | `pitch`, `pitch_gyro` | rad, rad/s | `INS.Pitch`, `INS.Gyro[0]` (`chassisR_task.c` 158-159) |
| 14, 15, 16 | `roll`, `yaw_total`, `yaw_gyro` | rad, rad, rad/s | `INS.Roll`, `INS.YawTotalAngle`, `INS.Gyro[2]` |
| 17, 18 | `x_filter`, `v_filter` | m, m/s | `chassis_move.x_filter`, `v_filter` (`observe_task.c` 87-88) |
| 19, 20 | `F0_R`, `F0_L` | N | `vmc->F0` (after its ±100 N clamp) |
| 21, 22 | `Tp_R`, `Tp_L` | N·m | `vmc->Tp` |
| 23 to 26 | `tau_req_R0`, `tau_req_R1`, `tau_req_L0`, `tau_req_L1` | N·m | `vmc->torque_set[i]` **before** the ±7 N·m clamp |
| 27, 28 | `wheel_req_R`, `wheel_req_L` | N·m | `wheel_T` **before** the ±2 N·m clamp |
| 29, 30, 31 | `leg_set`, `turn_T`, `roll_out` | m, N·m, N | leg target, yaw PD output, roll PID output |

The firmware clamps in place (`mySaturate`). The logger must copy the value before the clamp; the value after the clamp is already in CAN_TX. That gives the Studio pair `request` / `applied` (SOURCE `motion_control_eval.m`, last line).

### 3.2 DM frames inside CAN_TX / CAN_RX

- Command (MIT): `P16 | V12 | Kp12 | Kd12 | T12`. SOURCE: `h7-advisory-fw/docs/porting.md` line 71. DM-jump sends p = v = Kp = Kd = 0 with a torque (`chassisR_task.c` 79-84). Zero frame: `7F FF 7F F0 00 00 07 FF`, tested.
- Feedback (INFERRED from the public Damiao manual; confirm with one captured frame, EXPERIMENTS R0): `D0 = id | err<<4`, `D1-2 pos16`, `D3-4 vel12`, `D4-5 tau12`, `D6 T_MOS °C`, `D7 T_rotor °C`. The importer matches feedback to a motor by `(bus, D0 & 0xF)`, because the reply's CAN id is the master id.
- Ranges come from META per motor. The defaults in `spec.default_meta()` (PMAX 12.5 rad, VMAX 30 rad/s for the 4310 and 45 for the 6215, TMAX 10 N·m) are **placeholders**. Read them from each motor with the Damiao tool before trusting any number. Resolution with these placeholders: position 3.8e-4 rad, velocity 0.015 or 0.022 rad/s, torque 0.0049 N·m.

## 4. Rates, time stamps and sync

### 4.1 Rates (profile FULL)

| Stream | Rate | Basis |
|---|---|---|
| IMU | 1000 Hz | AGV insTask 1 kHz (SOURCE unbundled protocol reference §2); Studio sensor tick 1 ms (SOURCE `sensor_params.m` `tick_s`) |
| CAN_TX, CAN_RX | 333.3 Hz per motor, 2000 frames/s each for 6 motors | DM-jump: three sends 1 ms apart per side, so a 3 ms cycle (SOURCE `chassisR_task.c` 79-84) |
| CTRL, EST, TIMING | 333.3 Hz | one per control cycle |
| SYNC, IMU_TEMP | 1 Hz | |
| META | once | |

The Studio runs its sensors every 1 ms and its controller every 2 ms (`control_params.m` `Ts`). The rates are not 1:1; the signals are. The importer resamples (4.3).

### 4.2 Time stamps and synchronisation

- **Clock:** a free-running 32-bit timer at 1 MHz (TIM2 or TIM5 on the H7). Do not use bare DWT `CYCCNT`: at 192 MHz it wraps every 22 s (INFERRED arithmetic). The importer unwraps the 71.6 min wrap; this is tested across the wrap.
- **What each stamp means** is in section 3. Sensor data is stamped at acquisition, never at transmission.
- **Studio sync (primary):** the experiment script writes `EVENT label="studio_t0"` at the instant it starts the command profile that the Studio mission or fixture starts at t = 0. The synthetic importer offset is exact to numerical precision. Physical alignment is limited by event-stamping and actuation uncertainty. Without the EVENT, `studio.align_offset` searches least-squares signal mismatch: about 0.8 µs on the synthetic chirp and about 1 µs on the smooth M13 trace after correcting the search bounds (`REHEARSAL.md` §7). Weak or ambiguous motion still requires an independent sync check.
- **External sync:** each SYNC record has a GPIO pulse on a test pin or LED at the same instant, for a scope, logic analyser or video. With `source = 1` the host sends its clock over USB and the firmware logs the pair, which gives host-to-MCU offset and drift (USB latency of about 1 ms limits this; INFERRED). The 24 MHz HSE drifts by tens of ppm (INFERRED), which matters only for long video alignment.
- **Loop timing:** `CTRL.imu_t_us_used` and the CAN_TX stamps give the sensor-to-command delay per axis directly (method A in `ident/delay.py`; exact in the rehearsal).

### 4.3 Resampling against the Studio

- **Sensor and state comparison:** linear interpolation of the real channel at the Studio sample times, after the offset is applied (`studio.compare`). The residual on the M13 run is float32 rounding plus interpolation error: pitch RMS about 1e-6 rad (`REHEARSAL.md` §7).
- **What the controller saw:** zero-order hold (`importer.interp(..., "zoh")`). This is causal and matches the firmware.
- **Replay through the Studio estimator:** `studio.write_wire_csv` writes the robot log as the 21-column M13 packet wire on the 1 ms tick: IMU = newest sample in the tick, encoders = zero-order hold of the newest CAN feedback. The `m13_boundary_worker.m` pattern can then run the Studio estimator on real packets, using the robot's measured `R_BS` and not the Studio fixture rotation.

## 5. Mapping to the Studio (1:1)

### 5.1 Signals

Studio indices are 1-based MATLAB indices.

- `wire` is the M13 packet (SOURCE `m13_sensor_block.m`): wire(1) is the IMU tick, wire(8) the encoder tick.
- `y` is the controller output (SOURCE `motion_control_eval.m`).
- `diag` is the estimator diagnostic (SOURCE `m13_packet_update.m` line 21).
- `truth` is the scorer truth (SOURCE `m13_score_prefix.m`).

The axis order is the Studio's: `L_hip3, L_hip4, R_hip3, R_hip4, L_wheel, R_wheel` (SOURCE `sensor_params.m` `P.axes`).

| Channel | Unit | Studio source | Robot source |
|---|---|---|---|
| `imu.gyro_S.{x,y,z}` | rad/s | wire(2:4) gyro, frame S | IMU g* × range / 32768 × π/180 |
| `imu.accel_S.{x,y,z}` | m/s² | wire(5:7) specific force, frame S | IMU a* × range / 32768 × 9.80665 |
| `enc.q.<axis>` (6) | rad, wrapped to [-π, π) | wire(9:14), 65536 counts/turn | CAN_RX pos × sign + offset, wrapped like `encoder_wrap.m` |
| `enc.dq.<axis>` (6) | rad/s | wire(15:20) | CAN_RX vel × sign |
| `ctrl.applied.<axis>` (6) | N·m | y(1:6) | CAN_TX MIT torque × sign (after the clamp) |
| `ctrl.request.<axis>` (6) | N·m | y(7:12) | CTRL `tau_req_*`, `wheel_req_*` × sign (before the clamp) |
| `ctrl.x.axle_x`, `axle_v` | m, m/s | y(13:14) | CTRL `x_filter`, `v_filter` × `x_sign` |
| `ctrl.x.leg_minus_pitch`, `_rate` | rad, rad/s | y(15:16) | from CTRL `alpha_*`, `d_alpha_*`, pitch (5.3) |
| `ctrl.x.pitch`, `pitch_rate` | rad, rad/s | y(17:18) | CTRL `pitch`, `pitch_gyro` × `pitch_sign` |
| `ctrl.L.L`, `ctrl.L.R` | m | y(19:20) | CTRL `L0_L`, `L0_R` |
| `ctrl.phi`, `ctrl.roll`, `ctrl.yaw` | rad | y(21:23) | CTRL pitch, roll, `yaw_total` wrapped, with signs |
| `est.ready, pitch, roll, yaw, body_speed` | 1, rad, rad, rad, m/s | diag(1:5) | EST |
| `est.L.L`, `est.L.R` | m | diag(6:7) | EST |
| `est.gbias.{x,y,z}` | rad/s | diag(8:10) | EST |
| `est.accel_bias` | m/s² | diag(11) | EST |
| `ctrl.imu_age` | s | `estimator_processing_ticks` × 1 ms | CTRL time − `imu_t_us_used` |
| `timing.loop_period` | s | `Ts` = 2 ms | TIMING |
| `truth.pitch`, `truth.speed`, `truth.L.L/R` | rad, m/s, m | scorer truth | **none on the robot** |
| `motor.tau_fb.<axis>`, `t_mos`, `t_rotor`, `err` | N·m, °C, °C, code | **none** (the M09 lag state is internal) | CAN_RX |
| `imu.temp`, `timing.exec`, `timing.log_drops`, `cmd.kp/kd/v.<axis>` | | **none** | IMU_TEMP, TIMING, CAN_TX |
| `imu.gyro_B`, `imu.accel_B` | rad/s, m/s² | the M13 estimator's `R_BS * payload` | `R_BS` from META applied to S |

`tests/test_format.py` checks the coverage: 18 wire values, 23 controller outputs and 11 diagnostics, each used exactly once. Studio gyro and accelerometer samples are in the sensor frame S. Compare them in the body frame B (`imu.*_B`), because the Studio's `R_BS` is a test fixture (30/−20/10°), not the robot mount.

### 5.2 Frames and signs

- Studio frames: world X forward, Y left, Z up; body B is the Studio control axes (SOURCE `sensor_params.m`).
- DM-jump uses `INS.Gyro[0]` as the pitch rate (SOURCE `chassisR_task.c` 159). The INS x axis is therefore the robot's lateral axis (INFERRED), so the robot's `R_BS` is not identity. It is measured in EXPERIMENTS R1.
- The left side negates pitch and gyro (SOURCE `VMC_calc.c` 109-110). The wheel speed estimate negates both motor velocities (SOURCE `observe_task.c` 73, 77). This suggests wheel `sign = -1` for "forward positive" (INFERRED). EXPERIMENTS R1 settles every sign by moving one joint at a time by hand.
- The legacy code had swapped roll and pitch labels (SOURCE unbundled protocol reference §3). Check the axis mapping first.

### 5.3 Native CTRL to Studio state

Here `m = -1` when `mirror_left` is set (the default, from DM-jump):

```
phi        = pitch_sign * pitch
pitch_rate = pitch_sign * pitch_gyro
alpha_c    = alpha_sign * (alpha_R + m * alpha_L) / 2 + alpha_offset
x          = [x_sign * x_filter, x_sign * v_filter,
              alpha_c - phi, alpha_sign * (d_alpha_R + m * d_alpha_L) / 2 - pitch_rate,
              phi, pitch_rate]                                  # Studio y(13:18)
```

This matches the Studio definition `x = [mean axle x, mean axle v, mean(alpha) - phi, mean(dalpha) - pitchRate, phi, pitchRate]` (SOURCE `motion_control_eval.m`). Whether DM's `alpha` is the same angle as the Studio's `fivebar_kinematics` leg angle is **UNKNOWN**. `alpha_sign` and `alpha_offset_rad` are set on robot day from one static pose (EXPERIMENTS R1).

## 6. Bandwidth budget

Record size = 13 + payload. Bytes per second = size × rate (`spec.budget`).

| Profile | Contents | Bytes/s | USB FS CDC (about 800 kB/s usable, INFERRED) | UART 2 Mbaud (200 kB/s) | UART 921600 (92 kB/s) | UART 460800 (46 kB/s) |
|---|---|---|---|---|---|---|
| FULL | all streams, section 4.1 | **209,374** | 26 % ✔ | 105 % ✘ | ✘ | ✘ |
| IMU_ONLY | IMU 1 kHz, temperature, SYNC | **27,041** | 3 % ✔ | 14 % ✔ | 29 % ✔ | 59 % ✔ |
| ONE_MOTOR | one motor at 1 kHz TX/RX/TIMING, SYNC, EVENT | **73,084** | 9 % ✔ | 37 % ✔ | 79 % (tight) | ✘ |

FULL breakdown (bytes/s): IMU 27,000; CAN_TX 52,000; CAN_RX 52,000; CTRL 51,000; EST 20,333; TIMING 7,000; SYNC 26; IMU_TEMP 15.

- **Storage:** FULL is about 12.6 MB per minute, so 377 MB for a 30-minute session. The text form measured 1.95 × the binary size on a synthetic FULL log, which is fine for bench profiles but not for FULL over UART.
- **CAN load:** logging adds no CAN frames; CAN_TX and CAN_RX are copies. Bus load stays at DM-jump's level, about 22 to 27 % per bus (SOURCE unbundled hardware planning reference, section 2).
- **MCU cost (INFERRED):** about 6,000 records/s. A table-driven CRC-16 plus memcpy is around 1 µs per record on a 192 MHz M7, so under 1 % CPU. Use a 32 KB ring buffer (150 ms of FULL) drained by USB or UART DMA from a low-priority task.
- **Link check:** EXPERIMENTS B0 measures the real CDC throughput with a counter pattern before trusting the 800 kB/s figure.

## 7. CSV forms

1. **Firmware text form** (bench fallback over UART; the importer reads it like binary): one record per line, `NAME,t_us,seq,<fields in section 3 order>*HH`. HH is the XOR of the characters before `*`. Bytes fields are hex. META carries hex-encoded JSON. Example: `IMU,1000137,0,12,-3,0,15,-8,5461,0*4B` (checksum illustrative).
2. **Long export** (`importer.write_long_csv`): `t_s,channel,value`, with every channel at its own time stamps. Lossless.
3. **Wide export** (`importer.write_wide_csv`): `# key: value` header lines (source hash, offset, resample method), then `t_s,<channels>` on a chosen grid (normally the Studio grid), using zero-order hold or linear interpolation.
4. **Studio wire** (`studio.write_wire_csv`): 21 columns, `t, imu_tick, gyro_S(3), accel_S(3), enc_tick, q(6), dq(6)`, with −1 ticks for missing packets. `studio.load_wire_csv` reads the same layout. Studio-side export, when MATLAB is available: write `[t, acquired_packets]` from a loop raw file, where `acquired_packets` is the 20-wide `To Workspace` signal in `build_sensor_loop.m`. `studio.load_estimation_csv` reads the existing `*_estimation.csv` files as they are; this is tested on `loop3_estimation.csv`.

## 8. Open items (must be closed on robot day; all UNKNOWN today)

- CAN buses, ids, master id and DM ranges per motor (unbundled hardware planning reference, question 4). The META defaults are placeholders.
- The DM feedback byte layout and the frame position of `tau` (INFERRED), checked against one captured frame.
- Whether a USB CDC endpoint exists in the WBR firmware (DM-jump has no USB; the AGV firmware does).
- IMU sensor ranges and filter settings on the robot. The course driver uses ±3 g and ±250 °/s, 100 Hz (unbundled sensor-driver reference, lines 37-43; two lines marked CHECK-DATASHEET). The ±250 °/s range saturates at 4.4 rad/s, which a falling robot exceeds (INFERRED).
- The IMU stamp source (data-ready interrupt or polled read) and the INS filter's group delay.

## 9. Completion clarifications and importer limits

- `python -m wbrlog` is the offline CLI; see README for native-time export, EVENT alignment, Studio comparison and replay-wire examples. An adjacent JSON receipt records input SHA256, META, offset and decode diagnostics. META overrides replace the complete object. Missing META is explicitly reported as placeholder defaults; do not accept that output as calibrated hardware data.
- The v1 CAN record supports classic standard-ID eight-byte MIT frames. FD/extended flags and DLC other than eight are counted and excluded from decoded motor channels. A future extended-ID schema needs a wider CAN-id field. Enable, disable and zero-position command patterns are excluded from torque samples.
- Binary parsing recovers after corrupt lengths as well as bad CRC. Malformed records, truncated tails, unknown types and sequence gaps are reported separately. Text parsing checks ranges/field widths, skips malformed rows and counts sequence gaps. A truncated tail cannot be reconstructed.
- Clock unwrapping assumes less than half a timer wrap between adjacent received records; use a new file after reboot. Keep global sequence order even when acquisition stamps are slightly reordered. IMU age uses modular subtraction against the associated CTRL stamp, including after multiple timer wraps. Zero remains the documented no-sample sentinel; that exact timer instant cannot identify a used sample in v1.
- Capture CTRL as a coherent snapshot and document which control task owns it. Reference DM-jump has separate side tasks; the rehearsal assumes synchronized snapshots. Actual per-side phases and the sample used for each command require confirmation or a versioned per-side record extension. The synthetic motor send order follows META motor order; it is not a verified assignment of hip3/hip4 to physical first/second slots. The inspected right task sends native joint[1], joint[0], wheel.
- Method A pairs TX only within the matching control interval and skips intervals with no matching frame. It measures sample-to-FIFO-enqueue latency. Arbitration, wire transmission, feedback sample age, filter phase and motor lag are separate. ONE_MOTOR as budgeted does not include CTRL/IMU: use FULL or recalculate a custom instrumented profile for Method A.
- EST values must be expressed in the declared Studio frame before logging, except the scalar signs explicitly mapped in section 5. Raw gyro-bias axes require the firmware's documented frame transform; `R_BS` is only applied automatically to raw IMU samples. Unavailable EST fields must be NaN. Verify mirrored leg-angle conventions from measured geometry rather than treating the default sign table as calibrated.
- Wrapped-angle comparisons use shortest-arc interpolation and circular errors for `enc.q.*` and yaw. ZOH has no expiry and linear interpolation has no anti-alias filtering or gap rejection. Mask gaps and stale packets before physical fitting. Replay wire rebins time to 1 ms and cannot preserve individual sub-ms encoder ages; use the native-time long export for delay analysis.
- Synthetic writers mark `measurement_domain=synthetic`; hardware provenance must be attached only to measured captures. No real stationary capture has been fitted in this package. See `IDENTIFIABILITY.md` for the open hardware acceptance gate.
