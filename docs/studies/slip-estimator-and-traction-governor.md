# Wheel slip: slip-gated estimator and traction governor

## Tóm tắt tiếng Việt

[COMPUTED: EST1, TRAC2 và FW9] Khi cảnh báo trượt bật, bộ ước lượng không còn kéo tốc độ theo bánh xe, và bộ điều tốc bám đường giới hạn tham chiếu tốc độ quanh tốc độ ước lượng. Trong ma trận 16 ca, số ca ngã giảm từ 3 xuống 2. Hai ca vẫn ngã đều là mô hình 6.2 kg: phanh 0.35 s trên μ 0.15 (đòi gia tốc vượt giới hạn ma sát và nghiêng người trước khi có dấu hiệu trượt) và bậc 5 mm trên μ 0.15 (cần chính sách tiếp cận bậc). Chỉ là mô phỏng.

**Result in one line.** [COMPUTED: FW9 integration run] The changes remove 1 of the 3 falls in a 16-case slip matrix. **2 of 16 cases still fall**, both with the assumed 6.2 kg model. This is a partial fix, not slip safety.

## What changed in the firmware

- **Slip-gated speed recapture.** [FROM FILE: `firmware/control/s10_estimator.c`] The estimator's robust speed recapture pulled the speed estimate towards the wheels whenever the two wheels agreed. Under common slip both wheels can agree and both be wrong. The existing slip alarm now also blocks that recapture; inertial propagation continues while the alarm is set. No new threshold was added.
- **Traction governor.** [FROM FILE: `firmware/control/balance_ctrl.c`] When the slip alarm fires with the robot grounded in BALANCING, CONTROLLED_STOP or REST, the speed reference restarts from the estimated speed and stays within [ASSUMED] ±0.10 m/s of it. It then slews to the supervisor's reference at [ASSUMED] 0.5 m/s², with no host acceleration feed-forward, and releases after [ASSUMED] 300 ms without the alarm. Without an alarm the reference passes through unchanged. All three values are development values, not hardware constants.
- **Frozen-encoder gate.** [FROM FILE: `firmware/control/balance_ctrl.c`] A frozen or chattering encoder also corrupts the wheel speed the alarm compares. The governor is therefore off while the S11 frozen or advisory masks are set.

## Slip matrix

[ASSUMED: scenario settings] Two models (nominal 5.06 kg and an assumed 6.2 kg with a battery lump) × eight cases: clean drive, one-sided slip, full-slip acceleration, a commanded 0.35 s brake on μ 0.15, the same brake on μ 0.8, a 5 mm step on μ 0.15, a lateral push on μ 0.25, and a wheel lift. MuJoCo at 1 kHz with the C firmware at 500 Hz.

| Case | No change [COMPUTED] | Slip-gated estimator only [COMPUTED] | Estimator + governor + gate [COMPUTED] |
|---|---|---|---|
| 5.06 kg, 0.35 s brake, μ 0.15 | falls at 4.256 s | falls at 4.256 s | **no fall** |
| 6.2 kg, 0.35 s brake, μ 0.15 | falls at 4.248 s | falls at 4.248 s | **falls at 4.254 s** |
| 6.2 kg, 5 mm step, μ 0.15 | falls at 2.448 s | falls at 2.442 s | **falls at 2.444 s** |
| other 13 cases | no fall | no fall | no fall |

[COMPUTED: EST1 slip study] The estimator change shrinks the worst speed errors. Nominal full-slip acceleration: speed-error peak [COMPUTED] 0.3681 → 0.1885 m/s and integrated odometry error [COMPUTED] 0.06936 → 0.04477 m. 6.2 kg step: [COMPUTED] 0.6721 → 0.08374 m/s. Cases without an alarm are unchanged. A residual error remains because the alarm's own fast reference relaxes towards wheel speed (time constant [FROM FILE] 0.085 s), so a sustained common slip can clear the alarm while the wheels still slide.

[COMPUTED: TRAC2 brake sweep] Over μ 0.1–0.8 × 0.3–0.8 m/s × both models: falls with the commanded 0.35 s brake went from [COMPUTED] 51/96 to 37/96; falls with the supervisor's host-loss stop went from [COMPUTED] 2/96 to 0/96. Normal-friction stopping distance (μ 0.8) is unchanged: after/before ratio [COMPUTED] 1.000 in all [COMPUTED] 26 comparisons, because the alarm stays silent there.

## Why two cases still fall

**6.2 kg, commanded brake on μ 0.15.** [COMPUTED: TRAC2 physics bound] Friction limits the deceleration to μ·g = [COMPUTED] 1.471 m/s². The commanded 0.35 s brake from 0.5 m/s asks for a peak of [COMPUTED] 2.68 m/s², [COMPUTED] 1.82 times that limit. The controller brakes lean-first and commits the lean within about one pendulum time constant ([COMPUTED] 0.116 s), before slip is visible. The 5.06 kg robot's forward push slips early, so its alarm fires at [COMPUTED] 4.034 s with pitch still above [COMPUTED] −1° and the governor saves it. The heavier robot's push does not slip; its alarm fires only at [COMPUTED] 4.120 s with pitch near [COMPUTED] −11°, already past the roughly [COMPUTED] −8.5° lean that friction can hold. From there the wheels saturate at the [ASSUMED] 2 N·m model limit, spin, and the body falls.

[COMPUTED: TRAC2 physics bound] No fixed firmware limit fixes this without hurting normal floors: the shortest possible stop from 0.5 m/s on μ 0.15 is [COMPUTED] 0.085 m, while the normal-friction stop is [COMPUTED] 0.064 m (5.06 kg) and [COMPUTED] 0.066 m (6.2 kg), so a [ASSUMED] +10 % distance allowance stays well below 0.085 m. The firmware would have to know the friction before the lean is committed, and it has no such signal.

**6.2 kg, 5 mm step on μ 0.15.** [COMPUTED: TRAC2] The wheel stalls on the edge at about [COMPUTED] 2.16 s, the body keeps its forward momentum and pitches forward, and the wheel slides back off the slippery corner. Catching a forward lean needs the wheel to move forward, which the edge blocks. A traction governor cannot supply that; this needs a step-approach policy.

## Side effect and the frozen-encoder gate

[COMPUTED: FW9 integration run] On the [COMPUTED] 108-run frozen-encoder set, every build had [COMPUTED] 0 falls and [COMPUTED] 72 detected stops, but a frozen wheel can fake the slip alarm.

| Frozen-encoder set, 108 runs | No governor | Governor, no gate | **Governor + gate** |
|---|---|---|---|
| Runs where the governor engaged | — | [COMPUTED] 31 | [COMPUTED] 26 |
| Peak pitch, detected freezes (72 runs) | [COMPUTED] 0.772° | [COMPUTED] 1.650° | [COMPUTED] 0.786° |
| Peak pitch, advisory-only freezes (24 runs) | [COMPUTED] 1.843° | [COMPUTED] 4.627° | [COMPUTED] 1.843° |
| Peak pitch, freezes never flagged (12 runs) | [COMPUTED] 2.073° | [COMPUTED] 4.926° | [COMPUTED] 4.926° |
| Worst drift (hip freeze while driving) | [COMPUTED] 0.087 m | [COMPUTED] 0.571 m | [COMPUTED] 0.088 m |

[COMPUTED: FW9 integration run] The gate cannot see freezes that the frozen-encoder detector never flags: [COMPUTED] 6 such runs still engage the governor and reach [COMPUTED] 4.93° peak pitch, below the [FROM FILE] 15° pitch event, with no fall. Closing this needs a slip criterion that a stopped wheel cannot fake.

[COMPUTED: FW9 integration run] Other gates on the final firmware: CTest [COMPUTED] 52/52, firmware in the loop [COMPUTED] 6/6 with output identical to the earlier firmware, the [COMPUTED] 32-run figure-8 matrix and the [COMPUTED] 16-case lateral push grid unchanged.

## What this teaches

[COMPUTED: TRAC2 brake sweep] On low friction, the safe request is set by the floor, not by the controller: with the current lean-first controller, brakes whose peak deceleration stayed below about [COMPUTED] 0.7·μ·g never fell. For μ = 0.15 that is about [COMPUTED] 1.0 m/s², so a smooth stop from 0.5 m/s needs about [COMPUTED] 0.94 s. [STANDARD PRACTICE] An alarm that changes estimator fusion is not a safety stop; slip evidence that arrives after the lean is committed is too late.

## What the real robot must confirm

[UNKNOWN] Simulation only, firmware in the loop; not run on a robot. The 6.2 kg model is an [ASSUMED] mass lump, not a measured assembly; tyre compliance, real friction and contact chatter are unmeasured. The governor values are [ASSUMED] development values. A complete fix needs either a host-declared low-friction mode or a controller that limits the lean-first excursion, and the step case needs an approach policy. Measure floor friction and stopping distance before any low-friction trial.
