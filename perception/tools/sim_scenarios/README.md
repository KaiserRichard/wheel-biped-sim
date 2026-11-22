# ASSUMED simulation scenarios and advisory scorer

This library supplies scene JSON to `tools/simcam`; it does not render depth or
implement a detector. Every scene has `status: ASSUMED scene`. Expected labels
come from configured thresholds and assumed visibility, not measured robot data.
Advisories are informational and never grant jump permission.

## Scenario library

Times below use seconds from scene start. Windows are half-open `[t_from, t_to)`.
JUMP and STOP abbreviate `JUMP_CANDIDATE` and `STOP_CANDIDATE` in this table only.
JSON files always use the full `Advisory` enum names.

| Name | What it tests | Expected gated advisory |
|---|---|---|
| [lab_empty](scenarios/lab_empty.json) | Lab 1-empty; valid visible floor | NONE throughout |
| [lab_books46](scenarios/lab_books46.json) | Lab 2-books46; 46 mm box at 1 m | JUMP throughout |
| [lab_box132](scenarios/lab_box132.json) | Lab 3-box13; 132 mm box at 1 m | STOP throughout |
| [lab_approach_box132](scenarios/lab_approach_box132.json) | Lab 4-approach; box initially 2 m away, speed 0.25 m/s | STOP throughout 5 s |
| [edge_box110](scenarios/edge_box110.json) | 10 mm below the 120 mm jump limit | JUMP throughout |
| [edge_box130](scenarios/edge_box130.json) | 10 mm above jump limit, fresh decision history | STOP throughout |
| [tiny_box015](scenarios/tiny_box015.json) | 15 mm box, below the 20 mm detector minimum | NONE throughout |
| [at_min_height_020](scenarios/at_min_height_020.json) | Box exactly at the 20 mm detector minimum (no hysteresis there) | Not scored. simcam gives UNCERTAIN on 61/61 ticks (height 0.0202 m, MEASURED in sim). A 19-flip flicker in the first run came from the old ±2 m floor edge, not the detector |
| [range_entry_jump](scenarios/range_entry_jump.json) | Face enters 3 m range at 2 s; cycle starts at 3 s | NONE 0–2; JUMP 2–4; UNCERTAIN 4–5.51; JUMP 5.51–9 |
| [step080](scenarios/step080.json) | 80 mm step with valid floor | JUMP throughout |
| [ramp100](scenarios/ramp100.json) | 100 mm rise, 0.8 m length; assumed raised-surface cluster | JUMP throughout, segmentation dependent |
| [two_boxes](scenarios/two_boxes.json) | Near 46 mm versus far 132 mm box; nearest wins | JUMP throughout |
| [jump_box110](scenarios/jump_box110.json) | Approach at 0.1 m/s; cycle starts at 1 s | JUMP 0–2; UNCERTAIN 2–3.51; JUMP 3.51–7 |

## Shared format and threshold derivation

Coordinates and dimensions are metres. `x_face` and `x_start` are world forward
coordinates; axle position is `robot.start_x + robot.speed_m_s * t`. Range is
measured by the detector from its camera ground projection, so a nonzero camera
forward offset shifts the nominal range-entry time. These scenes assume zero
forward offset. The approach ends before the axle reaches the box; no collision
response or motor control is implied.

The shared JSON keys are `name`, `description`, `status`, `robot`, `obstacles`,
and optional `expected`. Robot keys are `start_x`, `speed_m_s`, `duration_s`,
`l0`, `jump_at_s` (number or null). Obstacle keys depend on `type`:

- `box`: `x_face`, `y`, `depth`, `width`, `height`, `yaw_deg`.
- `step`: `x_face`, `height`.
- `ramp`: `x_start`, `length`, `rise`, `width`.

Expected entries contain `t_from`, `t_to`, `advisory`, `note`. Notes cite constant
names and identify additional assumptions. The validator rejects unknown keys,
nonfinite numbers, booleans used as numbers, nonpositive dimensions/duration/leg
length, negative forward speed, obstacles initially behind the axle, invalid jump
times, invalid advisory names, and overlapping or out-of-duration windows.
An obstacle may leave the forward view later; this is allowed. Optional windows
can have gaps; unlabelled intervals are not scored. No expected windows yields
FAIL because there is no scoring claim to evaluate.

[Decision policy](../obstacle_decision/decide.py) defaults:

- `DecisionParams.jump_min_h = 0.03`, `jump_max_h = 0.12`: inclusive initial
  JUMP interval; larger heights produce STOP from NONE/UNCERTAIN.
- `height_hysteresis_m = 0.01`: an existing JUMP changes to STOP only above
  0.13 m; an existing STOP changes to JUMP only below 0.11 m and at least 0.04 m.
  The 110/130 mm static scenes start with fresh history; they do not establish
  transition behavior at those strict hysteresis boundaries.
- `confirm_n = 3`, `confirm_m = 5`, `ema_alpha = 0.4` describe temporal
  confirmation/smoothing. `hysteresis_frames = 2` delays clearing to NONE.
- `near_m = 1.5` controls fallback STOP for an invalid plane and remembered
  near geometry. It is not the valid-plane detection range limit.

[Detector](../obstacle_decision/detect.py) defaults are
`Params.min_forward = 0.30`, `max_forward = 3.0`, and
`min_obstacle_height = 0.02` with a strict `h > min_obstacle_height` mask.
Thus an ideal 20 mm box is excluded; a surviving cluster below
`DecisionParams.jump_min_h` would produce UNCERTAIN instead. The range scene
uses the 3 m detector default. The external lab-day flicker report used a
separate 1.3 m nearest-face cutoff and 0.10 m retention hysteresis; that override
is not encoded in this shared format. These lab mirrors reproduce the requested
object dimensions/placements, not recorded camera trajectories or frame labels.

The ramp expected label assumes the detector preserves the flat floor plane and
extracts a raised surface whose estimated height is 30–120 mm. Its slope is
about 7.1 degrees, within `Params.top_normal_deg = 25`; this alone does not
ensure a cluster or correct plane. The two-box scene assumes sufficient visible,
separate support and uses `process_frame`'s nearest-cluster selection.

## Jump timing contract

`jump_at_s` is the start of CROUCH. That is how `tools/simcam/run.py` uses it, and the two workers had
assumed different meanings until the maintainer's first scored run on 2026-10-30. From there the
[`wbr_pose.jump_demo`](../wbr_pose/jump_demo.py) phases follow:
CROUCH 0.40, PUSH 0.12, FLIGHT 0.55, LAND 0.04, SETTLE 0.40, which is 1.51 s
(`total_time()` 2.51 s minus `STAND_S` 1.00 s).
Both jump scenes expect UNCERTAIN from CROUCH through SETTLE. Fresh confirmation
follows recovery; the scoring margin allows that startup transient.

This follows the proposed gating table in
[TILT_JUMP_RESEARCH.md](../../docs/scene_twin/TILT_JUMP_RESEARCH.md):
CROUCH/thrust, FLIGHT, LAND, SETTLE and stale/unknown phase inhibit useful
advisories. These scene windows assume fresh phase input and a clean cycle.
The times are illustrative, not measured firmware/contact timing. If simcam
uses a different phase origin or extra settling dwell, reconcile the explicit
contract before interpreting scores. Raw decision output may remain JUMP/STOP
during flight; only the separate gate is expected to emit UNCERTAIN.

## Validate and score

Run from repository root with Python 3; this package requires only the standard
library and the existing advisory enum.

```sh
python3 -m tools.sim_scenarios.validate
python3 -m tools.sim_scenarios.score tools/sim_scenarios/scenarios/lab_box132.json /path/to/timeline.csv
python3 -m tools.sim_scenarios.score score_all /path/to/results
python3 tools/test_support/run_unittest.py tools/sim_scenarios
python3 tools/lint_markdown.py tools/sim_scenarios/README.md
```

A timeline needs `t`, `raw_advisory`, `gated_advisory`. Also accepted:
`time`, `time_s`, `t_s`; `raw`, `advisory_raw`; `gated`, `advisory_gated`.
Spaces in headers normalize to underscores. Labels can be enum names or integer
codes 0–3. Other columns are ignored. Both channels are mandatory. Timestamps
must be finite, nonnegative and strictly increasing in scene-relative seconds.

Batch matching accepts `RESULTS/NAME/timeline.csv` or `RESULTS/NAME.csv` for each
scenario's `name`. Missing files, both matching layouts, malformed CSV, invalid
scenarios and empty result timelines fail. Use `--scenarios-dir PATH` to score
a subset library. Batch output includes failures and continues to other scenes.
Exit status is zero only when every scenario passes; validation/scoring failures
return one. The scorer prints Markdown per-window and scenario summary rows.

For each channel in each expected window, the scorer reports:

- **Agreement:** percentage of samples matching the expected enum after removing
  an ASSUMED 0.5 s settling margin from the beginning of each window.
- **First correct:** absolute scene time of the first matching sample anywhere
  in the full window, including settling; `never` if no sample matches. Subtract
  `t_from` for latency. This is first occurrence, not time to sustained agreement.
- **Flicker:** number of adjacent sample label changes within the full window.
  A legitimate change across an expected boundary is excluded. This is a
  transition count, not a diagnosis of detector noise or a PASS threshold.
- **Coverage:** fraction of the post-margin interval supported by observed
  timestamps, using forward hold capped at 1.5 times the median timeline sample
  period. The final sample extends one median period. Large gaps and truncated
  captures lose coverage instead of receiving an unlimited last-value hold.

A window passes when **gated agreement and coverage are both at least 90%** and
there is at least one post-margin sample. Every expected window must pass for a
scenario to pass. Raw metrics are diagnostic and do not control PASS. A window
shorter than or equal to the margin fails. Override the ASSUMED acceptance
settings with `--settling-s 0.5 --min-agreement 0.9` (fraction, not percent).
Agreement is sample weighted, so irregular capture rates can bias it. Coverage
is an incompleteness check, not an independent timing/freshness guarantee.

## Limits and local validation receipt

Simulated depth is ideal. These expectations do not model RealSense holes,
flying pixels, mount error, body vibration, USB latency, unavailable floor,
plane-selection mistakes, or contact-estimator uncertainty. See the existing
[obstacle report](../obstacle_decision/REPORT.md) for its historical synthetic
boundary errors; threshold-derived labels do not guarantee detector estimates.
A PASS permits up to 10% disagreement, including within inhibited windows;
it does not certify absence of false jump candidates or robot safety. Keep the
raw timeline and inspect any gate violation independently of the summary score.

On 2026-10-30, the validator accepted 12/12 scenes and
`python3 tools/test_support/run_unittest.py tools/sim_scenarios` passed 15 tests with zero
skips. Hand-made timelines cover perfect, flickery (13/15 = 86.67%), late
(10/15 = 66.67%) and wrong (0%) labels, settling tolerance, boundary accounting,
missing/truncated/gappy data, schema errors, CSV aliases and batch matching.
Both scorer CLI modes also passed using generated expected-label CSV fixtures
for all 12 scenes (100% by construction; scorer smoke test only). Markdown lint
returned 0 issues. No MuJoCo or measured replay result is claimed. The optional
`tools/tilt_sweep/REPORT.md` was absent locally and in `origin/main`, so skipped.
Only files under `tools/sim_scenarios` were authored; maintainer owns integration,
commits and any broader worklog updates.

## maintainer run (2026-10-30)

All 13 scenarios were run through `tools/simcam/run.py` (MuJoCo depth, the real detector and the jump gate) and scored with `score_all`.

- 12/12 scored scenarios PASS; `at_min_height_020` is NOT SCORED.
- Scenario corrections from that run:
  - `jump_at_s` = start of CROUCH;
  - `max_forward` is measured from the camera, which sits 0.15 m ahead of the axle;
  - the tiny box is 0.015 m (0.020 m equals the detector minimum).
- One simcam fix: the floor now covers the scenario. A ±2 m floor edge showed up as STOP for an 80 mm box.

- Gate fix (#178): jump windows now end at LAND + settle dwell (the gate spec), not at the end of the scripted SETTLE. The earlier pass came from simcam wiping detector history. Re-scored: 12/12 PASS.
