# WBR kinematic jump pose demo

This is a visual, mocap-driven closed five-bar model. MuJoCo physics, contact,
actuation, and balance are disabled. Each side's four links, wheel, and the body
are separate mocap bodies. `set_pose` places their geometry from `(l0, theta0)`;
the wheels' lowest point sits at the floor, with `vertical_offset` used for flight.

## Run

```bash
./.venv/bin/python -m tools.wbr_pose.jump_demo --record tools/wbr_pose/results --fps 20
./.venv/bin/mjpython -m tools.wbr_pose.jump_demo
python3 tools/test_support/run_unittest.py tools/wbr_pose
```

The first command is headless and writes a GIF plus three key PNGs. Use
`mjpython` for the interactive window on macOS. `state_at(t)` exposes the
scripted phases and values for inspection.

## Geometry and status

| Quantity | Value | Status |
|---|---:|---|
| Hip spacing `d` | 0.08 m | SOURCE-SUPPORTED: `sim/models/make_wbr_3d_simple.py`, team report Fig. 2.1 / DM-jump VMC initialization reference |
| Upper links `l1=l4` | 0.075 m | SOURCE-SUPPORTED: same generator/source references |
| Lower links `l2=l3` | 0.14 m | SOURCE-SUPPORTED: same generator/source references |
| Wheel radius | 0.0603 m | SOURCE-SUPPORTED: generator's DM-jump observer constant |
| `l0` operating range | 0.06–0.20 m | SOURCE-SUPPORTED: team report range stated in task |
| Body envelope, hip offset, lateral stack, link radius, tyre width | generator constants | ASSUMED/INFERRED: see source comments and `tools/imu_view/README.md` |
| Stand/crouch/push/flight/land/settle timing and flight height | 1.00/0.40/0.12/0.55/0.04/0.40 s; ballistic peak 0.371 m | ASSUMED: display-only values in `jump_demo.py` |

The intended firmware convention check is at
`User/Algorithm/VMC/VMC_calc.c` (the `VMC_init` / `VMC_calc` `phi1` and `phi4`
path). GitHub could not be resolved from this execution environment, so the
source was not fetched and the signed-angle equivalence is **UNVERIFIED**.
`fivebar.py` currently reports signed upper-link angles from +x, counterclockwise
positive; do not use those values as motor commands without comparing the source.

## Limits

`theta0` changes the axle point as `(l0 sin(theta0), -l0 cos(theta0))` in the
leg plane. Inverse kinematics selects the downward axle / outward knee branch.
The demo holds `theta0=0` and uses a prescribed ballistic height; neither the
trajectory nor the rendered part layout is hardware validated. The current
automation shell has no active CoreGraphics connection, so evidence uses a
labelled orthographic Pillow projection of MuJoCo-forwarded geom poses. This
preserves the model's part placements but is not MuJoCo's shaded renderer.
