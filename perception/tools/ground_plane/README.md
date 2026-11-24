# Ground-plane and obstacle geometry (offline NumPy)

This package renders synthetic depth frames, estimates a per-frame floor plane,
and compares plane-relative obstacle labels with a nominal fixed-mount model.
It uses NumPy; the evaluation plots use Matplotlib.

## Frames and conventions

Depth is axial camera Z in metres. Camera points use the ROS optical frame:
x right, y down, z forward. Transforms use `T_A_B` to map coordinates from B
into A. The renderer's world frame is x right, y forward, z up. Camera pitch is
positive when the optical axis points down; roll is a right-handed rotation
about optical +z. A positive pitch p gives the plane normal
`[0, -cos(p), -sin(p)]` in camera coordinates.

The fitted plane is `n dot p + d = 0`, with unit normal oriented toward the
camera. Its signed height function is `n dot p + d`; `d` is camera height above
the floor. Default intrinsics are 640x480, fx=fy=386.953 px and
cx=321.034, cy=241.040 px (MEASURED factory values supplied for this task).
The default stereo baseline is 0.050 m (SOURCE-SUPPORTED datasheet parameter
supplied for this task). Nominal mount height 0.40 m is ASSUMED.

## API

- `synth.render(...)` returns `(depth_m, labels, true_height_m)`. Labels are
  uint8: 0 none/invalid, 1 floor, 2 obstacle. Boxes have world-frame
  `(right, forward, up)` center positions and `(width, depth, height)` sizes;
  boxes are expected to rest on the floor. Stereo noise uses
  `sigma_Z = Z**2 * sigma_d / (f * B)`.
- `plane.backproject(depth, ..., stride=1)` returns valid 3D points and pixel
  indices. `plane.estimate_plane(...)` runs seeded three-point RANSAC, optional
  previous-plane hypothesis gating, then SVD refinement. `PlaneEstimate.valid`
  indicates sufficient support; invalid fields are NaN.
- `obstacles.height_image(...)` returns signed plane-relative height per pixel.
  `label_obstacles(...)` applies the 0.05 m threshold and range limit.
  `fixed_mount(...)` applies nominal height and pitch. The first angular height
  residual from a pitch error is `d * tan(delta_pitch)` at horizontal distance d.

The reported plane pitch/roll are extracted from the camera-facing normal;
pitch is exact for pure pitch, and roll is the normal's x tilt under the stated
convention. Combined rotations are a compact normal-based decomposition.

## Run

From the repository root:

```sh
python3 -m unittest discover -s tools/ground_plane/tests -v
python3 tools/ground_plane/evaluate.py --quick
python3 tools/ground_plane/evaluate.py
```

Evaluation writes `results/metrics.json` and four plots. The default full sweep
uses five pitch values from 0 to 6 degrees, three disparity-noise values, and
three seeds. RANSAC metrics use stride 4; timing measures the same synthetic
frame at stride 1 and 4 on the host machine. No Pi timing is represented.
