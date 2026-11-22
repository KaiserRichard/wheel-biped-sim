# Offline stereo depth from the recorded IR MCAP

This tool reads the rectified mono8 infra1/infra2 topics, pairs images by the
identical ROS header stamp, and computes metric depth with OpenCV
`StereoSGBM`. It streams the MCAP; it does not retain the full recording in
memory. Calibration is taken from the recorded `camera_info` messages.

## Run

The project perception environment already contains the required Python
packages. The default command processes every 15th pair from the marked-loop
recording and writes results outside the repository:

```sh
python3 \
  tools/stereo_depth/sgbm_depth.py
```

Options:

```text
--mcap PATH       Input MCAP (defaults to the recorded marked loop)
--output PATH     Output directory (defaults to the dated derived directory)
--every N         Process every Nth pair, starting with pair index 0 (default 15)
--frames N        Process at most N selected pairs; still scans the whole MCAP
--max-depth M     Preview color scale in metres (default 8)
```

For example, a short development run can use `--frames 2`; its summary will
still report complete bag pairing counts. Each selected frame produces a
compressed `.npz` with `depth_m` as float32 metres, a 16-bit PNG in millimetres,
and a Turbo-colormapped preview PNG. `summary.json` records the measured
calibration, algorithm parameters, pairing counts, per-frame valid fraction
and depth percentiles, and full/half resolution compute times. The contact
sheet shows left IR and depth previews for up to 12 approximately evenly spaced
selected frames.

Invalid depth is encoded as zero. Validity requires positive left disparity and
a left/right disparity agreement within 1.5 px. The right disparity is computed
by matching swapped images with a negative disparity search range.

## Parameters

The full-resolution disparity count is the next multiple of 16 covering
`fB / 0.3 m`; the recorded calibration selects 80 disparities. The matcher
uses a 5x5 block, `P1=8*5^2=200`, `P2=32*5^2=800`, `disp12MaxDiff=1`,
`preFilterCap=31`, `uniquenessRatio=10`, `speckleWindowSize=100`,
`speckleRange=2`, and `STEREO_SGBM_MODE_SGBM_3WAY`. Depth is `Z=fB/d`.

## Tests

```sh
python3 \
  -m unittest discover -s tools/stereo_depth/tests -v
```

See [REPORT.md](REPORT.md) for the full-bag run, measured values, and limits.
