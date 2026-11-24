# Full-duration replay and long stereo capture report

## Implemented

- `replay_map_points.py` accepts `--seconds` (default `20`; `0` reads the bag's
  `metadata.yaml`) and `--rate` (default `0.3`). The player command uses both
  selected values.
- Playback deadline is `ceil(seconds / rate + 30)` seconds. Work and overall
  deadlines expand for longer requests while retaining the existing default
  deadline windows. Receipt JSON and successful validation JSON record the
  requested/resolved duration, rate and player deadline.
- Added no-ROS unittest coverage for option defaults, deadline arithmetic,
  metadata duration parsing and playback command construction.
- `MAP_POINTS.md` documents full-bag invocation. `scripts/camera_day/README.md`
  describes direct stereo launch plus foreground recording beyond 180 seconds,
  with the measured storage estimate and explicit Pi verification limits.

## Verification

Command:

```text
python3 -m unittest discover -s ros2/wbr_orb_node/tests -p 'test_replay*.py' -v
```

Output:

```text
test_sigterm_cleans_owned_process_groups (test_replay_cleanup.ReplayCleanupTest.test_sigterm_cleans_owned_process_groups) ... ok
test_duration_and_rate_defaults_preserve_existing_replay (test_replay_options.ReplayOptionsTest.test_duration_and_rate_defaults_preserve_existing_replay) ... ok
test_play_command_uses_requested_duration_and_rate (test_replay_options.ReplayOptionsTest.test_play_command_uses_requested_duration_and_rate) ... ok
test_player_deadline_is_play_time_plus_margin (test_replay_options.ReplayOptionsTest.test_player_deadline_is_play_time_plus_margin) ... ok
test_reads_rosbag_metadata_duration (test_replay_options.ReplayOptionsTest.test_reads_rosbag_metadata_duration) ... ok

----------------------------------------------------------------------
Ran 5 tests in 1.001s

OK
```

No ROS playback, full recording replay or Pi camera capture was run. Long
capture behavior and full-duration map output remain **UNVERIFIED on the Pi**;
the README labels the capture path and extrapolated storage rate accordingly.
