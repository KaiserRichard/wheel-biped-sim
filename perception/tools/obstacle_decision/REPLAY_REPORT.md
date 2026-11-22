# C2-BE2 RGB-D bag evaluation report

## Commands run

Commands ran from the repository root, using the supplied perception Python. No
ROS runtime, camera, Pi, network, or package installation was used.

```sh
MPLCONFIGDIR=/private/tmp/wbr-mpl-cache MPLBACKEND=Agg \
  python3 -m unittest discover \
  -s tools/obstacle_decision/tests -v

MPLCONFIGDIR=/private/tmp/wbr-mpl-cache MPLBACKEND=Agg \
  python3 tools/obstacle_decision/eval_runs.py \
  /private/tmp/wbr-c2-be2/run_sheet.csv --out /private/tmp/wbr-c2-be2/eval
```

The test suite constructs synthetic 640×480 `16UC1` frames with
`tools/ground_plane/synth.py`, writes them through `mcap_ros2.writer.Writer`,
then copies their CDR messages using `rosbags.rosbag2.Writer` into SQLite and
MCAP rosbag2 directories. It also replays a standalone `.db3`. The three-row
sheet evaluation uses a synthetic 5 cm box at 1.0 m, an empty floor, and a
fronto-parallel 1.0 m wall plane. It includes an extra `operator_tag` column to
check that additional run-sheet values survive in JSON.

## Actual results

- Unittest discovery: **11 tests passed**.
- Bag inputs: standalone MCAP replay passed; rosbag2 SQLite directory replay
  passed; rosbag2 MCAP directory replay passed; standalone `.db3` replay passed.
- Box row: six frames; estimated height 0.050064 m (error +0.000064 m), front
  distance 1.000000 m (error 0), width 0.299991 m; expected and reported
  `JUMP_CANDIDATE`; **PASS**.
- Empty-floor row: six frames; `NONE` on all six; false-detection rate 0.0;
  camera-height error −0.000024 m; plane-height std 0 m; pitch std 0°; **PASS**.
- Wall row: six frames; centered 80×80 px ROI median 1.000000 m versus tape
  distance 1.000 m; error and relative error 0; **PASS**.
- Markdown output contains distinct box, empty-floor, and wall sections. The
  extra run-sheet column is present under that row's `run_sheet` object in JSON.

## Limits

These are deterministic synthetic scenes; they do not establish real-camera
accuracy. All processed frames are treated as one evaluation interval, so
stationary segments should be trimmed or recorded separately if bags include
setup/motion. False detection measures detector corridor candidates, not the
temporal advisory state. Camera stability limits and box tolerances are
evaluation criteria, not hardware safety guarantees. Replay timing is host-side
detector execution only.
