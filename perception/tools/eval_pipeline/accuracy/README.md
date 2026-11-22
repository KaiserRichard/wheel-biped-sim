# ORB-SLAM3 EuRoC accuracy pipeline

This pipeline launches repeated official `stereo_inertial_euroc` runs on a Pi and evaluates their saved output on the Mac or Pi. It uses `tools/trajectory_eval/traj_eval.py` for parsing, time association, SE3/Sim3 alignment, ATE and RPE; it does not reimplement trajectory math. The pipeline is an offline EuRoC benchmark. **EuRoC accuracy is not a D435i result and does not pass a D435i or project gate.**

## Pi run

Do not run the launcher until the Pi build, dataset and storage paths are ready. From the repository root on the Pi, pass the executable, vocabulary, settings, extracted sequence directory, matching `EuRoC_TimeStamps` file, sequence label, repeat count and a new output directory:

```sh
bash tools/eval_pipeline/accuracy/run_euroc.sh \
  "$HOME/orbslam3/ORB_SLAM3/Examples/Stereo-Inertial/stereo_inertial_euroc" \
  "$HOME/orbslam3/ORB_SLAM3/Vocabulary/ORBvoc.txt" \
  "$HOME/orbslam3/ORB_SLAM3/Examples/Stereo-Inertial/EuRoC.yaml" \
  "$HOME/datasets/euroc/MH01" \
  "$HOME/orbslam3/ORB_SLAM3/Examples/Stereo-Inertial/EuRoC_TimeStamps/MH01.txt" \
  MH01 3 "$HOME/wbr-evidence/euroc-accuracy/MH01-$(date -u +%Y%m%dT%H%M%SZ)"
```

The script takes a host-wide nonblocking `flock` under `${TMPDIR:-/tmp}`, refusing overlap even when callers select different output directories. It refuses pre-existing run directories, refuses unresolved `ldd` libraries, hashes the executable, `libORB_SLAM3.so`, vocabulary, settings and timestamps, and writes one run directory per invocation with command, UTC start/end, exit code, stdout/stderr, library resolution and `vcgencmd get_throttled` before/after. UTC values are from the Pi clock; they are provenance labels and are not used for trajectory time arithmetic. The sequence argument labels output only; it does not validate dataset identity. A run with nonzero engine exit remains available for analysis but is marked FAIL and is not accuracy evidence. Missing exit status is BLOCKED. For older evidence, a matching `runN end ... rc=N` row in the run bundle's parent `runs.log` can supply the exit status.

## Evaluation

Use the exact EuRoC `state_groundtruth_estimate0/data.csv` and the run directories. Pass the timestamp file to calculate pose fraction from the number of timestamp rows. If only the documented row count is available, `--expected-frames` records that explicit caller-supplied denominator in `summary.json`.

```sh
python3 tools/eval_pipeline/accuracy/evaluate.py \
  --sequence MH01 \
  --runs /path/to/run01 /path/to/run02 /path/to/run03 \
  --ground-truth /path/to/MH01/mav0/state_groundtruth_estimate0/data.csv \
  --times /path/to/EuRoC_TimeStamps/MH01.txt \
  --out /path/to/new/evaluation
```

The evaluator detects exactly one `f_*.txt`, `CameraTrajectory.txt`, or `KeyFrameTrajectory.txt` per run. It detects seconds versus nanoseconds from the first non-comment timestamp using the documented magnitude cutoff in `thresholds.json`, then passes explicit format/unit flags to the existing evaluator. This is an inferred format rule for EuRoC absolute epoch timestamps; check it against the exact sequence time domain. The chosen format and unit are saved in `summary.json`. EuRoC CSV is parsed as nanoseconds. Ambiguous or missing output is BLOCKED. SE3 ATE is primary for metric-scale stereo-inertial; Sim3 is retained as an explicitly secondary diagnostic. RPE is calculated in two independent evaluator passes, at 1 m and 10 associated frames. The explicit zero-second estimate offset and shared EuRoC dataset time-base assumption are in every underlying result. EuRoC timestamp/domain context is described in the [EuRoC dataset documentation](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) and [Burri et al., 2016](https://doi.org/10.1177/0278364915620033).

`summary.json` stores per-run hashes, alignment metrics, initialization detection, stdout phrase counts, coverage, per-run evaluator provenance, across-run mean/population standard deviation/min/max, criteria and thresholds. `report.md` gives the comparison table. Initialization is detected by `IMU initialized` or the upstream `end VIBA 2` text. Tracking loss and resets are phrase counts from stdout: logging omissions or upstream wording changes can hide events. Pose fraction is output pose rows divided by input timestamp rows; it does not prove uniform temporal coverage. Camera and IMU body-frame labels are explicit and no extrinsic conversion is applied.

## Acceptance and evidence labels

All numeric limits and evaluation deltas reside in [thresholds.json](thresholds.json), with a rationale/source next to each. The proposed MH01 SE3 ATE limit is 0.046 m: the [ORB-SLAM3 paper, Table II](https://arxiv.org/abs/2007.11898) reports 0.036 m stereo-inertial RMS ATE for MH01, with a proposed 0.010 m margin. The proposed 0.95 minimum output-pose fraction is a project coverage guard, not an ORB-SLAM3 publication result. These limits are **PROPOSED** and passing them is not a project gate PASS. Evaluation outputs identify calculated results separately from source-supported assumptions and proposed criteria.

## Validation and limits

The saved [validation report](validation/report.md) and [summary](validation/summary.json) reproduce the read-only N4A evidence (0.041461, 0.036844, 0.041604 m, rounding to 0.0415, 0.0368, 0.0416 m). It used three historical trajectories, the saved EuRoC ground truth CSV, and the evidence report's 3,682 camera-frame count because the timestamp file was not part of that saved evidence bundle. This is offline tool validation only.

Run tests and preserve their output:

```sh
python3 -m unittest discover -s tools/eval_pipeline/accuracy/tests -v 2>&1 | tee tools/eval_pipeline/accuracy/tests/last-run.log
bash -n tools/eval_pipeline/accuracy/run_euroc.sh
```

Tests exercise PASS, FAIL and BLOCKED status decisions and, where the read-only shared evidence exists, run the complete three-run real-data validation. No test launches ORB or accesses hardware. This pipeline does not establish calibration, frame equivalence from label text, live clock synchronization, D435i accuracy, sustained Pi timing, or ground-truth perfection. Experiment timing remains constrained by the measured under-voltage/thermal issue described in the source evidence.
