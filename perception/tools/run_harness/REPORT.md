# Q3 / C8 — repeatable run harness report

Completed 2026-10-14. INFERRED implementation of the queued C8 requirements;
nothing here establishes a gate PASS.

Built entirely under `tools/run_harness/`:

- `harness.py`: YAML CLI, argv templating, isolated per-repeat working directories,
  headless attestation/display removal, streamed stdout/stderr, POSIX process-group
  timeout, preserved partial trajectories, four engine statuses and incremental
  receipts/summary retaining every planned trial in the denominator.
- Input hashes for dataset manifest/settings/binary plus vocabulary/times/spec and
  optional mount/reference/evaluator; output hashes, input-change diagnostics,
  direct read-only Git HEAD/ref provenance without Git commands, host information,
  UTC/monotonic timestamps and monotonic wall duration.
- Existing `tools/frames` reused for parsing, explicit named transforms and optional
  base_link conversion with observed epoch sidecars. Reference-free path/jump,
  epoch reset/segment, timestamp-order and ORB log phrase/map diagnostics.
- Existing trajectory evaluator reused through its CLI for ATE/RPE with explicit
  frame/clock assumptions, normalized input conventions and no plots. This worktree
  lacks `tools/trajectory_eval`; `ground_truth.evaluator` selects existing source.
- `examples/orb_pi.yaml`: Pi stereo_inertial_euroc spec, all real input paths/domain
  marked TO-FILL, fixed output suffix, optional mount/reference instructions.
- `README.md`, `SOURCES.md`, scripted fake engine and stdlib unittest suite.

Exact test command, run from repository root:

```sh
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tools/run_harness/tests -v
```

MEASURED result: **17 tests passed, 0 failures/errors/skips**, final run 19.175 s.
Environment: Python 3.11, NumPy 1.26.4, PyYAML 6.0.3. The tests used only temporary
synthetic fixtures; no retained real data was copied or evaluated. The real
first-party evaluator source was invoked read-only at
`<local workspace>/Desktop/Research/wbr-perception-phase1-runtime/tools/trajectory_eval/traj_eval.py`
with synthetic estimates/references and temporary output directories.

MEASURED verification:

- Success/crash/empty/partial/reset/timeout and launch failure; repeated trials,
  process exit codes, log retention, SHA-256 provenance, receipt clock labels,
  fresh directories, spaces in paths and CLI success/failure codes.
- Six mixed repeats: 3 OK, 1 CRASH, 1 NO_TRAJECTORY, 1 TIMEOUT, denominator 6,
  OK fraction 0.5. Crashed partial trajectories remain valid retained artifacts.
- Known reset example: path length 3 m, one jump above 1.5 m, two observed
  segments, one reset-counter transition. The 98 m cross-epoch displacement
  does not contribute to path length or jumps.
- Nonidentity mount rotation/lever arm through the existing frames library;
  reset boundaries suppress backward-difference velocity. Missing epoch metadata
  remains UNKNOWN rather than fabricated. Malformed poses, sidecar/order faults,
  artifact escape symlinks, duplicate YAML, empty times, output/input collisions,
  unsafe templates and clock/frame disagreements are rejected.
- Synthetic identical square: ATE/RPE translation RMSE 0 within 1e-12 m and three
  RPE pairs. Synthetic doubled-reference square with alignment none: ATE and
  RPE translation RMSE 1 m. Nonidentity world-to-sensor/ns/wxyz input normalizes
  correctly, including rotation RPE. Evaluator failure remains separate from
  engine OK status and remains in the summary.
- ORB map-point count messages are not parsed as map IDs; explicit Atlas map
  counts and creation IDs are distinguished. Changed input hashes are visible.

Additional MEASURED checks: CLI `--help` exited 0; AST syntax checks passed for
all three Python files; the example YAML parsed and its `wbr` suffix matches
`f_wbr.txt`/`kf_wbr.txt` artifact names.

SOURCE-SUPPORTED API, export and log contracts are cited with upstream URLs and
source lines in `SOURCES.md`. Status precedence, jump threshold, count ratio,
receipt schema and reset/evaluation refusal policies are INFERRED choices.

UNKNOWN / NOT verified:

- ORB-SLAM3 is not built on this Mac. No actual engine, Pi, camera, SSH, physical
  calibration, mounted frame accuracy, real logs, live timestamps or hardware
  performance was tested. No packages installed, network downloads, Git commands
  or changes outside the assigned implementation directory occurred.
- `headless: true` is caller attestation; the supplied binary must disable its
  viewer. Removing display variables alone cannot do that.
- Dataset hashes cover manifest bytes only, not independent rehashing of all
  image/IMU files. Binary hashes exclude shared libraries. The recorded Git HEAD
  is this repository, with dirty state/engine source revision UNKNOWN.
- Stock ORB text exports lack per-pose epochs. Actual mount conversion needs
  instrumented map/reset metadata and honest mount/SDK extrinsics. Log resets
  are phrase counts, not an authoritative per-pose tracking/reset history.
- Full input/temporal coverage is unverified; pose count divided by times rows is
  only a diagnostic proxy. Exit-0 partial trajectories can be OK.

Open issues / integration constraints:

1. Provide an existing evaluator path on each host; no local evaluator was copied.
2. ATE/RPE across resets/maps or timestamp reversals is refused because the reference
   has no bound epoch identity. Separate aligned-epoch runs are needed to compare
   such trials; requested analysis failure is explicitly retained.
3. No resume protocol or disk/interruption recovery is supplied. Completed receipts
   and the last incremental summary survive an interruption, with planned repeats
   still the denominator. Uncompleted trials are visible as missing completion.
4. Engine execution trusts the supplied binary to keep inputs read only and artifacts
   in its working directory. Long log/trajectory analysis currently reads files
   into memory; large-run resource use remains UNKNOWN. No reproducibility or
   accuracy qualification follows from these synthetic tests.
