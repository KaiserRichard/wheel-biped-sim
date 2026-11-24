# phase-16 deployment report

## Built

- `make_bundle.sh` creates a commit-named tarball for Pi-owned runtime paths,
  excludes host simulation and firmware trees, and records build UTC plus
  per-file SHA-256 in `MANIFEST.json`. Dirty checkout refusal is default;
  `--allow-dirty` is explicit and recorded.
- `deploy.sh` supports remote `scp`/`ssh`, a command-printing dry run, local
  filesystem simulation, and rollback. It installs into a new release folder,
  validates contents before switching `current`, and keeps `previous` for
  rollback. Remote rollback accepts a custom `--dest` and selects the same
  deployment parent.
- `pi_smoke.sh` checks the manifest, Python/modules, CLI help, fixture analysis,
  and hardware-free unittest suites, then writes a PASS/FAIL/BLOCKED
  `result.json` and raw logs. It stops before executing a bundle with an absent,
  unreadable, or invalid manifest and writes a contract result.

## Verification

The implementation tests are in `tests/test_deploy.py`; all **10 tests passed**
and output is saved to `tests/last-run.log`. They cover a temporary git bundle,
dirty-tree rejection, tamper detection, missing/unreadable manifest reporting,
fake-host dry runs, local install/rollback, custom-path remote rollback, status
precedence across PASS/FAIL/BLOCKED, and a manifest containing the saved real
power/USB evidence fixture. All three shell scripts pass `bash -n`.

The smoke runner was also run against an extracted bundle on the Mac.
Manifest verification, CLI help, fixture analysis, tool availability, and all
17 selected bundled unit suites passed. Overall status was **BLOCKED** because
this Mac has Python 3.11.0rc2 and phase-16 requires Python 3.12 or newer on the Pi.
An earlier smoke run hit a timing-sensitive 50 ms camera callback-gap fixture
limit; the suite passed both in isolation and in the final bundled rerun. The
summary is saved in `tests/smoke-last-run.log`.

## Not verified and risks

- No SSH, SCP, Pi, camera, ROS graph, or serial device was accessed for this
  task. Remote shell/scp behavior and Python 3.12 execution remain UNKNOWN
  until the operator runs the documented commands on the intended Pi.
- `lab/d435i_raw/tests` has a timing-sensitive fake SDK callback-gap test. It
  passed the final run, but a transient observed gap above its configured limit
  caused one earlier suite run to fail; that test is outside this task's
  owned path.
- `current` is switched atomically after extracted-file verification. The
  remote destination is commit-named and never overwritten. A failed extraction
  can leave an incomplete destination directory; remove it manually only after
  inspecting it before retrying.
- Existing tools vary in CLI shape. The smoke runner checks documented
  `--analyze-only` paths where they exist; tools without offline analyze modes
  are covered by their help output and fixture-backed unittest suites.
- The deploy builder tests run on the Mac checkout because the extracted bundle
  has no `.git`; the link-bench host simulator suite is not included because
  its simulator source is deliberately excluded from the Pi bundle.
- Fixture PASS is not a hardware or project gate PASS.

Proposed commit message: `Add reproducible Pi deployment bundle and smoke test`
