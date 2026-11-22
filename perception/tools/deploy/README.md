# Pi deployment bundle

`tools/deploy/` creates a versioned, checksummed copy of Pi runtime code and
provides a remote deployer plus a software-only smoke check. The bundle includes
`lab/`, `tools/` except host simulator/firmware paths, `calibration/lab/pi/`,
`scripts/`, `ros2/` when present, and three small imported support modules from
the September sensor-validation tooling. It contains no other experiment data
or `third_party/`. The exact
selection and Python/module policy are in [policy.json](policy.json).

## Build and deploy from the Mac

Run from any directory in this worktree. A dirty tree is refused by default;
`--allow-dirty` records the dirty checkout in the manifest and packages current tracked
and non-ignored untracked files. The tarball name includes the checked out
commit and build UTC, and contains `MANIFEST.json` with each included path's
SHA-256.

```bash
bash tools/deploy/make_bundle.sh --outdir /tmp/wbr-bundles
bash tools/deploy/deploy.sh pi5
```

The default destination is `$HOME/wbr-tools/<full-commit>` on the Pi. A custom
destination can be supplied with `--dest '$HOME/wbr-tools/<commit>'`. If using
a dirty checkout, pass `--allow-dirty` to `deploy.sh`; the release directory is
still keyed by its base git commit. Deployment uses `scp` and `ssh`, refuses an existing
release directory, verifies the extracted manifest on the Pi, and changes
`current` only after verification. `previous` tracks the prior current release.
The bundle is first copied to a commit-named temporary archive in the Pi user's
home directory, so the `wbr-tools/` parent does not need to exist beforehand.
To restore it, run:

```bash
bash tools/deploy/deploy.sh pi5 --rollback
```

For a release installed with a custom `--dest`, pass that destination to
`--rollback` as well so it switches the `current` and `previous` links in the
same parent directory.

Before using a real host, review the printed commands without connecting:

```bash
bash tools/deploy/deploy.sh fake-pi --dry-run --allow-dirty
```

`--local-root DIR` is a filesystem-only transport simulator for tests. It
extracts and verifies into a new commit directory and atomically switches local
`current`/`previous` symlinks. It does not contact a host.

## Pi smoke test

Run in the unpacked release directory, with no camera or serial device needed:

```bash
cd "$HOME/wbr-tools/current"
bash tools/deploy/pi_smoke.sh --outdir "$HOME/wbr-evidence/deploy-smoke-20261018-01"
```

The script verifies every manifest hash, records Python and optional `numpy`,
`yaml`, and `rclpy` availability, checks help for lab and evaluation CLIs, runs
saved-fixture analyzers, then runs the hardware-free unittest suites. The EuRoC
launcher has no `--help`, `--plan`, or analyze-only mode; smoke checks its
no-argument usage path without starting the engine. Other current CLIs do not
expose a `--plan` flag; the smoke tool uses their documented
`--analyze-only` modes where available and reports expected fixture FAIL or
BLOCKED outcomes as valid execution. The C++ VIO adapter suite builds and tests
a temporary copy so the verified release stays unchanged. Builder tests run on
the Mac checkout because the extracted bundle has no `.git`; the link-bench
host-simulator suite is excluded because its host-only simulator is not bundled.
The smoke check never starts a capture, publisher, recorder, serial link, or
network client. Missing optional modules are reported in `modules.json`; the
mandatory Python version criterion is Python 3.12 or newer on the Pi.

Output includes `result.json`, per-command stdout logs, fixture outcomes and
suite logs. Overall status follows the common FAIL > BLOCKED > PASS rule. A
PASS means the packaged scripts execute their offline checks on this host; it
does not pass Pi power, storage, thermal, camera, ROS transport, calibration,
SLAM, or any physical gate. A fixture PASS is tooling evidence only.

## Mac tests

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tools/deploy/tests -v \
  > tools/deploy/tests/last-run.log 2>&1
for f in tools/deploy/*.sh; do bash -n "$f"; done
```

Tests build from a temporary git repository, refuse dirty input by default,
detect a modified manifest payload, print deployment commands against a fake
host, and exercise install/rollback symlinks in a temporary local directory.
