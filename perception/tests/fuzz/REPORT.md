# phase-25 report — parser robustness fuzzing

## Built

Added deterministic Python parser mutation tests and sanitizer-backed C mutation drivers under `tests/fuzz/`, with fixed inputs and thresholds in `fuzz_config.json`. The tests exercise perception-health stream parsing, perception-health C decoding, the MCU-side `ph_rx` decoder, link_proto COBS/frame/stream handling, both ROS CDR header readers, and trajectory file parsing. A MEASURED real rosbag header fixture is used alongside frozen/synthetic vectors. Parser implementations were not modified.

## Verification

`make -C tests/fuzz test` completed successfully on the development Mac with Python 3.11 and the available C compiler. Six Python unittest cases passed; all three C programs completed 1,200 seeded mutations under AddressSanitizer and UndefinedBehaviorSanitizer. The full runner also discovered the suite through both its optional Makefile and Python-directory paths. Full fuzz output is saved in `tests/last-run.log`; the run is bounded fixture/tooling evidence only, not a hardware gate PASS.

`bash -n tools/run_all_tests.sh` passed. The final `bash tools/run_all_tests.sh` returned exit 1 because the unrelated existing `tools/link_bench` suite fails on this Mac's unsupported 460800-baud pseudo-terminal. The runner summary shows `make:tests/fuzz:test PASS`, `python:tests/fuzz PASS`, and `python:calibration/allan/tests PASS`; no owning suite was changed by this review.

## Not verified and risks

No Pi, MCU hardware, or hardware transport was used. Python timeouts use Unix `SIGALRM`, and tests require NumPy through the existing trajectory evaluator dependency. Short deterministic mutation runs can miss parser defects. See `FINDINGS.md`.

Proposed commit message: `Add bounded parser mutation fuzz coverage`
