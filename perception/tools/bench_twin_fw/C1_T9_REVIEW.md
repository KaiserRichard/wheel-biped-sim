# C1-phase-9 local milestone review — 2026-10-30

- PRs: none; owner explicitly requested uncommitted changes in this directory.
- Silent loss mechanism found: in the old mux a stray non-C/E/D start enters
  binary mode. Binary overflow stays in dropping state until zero, without an
  error callback. Arbitrarily many valid C-lines can be consumed there with
  no parser/command counter change. New overflow recovery is immediate and
  counted; partial-record idle timeouts are counted. This reproduces a source
  failure mechanism, not the exact cause of the owner's hardware run.
- CR now terminates a command; CRLF is one command. RX queue publication and
  release ordering are explicit; overflow/error gap markers reset parser state.
  `text_ok = sent + rejected` is the command-stage accounting invariant.
- No pre-existing TX reserve check gates CAN command admission. Output record
  reserve failures were counted, including skipped STAT; if later STATs also
  drop, the counter is invisible to a host. A permanent 1536-byte reservation
  now protects status, with a retained deadline and retry on physical stalls.
- Runtime UART is nominal 460800 8N1, actual divider 35 (~457143 baud). ROM
  bootloader/flash_safe remains 115200 8E1. M defaults to 20 Hz/ID on feedback
  arrivals; `M_HZ=0..1000` configures it. Default conservative TX 12796 B/s,
  versus 46080 nominal wire capacity. No firmware/HSI timing measurement.
- `make -C tools/bench_twin_fw`: PASS; final 17904-byte image, BSS 7000 bytes.
- `make -C tools/bench_twin_fw test`: 18 PASS, including C receiver, queue,
  reservation, format bounds, reference comparisons and mocked runner.
- `DMJUMP_REF=/private/tmp/c1-t9-missing-reference make -C tools/bench_twin_fw test`:
  15 PASS, three reference-only comparisons explicitly SKIP. Missing references
  no longer hide the local firmware and runner regressions.
- Final SHA-256:
  `37cb50dd53f658caecfd6da993fdaa598c0fafa505924e739a40ceb44b03a777`.
- Limits: no device/SSH/flash access; no measured 100 Hz acceptance result.
  Queue tests are deterministic native tests, not interrupt timing emulation.
  STAT reservation prevents admission loss, not physical-link stalls or main
  loop hangs. Error event counters cannot reconstruct exact lost wire bytes.
  Decimated M counts no longer equal fed_back. Runner first STAT is a baseline;
  early writes precede it. Fresh MCU sequence state is required per README.
- Recommendation for subsequent owner-authorized work: on an isolated fixture,
  use pi_loop.py with a fresh session, retain RX/write logs, then compare
  rx_bytes/queued/processed, text_ok, sent/rejected/fed_back and parser counters.
- Claude handoff: review this local C1-phase-9 milestone and its hardware limitations;
  no PR was opened and no hardware acceptance is claimed.
