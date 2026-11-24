# IMU + dual-CAN bench check (Nucleo-F446RE)

A small bare-metal image (no micro-ROS, no motor protocol) plus a Pi-side checker.
It answers one question: do the IMU sensor and both CAN transceivers on the bench work?
It also separates firmware faults from wiring faults.

## What the firmware does

The firmware runs on HSI 16 MHz only. It prints text on USART1 (PA9/PA10, 115200 8N1)
to the Pi's `/dev/ttyAMA0`.

| Phase | What it proves |
|---|---|
| `PIN`: CAN RX pins read with pull-down; each CAN TX driven low for 20 µs | RX reads 1 when idle, so the transceiver is powered and drives RO. Own RX goes low, so CTX/CRX are wired to that module. The *other* RX goes low, so CANH/CANL join the two buses. SCL/SDA read 1, so the IMU sensor pull-ups are powered. |
| `LOOP`: 10 frames per controller in silent loopback | Controller, filters and bit timing, with no wiring involved |
| `NORM`: leave init mode in normal mode | RX saw 11 recessive bits. This exit has a 50 ms timeout instead of the hang in the AGV image. |
| CAN1 sends `0x101` (counter + inverse) at 100 Hz; CAN2 echoes it as `0x202`; CAN1 checks it | Real frames cross both transceivers and the bus at 500 kbit/s |
| `I,...` lines: IMU sensor at 0x68/0x69, WHO_AM_I, 100 Hz samples from DATA_RDY (±4 g, ±500 dps) | Sensor identity, rate, 1 g, gyro bias. A failed read or an all-0x00/0xFF block counts as an error and triggers a re-init after 500 ms. |
| `STAT` once a second: TX ok/busy/err, RX, overruns, bus-off events, raw ESR (TEC/REC/LEC) | Error counters to tell "no ACK" (busy, TEC→128, LEC=3) from "bit error" (bus-off) |

The LEDs mirror the result:

- D4 green: IMU ok.
- D9 blue: CAN round trip within the last second.
- D6 red: any fault.
- D5 yellow: 1 Hz heartbeat.

## Use

```sh
make            # firmware/prebuilt/imu_can_bench.bin + SHA256SUMS
make test       # checker unit tests, no hardware
```

On the Pi, copy `pi/*`, `../link_bench/pi/stm32boot.py` and the `.bin` into `~/imu-can-bench/`. Then:

```sh
pkill -f "micro_ros_agent serial"        # the agent owns /dev/ttyAMA0
./flash.sh imu_can_bench.bin <sha256> ~/wbr-evidence/<dir>/flash   # fit BOOT0 FIRST, then RESET
# remove BOOT0, press RESET
python3 bench_check.py --seconds 20 --out ~/wbr-evidence/<dir>/run   # PASS/FAIL per check
./restore_canfix01.sh                     # BOOT0 + RESET again; puts the WBR AGV image back
```

`flash.sh` refuses to run on an image SHA256 mismatch or a busy port. It reads the full
512 KiB twice and writes only if both reads are identical. It then erases, writes and
verifies by reading back. For the CANH-unplug test, run `bench_check.py --expect-can-fail`
and pull CANH during the run.

## Results, 29 Oct 2026 (run-02, 20 s, board flat and still)

All rows are MEASURED. SUMMARY PASS 10/10.

| Check | Result | Status |
|---|---|---|
| Pins | All 8 as expected; both TX pulses reach both RX lines | PASS |
| IMU WHO_AM_I | 0x68 at address 0x68 | PASS |
| IMU rate | 99.48 Hz (2002 samples) | PASS |
| IMU accel magnitude | 9.629 m/s², mean (−0.53, −0.11, 9.61) | PASS |
| Gyro bias at rest | (−3.25, +0.02, −0.18) °/s, std ≈ 0.035 °/s | PASS (datasheet ZRO ±20 °/s) |
| IMU read errors / re-inits | 0 / 0 | PASS |
| CAN loopback | 10/10 on both controllers | PASS |
| CAN normal-mode init | Both left init mode | PASS |
| CAN1→CAN2→CAN1 round trips | 1900/1900, bad 0, busy 0, TEC/REC 0 on both, no bus-off | PASS |
| CANH disconnect fails the test | Not run (owner skipped it; modules already confirmed) | UNKNOWN |
| 120 Ω termination value | Not measured (no multimeter) | UNKNOWN |
| Transceiver supply | 3V3 rail; works at 500 kbit/s with ACK | MEASURED (functional only) |

- The cumulative counter showed 4 CAN1 bus-off events in the ~80 s before the measured
  window, when the board was being reset and handled. None occurred inside the window.
- The \|a\| of 9.63 is 1.8 % low. That is within typical IMU sensor accel offset/gain
  tolerance; there is no calibration yet.
- Raw logs, the two flash backups and agent logs are in
  `~/wbr-evidence-archive/imu-can-bench-20261029/`. The summary is in
  `results/2026-10-29-run-02-summary.json`.

## Known limits

- **Bootloader jump does not work (FAIL):** sending `BOOTLOADER` stops the firmware, but
  the ROM bootloader never answered on USART1. Two causes were found and fixed on the
  host side:
  - RX is polled between ~1.7 ms I2C reads, so back-to-back bytes overrun.
  - A trailing byte locks the bootloader's autobaud.

  Even with both fixes it stayed silent. The suspected cause (INFERRED) is that the
  bootloader also probes CAN2 (PB5/PB13) and I2C1 (PB6/PB7), which are wired on this
  bench. Use BOOT0.
- With BOOT0, fit the jumper **before** pressing RESET. A running bench image prints
  `y` (0x79), which `stm32boot.py`'s sync loop takes for an ACK.
- On the joined bench bus, `agv_home_test.py` cannot reach 8/8 with `canfix01`:
  - `loopback_fake` answers only for the left wheel, and its echo applies `LEFT_SIGN`
    twice.
  - Both fake motors share ID 1 on a single bus.

  Its 5/8 result is not evidence for or against the CAN hardware. This tool is the
  hardware check.

- **GO start (FAIL):** started by the ROM bootloader's GO command (BOOT0 fitted), the bench
  image prints garbled UART, at no standard baud the Pi could read. That happened on v2 and
  v3 (v3 resets USART1/CAN/GPIO and enables IRQs at startup). It runs correctly only after a
  real RESET with BOOT0 removed. The cause is not found (INFERRED: clock state left by the
  bootloader). The canfix (micro-ROS) images do start correctly via GO.
- `pi/flash_safe.py` is `stm32boot.py` with a sync that accepts ACK/NACK only on an
  otherwise quiet line, so a running app cannot fake it (worked once, 30 Oct). Open it
  **before** pressing RESET: reopening the port after the bootloader has started got no reply.
- `pi/flash_magic.py` (send the reset command, then flash on the same open port) has **never succeeded**: its only run targeted a GO-started image whose UART was garbled. Treat it as untested.
