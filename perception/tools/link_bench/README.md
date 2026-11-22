# Pi ↔ STM32 link bench test (Nucleo-F446RE stand-in)

**OPEN / diagnostic.** Bench tooling around the CANDIDATE codec in
[`../link_proto`](../link_proto/README.md). The Nucleo-F446RE stands in for the
STM32H7 for the link and message format only: no balancing, LQR/VMC, FSM,
actuator or motor code exists here. Results are diagnostic numbers, not a gate PASS.

## What it measures

| Phase | What happens | What it shows |
|---|---|---|
| preflight | Pi listens 1 s, sends nothing | MCU→Pi direction alone (20 Hz `McuStatus` heartbeat, state `NO_LINK`) |
| rtt | Pi sends `PerceptionHealth` at 50 Hz for 60 s; MCU replies with `McuStatus` right after each packet | round-trip P50/P95/P99/max, round-trip loss, wire errors per direction |
| faults | corrupted CRC, duplicate sequence, wrong session (10 each) | MCU rejects with `WIRE` / `SEQUENCE` / `SESSION` and recovers on the next valid packet |
| timeout | Pi goes silent for 100 ms (5×) and 300 ms (5×) | 100 ms: link held. 300 ms: MCU declares `NO_LINK` at its 150 ms silence rule, and the link recovers when packets resume |

Clock domains are never mixed. Round trip = Pi `CLOCK_MONOTONIC` at reply decode
minus Pi `CLOCK_MONOTONIC` before `write()`. MCU silence = `action_mcu_ns` of the
`NO_LINK` status minus `sample_mcu_ns` of the reply to the last packet, both from
the MCU's TIM2 clock. "Pi detect" is Pi time from the last send to the `NO_LINK`
status arriving.

The health packets are deliberately **ineligible** (`flags = 0`, tracking
`STARTING`, no clock mapping): there is no Pi→MCU clock mapping yet, so the test
exercises link liveness, CRC/sequence/session checks and the silence rule, not
estimate freshness or references. Linked state therefore reads
`RESET_PENDING_REENABLE` (no local enable is ever issued), and `NO_LINK` during silence.

Test-only provisioning: the firmware binds at boot to the fixed session
`WBRLINK1` with mapping/transform revision `0xF446`, and the Pi script uses the
same values. That stands in for the SPEC's trusted local bind; it is not a
handshake design. The Pi starts its sequence at its monotonic milliseconds, so
reruns within one Pi boot keep increasing. If the MCU still rejects with
`SEQUENCE` (for example after a Pi reboot), press the Nucleo's black RESET button.

## Firmware

Bare-metal C99, register level, no HAL/CMSIS download: needs only
`arm-none-eabi-gcc`. It compiles `../link_proto/link_proto.c` unchanged.

* USART1 PA9 TX / PA10 RX (Arduino D8 / D2) at 460800 8N1: Pi GPIO path.
* USART2 PA2/PA3 at 460800 8N1: ST-Link virtual COM port over the Nucleo USB (fallback path).
  Both ports run the same logic independently.
* PA8 (Arduino D7, also morpho CN10 pin 23) toggles once per USART1 packet that passes
  COBS/CRC/layout decoding, before the receiver policy runs: the MCU timing marker.
* LD2 (green user LED, PA5) is on while either port holds a link, off in `NO_LINK`.
* 84 MHz from the ST-Link 8 MHz MCO (HSE bypass), HSI fallback; baud error +0.16 %.
* TIM2 free-running at 1 MHz is the MCU monotonic clock. The silence/staleness rule
  runs every main-loop pass; status replies are sent from the main loop, never from the ISR.
* `McuStatus.session` is the MCU boot ID (chip UID + boot counter in RTC backup register 0),
  so an MCU reset shows up on the Pi as `mcu_reboots_seen`.

Prebuilt binary: [`firmware/prebuilt/link_bench_f446.bin`](firmware/prebuilt/)
(SHA-256 in `SHA256SUMS`, built with Arm GNU Toolchain 13.2.rel1 from this tree).
Rebuild with `make -C tools/link_bench/firmware`.

### Prebuilt binary history

| Image | SHA-256 |
|---|---|
| Previous image, before advisory LEDs | `91d7702fb81625d31b641419597770d352f51a72e7be2340bb4c6dbf2219fcce` |
| Current image with advisory LEDs | `07f0af8dfe1759e76432205df35527ce8a6febfdf3804cfe6ebd74fd0266551a` |

Flash (pick one):

```sh
cp tools/link_bench/firmware/prebuilt/link_bench_f446.bin /Volumes/NOD_F446RE/   # ST-Link USB drive
st-flash --reset write tools/link_bench/firmware/prebuilt/link_bench_f446.bin 0x08000000   # brew install stlink
```

## Wiring (both sides 3.3 V logic)

| Pi 5 header | Nucleo-F446RE |
|---|---|
| pin 8, GPIO14 TXD | PA10 USART1_RX (CN9 D2) |
| pin 10, GPIO15 RXD | PA9 USART1_TX (CN5 D8) |
| pin 6, GND | any GND |

Power and flash the Nucleo from the Mac over its USB. Do not connect any Nucleo
5V/3V3 pin to the Pi.

## Pi side

Standard library only. Copy `pi/link_bench.py` and `../link_proto/py/link_proto.py`
into one directory on the Pi, then:

```sh
python3 link_bench.py --port /dev/ttyAMA0 --out ~/link-bench-$(date -u +%Y%m%dT%H%M%SZ)
python3 link_bench.py --port /dev/ttyAMA0 --out OUT --loopback   # no Nucleo: jumper pin 8 to pin 10
python3 link_bench.py --port /dev/ttyACM0 --out OUT              # fallback: Nucleo USB into the Pi (USART2 via ST-Link)
```

On a Pi 5, GPIO14/15 appear as `/dev/ttyAMA0` only when UART0 is enabled in
`/boot/firmware/config.txt` (`dtparam=uart0=on`, then a reboot; INFERRED from
Raspberry Pi firmware conventions, verify with `ls -l /dev/ttyAMA0`). The user needs
the `dialout` group or `sudo`.

Writes `summary.json`, `rtt.csv`, `faults.csv`, `timeout.csv`.

## TerrainAdvisory link (bench only)

Type 4 carries the proposed 42-byte `TerrainAdvisory`; type 5 returns the
27-byte MCU echo with accept/reject reason, counters, staleness, and age. The
MCU only validates and reports these records. It has no advisory-driven action.
Both USART1 and USART2 use this same path, session `WBRLINK1`, and increasing
advisory sequence numbers. The advisory becomes stale after 300 ms without an
accepted packet; while stale the MCU sends echoes at 10 Hz.

Build and flash the bench image (same wiring shown above):

```sh
make -C tools/link_bench/firmware
cp tools/link_bench/firmware/build/link_bench_f446.bin /Volumes/NOD_F446RE/
# or: st-flash --reset write tools/link_bench/firmware/build/link_bench_f446.bin 0x08000000
```

Run on the Pi from the repository root (or copy this tree so the shared Python
modules resolve):

```sh
python3 tools/link_bench/pi/advisory_link.py --port /dev/ttyAMA0 --out OUT --synthetic 30 60
python3 tools/link_bench/pi/advisory_link.py --port /dev/ttyAMA0 --out OUT --bag /path/to/rosbag2_or_mcap --rate 1.0 --stride 1
python3 tools/link_bench/pi/advisory_link.py --port /dev/ttyAMA0 --out OUT --synthetic 10 5 --faults
```

Install `rosbags`, `numpy`, and the MCAP ROS 2 support package for bag replay.
The bag mode calls the existing `process_frame` and temporal `update` functions;
it stores each camera frame stamp but uses only Pi `CLOCK_MONOTONIC` for detector
time and send→echo round-trip latency. `--marker-gpio 17` toggles the Pi marker
after the per-packet send timestamp, as in the standard bench script. Outputs
are `summary.json` and `packets.csv` in `--out`.

The `--faults` phase sends bad CRC, old sequence, wrong session, a NaN float,
waits silently for 400 ms, then sends a valid recovery packet. Expected echo
reasons are `WIRE`, `SEQUENCE`, `SESSION`, and `FIELDS`; stale must assert during
the silence and clear after recovery.

## LED test

The four external LEDs show the latest accepted TerrainAdvisory from USART1.
Each LED is active high with one 330 Ω series resistor. The STM32 pins are
push-pull, low speed, and start low (all LEDs off).

| Nucleo Arduino pin | MCU pin | Color | Accepted advisory |
|---|---|---|---|
| D4 | PB5 | Green | `NONE` |
| D5 | PB4 | Yellow | `JUMP_CANDIDATE` |
| D6 | PB10 | Red | `STOP_CANDIDATE` |
| D9 | PC7 | Blue | `UNCERTAIN` |

Wire each LED in series with its 330 Ω resistor from the listed pin to GND.
Only one advisory LED is lit at a time. A rejected packet leaves the previous
accepted indication unchanged. Before the first accepted advisory, if no
advisory arrives for 300 ms, or after reset, all four are off and RED blinks at
2 Hz to show that the advisory feed is absent or stale. A fresh `NONE` advisory
lights GREEN; it is distinct from the no-feed indication.

For the operator check, run the advisory sender with the obstacle decision
pipeline active. Approach an obstacle slowly while watching the LEDs. Stop as
soon as RED lights, then use a tape measure to record the obstacle distance.
This is a diagnostic observation of the advisory threshold; the LEDs do not
control the robot or grant permission to jump.

For a logic-analyzer capture, keep the wiring/probes and 12 MHz analyzer setup
above, and use:

```sh
python3 tools/link_bench/pi/advisory_link.py --port /dev/ttyAMA0 --out OUT --synthetic 30 10 --marker-gpio 17
```

Probe D0/D1/D2/D3 at PA10/PA9/Pi GPIO17/PA8. For a 62-byte decoded advisory
record, COBS plus delimiter is 64 bytes maximum: `64 × 10 / 460800 = 1.389 ms`
(CALCULATED). The 47-byte decoded echo record is 49 wire bytes maximum:
`49 × 10 / 460800 = 1.063 ms` (CALCULATED). The full-duplex serialization floor
is therefore 2.452 ms per send/echo pair. At 30 Hz each direction uses at most
19.2 kbit/s and 14.7 kbit/s respectively (CALCULATED); measured Pi RTT also
includes scheduling and UART wake-up.

## Hardware timing with a logic analyzer (optional)

The Pi-clock RTT includes Python and kernel time on both ends. A logic analyzer puts
every stage on one clock. `--marker-gpio 17` makes the Pi toggle BCM GPIO17
(header pin 11) right after it takes its send timestamp, through the GPIO character
device (stdlib ioctl; needs read/write on `/dev/gpiochip*`, else `sudo`). The firmware
toggles PA8 on every decoded USART1 packet.

| Analyzer | Probe point | Signal |
|---|---|---|
| D0 | Nucleo morpho CN10 pin 33 (PA10) | Pi TX → MCU RX |
| D1 | Nucleo morpho CN10 pin 21 (PA9) | MCU TX → Pi RX |
| D2 | Pi header pin 11 (GPIO17) | Pi send marker |
| D3 | Nucleo morpho CN10 pin 23 (PA8) | MCU decode marker |
| GND | Nucleo CN10 pin 20, or Pi pin 9 | common ground (Pi and Nucleo grounds are already joined) |

The morpho headers are male pins, so the analyzer leads clip there while the Pi
jumpers stay on the Arduino D2/D8 sockets. Settings for a Saleae-compatible 8-channel
analyzer: 3.3 V logic (threshold about 1.5 V), **12 MHz** sample rate (≈26 samples per
2.17 µs bit, 83 ns resolution), 4 channels, 15 s capture. The capture covers an
RTT-only run, because the fault phase sends packets that never decode:

```sh
python3 link_bench.py --port /dev/ttyAMA0 --out OUT --phases rtt --duration 10 --marker-gpio 17
```

Export: in PulseView, *File → Export → Value Change Dump*. With sigrok-cli, use
`sigrok-cli -d fx2lafw --config samplerate=12m --channels D0,D1,D2,D3 --time 15s -O vcd -o cap.vcd`.
In Logic 2, export the digital data as CSV (channels named D0–D3, or pass
`--pi-tx/--mcu-tx/--pi-mark/--mcu-mark`). Then run on the Mac:

```sh
python3 tools/link_bench/pi/la_analyze.py cap.vcd --summary OUT/summary.json --out OUT/la.json
```

It pairs each Pi marker with the next Pi TX burst, MCU marker and MCU reply burst,
and reports P50/P95/P99 for `pi_tx_sw` (Pi send to the first start bit),
`pi_tx_wire`, `mcu_decode` (end of the delimiter to decode), `mcu_reply`,
`reply_wire` and `onwire_rtt`. Pi-clock RTT percentiles minus on-wire RTT percentiles
give an INFERRED Pi receive-path cost: kernel UART RX, wake-up, Python decode, plus
the marker ioctl. That is a comparison of distributions, never a per-packet
subtraction across the two clocks. Expect `pi_tx_wire` ≈ 2210 µs and `reply_wire`
≈ 1430 µs (CALCULATED at 461538 baud). If they differ, the baud rate or the burst
detection is off, and the other stages should not be trusted.

## Expected numbers (CALCULATED / INFERRED, not measured on hardware)

* Serialization floor of one round trip: (102 + 66) bytes × 10 bits / 460800 baud ≈ 3.65 ms.
  RTT P50 should sit a little above that on the GPIO path. Python wake-up and
  scheduling on a loaded Pi add the tail, so P99 is the number to watch.
* USB fallback adds ST-Link bridge and USB polling delay; its RTT is not
  representative of the GPIO UART link.
* Loss and wire errors: 0 expected on a short bench cable.
* 300 ms gap: MCU silence about 150–151 ms; Pi detect about 150 ms plus one status
  record time. 100 ms gap: no `NO_LINK`.

## Verification done here

`make -C tools/link_bench test` (MEASURED, host only):

* Python decoder reads the frozen C `McuStatus` vector from `../link_proto/tests/vectors.txt`;
  every single-byte corruption of a bench health packet is rejected.
* `la_analyze.py` recovers known delays from synthetic VCD and Logic 2 CSV captures
  (within 1.5 bit times), and counts unmatched packets instead of mis-pairing them.
  The GPIO marker's ioctl request layout matches `linux/gpio.h` (faked ioctls; no GPIO chip here).
* Host simulator (`host/sim.c`, the firmware's `bench.c` behind a pseudo-terminal, real clock):
  all four phases, zero loss, 3/3 of each fault kind rejected with the expected reason,
  100 ms gap held, 300 ms gap tripped with MCU silence in [150, 200) ms.
* The actual firmware sources (with `-DBENCH_EMU`, which only skips RCC/UID) on QEMU
  `netduinoplus2` (STM32F405, same USART1/TIM2 addresses): vector table, USART
  interrupts, ring buffers and codec work end to end. QEMU does not model the F446
  clock tree, so its timing is ignored.

UNKNOWN until the bench run: real UART timing and error rate, the Pi's UART0
state, Pi-side scheduling tail, the GPIO marker on real RP1 hardware, F446 clock start-up (HSE MCO vs HSI fallback) on this board.
