# F446 live bench twin (phase-3 redo)

**Not micro-ROS, not AGV-ready, bench 500k.** This single bare-metal F446RE
image combines the IMU sensor stream, TerrainAdvisory LEDs/echo and a DM CAN
command/feedback loop. The real WBR DM-jump firmware uses PS2 + UART and has no
micro-ROS (**SOURCE-SUPPORTED**, owner's corrected session, 2026-10-30).
No robot motor control, flashing, serial device access or Pi access was performed.

## Evidence and scope

**SOURCE-SUPPORTED** below means source configuration or wire layout, not a
hardware measurement. DM sources are the owner-provided, read-only DM-jump
commit `6855ef8` snapshot at `/private/tmp/AI-assistance/dmjump-ref/`.
File:line citations in the next two tables are relative to that snapshot.
The four supplied files have no Git metadata; the commit identity is provided
by the owner. Tests compile verbatim function bodies from the snapshot and
check that its driver SHA-256 remains unchanged.

| Parameter / layout | Value | Evidence label and source |
|---|---|---|
| Mode ID offsets | MIT `0x000`, position `0x100`, speed `0x200`; add motor ID | SOURCE-SUPPORTED `DM_Motor/dm4310_drv.h:29-31`; addition `DM_Motor/dm4310_drv.c:151,174,202,236,269,298` |
| DM-J4310 ranges | p ±12.5 rad, v ±30 rad/s, t ±10 Nm | SOURCE-SUPPORTED `DM_Motor/dm4310_drv.h:34-43` |
| DM-H6215 ranges | p ±12 rad, v ±45 rad/s, t ±18 Nm | SOURCE-SUPPORTED `DM_Motor/dm4310_drv.h:46-55` (source calls it DM6215) |
| Both MIT gains | kp 0..500 Nm/rad, kd 0..5 Nm·s/rad | SOURCE-SUPPORTED `DM_Motor/dm4310_drv.h:38-41,50-53` |
| Conversion | truncate `(x-lo)*(2^bits-1)/(hi-lo)`; inverse `u*(hi-lo)/(2^bits-1)+lo` | SOURCE-SUPPORTED `DM_Motor/dm4310_drv.c:50-69` |
| MIT payload | p16, v12, kp12, kd12, t12; big-endian bit fields | SOURCE-SUPPORTED `DM_Motor/dm4310_drv.c:204-217,238-251` |
| Enable | seven `FF` bytes then `FC`, 8-byte payload, mode + ID | SOURCE-SUPPORTED `DM_Motor/dm4310_drv.c:148-162` |
| Disable | seven `FF` bytes then `FD`, 8-byte payload, mode + ID | SOURCE-SUPPORTED `DM_Motor/dm4310_drv.c:171-185` |
| Feedback payload | byte 0 state high nibble / ID low nibble; bytes 1..2 p16; byte 3 + high nibble 4 v12; low nibble 4 + byte 5 t12; bytes 6/7 MOS/coil °C | SOURCE-SUPPORTED `DM_Motor/dm4310_drv.c:102-115,125-138` |
| Save-zero | not implemented by this bench; do not send a guessed opcode | SOURCE-SUPPORTED gap: supplied `DM_Motor/dm4310_drv.c:1-307` and `.h:1-125` contain no save-zero function/opcode |
| CAN reply arbitration ID | `0x700 + motor_id`, standard 11-bit, DLC 8 | ASSUMED isolated-bench routing; reference supplies feedback payload but no configured reply IDs |
| Range clipping | clip to the applicable range before conversion | ASSUMED bench extension; reference conversion itself does not clip |

| Selected fake motor | Original DM-jump placement / ID | Evidence |
|---|---|---|
| Hip A, DM-J4310 | right rear, FDCAN1, ID 6 | SOURCE-SUPPORTED `chassisR_task.c:105,120`; transmit bus `chassisR_task.c:78-81` |
| Hip B, DM-J4310 | right front, FDCAN1, ID 8 | SOURCE-SUPPORTED `chassisR_task.c:106,115`; transmit bus `chassisR_task.c:78-81` |
| Wheel R, DM-H6215 | FDCAN1, ID 0 | SOURCE-SUPPORTED `chassisR_task.c:107,125,83` (executable initialization is 0 despite its comment mentioning 1) |
| Wheel L, DM-H6215 | FDCAN2, ID 1 | SOURCE-SUPPORTED `chassisL_task.c:89,107,64` |

**ASSUMED topology:** CAN1 is the command/feedback endpoint; CAN2 emulates all
four selected motors on one joined wire. The full DM-jump robot also has left
hip IDs 6/8 on FDCAN2 (`chassisL_task.c:87-88,60-62`); those extra hips are
omitted from this four-motor task. No motor ID was invented or renumbered.
This reduced topology does not reproduce the robot's two separate buses.

## Build and host verification

```sh
make -C tools/bench_twin_fw
python3 -m unittest discover -s tools/bench_twin_fw/tests -v
python3 tools/lint_markdown.py tools/bench_twin_fw/README.md tools/bench_twin_fw/WORKLOG.md tools/bench_twin_fw/REVIEW.md
/usr/local/bin/arm-none-eabi-size tools/bench_twin_fw/build/bench_twin.elf
(cd tools/bench_twin_fw/firmware && shasum -a 256 -c SHA256SUMS)
```

The default compiler is `/usr/local/bin/arm-none-eabi-gcc`; output is
`firmware/bench_twin.bin` and `firmware/SHA256SUMS`. No HAL, RTOS, libc or
micro-ROS is linked. Canonical `tools/link_proto` is included through tiny
wrappers, avoiding an independent protocol fork. Host tests use `cc`, ctypes
and Python unittest; no pip packages or hardware are needed. Override
`DMJUMP_REF` only to another read-only checkout of the same reference.
Missing reference driver .c/.h files skip only the three reference comparisons
with an explicit DMJUMP_REF message; firmware/host regression tests still run.

Tests execute the actual bench C pack/unpack, model, line parser/formatter and
mixed UART receiver. They compile the reference converters, MIT packers and
feedback decoders with a stub CAN-send boundary, then compare bytes/values.
They also exercise clipping, enable/disable, saturation, advisory session/sequence
rejection, CRC/nonfinite rejection, sequence wrap, stale LEDs, overflow recovery,
byte-at-a-time multiplexing and the existing `tools.imu_view.stream` parser.
Build/test observations are **MEASURED on the Mac only**, recorded in WORKLOG.

## UART records and multiplexing

**C1-phase-9 application configuration:** USART1 PA9/PA10, nominal **460800 baud
8N1**, HSI 16 MHz. Divider 35 gives approximately 457143 baud (-0.79%, calculated).
`flash_safe` and the ROM bootloader still use **115200 8E1**; this change only
sets the running application baud. No bootloader tool was changed.
All text grammar, rates and queue limits below are **ASSUMED bench choices**.

| Direction | Record | Meaning |
|---|---|---|
| Host → MCU | `C,seq,id,p,v,kp,kd,t\n` | MIT target: rad, rad/s, Nm/rad, Nm·s/rad, Nm |
| Host → MCU | `E,seq,id\n` / `D,seq,id\n` | Send DM enable / disable special frame |
| Host → MCU | `00 + link_proto TerrainAdvisory wire` | Optional leading zero, then unchanged COBS/CRC type 4 packet ending in zero |
| MCU → host | `I,ms,fresh,ax,ay,az,temp,gx,gy,gz\n` | Exact imu_can_bench order and raw signed int16 values; fresh=1 for data-ready samples |
| MCU → host | `M,ms,id,pos,vel,tor,temp,err\n` | Three decimals for rad/rad/s/Nm; integer MOS °C; `err` is the raw DM state nibble |
| MCU → host | `STAT key=value ...\n` | Once per second, cumulative counts and interval latency statistics |
| MCU → host | `00 + link_proto AdvisoryEcho wire + LF` | Unchanged type 5 packet inside a mixed-stream envelope |

IDs are decimal 6, 8, 0, 1. `seq` is uint32, global across C/E/D. Only successfully
queued commands advance its high-water mark; retries use a newer sequence after
acceptance. Rejected/busy commands are counted, with no per-command ACK; `seq`
is not present in the DM wire or M-line. Text permits signed decimal numbers
with 1..6 digits before and at most 6 after the decimal, no exponent/NaN/Inf,
no spaces; LF, CRLF and CR terminate text commands. Text buffer is 128 bytes including NUL.
Commands outside DM physical ranges clip; invalid grammar/IDs reject.
All fake motors start disabled. `host.py` supplies command/advisory encoders and
a byte-stream decoder; it never opens a serial device.

The MCU recognizes C/E/D at a record boundary as text; other nonzero starts
enter the COBS parser until zero. A raw existing type-4 `link_proto` packet is
accepted without the optional prefix. Prefix zero recovers an interrupted
binary record; finish an interrupted text record with LF or zero. Do not
interleave bytes from separate writers. UART receive uses a 1024-byte interrupt
queue, so I2C polling does not depend on per-byte main-loop service. UART output
uses a 4096-byte ring and admits or drops complete records. Counters expose
receive overflow/error and output record drops. The ISR publishes a volatile
byte plus gap marker before advancing head; main consumes it before releasing
tail. A gap resets partial parser state. RX service is capped at 256 bytes/pass
so continuous input cannot starve CAN, TX and status. Parser idle timeout is
100 ms. Binary overflow is counted immediately and exits binary mode, rather
than waiting indefinitely for a zero delimiter.

The binary echo can itself contain LF. Use `host.Decoder` before dispatching
records to consumers. Each binary output envelope ends in LF, so subsequent
I-lines remain complete and compatible with `tools/imu_view` even in a raw
newline reader; binary fragments may count as viewer garbage. A single UART
reader can distribute decoded I-lines and motor/echo records to separate tools.
This is the advisory subset: no PerceptionHealth/BodyVelocityRef controller,
20 Hz McuStatus stream, clock synchronization or motion authority is provided.

### Advisory policy and LEDs

**SOURCE-SUPPORTED bench policy:** session `0x5742524C494E4B31`, stale deadline
300 ms and stale echo period 100 ms from `tools/link_bench/firmware/bench.h:14,18-19`.
Accepted sequence must be newer modulo uint32; valid type-4 fields follow
`tools/link_proto/SPEC.md` and its canonical codec. Echo is immediate for accepted
or identifiable rejected advisories (including CRC/field errors), plus 10 Hz
while stale. Undecodable packets cannot always be associated with an advisory;
they increment parse errors. Rejection preserves the last accepted LED until
its original deadline. Stale/no advisory turns **all LEDs off**, matching
`tools/link_bench/firmware/bench.c:106-116`.

| Advisory | LED / pin | Evidence |
|---|---|---|
| NONE 0 | green D4 / PB5 | SOURCE-SUPPORTED `tools/link_bench/firmware/bench.c:111`, `tools/imu_can_bench/firmware/main.c:12-13` |
| JUMP_CANDIDATE 1 | yellow D5 / PB4 | SOURCE-SUPPORTED `tools/link_bench/firmware/bench.c:112`, IMU bench pin comment above |
| STOP_CANDIDATE 2 | red D6 / PB10 | SOURCE-SUPPORTED `tools/link_bench/firmware/bench.c:113`, IMU bench pin comment above |
| UNCERTAIN 3 | blue D9 / PC7 | SOURCE-SUPPORTED `tools/link_bench/firmware/bench.c:114`, IMU bench pin comment above |

Echo timestamps use extended MCU milliseconds, never host/source epoch subtraction.
The diagnostic header session is UID-derived and repeats across resets
(**ASSUMED limitation**); reset the host decoder and command/advisory sequence
state whenever the BOOT line appears. It is not a globally unique MCU boot ID.

### C1-phase-9 rates and Pi runner

M telemetry defaults to **20 Hz per motor**, emitted on feedback arrivals only;
it does not change `fed_back` counting or the one-pending-command gate.
Configure with `make -B -C tools/bench_twin_fw M_HZ=50` (0 disables M-lines;
valid range 0..1000). Rebuild with `M_HZ=20` for the default artifact.
`m_decimated` counts deliberately omitted feedback records.

At 100 aggregate commands/s, conservative TX budget is 100×64 I + 80×48 M
+ 20×51 echoes + 1536 STAT = **12796 B/s (27.77% of 46080 B/s)**.
RX commands/advisories budget is 100×128 + 10×65 = **13450 B/s (29.19%)**.
At 400 aggregate commands/s the conservative RX bound exceeds wire capacity;
use actual encoded lengths when choosing a different rate. These calculations
are capacity bounds, not measured firmware throughput.

Ordinary records leave **1536 bytes reserved for STAT** in the 4096-byte TX
ring. Tests bound the maximum status length (all counters at ten digits) to
939 bytes. A due STAT is retained until admission succeeds if the physical TX
stalls; `stat_defer` counts retry iterations. Latency intervals reset only when
STAT is queued. Reserved space prevents telemetry admission from dropping STAT;
it cannot guarantee delivery through a disconnected or stalled physical link.

For a future owner-run isolated fixture test (not executed by this task):

```sh
python3 -m pip install pyserial
python3 tools/bench_twin_fw/pi_loop.py /dev/ttyUSB0 --rate 100 --seconds 10 \
  --raw-log c1-t9-rx.jsonl > c1-t9-report.jsonl
```

Use a fresh application session with no other UART owner. The runner emits
**C only, p=v=kp=kd=t=0**; it never enables a motor. One thread owns reads and
`host.Decoder`; the writer uses absolute deadlines from integer schedule slots,
round-robins IDs 6/8/0/1, retries partial writes and skips overdue slots rather
than bursting. Every complete write includes timestamp, sequence and length.
Raw RX chunks have monotonic timestamps. STAT reports normalize cumulative
counter deltas by MCU elapsed time, including sent/fed_back/rejected and drop
counters. BOOT/reset, decoder errors, serial failures and 3 seconds without STAT
fail the run. It drains for 2.2 seconds after writing; first STAT is a baseline,
so per-second reports do not account for commands sent before that baseline.
Writer totals count host write completions, not proof of MCU receipt.

New diagnostics: `rx_bytes` counts SR/DR receive/error events, `rx_queued` and
`rx_processed` trace the byte queue; `rx_drop` counts full-queue bytes and
`rx_error` hardware error events (overrun can represent more than one lost wire
byte). `text_ok` must equal `sent + rejected` modulo uint32. `rx_gap` counts
parser resets at queued gap markers. `binary_bytes`, `binary_overflow`,
`mux_timeout`, `text_overflow`, `parse_error` expose parser routing/loss.
`uart_drop` counts whole ordinary records refused at TX admission;
`tx_byte_drop` counts defensive character overflow; `m_format_drop` and
`echo_encode_drop` cover formatter failures. `m_decimated` is intentional
telemetry suppression, not CAN loss. These counters overlap; do not sum all
of them as unique lost commands.

## CAN/model/counter behavior

CAN1 sends standard-ID MIT and special frames. CAN2 recognizes the selected
IDs, applies enable/disable and holds each last MIT target. It steps at 1 ms,
then responds to each received command using the DM feedback layout. Normal
CAN operation needs external transceivers and a connected/terminated bus;
this image does not silently substitute bxCAN internal loopback.

**ASSUMED model parameters:** velocity lag 0.05 s; torque-to-speed damping
0.2 Nm·s/rad; explicit Euler step 0.001 s. Effective torque is
`t_eff = clip(kp*(p_cmd-p) + kd*(v_cmd-v) + t_cmd, ±Tmax)` while enabled, zero
while disabled. Update `v += dt*(t_eff/0.2-v)/0.05`, clip v to its motor range,
then `p += dt*v`, clip p. Model catch-up is capped at 10 steps per service pass;
skipped milliseconds are counted. Position clipping is a numeric bound, not a
simulated mechanical stop. State nibble 0=disabled, 1=enabled and both
25 °C temperatures are **ASSUMED synthetic diagnostics**, not measured motor
state/error semantics or thermal physics. Disable clears target/torque and lets
velocity decay. Enable also clears the held target. No command watchdog,
thermal protection, back EMF, current loop, gearing or persistence is modeled.

Worked **CALCULATED** example: enabled, p=v=0, kp=kd=0, t=1 Nm gives
`v_next=0.001*(1/0.2)/0.05=0.1 rad/s`, `p_next=0.0001 rad` before command
quantization. The host test uses the decoded torque and checks the actual
100-step C trajectory against this recurrence.

`sent` counts CAN1 mailbox admissions, `fed_back` counts matching feedback,
`lost` counts commands still pending at 100 ms, and `pending=sent-fed_back-lost`
is **CALCULATED** modulo uint32. There is at most one outstanding command per ID.
On timeout, abort the associated CAN1 mailbox retry. Busy or old-sequence commands
increment `rejected`; malformed lines increment `parse_error`. `unsolicited`
counts valid feedback with no timely pending command. `c1_txok/txerr`, both
FIFO overrun counts, ESR words, and `fb_busy` expose controller/return congestion.
Feedback with extended ID, RTR or DLC other than 8 is not accepted.

Send-to-feedback latency is **MEASURED only when run on the bench**, from DWT
cycles immediately before CAN1 mailbox enqueue to CAN1 FIFO read, divided by
16 cycles/µs. It includes polling/I2C service delay; it is not a bus-analyzer
wire latency. `lat_last_us` retains the last observation; mean/max/n refer to
the last one-second interval. Subtract cycles before division to handle DWT
wrap. No measurements exist yet. DM carries no command sequence in feedback:
matching assumes isolated immediate fake responses; a sufficiently late reply
could alias a later command for the same ID. The nominal 5 Hz/ID spacing exceeds
the 100 ms timeout but is not proof against arbitrary delayed traffic.

## Wiring and owner-only flash procedure

| Function | F446RE pins / setting | Evidence |
|---|---|---|
| USART1 TX / RX | PA9 / PA10, 3.3 V UART, common GND | SOURCE-SUPPORTED `tools/imu_can_bench/firmware/main.c:383-385`; external 3.3 V adapter ASSUMED |
| I2C1 SCL / SDA | PB6 / PB7, IMU sensor IMU sensor | SOURCE-SUPPORTED `tools/imu_can_bench/firmware/main.c:173-175`; device discovery/config `:211-226` |
| IMU sensor configuration | addresses 0x68/0x69; WHO_AM_I 0x68; 100 Hz; ±4 g, ±500 °/s | SOURCE-SUPPORTED `tools/imu_can_bench/firmware/main.c:211-226` |
| CAN1 RX / TX | PB8 / PB9 to transceiver A RXD / TXD | SOURCE-SUPPORTED `tools/imu_can_bench/firmware/main.c:276-282` |
| CAN2 RX / TX | PB12 / PB13 to transceiver B RXD / TXD | SOURCE-SUPPORTED same source lines |
| Joined bench wire | A CANH↔B CANH, A CANL↔B CANL, common GND, appropriate end termination | ASSUMED isolated fixture; no real motors connected |
| CAN timing | 500 kbit/s, 16 time quanta, 87.5% sample point, HSI 16 MHz | SOURCE-SUPPORTED inherited configuration `tools/imu_can_bench/firmware/main.c:76-81`; 500k selected ASSUMED bench-only |

**Owner only; never on the robot.** The runner did not flash and must not flash.
Only the maintainer/owner runs any later physical checks on an isolated Nucleo.

1. Disconnect all real motor/robot CAN wiring. Use only the isolated transceiver
   pair, sensor and LEDs. Stop all readers/writers of the selected UART.
2. Verify `SHA256SUMS`. Before any erase/write, back up the **current** complete
   512 KiB flash twice, compare the reads byte-for-byte, and archive both reads
   and hashes. Keep that backup independent of this image.
3. For ROM UART access, fit **BOOT0 before pressing RESET**. Do not infer ROM
   entry from a random `0x79` in running application output. Use the existing
   `tools/link_bench/pi/stm32boot.py` backup/readback workflow only under owner
   control. That tool's flash path backs up twice before erasing; the runner
   has not run it or opened any UART device.
4. After a verified backup, owner writes `firmware/bench_twin.bin` at flash base
   `0x08000000` and performs binary readback verification. Remove BOOT0 and then
   RESET to boot the image. This procedure is documentation, not runner flash
   authorization. There is no UART-triggered reset/flash command in this image.
5. Owner observes BOOT, live I-lines, all four LED classes and stale-off behavior,
   then enable/C/disable feedback and STAT on the isolated CAN fixture. Preserve
   raw logs; temperature/model values remain synthetic regardless of success.

Flash capacity/base and backup ordering are **SOURCE-SUPPORTED** by
`tools/can_monitor/firmware/f446.ld:5-6`,
`tools/imu_can_bench/pi/flash.sh:2-10`, and
`tools/link_bench/pi/stm32boot.py:51-66`; physical setup is **ASSUMED** until verified.

## Optional AGV codec

RobStride remains only as an **optional compile-time AGV codec**, excluded by
the default build. `make -B -C tools/bench_twin_fw AGV_CODEC=1` compiles the legacy
codec with `BENCH_OPTIONAL_AGV_CODEC=1`; unreferenced functions are discarded
from the image. This does not select an AGV runtime or change the DM loop.
The legacy codec supports experimental type-1/type-2 packing only.
Its **SOURCE-SUPPORTED** constants/layout come from
`tools/can_monitor/firmware/robstride_decode.c:4-16,19-21,32-44,54-58`
(position ±12.57, feedback velocity ±30, torque ±60, unsigned 16-bit scaling,
identifier fields, payload byte order, fault bits, temperature ×0.1 °C),
`tools/can_monitor/firmware/robstride_decode.h:7` (types 1/2), and
`docs/lessons/01_can_and_robstride.md:196-203` (truncated zero code `0x7fff`).

**SOURCE-SUPPORTED by owner correction, 2026-10-30:** the AGV uses RobStride at
1 Mbit/s, ID 1 on each bus, master 0xFD, velocity mode. Its velocity-mode
configuration/lifecycle is not implemented here. This DM image remains
**not micro-ROS, not AGV-ready, bench 500k** with either compile option.
