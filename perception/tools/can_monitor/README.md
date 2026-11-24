# Passive dual bus CAN monitor (Nucleo-F446RE)

This node listens to the AGV's two independent 1 Mbit/s CAN buses so the lab can
see what the STM32H723 sends and what the RobStride motors answer. Use it to
debug command/feedback rates, motor speed/position/torque/temperature, reported
faults, and CAN errors.

## Safety: receive-only

**Both bxCAN controllers are configured with `CAN_BTR.SILM = 1` (SILENT
listen-only mode). They do not transmit, acknowledge received frames, or send
error frames. The firmware contains no CAN transmit function or TX code path.**
All IDs are accepted by zero-mask filters. CAN2 uses filter banks beginning at
`CAN2SB=14`.

## Wiring

Use a T connection at each motor's existing cable so the H7 and motor remain
connected. The H7 has two physically independent buses; keep them separate:

| Monitor | Nucleo-F446RE pins | Connect to |
|---|---|---|
| CAN1 | PB8 RX, PB9 TX pin mux | Left motor CAN bus |
| CAN2 | PB12 RX, PB13 TX pin mux | Right motor CAN bus |
| Both transceivers | CANH, CANL, GND | Corresponding bus CANH/CANL and common signal ground |
| Power | Nucleo 3V3 and GND | SN65HVD230 logic supply and ground |
| Agent UART | PA9 USART1 TX, PA10 USART1 RX, GND | Pi GPIO UART RX, TX, GND respectively |

The SN65HVD230 modules are 3.3 V transceivers. Tap CANH/CANL/GND only into
each existing, terminated bus. **Do not add a termination resistor at the
monitor.** The monitor must not connect the two motor buses together. Check
module pin labels and logic supply before applying power; transceiver TXD is
driven by the MCU peripheral pin but the bxCAN controllers stay silent.
Connect each module's receiver output (`RO`/`RXD`) to its listed MCU RX pin,
and its logic input (`DI`/`TXD`) to the listed MCU TX pin. The module's CANH and
CANL join only the corresponding bus; module GND, Nucleo GND and bus signal GND
must be common.

The agent listens on the Pi's `/dev/ttyAMA0` at 115200 baud, 8-N-1. Connect
Nucleo PA9 (TX) to Pi RX, PA10 (RX) to Pi TX, and grounds together. Do not
connect the Nucleo UART pins to 5 V logic.

## Build

Build with the local micro-ROS archive. Override `LIBROOT` if it is elsewhere:

```sh
cd tools/can_monitor
make LIBROOT="$HOME/Desktop/Research/agv-libbuild/micro_ros_stm32cubemx_utils/microros_static_library/libmicroros"
```

The build writes `firmware/prebuilt/can_monitor.bin` and
`firmware/prebuilt/SHA256SUMS`, and prints the ELF size. Run host decoder unit
tests with `make test`.

## Flashing (bench step only)

This session did not flash hardware. When the owner proceeds with the physical
bench step, the documented ST-Link command is:

```sh
st-flash --connect-under-reset --freq=100k --reset write firmware/prebuilt/can_monitor.bin 0x08000000
```

The Nucleo UART bootloader is an alternative. Select its USART1 boot pins and
boot procedure from the STM32F446RE documentation; the monitor application
uses PA9/PA10 after startup.

## Start the Pi agent and viewer

On the Pi with ROS 2 Jazzy and `micro_ros_agent` installed:

```sh
source /opt/ros/jazzy/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyAMA0 -b 115200
```

In another sourced ROS 2 shell:

```sh
python3 tools/can_monitor/view_can_monitor.py --csv can-monitor.csv
```

The helper prints rates, commanded speed, feedback position/velocity/torque,
temperature, error counters and fault/mode values. Omit `--csv` to disable
recording.

## Topics and statistics layout

- `/can_monitor/motors`, `sensor_msgs/msg/JointState`, 20 Hz: names are
  `left`, `right`; position (rad), velocity (rad/s), and effort (Nm) use the
  latest RobStride feedback frame for each bus. Before the first feedback,
  values are zero.
- `/can_monitor/stats`, `std_msgs/msg/UInt32MultiArray`, 5 Hz: data is two
  consecutive 15-element records, first CAN1/left then CAN2/right. Indexes:

| Index | Value | Units / encoding |
|---:|---|---|
| 0 | All received frames | frames/s, 200 ms sample × 5 |
| 1 | Feedback and active-report frames | frames/s |
| 2 | Command and parameter-write frames | frames/s |
| 3 | Last commanded speed | rad/s × 1000, signed 32-bit two's-complement carried in uint32 |
| 4 | Transmit error counter (TEC) | CAN error counter |
| 5 | Receive error counter (REC) | CAN error counter |
| 6 | Last error code (LEC) | raw 3-bit CAN_ESR code |
| 7 | Bus-off | 0 or 1 from CAN_ESR |
| 8 | RX FIFO0 overruns | cumulative count |
| 9 | RobStride decode failures | cumulative count for recognized frame types |
| 10 | Last feedback fault bits | raw 6-bit field |
| 11 | Last feedback mode/status | raw 2-bit field |
| 12 | Latest feedback temperature | °C × 10 |
| 13 | Last commanded run mode | raw value written to `RUN_MODE (0x7005)` |
| 14 | Last type-21 fault-frame word | raw big-endian bytes 0..3 as uint32; no further H7 field interpretation is defined here |

RobStride 29-bit `rs_ext_*` types decoded are get-ID (0), control (1), feedback (2),
enable (3), stop (4), set-zero (6), parameter read (17), parameter write
(18), fault feedback (21), save-parameter (22), set-baud (23), active report
(24), protocol-switch (25), and version-read (26). Feedback follows the H7
driver: position `[-12.57, 12.57]` rad, feedback velocity `[-30, 30]` rad/s,
torque `[-60, 60]` Nm, and the last two data bytes are temperature in tenths
of °C. Type-18 writes for `RUN_MODE (0x7005)` and `SPD_REF (0x700A)` are
decoded; type-1 control frames' packed velocity command is also reflected in
the last commanded speed. Type-1 frames decode packed torque, position,
velocity, Kp, and Kd. The most recent type-18 run-mode write is available as
stats index 13. Bytes 0..3 from the latest type-21 fault frame are exposed raw
at index 14. The rate counter counts command frames and all parameter writes.

## Bench checks

1. **No-robot check:** power the Nucleo and start the agent. Confirm the
   `can_monitor` node and both topics appear, and that stats remain at 0 frames/s
   with zero overruns. Wiring the two monitor modules to each other is not a
   valid traffic test: both are silent, neither acknowledges nor transmits, so
   there is no frame source. Do not inject traffic with this monitor.
2. **On-robot check:** place the robot securely on a stand with drive wheels
   clear of the floor. Verify the existing H7/motor connections remain intact,
   add each monitor T separately, and check bus power and common ground before
   powering the motor system. Start the Pi agent/viewer, then power the system
   according to the lab's normal procedure. Confirm both buses show frames,
   feedback rates, plausible temperatures and changing feedback only when the
   H7 normally commands motion. Compare logged commanded speeds against H7
   behavior. Stop and inspect wiring if either bus goes bus-off or errors rise.

## Foxglove

Add `/can_monitor/motors` and select Plot. Plot `velocity[0]` and `velocity[1]`
for left/right wheel speed. `position[0..1]` and `effort[0..1]` are also
available. Add `/can_monitor/stats` as a table or raw message view for rates,
error counters and temperatures (temperature is index 12 of each bus record,
divided by 10).

## Verification status

Host decoder tests and cross-compilation are local software checks. UART
interoperability, silent electrical behavior on a physical bus, controller
timing, real H7 frame capture, message-rate stability, and on-robot temperature
and error interpretation remain **UNVERIFIED** until the bench checks are
performed.
