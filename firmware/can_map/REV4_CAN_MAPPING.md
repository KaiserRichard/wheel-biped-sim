# CTRL rev4 CAN mapping

**Tóm tắt [FROM FILE].** Firmware dùng CAN1 cho cả chân phải, CAN2 cho cả chân trái; CAN3 để dự phòng. Thứ tự sáu trục và CAN ID duy nhất được giữ nguyên.

**[FROM FILE — `hardware/CTRL/CTRL.kicad_pcb` connector values J5/J6/J7; `firmware/{can_sched,damiao_mit}.c`; `firmware/target/target_fdcan.c`]**

| Connector | Peripheral / bus_idx / can_bus | Role | Axes | Command IDs | Feedback IDs | RX / TX |
|---|---|---|---|---|---|---|
| J5 | FDCAN1 / 0 / 1 | Right leg | 3 front, 4 rear, 5 wheel | 0x004, 0x005, 0x006 | 0x014, 0x015, 0x016 | PD0 / PD1 AF9 |
| J6 | FDCAN2 / 1 / 2 | Left leg | 0 front, 1 rear, 2 wheel | 0x001, 0x002, 0x003 | 0x011, 0x012, 0x013 | PB5 / PB6 AF9 |
| J7 | FDCAN3 / 2 / 3 | Spare | None | None | None | PD12 / PD13 AF5 |

**[FROM FILE — `target_fdcan.c`, `target_fdcan_init`]** Each active bus has two
dual-ID filters: its first two reply IDs, then its third ID duplicated. J7 has
zero standard filters. Global filters reject unmatched standard, extended and
remote frames on every bus. The software dispatcher also rejects wrong-leg and
spare traffic. The low-nibble motor-ID check remains in the hardware RX adapter.

**[FROM FILE — `can_sched.c`; `target_fdcan.c`]** Normal torque, enable,
immediate disable, asynchronous abort completion, and the +1 ms disable repeat
all cover three motors per active bus. No motor frames are sent on J7.
FDCAN3 remains initialized and monitored; a reported bus-off on any peripheral
keeps the conservative whole-system fault policy. Queue capacity remains
eight frames per bus; message RAM offsets remain 0/256/512 words.

**[COMPUTED — arithmetic below]** Moving from two to three motors on an active
bus changes wire occupancy. At 500 Hz, three command and three feedback frames
per 2 ms require 3000 frames/s per leg, assuming one reply per command. Using a
conservative 135 bits/frame at 1 Mbit/s gives 40.5% occupancy before other
traffic. This is arithmetic, not a timing measurement or a bus-load simulation.
Earlier latency results for the previous two-motors-per-bus split do not
validate this harness.

**[UNKNOWN]** The physical harness, configured drive command/master IDs, wheel
register limits, termination, reply cadence and loaded-bus latency must be
confirmed before powered operation. The board connector labels do not prove the
actual assembly. USB enumeration and IMU sensor integration remain separate
bring-up items.
