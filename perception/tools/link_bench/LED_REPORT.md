# TerrainAdvisory LED report

Implemented the USART1 advisory indication on the Nucleo-F446RE bench firmware.
The pure `bench_led_mask()` function maps the last accepted, non-stale advisory
to exactly one LED. No advisory or a stale advisory gives mask zero; the main
loop blinks RED at 2 Hz while the mask is zero. Rejected CRC and old-sequence
packets preserve the prior indication. The LEDs are display-only; the firmware
still sends no actuator output.

## Build and image

Command: `make -C tools/link_bench/firmware`

| Section | Bytes |
|---|---:|
| text | 7568 |
| data | 0 |
| bss | 5120 |
| ELF total (text + data + bss) | 12688 |
| prebuilt binary | 7620 (post-d29beae; 7572 before) |

Current (post-d29beae) SHA-256 of `firmware/prebuilt/link_bench_f446.bin`:

`bf5721b44c0d9eafa4b5b53fb75f1ceec8c6e3c72a0c81d5a6bfd7635fe003ed`

Pre-d29beae image SHA-256:

`07f0af8dfe1759e76432205df35527ce8a6febfdf3804cfe6ebd74fd0266551a`

## Host verification

`make -C tools/link_bench test` passed. The dedicated C test checks all four
advisory values, the initial empty state, stale clearing, and preservation after
a bad CRC and an old sequence. The Python suite passed 9 tests; the QEMU test was
skipped because `qemu-system-arm` and `arm-none-eabi-gcc` are not both available
to that test's runtime environment. No board flashing or physical LED test was
performed.
