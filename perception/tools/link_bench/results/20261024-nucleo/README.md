# 2026-10-24 first real-UART run, Pi 5 ↔ Nucleo-F446RE (MEASURED)

Wiring: Pi pin 8/10/6 → D2/D8/GND, Nucleo powered from Pi pin 2 → E5V (JP5 on E5V), no USB.
Flashed from the Pi over the ROM UART bootloader with `pi/stm32boot.py` (keeps the port open;
`stm32flash` failed to sync, INFERRED: TX glitch on port open). The OS-project firmware that was on the
board was backed up first: 512 KB, read twice identical, SHA in `os-firmware-backup.SHA256SUMS`, file in
`~/wbr-evidence-archive/nucleo-backup-20261024/` (Mac) and `~/wbr-evidence/` (Pi).
The bench firmware does not run when started with the bootloader GO command (no VTOR set, INFERRED);
it runs after a normal reset with BOOT0 low.

| run | result |
|---|---|
| health bench (`link_bench.py`) | 3000/3000, RTT P50/P99 4.07/4.08 ms; CRC, duplicate, session faults 10/10 rejected; 100 ms gap held; 300 ms gap → NO_LINK after 150.0 ms MCU silence, recovered 5/5 |
| advisory synthetic 15 Hz × 60 s + faults | 900/900, RTT P50/P99 2.81/2.82 ms; WIRE, SEQUENCE, SESSION, FIELDS each rejected; 400 ms silence → stale, recovered |
| advisory, B-H08 bag replayed on the Pi | 139/139, detector + decision P50/P95 78/83 ms, RTT P50/P99 2.82/4.24 ms |

All times are Pi CLOCK_MONOTONIC. Camera-to-Pi delivery latency is not included (no clock mapping).
The first advisory runs read ~66–81 ms RTT because the sender slept without reading echoes; fixed
(drain while waiting) before the numbers above. Logic-analyzer timing not done yet.
