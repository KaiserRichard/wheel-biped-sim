#!/usr/bin/env python3
"""Reflash while a bench image (ver>=2) runs and BOOT0 stays fitted, with no RESET press.

One port open for the whole run: send "BOOTLOADER" at the app's 115200 8N1 (the app does a
system reset, which lands in the ROM bootloader because BOOT0 is high), switch the same port
to 57600 8E1 and continue exactly like stm32boot.py (double backup, erase, write, verify, GO).
Reopening the port after the bootloader has started can lock its autobaud on a glitch.
Usage: flash_magic.py --out DIR --flash IMAGE [--port /dev/ttyAMA0]
"""
import sys, time
import serial
import stm32boot

real_init = stm32boot.Boot.__init__


def init(self, port, baud):
    real_init(self, port, baud)
    self.p.baudrate, self.p.parity = 115200, serial.PARITY_NONE
    for c in b"BOOTLOADER":  # 5 ms per byte: the app polls RX between I2C reads
        self.p.write(bytes([c])); self.p.flush(); time.sleep(0.005)
    time.sleep(0.5)
    self.p.baudrate, self.p.parity = baud, serial.PARITY_EVEN
    self.p.reset_input_buffer()
    print("magic sent", flush=True)


stm32boot.Boot.__init__ = init
if "--sync-seconds" not in sys.argv:
    sys.argv += ["--sync-seconds", "20"]
stm32boot.main()
