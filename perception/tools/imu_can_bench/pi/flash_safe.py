#!/usr/bin/env python3
"""stm32boot.py whose sync ignores a running app: an ACK/NACK byte counts only if no other
byte follows within 150 ms (app output can contain 0x79 = ACK). Same arguments as stm32boot.py."""
import time
import stm32boot


def quiet_sync(self, seconds):
    end = time.time() + seconds
    while time.time() < end:
        self.p.reset_input_buffer(); self.p.write(b"\x7f"); self.p.timeout = 0.3
        b = self.p.read(1)
        if b in (bytes([stm32boot.ACK]), bytes([stm32boot.NACK])):
            self.p.timeout = 0.15
            if not self.p.read(16):
                return b.hex()
    raise IOError("no bootloader reply")


stm32boot.Boot.sync = quiet_sync
stm32boot.main()
