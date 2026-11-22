#!/usr/bin/env python3
"""STM32 ROM UART bootloader (AN3155) over one continuously open port: sync, backup, flash.

stm32flash reopens the port per run; a TX glitch on open can lock the bootloader's
autobaud on a bad byte. This keeps the port open from before the RESET to the end.
Writes nothing unless the full backup reads back identically twice.
"""
import argparse, hashlib, pathlib, sys, time
import serial

ACK, NACK = 0x79, 0x1F

class Boot:
    def __init__(s, port, baud): s.p = serial.Serial(port, baud, parity=serial.PARITY_EVEN, timeout=1)
    def ack(s, timeout=2):
        s.p.timeout = timeout; b = s.p.read(1)
        if b != bytes([ACK]): raise IOError(f"expected ACK, got {b.hex() or 'nothing'}")
    def cmd(s, c): s.p.write(bytes([c, c ^ 0xFF])); s.ack()
    def addr(s, a):
        b = a.to_bytes(4, "big"); s.p.write(b + bytes([b[0] ^ b[1] ^ b[2] ^ b[3]])); s.ack()
    def sync(s, seconds):
        end = time.time() + seconds
        while time.time() < end:
            s.p.reset_input_buffer(); s.p.write(b"\x7f"); s.p.timeout = 0.3; b = s.p.read(1)
            if b in (bytes([ACK]), bytes([NACK])): return b.hex()
        raise IOError("no bootloader reply")
    def get_id(s):
        s.cmd(0x02); n = s.p.read(1)[0]; pid = s.p.read(n + 1); s.ack(); return pid.hex()
    def read(s, a, n):
        s.cmd(0x11); s.addr(a); s.p.write(bytes([n - 1, (n - 1) ^ 0xFF])); s.ack()
        s.p.timeout = 2; d = s.p.read(n)
        if len(d) != n: raise IOError(f"short read at {a:#x}")
        return d
    def dump(s, base, size):
        out = bytearray()
        for a in range(base, base + size, 256):
            out += s.read(a, 256)
            if (a - base) % 65536 == 0: print(f"  read {a - base >> 10} KB", flush=True)
        return bytes(out)
    def mass_erase(s):
        s.cmd(0x44); s.p.write(b"\xff\xff\x00"); s.ack(timeout=60)
    def write(s, a, data):
        s.cmd(0x31); s.addr(a); n = len(data); chk = n - 1
        for x in data: chk ^= x
        s.p.write(bytes([n - 1]) + data + bytes([chk])); s.ack()
    def go(s, a): s.cmd(0x21); s.addr(a)

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--port", default="/dev/ttyAMA0"); ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--out", required=True); ap.add_argument("--flash"); ap.add_argument("--sync-seconds", type=float, default=180)
    ap.add_argument("--size", type=int, default=512 * 1024)
    a = ap.parse_args(); out = pathlib.Path(a.out); out.mkdir(parents=True, exist_ok=True)
    b = Boot(a.port, a.baud); base = 0x08000000
    print("waiting for bootloader (press RESET with BOOT0 high)...", flush=True)
    print("sync reply", b.sync(a.sync_seconds)); print("product id", b.get_id(), flush=True)
    d1 = b.dump(base, a.size); d2 = b.dump(base, a.size)
    nonff = sum(x != 0xFF for x in d1); sha = hashlib.sha256(d1).hexdigest()
    (out / "flash-backup.bin").write_bytes(d1); (out / "SHA256SUMS").write_text(f"{sha}  flash-backup.bin\n")
    print(f"backup {len(d1)} bytes, non-0xFF {nonff}, identical re-read {d1 == d2}, sha256 {sha}", flush=True)
    if d1 != d2 or nonff < 1024 or len(d1) != a.size: sys.exit("BACKUP NOT USABLE - not flashing")
    if not a.flash: return
    img = pathlib.Path(a.flash).read_bytes(); img += b"\xff" * (-len(img) % 256)
    print("mass erase...", flush=True); b.mass_erase()
    for off in range(0, len(img), 256): b.write(base + off, img[off:off + 256])
    back = b.dump(base, len(img)); print("verify", back == img, flush=True)
    if back != img: sys.exit("VERIFY FAILED")
    b.go(base); print("started new firmware at", hex(base))

if __name__ == "__main__": main()
