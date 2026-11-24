# Session facts addendum — 2026-10-14 (read with 2026-10-13-SESSION_FACTS_FOR_SLIDES.md)

Labels: MEASURED / INFERRED / UNKNOWN. Nothing here is a gate PASS. Evidence lives on the Pi under `~/wbr-evidence/2026-10-14-*` unless noted.

1. **8 GB Pi works with the same card** (MEASURED): Raspberry Pi 5 Rev 1.1, 7.8 GiB RAM; the 32 GB card boots unchanged (same software, IP pi-host on Ethernet, Wi-Fi profiles incl. "M625 Mechatronic Lab"). Under-voltage already within 3 min of boot on the same 5 V/3 A supply → power is not fixed by the board change.
2. **Camera streams fine for 19 s** (MEASURED, 20 s still take): IR1/IR2 30 Hz, gyro, accel delivered; one ~0.7 s gap in the final second at shutdown.
3. **First 100 s hand-moved take unusable** (MEASURED): gaps of 0.3–2.9 s every ~5 s after 22 s. Cause (INFERRED, strong): the microSD could not sustain the write rate (1.66 GB in 100 s = 16.6 MB/s vs 18.4 MB/s needed) → recorder blocked → frames dropped. No under-voltage or USB errors in the kernel log during the take.
