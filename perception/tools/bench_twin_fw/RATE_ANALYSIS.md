# C1-phase-8: phase-3 command throughput — 2026-10-30

Scope: read-only source analysis plus proposed changes; no firmware/host edits,
commit, device access or flash. Hardware figures below are owner-reported.
All source paths below are relative to `tools/bench_twin_fw/`.

## Finding: the supplied counters do not establish a 32 cmd/s limit

- `firmware/main.c:319-330` increments cumulative `sent` once per successful CAN1
  mailbox admission, including E/D. `main.c:420-434` resets only interval latency
  accumulators, not sent/fed_back. Thus `sent=32` means 32 since boot, about
  3.2/s over a confirmed 10 s boot interval, not 32/s. Use STAT deltas and `ms`.
- `fed_back=32,lost=0,pending=0` accounts for every admitted command. With
  `rejected=parse_error=rx_drop=rx_error=0`, this image has no recorded mechanism
  silently turning ~1,000 complete, valid C-lines into 32 admissions.
- `115 M` records versus cumulative `fed_back=32` needs `unsolicited`, BOOT/reset
  boundaries and raw timestamps: M is emitted even for unsolicited feedback
  (`main.c:381-391`). The totals alone cannot identify a firmware bottleneck.
- `host.py:1-24,35-72` supplies codecs only, not the hardware sender/reader loop.
  Its timing, actual returned write lengths, read blocking and log coverage are
  unavailable. A requested write rate is not a measured delivery rate.
- `c1_txok=27` is not an independent exact frame counter: completion bits are
  polled after RX processing (`main.c:298-305,401`); a mailbox can be reused before
  polling and multiple completions can collapse into one observed status bit.

## Candidate mechanisms, with source evidence

| Candidate | Actual behavior and implication |
|---|---|
| Per-ID in-flight gate | `main.c:313-330,385-399`: one pending command/ID, cleared on reply or 100 ms timeout; busy increments `rejected`. A 5,749 us mean suggests ~174/s/ID if representative, not a 32/s ceiling; mean is not a worst-case bound. At 25/s/ID the interval is 40 ms. |
| Three CAN TX mailboxes | `main.c:272-282,328`: a four-line burst can reject its fourth command if all three are occupied. Counted as busy/rejected; not silent coalescing. |
| RX buffer/parser | `main.c:130-141,362`: 1,023 usable RX bytes, interrupt-fed, drained before CAN service. `firmware/twin.h:12`, `twin.c:89-105`: 128-byte text buffer; overflow/bad lines count parse errors. `host.py:20-24` bounds encoded lengths. Zero error counters argue against these for this boot. |
| Coalescing | `twin.c:98-103` dispatches each complete line immediately; `main.c:322-330` admits or rejects it. No latest-command queue. `firmware/dm.c:59-62` holds the last target only after CAN reception. |
| Output congestion | `main.c:96-128`: nonblocking 4,096-byte TX ring, whole-record admission/drop; no TX-space check gates commands. Saturation loses output and increments `uart_drop`; zero does not show saturation. |
| Slow peripheral service | UART TX is polled only at loop top (`main.c:361`); I2C waits can take 2 ms each (`:155-160,201-213`). IMU faults retry every 500 ms (`:402-418`). This can reduce telemetry/service cadence; inspect `imu_ok/err/stale`, `model_skip_ms`, and MCU timestamps before attribution. |

Only 54 I-lines in 10 s does not prove UART overload: I-lines are emitted only
on fresh, successful IMU reads (`main.c:404-415`). Compare raw I count with
delta `imu_ok` and `uart_drop`, then check host decoder errors and BOOTs. A
single-thread host waiting on reads before each write is a plausible throttle,
but cannot be confirmed without the actual hardware runner.

## UART budget: 115200 baud, 8N1

Each byte costs 10 bits: **11,520 B/s per direction**, full duplex. Divider 139
(`main.c:340`) actually gives ~11,510.8 B/s. README:143-172 and
`host.py:75-87` budget only **20 aggregate commands/s**, not 100 or 400.
Using their conservative lengths (including LF):

| MCU TX stream | Requested bytes/s |
|---|---:|
| 100 Hz I, 64 B | 6,400 |
| 4 × 100 Hz M, 48 B | 19,200 |
| 1 Hz STAT, 768 B bound | 768 |
| Total without echoes | **26,368 = 228.9%**, requiring ≥263,680 baud |
| Plus stale echoes, 10 × 51 B (`twin.c:85-87`) | **26,878 = 233.3%** |

Thus 100 I + 400 M/s cannot fit at 115200 with this budget; a finite ring cannot
fix sustained overload. M is one record per feedback (`main.c:381-391`), not
an autonomous 100 Hz/motor stream (README:147-148).
The reported **100 aggregate C/s means 25 Hz/motor and ~100 M/s**, not 400:
6,400 + 4,800 + 768 = 11,968 B/s (103.9%); with stale echoes 12,478 (108.3%).
Actual short records can fit; these bounds do not prove overload on this run.
Typical `host.command(...,v=.5,kd=1)` is 51–60 B with 1–10 sequence digits:
5,100–6,000 B/s host TX at 100/s, independently below RX wire capacity.

## Why velocity can read −0.007 rad/s

`dm.c:14-20,39-50` truncates a symmetric 12-bit velocity range: exact zero
encodes to 2047, decoding to −30/4095 = −0.007326 rad/s for IDs 6/8;
IDs 0/1 give −45/4095 = −0.010989. `twin.c:38-46` rounds to three decimals.
This is the zero-bin bias, not evidence of reverse physical motion.
All models start disabled; E sets enabled and clears the target
(`dm.c:58-62`). Inspect M `err` (synthetic state: 0 disabled, 1 enabled,
`dm.c:73-76`), per ID. Immediate feedback follows a zero-duration step
(`main.c:375-377`), so the first C reply can legitimately show zero velocity.
With sustained enabled C(p=0,v=.5,kp=0,kd=1,t=0), `dm.c:64-71` predicts
v∞ = (kd·v_cmd+t)/(kd+0.2) ≈ 0.4167 rad/s before quantization, not .5.
Native execution of the actual `dm.c` for 1,000 × 1 ms gave internal velocities
0.407000 (hips)/0.408425 (wheels), decoded feedback 0.402931/0.406593 rad/s.
Persistent zero therefore suggests disabled/reset targets, missing MIT commands,
or mismatched logs/image; low command rate alone cannot explain it: targets hold.

## Smallest proposed fix and discriminating host test

First change the hardware runner: one continuously draining reader using
`host.Decoder`, and a writer driven by absolute monotonic deadlines, independent
of read timeouts. Stagger IDs 6,8,0,1 every 10 ms (100 aggregate C/s), avoiding
four-mailbox bursts. Preserve the firmware gate; there is no proven need to
remove it. This is a proposed host fix, pending inspection of the missing runner.
For guaranteed conservative TX headroom at 100 aggregate C/s, the smallest
firmware rate change is to emit I at 50 Hz while keeping sensor polling at 200 Hz:
3,200 + 4,800 + 768 + 510 = 9,278 B/s (80.5%). Retain `imu_ok` sample counting.
If 100 I + 400 M/s is required, raise both endpoints to 460800 and budget again;
TX interrupt/DMA service may also be needed because the present pump is polled.

Host acceptance procedure (proposed, not executed):
1. On the isolated fake-motor fixture, save all raw RX bytes with monotonic times;
   decode text/binary with one Decoder. Abort the measurement on any BOOT/reset.
2. Send four E commands sequentially, waiting for each enabled M; then capture
   baseline STAT. Use one globally increasing uint32 sequence across E/C/D.
3. For exactly 10 s send 1,000 C-lines, 250/ID, at absolute 10 ms deadlines;
   log each actual write length/time, retry partial writes, count deadline misses.
   Do not wait for feedback between scheduled C writes. Keep draining RX.
4. Stop writing, drain pending replies, capture final STAT. Pass: 1,000 complete
   writes, delta sent=delta fed_back=1,000, pending=0, zero delta rejected/lost,
   RX/parse/UART drops, CAN FIFO overruns and fb_busy; ~1,000 decoded M-lines.
   Use write timestamps for 100/s and STAT ms/deltas for CAN throughput; retain
   intermediate STATs to expose stalls. Check enabled state and positive velocity.
5. If counts disagree, compare raw UART capture/MCU boot identity before changing
   firmware. This test establishes 100 aggregate cmd/s, not 100 Hz per motor.
