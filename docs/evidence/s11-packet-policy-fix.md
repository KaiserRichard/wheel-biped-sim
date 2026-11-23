**Tóm tắt [FROM FILE / COMPUTED].** Lần kiểm tra độc lập thứ hai phát hiện sáu lỗi biên và bản sửa xử lý cả sáu. Hồ sơ nguồn đạt 18/18 kiểm thử ở Debug và Release; firmware tích hợp thêm sửa sai số lượng tử vận tốc.

# S11 packet policy review and fixes

[FROM FILE] This page edits the supplied packet-policy draft against the round-two correction report and independent review. The draft's matched-schedule totals were removed because that replay is not an execution of the final integrated firmware.

| Independent finding [FROM FILE] | Implemented correction |
| --- | --- |
| Slow frozen angle could evade detection | Accumulate implied displacement as well as unchanged duration. |
| An unwritten axis could latch an unhealthy startup fault | Preserve unknown status until a valid sample is received. |
| Atomic generation did not protect ordinary payload members | Use an ownership-safe triple buffer with an explicit single-writer/single-reader contract. |
| Generation wrapping to zero looked unwritten | Separate initialization from generation and message sequence. |
| Arrival delay and held-data age were conflated | Derive transport and total-age budgets separately and document revised structures. |
| Release builds could erase assertions and setup calls | Mandatory checks remain active with optimization and `NDEBUG`. |

[FROM FILE] Saved final source executions report 18/18 passing in both [Debug](receipts/s11-debug.txt) and [Release](receipts/s11-release.txt). These are supplied execution receipts, not new runs in this staging pass. The integrated [firmware](../../firmware/README.md) includes those tests plus controller tests; its stuck detector also subtracts a quantization deadband, avoiding a false fault on decoded zero velocity.

[FROM FILE] The integrated policy defaults use a 6 ms transport ceiling and a 16 ms total-age ceiling; these are engineering budgets rather than measured target limits. See `S11Config` defaults in [source](../../firmware/firmware/s11_packet_policy.c).

## What this teaches

[STANDARD PRACTICE] `total_age = arrival_age + held_time`. A fresh host sequence proves a mailbox update, not sensor motion. Acquire/release coordination needs exclusive payload ownership; a retry after a data race does not repair it. Tests must execute their invariants in every build type.

## Figures

[FROM FILE] This is code and test evidence; no new plant figure or replay is claimed. [Earlier packet-tolerance simulation](s11-packet-tolerance.md) has a different execution scope.

## What the real robot must confirm

[UNKNOWN] Measure acquisition, arbitration, bridge and ISR latency separately. Qualify target concurrency, compiler behavior, cache coherence, update deadlines and the configured age ceilings before relying on the policy on hardware.
