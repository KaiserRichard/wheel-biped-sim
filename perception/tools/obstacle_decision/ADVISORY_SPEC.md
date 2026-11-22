# TerrainAdvisory — PROPOSED

Firmware owner review is required before any firmware use. This message is
informational only. It does not grant jump permission or request actuation;
the STM32 retains balance, jump, and all safety decisions. No transport is
implemented.

Framing reuses `tools/link_proto/py/link_proto.py`: `COBS(header || payload ||
CRC-32C) || 00`, version 1, type 4, session u64, packet sequence u32, little
endian. Type 4 is unused in the current SPEC. Decoded header is the existing
16-byte layout and CRC is four bytes. Payload is 42 bytes:

| Offset | Type | Field |
|---:|---|---|
| 0 | u64 | Pi monotonic timestamp, ns |
| 8 | u64 | source frame stamp, ns |
| 16 | u8 | advisory: 0 NONE, 1 JUMP_CANDIDATE, 2 STOP_CANDIDATE, 3 UNCERTAIN |
| 17 | u8 | plane_valid boolean |
| 18 | f32 | confidence, 0..1 |
| 22 | f32 | obstacle front-face distance, m |
| 26 | f32 | obstacle height, m |
| 30 | f32 | obstacle lateral width, m |
| 34 | f32 | plane pitch, rad |
| 38 | f32 | camera height above plane, m |

All floats must be finite. No obstacle is represented with finite zero distance,
height and width; advisory NONE distinguishes the no-obstacle case. This format
is proposed and requires firmware owner review before implementation.
