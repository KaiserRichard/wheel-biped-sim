# FROM FILE: S12_comms_in_loop/can_timing.py, bits; frozen oracle excerpt.
def bits(identifier, payload):
    """CRC-15 then real bit stuffing, with ACK/EOF/intermission accounted for."""
    serial = f'0{identifier:011b}0001000' + ''.join(f'{byte:08b}' for byte in payload)
    crc = 0
    for bit in serial:
        feedback = (crc >> 14) ^ int(bit)
        crc = ((crc << 1) & 0x7fff) ^ (0x4599 if feedback else 0)
    last, consecutive, stuffed = '', 0, 0
    for bit in serial + f'{crc:015b}':
        consecutive = consecutive + 1 if bit == last else 1
        last = bit
        if consecutive == 5:
            stuffed += 1
            last, consecutive = str(1-int(bit)), 1
    return 111 + stuffed

