"""Read incomplete MCAP captures without changing the source evidence."""
from __future__ import annotations

from contextlib import contextmanager
from pathlib import Path
import shutil
import struct
import tempfile

MAGIC = b'\x89MCAP0\r\n'
FOOTER = struct.pack('<BQQQI', 0x02, 20, 0, 0, 0) + MAGIC


def _last_complete_record(path: Path) -> tuple[int, int]:
    """Return (end offset, record count) for the complete top-level MCAP prefix."""
    size = path.stat().st_size
    with path.open('rb') as stream:
        if stream.read(8) != MAGIC:
            raise ValueError(f'not an MCAP file: {path}')
        opcode = stream.read(1)
        raw_size = stream.read(8)
        if opcode != b'\x01' or len(raw_size) != 8:
            raise ValueError(f'invalid MCAP header record: {path}')
        header_size = struct.unpack('<Q', raw_size)[0]
        if header_size > size - stream.tell():
            raise ValueError(f'truncated MCAP header record: {path}')
        stream.seek(header_size, 1)
        last_complete = stream.tell()
        records = 0
        while stream.tell() + 9 <= size:
            header = stream.read(9)
            record_size = struct.unpack('<Q', header[1:])[0]
            if record_size > size - stream.tell():
                break
            stream.seek(record_size, 1)
            last_complete = stream.tell()
            records += 1
        return last_complete, records


def messages(reader, path: str | Path, connections):
    """Use storage order for unindexed MCAP; otherwise use Reader's sorted stream."""
    path = Path(path)
    if path.suffix.lower() == '.mcap' and not getattr(reader.storage, 'chunks', None):
        return reader.storage.messages_scan(connections)
    return reader.messages(connections=connections)


@contextmanager
def readable_mcap(path: str | Path):
    """Yield (readable path, omitted trailing bytes), repairing only a temp copy."""
    path = Path(path)
    if path.suffix.lower() != '.mcap':
        yield path, 0
        return
    with path.open('rb') as stream:
        stream.seek(-8, 2)
        if stream.read(8) == MAGIC:
            yield path, 0
            return

    complete, records = _last_complete_record(path)
    size = path.stat().st_size
    dropped = size - complete
    if records == 0:
        raise ValueError(f'MCAP contains no complete records after its header: {path}')
    with tempfile.TemporaryDirectory(prefix='wbr-mcap-recovery-') as temp_dir:
        recovered = Path(temp_dir) / path.name
        with path.open('rb') as source, recovered.open('wb') as target:
            remaining = complete
            while remaining:
                block = source.read(min(8 * 1024 * 1024, remaining))
                if not block:
                    raise OSError(f'unexpected EOF while copying complete MCAP prefix: {path}')
                target.write(block)
                remaining -= len(block)
            target.write(FOOTER)
        yield recovered, dropped
