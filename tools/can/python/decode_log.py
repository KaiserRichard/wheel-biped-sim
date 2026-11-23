"""Offline candump, Vector ASC and BLF decoder. No channels are opened."""
import argparse
import csv
import json
import math
from pathlib import Path
import re
import can
import cantools
from dm_codec import SPECIAL, frame_bits

ROOT=Path(__file__).resolve().parents[1]


def frames(path):
    path=Path(path)
    if path.suffix in ('.asc','.blf'):
        cls=can.ASCReader if path.suffix=='.asc' else can.BLFReader
        with cls(path) as reader:
            for m in reader:
                if m.is_error_frame or m.is_remote_frame or m.is_extended_id or m.is_fd:
                    raise ValueError('only classical standard data frames supported')
                yield m.timestamp, int(m.channel)+1, m.arbitration_id, bytes(m.data)
        return
    rx=re.compile(r'^\((\d+(?:\.\d+)?)\) can([123]) ([0-9A-Fa-f]{1,3})#([0-9A-Fa-f]*)$')
    for line in path.read_text().splitlines():
        if not line.strip() or line.startswith('#'): continue
        match=rx.fullmatch(line)
        if not match: raise ValueError('malformed candump data line')
        t,b,i,h=match.groups()
        yield float(t),int(b),int(i,16),bytes.fromhex(h)


def decode(path, output):
    db=cantools.database.load_file(ROOT/'wbr_dm.dbc')
    mapping=json.loads((ROOT/'reference/motor_map.json').read_text())
    records=[]
    for t,bus,identifier,data in frames(path):
        if len(data)!=8: raise ValueError('DLC must be 8 for DAMIAO MIT')
        row=dict(time_s=t,bus=bus,id=identifier,hex=data.hex().upper(),axis='',kind='unknown',
                 Position=math.nan,Velocity=math.nan,Kp=math.nan,Kd=math.nan,Torque=math.nan,
                 MotorID=math.nan,State=math.nan,MOS_Temperature=math.nan,Rotor_Temperature=math.nan,
                 validation='unknown_id',frame_bits=frame_bits(identifier,data))
        found=[m for m in mapping if m['bus']==bus and identifier in (m['study_command_id'],m['study_feedback_id'])]
        if found:
            m=found[0]; row['axis']=m['axis']; row['validation']='ok'
            if identifier==m['study_command_id']:
                if data[:7]==b'\xff'*7 and data[7] in SPECIAL:
                    row['kind']=SPECIAL[data[7]]
                else:
                    row['kind']='command'; row.update(db.decode_message(identifier,data,decode_choices=False))
            else:
                row['kind']='feedback'; row.update(db.decode_message(identifier,data,decode_choices=False))
                if row['MotorID']!=m['study_command_id']:
                    row['validation']='payload_id_mismatch'
        records.append(row)
    records.sort(key=lambda r:r['time_s'])
    if not records: raise ValueError('empty log')
    with Path(output).open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=records[0]);w.writeheader();w.writerows(records)
    return records


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('input');p.add_argument('output')
    a=p.parse_args(); rows=decode(a.input,a.output);print(f'Decoded {len(rows)} frames')
