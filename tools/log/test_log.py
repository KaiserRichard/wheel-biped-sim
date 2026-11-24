import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'pi/link'))
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from pi_link.messages import TelemFast
from pi_link.protocol import encode_frame,StreamingDecoder,UsbLinkStatus
from convert import convert

def test_roundtrip_bit_exact():
    msg=TelemFast(euler_rad=(.125,-.25,.5),motor_tor_cmd_nm=(1,2,3,4,5,6))
    wire=encode_frame(0x81,msg.pack(),seq=7,timestamp_us=123456)
    d=StreamingDecoder(); out=[]
    for b in wire:
        s,h,p=d.feed_byte(b)
        if s==UsbLinkStatus.OK:out.append((h,p))
    assert len(out)==1 and out[0][1]==msg.pack()
def test_crc_rejection():
    w=bytearray(encode_frame(0x81,TelemFast().pack(),seq=1));w[4]^=0x40
    d=StreamingDecoder(); statuses=[d.feed_byte(b)[0] for b in w]
    assert UsbLinkStatus.ERR_CRC in statuses and d.crc_errors==1
def test_lost_count_and_csv_columns(tmp_path):
    p=tmp_path/'x.bin';p.write_bytes(b''.join(encode_frame(0x81,TelemFast().pack(),seq=n,timestamp_us=n*2000) for n in (1,3)))
    out=tmp_path/'x.csv'; result=convert(p,out)
    assert result['lost']==1
    import csv
    with out.open() as f:r=csv.DictReader(f); assert r.fieldnames==['time_s','seq','msg_type','lost_before','fsm_state','pitch_rad','x_est_m','v_est_m_s','wheel_speed_rad_s','wheel_torque_cmd_Nm','motor_torque_cmd_Nm']

def test_demo_fixtures_use_si_units():
    import json
    import math

    demo = Path(__file__).resolve().parents[4] / 'demo' / 'fil'
    expected = {
        'time_s', 'fsm_state', 'euler_rad', 'gyro_rad_s', 'v_est_m_s',
        'x_est_m', 'motor_vel_rad_s', 'motor_tor_cmd_nm',
        'true_pitch_rad', 'true_x_m',
    }
    for name in ('nominal.jsonl', 'draw_001.jsonl'):
        with (demo / name).open() as src:
            rows = [json.loads(line) for line in src]
        assert len(rows) == 6000
        assert all(set(row) == expected for row in rows)
        assert all(row['fsm_state'] in range(256) for row in rows)
        assert all(len(row['euler_rad']) == len(row['gyro_rad_s']) == 3 for row in rows)
        assert all(len(row['motor_vel_rad_s']) == len(row['motor_tor_cmd_nm']) == 6 for row in rows)
        for row in rows:
            scalars = ('time_s', 'v_est_m_s', 'x_est_m', 'true_pitch_rad', 'true_x_m')
            values = [row[key] for key in scalars]
            values += row['euler_rad'] + row['gyro_rad_s']
            values += row['motor_vel_rad_s'] + row['motor_tor_cmd_nm']
            assert all(math.isfinite(value) for value in values)
        assert all(abs(row['euler_rad'][0]) < 0.5 for row in rows)
        assert all(abs(row['true_pitch_rad']) < 0.5 for row in rows)
        assert all(abs(row['euler_rad'][0] - row['true_pitch_rad']) < 0.5 for row in rows)
