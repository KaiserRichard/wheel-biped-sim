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
