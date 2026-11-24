"""Convert framed USB link log to SI-labelled CSV; validates frames with Pi decoder."""
import csv,sys
from pathlib import Path
from pi_link.protocol import StreamingDecoder,UsbLinkStatus
from pi_link.messages import TelemFast,TelemHealth

def convert(path,out_csv):
    data=Path(path).read_bytes(); dec=StreamingDecoder(); records=[]; bad=0; prev={}
    for b in data:
        st,h,p=dec.feed_byte(b)
        if st==UsbLinkStatus.OK:
            lost=0
            if h.msg_type in prev:
                delta=(h.seq-prev[h.msg_type])&0xffffffff; lost=max(0,delta-1)
            prev[h.msg_type]=h.seq
            row={'time_s':h.timestamp_us*1e-6,'seq':h.seq,'msg_type':h.msg_type,'lost_before':lost}
            if h.msg_type==0x81:
                x=TelemFast.unpack(p); row.update(pitch_rad=x.pitch_rad,x_est_m=x.x_est_m,v_est_m_s=x.v_est_m_s,wheel_speed_rad_s=(x.motor_vel_rad_s[2]+x.motor_vel_rad_s[5])*0.5,wheel_torque_cmd_Nm=(x.motor_tor_cmd_nm[2]+x.motor_tor_cmd_nm[5])*0.5,motor_torque_cmd_Nm=x.motor_tor_cmd_nm[0],fsm_state=x.fsm_state)
            elif h.msg_type==0x82: TelemHealth.unpack(p)
            records.append(row)
        elif st==UsbLinkStatus.ERR_CRC: bad+=1
    cols=['time_s','seq','msg_type','lost_before','fsm_state','pitch_rad','x_est_m','v_est_m_s','wheel_speed_rad_s','wheel_torque_cmd_Nm','motor_torque_cmd_Nm']
    with open(out_csv,'w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=cols);w.writeheader();w.writerows(records)
    return {'frames':len(records),'crc_errors':bad,'lost':sum(r['lost_before'] for r in records)}
if __name__=='__main__': print(convert(sys.argv[1],sys.argv[2]))
