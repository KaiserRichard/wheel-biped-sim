"""Write shared firmware wire frames from FIL tick records (JSONL input)."""
import ctypes as C, json, pathlib, sys
from pi_link.protocol import StreamingDecoder, UsbLinkStatus
from pi_link.messages import TelemFast, TelemHealth

class Fast(C.Structure):
    _fields_=[('fsm_state',C.c_uint8),('s11_mode',C.c_uint8),('s11_max_misses',C.c_uint8),('flags',C.c_uint8),('fault_mask',C.c_uint32),('euler_rad',C.c_float*3),('gyro_rad_s',C.c_float*3),('v_est_m_s',C.c_float),('x_est_m',C.c_float),('leg_length_m',C.c_float*2),('leg_theta_rad',C.c_float*2),('motor_pos_rad',C.c_float*6),('motor_vel_rad_s',C.c_float*6),('motor_tor_cmd_nm',C.c_float*6),('motor_tor_fb_nm',C.c_float*6),('enc_age_us',C.c_uint16*6),('s9_trim_rad',C.c_float),('gyro_bias_y_rad_s',C.c_float),('v_bus_mv',C.c_uint16),('i_bus_ma',C.c_int16)]
class Health(C.Structure):
    _fields_=[('can_tec',C.c_uint8*3),('can_rec',C.c_uint8*3),('can_busoff_cnt',C.c_uint16*3),('can_rx_drop',C.c_uint16*3),('motor_state',C.c_uint8*6),('t_mos_c',C.c_uint8*6),('t_coil_c',C.c_uint8*6),('rc_rssi_dbm',C.c_int8),('rc_lq_pct',C.c_uint8),('rc_age_ms',C.c_uint16),('rc_switches',C.c_uint16),('power_io_bits',C.c_uint16),('reset_cause',C.c_uint8),('cpu_load_pct',C.c_uint8),('min_stack_words',C.c_uint16*5)]

def native_lib():
    root=pathlib.Path(__file__).resolve().parents[2]
    out=pathlib.Path(__file__).with_name('liblogpack.so')
    import subprocess
    subprocess.run(['cc','-shared','-fPIC','-std=c11','-I',str(root/'firmware/firmware'),str(root/'firmware/firmware/log_packer.c'),str(root/'firmware/firmware/usb_link.c'),str(root/'firmware/firmware/cobs.c'),str(root/'firmware/firmware/crc32c.c'),'-o',str(out)],check=True)
    return C.CDLL(str(out))
def _arr(typ,vals,n): return (typ*n)(*vals)
def write_jsonl(input_path,bin_path,truth_path):
    lib=native_lib(); lib.log_frame_telem_fast.argtypes=[C.c_uint32,C.c_uint64,C.c_uint8,C.POINTER(Fast),C.POINTER(C.c_uint8),C.c_size_t,C.POINTER(C.c_size_t)]; lib.log_frame_telem_health.argtypes=[C.c_uint32,C.c_uint64,C.c_uint8,C.POINTER(Health),C.POINTER(C.c_uint8),C.c_size_t,C.POINTER(C.c_size_t)]
    with open(input_path) as src, open(bin_path,'wb') as out, open(truth_path,'w') as truth:
        truth.write('time_s,true_pitch_rad,true_x_m\n')
        for n,line in enumerate(src):
            d=json.loads(line); f=Fast(); f.fsm_state=d.get('fsm_state',0); f.euler_rad=_arr(C.c_float,d.get('euler_rad',[0,0,0]),3); f.gyro_rad_s=_arr(C.c_float,d.get('gyro_rad_s',[0,0,0]),3); f.v_est_m_s=d.get('v_est_m_s',0); f.x_est_m=d.get('x_est_m',0); f.leg_length_m=_arr(C.c_float,d.get('leg_length_m',[.14,.14]),2); f.leg_theta_rad=_arr(C.c_float,d.get('leg_theta_rad',[0,0]),2)
            for field in ('motor_pos_rad','motor_vel_rad_s','motor_tor_cmd_nm','motor_tor_fb_nm'): setattr(f,field,_arr(C.c_float,d.get(field,[0]*6),6))
            f.enc_age_us=_arr(C.c_uint16,d.get('enc_age_us',[0]*6),6); f.v_bus_mv=d.get('v_bus_mv',24000); f.i_bus_ma=d.get('i_bus_ma',0)
            b=(C.c_uint8*256)(); sz=C.c_size_t(); ts=round(d['time_s']*1e6); st=lib.log_frame_telem_fast(n+1,ts,0,C.byref(f),b,256,C.byref(sz)); assert st==0; out.write(bytes(b[:sz.value])); truth.write(f"{d['time_s']},{d.get('true_pitch_rad',0)},{d.get('true_x_m',0)}\n")
            if n%25==0:
                h=Health(); st=lib.log_frame_telem_health(n//25+1,ts,0,C.byref(h),b,256,C.byref(sz)); assert st==0; out.write(bytes(b[:sz.value]))
if __name__=='__main__': write_jsonl(*sys.argv[1:4])
