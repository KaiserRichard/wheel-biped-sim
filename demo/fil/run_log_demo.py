from pathlib import Path
import ctypes, json, sys
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE/'fw/tests/fil'))
import run_fil as fil
import run_b18c_slice8 as b18
import mujoco, numpy as np

fw=HERE/'fw'
b18.ROOT=fw
b18.OUT_DIR=fw/'tests/fil/b18c_out'
b18.MODELS_DIR=b18.OUT_DIR/'models'
b18.BASE_XML=fw/'sim_model/draw_001.xml'
b18.MESH_DIR=(fw/'sim_model/meshes').resolve()
models=b18.build_models()
lib=fil.load_library(fw/'build/libwbr_balance_ctrl.so')

def run(label, key, outpath):
    xml=models[key]
    rows=[]
    def observer(state, rig, rec):
        # callback at 500 Hz, post integration; controller outputs from same tick are in rec/state.
        t=rec[0]
        gyro,acc=rig.read_imu()
        x=float(rig.d.qpos[0])
        pitch=float(rec[3])
        wheel_rates=rig.read_encoders()[1]
        # B18c record stores wheel torque/rates at axis indices 2 and 5.
        tau=(rec[18]+rec[19])/2
        speed=(rec[21]+rec[22])/2
        estpitch=float(rec[12])
        rows.append(dict(time_s=t, fsm_state=int(rec[8]), euler_rad=[float(np.deg2rad(estpitch)),0,0],
          gyro_rad_s=[float(v) for v in gyro], v_est_m_s=float(rec[14]), x_est_m=x,
          motor_vel_rad_s=[0,0,float(wheel_rates[2]),0,0,float(wheel_rates[5])],
          motor_tor_cmd_nm=[0,0,float(tau),0,0,float(tau)],
          true_pitch_rad=float(np.deg2rad(pitch)),true_x_m=x))
    b18.run_case(lib,label,'push_sagittal',12.0,xml,push_axis='sagittal',observer=observer)
    with open(outpath,'w') as f:
      for row in rows:f.write(json.dumps(row,separators=(',',':'))+'\n')
    print(f'{label}: {len(rows)} frames, duration={rows[-1]["time_s"]:.3f}s')

run('nominal_com0_5p06kg','nominal_com0_5p06kg',HERE/'nominal.jsonl')
run('draw_001_5p06kg','draw_001_5p06kg',HERE/'draw_001.jsonl')
