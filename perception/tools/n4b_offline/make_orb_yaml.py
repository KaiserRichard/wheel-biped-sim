#!/usr/bin/env python3
"""Generate N4B ORB Settings via the repository's validated calib_convert."""
from __future__ import annotations
import argparse,hashlib,json,subprocess,sys
from pathlib import Path
import yaml
ROOT=Path(__file__).resolve().parents[2]
FACTORY=ROOT/'calibration/lab/config/camchain_factory_serial-alias-6.yaml'
IMU=ROOT/'calibration/lab/config/imu_d435i_provisional.yaml'
CONVERTER=ROOT/'tools/calib_tools/calib_convert.py'
# SDK IR1 -> gyro transform from the saved 2026-10-13 calibration readback.
# MEASURED SDK values; spatial interpretation is nominal/inferred, not calibrated.
T_B_C1=[[1,0,0,-0.00552000012248755],[0,1,0,0.00510000018402934],[0,0,1,0.011739999987185],[0,0,0,1]]
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def main():
 p=argparse.ArgumentParser(); p.add_argument('--output-dir',required=True,type=Path); p.add_argument('--fps',type=int,default=30); a=p.parse_args()
 if a.fps<=0:p.error('--fps must be positive')
 factory=yaml.safe_load(FACTORY.read_text()); Tbc=T_B_C1
 # Kalibr wants T_cam_imu (IMU -> camera), inverse of the known camera->IMU transform.
 Tci=[[Tbc[j][i] for j in range(3)]+[0] for i in range(3)]+[[0,0,0,1]]
 for i in range(3):Tci[i][3]=-sum(Tci[i][j]*Tbc[j][3] for j in range(3))
 phase-10=factory['cam1']['T_cn_cnm1']; T1i=[[sum(phase-10[i][k]*Tci[k][j] for k in range(4)) for j in range(4)] for i in range(4)]
 doc={'cam0':dict(factory['cam0']), 'cam1':dict(factory['cam1'])}
 for name,T in [('cam0',Tci),('cam1',T1i)]:doc[name]['T_cam_imu']=T; doc[name]['timeshift_cam_imu']=0.0
 a.output_dir.mkdir(parents=True,exist_ok=True)
 chain=a.output_dir/'generated_camchain.yaml'
 with chain.open('x') as f:f.write(yaml.safe_dump(doc,sort_keys=False))
 proc=subprocess.run([sys.executable,str(CONVERTER),'--camchain',str(chain),'--imu',str(IMU),'--output-dir',str(a.output_dir),'--target','orb','--fps',str(a.fps),'--units-si'],text=True)
 if proc.returncode:return proc.returncode
 yamlpath=a.output_dir/'orbslam3.yaml'; text=yamlpath.read_text()
 comments='''# N4B generated Settings; source labels for every value group:
# Factory MEASURED SDK readback: Camera1/2 intrinsics, distortion, width, height, Stereo transform.
# Factory MEASURED readback / INFERRED nominal geometry: IMU.T_b_c1 from SDK IR1->gyro; not an independent calibration.
# ASSUMED provisional: IMU.Noise* and IMU.Frequency from imu_d435i_provisional.yaml.
# ASSUMED operating choices: Camera.fps, RGB, Stereo.ThDepth, extractor, Viewer, keyframe policy from calib_convert.
# ASSUMED 0 s cam/IMU shifts: factory file has no timeshift; UNKNOWN physical offset. Must be independently resolved.
# SOURCE-SUPPORTED schema/transforms: tools/calib_tools/README.md; ORB-SLAM3 Settings 1.0.
# Feed raw Y8 as configured by factory readback. N4B qualification remains OPEN.
'''
 header,body=text.split('\n',1); yamlpath.write_text(header+'\n'+comments+body)
 prov=json.loads((a.output_dir/'provenance.json').read_text()); prov['field_provenance']={'File.version':'SOURCE-SUPPORTED ORB Settings schema 1.0','Camera.type':'SOURCE-SUPPORTED ORB Settings schema',**{f'Camera{i}.{k}':'MEASURED factory SDK readback' for i in (1,2) for k in ('fx','fy','cx','cy','k1','k2','p1','p2')},**{k:'MEASURED factory SDK readback' for k in ('Camera.width','Camera.height','Stereo.T_c1_c2')},**{k:'MEASURED SDK readback / INFERRED nominal geometry' for k in ('IMU.T_b_c1',)},**{k:'ASSUMED provisional noise YAML' for k in ('IMU.NoiseGyro','IMU.NoiseAcc','IMU.GyroWalk','IMU.AccWalk','IMU.Frequency')},**{k:'ASSUMED operating choice' for k in ('Camera.fps','Camera.RGB','Stereo.ThDepth','IMU.InsertKFsWhenLost','ORBextractor.nFeatures','ORBextractor.scaleFactor','ORBextractor.nLevels','ORBextractor.iniThFAST','ORBextractor.minThFAST','Viewer.KeyFrameSize','Viewer.KeyFrameLineWidth','Viewer.GraphLineWidth','Viewer.PointSize','Viewer.CameraSize','Viewer.CameraLineWidth','Viewer.ViewpointX','Viewer.ViewpointY','Viewer.ViewpointZ','Viewer.ViewpointF')},'camera_imu_timeshift':'ASSUMED placeholder; physical value UNKNOWN'}; prov['n4b_source_inputs']={'factory_camera_yaml':{'path':str(FACTORY.relative_to(ROOT)),'sha256':digest(FACTORY),'label':'MEASURED SDK factory readback'},'imu_yaml':{'path':str(IMU.relative_to(ROOT)),'sha256':digest(IMU),'label':'ASSUMED provisional noise'},'converter':{'path':str(CONVERTER.relative_to(ROOT)),'sha256':digest(CONVERTER),'label':'SOURCE-SUPPORTED repository converter'},'imu_extrinsic':{'value':T_B_C1,'label':'MEASURED SDK readback; nominal interpretation INFERRED'},'timeshift_seconds':0.0,'timeshift_label':'ASSUMED placeholder; actual offset UNKNOWN'}
 prov['outputs']['orbslam3.yaml']['sha256']=hashlib.sha256(yamlpath.read_bytes()).hexdigest(); prov['n4b_source_inputs']['generated_camchain']={'path':str(chain),'sha256':digest(chain),'label':'CALCULATED from factory stereo transform and SDK camera-to-gyro readback'}
 (a.output_dir/'provenance.json').write_text(json.dumps(prov,indent=2)+'\n'); print(yamlpath); return 0
if __name__=='__main__':raise SystemExit(main())
