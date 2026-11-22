#!/usr/bin/env python3
"""Qualify EuRoC export structure/timing. --analyze-only reuses saved files."""
from __future__ import annotations
import argparse,csv,hashlib,json,platform,socket,sys
from datetime import datetime,timezone
from pathlib import Path
VERSION='1.0.0'
CONFIG=Path(__file__).with_name('qualify_config.json')

class ContractParser(argparse.ArgumentParser):
 def error(self,message):
  self.print_usage(sys.stderr)
  self.exit(3,f'{self.prog}: error: {message}\n')

def sha(p):
 h=hashlib.sha256(p.read_bytes()); return h.hexdigest()
def stamps(path):
 out=[]
 with path.open(newline='') as f:
  for row in csv.reader(f):
   if not row or row[0].startswith('#'): continue
   out.append(int(row[0]))
 return out
def criterion(i,desc,val,unit,th,status,evidence,source): return {'id':i,'description':desc,'measured':val,'unit':unit,'threshold':th,'status':status,'evidence':evidence,'threshold_source':source}
def evaluate(seq):
 cfg=json.loads(CONFIG.read_text()); criteria=[]; notes=[]; raw=[]
 cams=[seq/'mav0/cam0/data.csv',seq/'mav0/cam1/data.csv']; imu=seq/'mav0/imu0/data.csv'; times=seq/'times_ns.txt'; legacy_times=seq/'timestamps.txt'; meta=seq/'metadata.json'
 required=[*cams,imu,times,legacy_times,meta]
 absent=[str(p.relative_to(seq)) for p in required if not p.is_file()]
 if absent:
  return [criterion('layout','Required EuRoC inputs exist',None,'files',str(absent),'BLOCKED','missing_files.txt','task required files')],None,absent
 for p in required: raw.append({'path':str(p.relative_to(seq)),'sha256':sha(p)})
 try:
  a,b,ii=stamps(cams[0]),stamps(cams[1]),stamps(imu)
  tt=[int(x) for x in times.read_text().splitlines() if x.strip()]
  md=json.loads(meta.read_text())
 except (ValueError,OSError,json.JSONDecodeError,csv.Error) as e:
  return [criterion('parse','CSV, times and metadata parse',None,'',str(e),'FAIL','metadata.json','EuRoC integer-ns schema')],raw,[]
 lim=cfg['camera_count_difference']; delta=abs(len(a)-len(b))
 criteria.append(criterion('camera_counts','Camera image counts equal',delta,'frames',f"<= {lim['maximum']}",'PASS' if delta<=lim['maximum'] else 'FAIL','mav0/cam*/data.csv',lim['source']))
 stereo= a==b
 png_ok=all((seq/'mav0'/cam/'data'/f'{t}.png').is_file() for cam,ts in [('cam0',a),('cam1',b)] for t in ts)
 lim=cfg['missing_png_file_count']; missing_png=0 if png_ok else 1
 criteria.append(criterion('png_files','Every CSV image reference exists as PNG',missing_png,'missing files',f"<= {lim['maximum']}",'PASS' if missing_png<=lim['maximum'] else 'FAIL','mav0/cam*/data',lim['source']))
 lim=cfg['stereo_stamp_mismatch_count']; skew=sum(x!=y for x,y in zip(a,b))
 criteria.append(criterion('stereo_stamps','Stereo timestamps exactly equal',skew,'mismatches',f"<= {lim['maximum']}",'PASS' if stereo and skew<=lim['maximum'] else 'FAIL','mav0/cam*/data.csv',lim['source']))
 lim=cfg['image_nonincreasing_pairs']; bad_images=sum(y<=x for x,y in zip(a,a[1:]))
 criteria.append(criterion('image_order','Image timestamps strictly increase',bad_images,'non-increasing pairs',f"<= {lim['maximum']}",'PASS' if bad_images<=lim['maximum'] else 'FAIL','mav0/cam0/data.csv',lim['source']))
 strict=all(y>x for x,y in zip(ii,ii[1:]))
 lim=cfg['imu_nonincreasing_pairs']; bad_order=sum(y<=x for x,y in zip(ii,ii[1:]))
 criteria.append(criterion('imu_order','IMU timestamps strictly increase',bad_order,'non-increasing pairs',f"<= {lim['maximum']}",'PASS' if strict and bad_order<=lim['maximum'] else 'FAIL','mav0/imu0/data.csv',lim['source']))
 margin=min(a[0]-ii[0],ii[-1]-a[-1]) if a and ii else None
 req=cfg['imu_image_coverage_margin_ns']; ok=margin is not None and margin>=req['minimum']
 criteria.append(criterion('imu_coverage','IMU covers image span with margin',margin,'ns',f">= {req['minimum']}",'PASS' if ok else 'FAIL','mav0/imu0/data.csv',req['source']))
 for label,seqs,lim in [('image',a,cfg['image_rate_hz']),('imu',ii,cfg['imu_rate_hz'])]:
  gaps=[y-x for x,y in zip(seqs,seqs[1:]) if y>x]; rate=1e9/(sum(gaps)/len(gaps)) if gaps else 0
  valid=lim['minimum']<=rate<=lim['maximum']
  criteria.append(criterion(label+'_rate',f'{label} mean rate in configured range',rate,'Hz',f"{lim['minimum']}..{lim['maximum']}",'PASS' if valid else 'FAIL',str(cams[0].relative_to(seq) if label=='image' else imu.relative_to(seq)),lim['source']))
  gaplim=cfg[label+'_gap_ns']; maxgap=max(gaps,default=0); validgap=maxgap<=gaplim['maximum']
  criteria.append(criterion(label+'_gap',f'{label} maximum gap within configured limit',maxgap,'ns',f"<= {gaplim['maximum']}",'PASS' if validgap else 'FAIL',str(cams[0].relative_to(seq) if label=='image' else imu.relative_to(seq)),gaplim['source']))
 legacy=[int(x) for x in legacy_times.read_text().splitlines() if x.strip()]
 consistent=tt==a and legacy==tt and bool(md.get('clock_policy',{}).get('selected_stamp')=='ROS sensor_msgs header.stamp')
 lim=cfg['times_file_mismatch_count']; mismatch=0 if consistent else 1
 criteria.append(criterion('times_file','times_ns.txt and timestamps.txt match cam0 stamps and metadata clock policy',mismatch,'mismatches',f"<= {lim['maximum']}",'PASS' if mismatch<=lim['maximum'] else 'FAIL','timestamps.txt + metadata.json',lim['source']))
 return criteria,raw,notes

def main():
 p=ContractParser(); p.add_argument('sequence',type=Path,nargs='?'); p.add_argument('--analyze-only',type=Path); p.add_argument('--outdir',type=Path); a=p.parse_args()
 seq=a.analyze_only or a.sequence
 if seq is None or a.outdir is None: p.error('provide SEQUENCE (or --analyze-only DIR) and --outdir DIR')
 start=datetime.now(timezone.utc).isoformat(); out=a.outdir; out.mkdir(parents=True,exist_ok=True)
 criteria,raw,notes=evaluate(seq)
 if any(c['status']=='BLOCKED' for c in criteria):
  missing=out/'missing_files.txt'; missing.write_text('\n'.join(str(p) for p in [seq/'mav0/cam0/data.csv',seq/'mav0/cam1/data.csv',seq/'mav0/imu0/data.csv',seq/'times_ns.txt',seq/'timestamps.txt',seq/'metadata.json'] if not p.is_file())+'\n')
  raw=raw or []; raw.append({'path':'missing_files.txt','sha256':sha(missing)})
 states=[c['status'] for c in criteria]; status='FAIL' if 'FAIL' in states else 'BLOCKED' if 'BLOCKED' in states else 'PASS'
 evidence_name='missing_files.txt' if status=='BLOCKED' else 'qualification.txt'
 for c in criteria:c['evidence']=evidence_name
 with (out/'qualification.txt').open('w') as f:
  f.write('criterion | status | measured | threshold\n')
  for c in criteria:f.write(f"{c['id']} | {c['status']} | {c['measured']} {c['unit']} | {c['threshold']}\n")
 if status!='BLOCKED':raw.append({'path':'qualification.txt','sha256':sha(out/'qualification.txt')})
 end=datetime.now(timezone.utc).isoformat()
 result={'check':'qualify_euroc','tool_version':VERSION,'status':status,'started_utc':start,'ended_utc':end,'host':{'hostname':socket.gethostname(),'platform':platform.platform(),'python':sys.version.split()[0]},'criteria':criteria,'raw_files':raw,'notes':notes}
 (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
 print((out/'qualification.txt').read_text(),end=''); print('Overall:',status)
 return {'PASS':0,'FAIL':1,'BLOCKED':2}[status]
if __name__=='__main__': raise SystemExit(main())
