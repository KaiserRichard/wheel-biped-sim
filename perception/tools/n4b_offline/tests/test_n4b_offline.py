import csv,importlib.util,json,os,sqlite3,struct,subprocess,sys,tempfile,unittest
from pathlib import Path
HERE=Path(__file__).resolve().parents[1]
ROOT=HERE.parents[1]
spec=importlib.util.spec_from_file_location('db3',HERE/'db3_to_euroc.py'); db3=importlib.util.module_from_spec(spec);sys.modules['db3']=db3;spec.loader.exec_module(db3)
def cdr():return bytearray(b'\x00\x01\x00\x00')
def put(buf,fmt,*vals):
 n=struct.calcsize('<'+fmt); a=min(struct.calcsize('<'+fmt[0]),8); buf.extend(b'\0'*((-len(buf))%a));buf.extend(struct.pack('<'+fmt,*vals))
def string(buf,s):
 b=s.encode()+b'\0';put(buf,'I',len(b));buf.extend(b)
def header(buf,t):
 sec,nsec=divmod(t,10**9);put(buf,'iI',sec,nsec);string(buf,'sensor')
def image_msg(t,seed):
 b=cdr();header(b,t);put(b,'II',480,640);string(b,'mono8');put(b,'B',0);put(b,'I',640);pix=bytes([(seed+i)%256 for i in range(640*480)]);put(b,'I',len(pix));b.extend(pix);return bytes(b)
def imu_msg(t,g,a):
 b=cdr();header(b,t);put(b,'dddd',0,0,0,1);put(b,'d'*9,*([0.]*9));put(b,'ddd',*g);put(b,'d'*9,*([0.]*9));put(b,'ddd',*a);put(b,'d'*9,*([0.]*9));return bytes(b)
def make_bag(path,reverse=False):
 conn=sqlite3.connect(path);conn.executescript('CREATE TABLE topics(id INTEGER PRIMARY KEY,name TEXT,type TEXT);CREATE TABLE messages(id INTEGER PRIMARY KEY,topic_id INTEGER,timestamp INTEGER,data BLOB);')
 ids={}
 for i,(k,t) in enumerate(db3.TOPICS.items(),1):conn.execute('INSERT INTO topics VALUES(?,?,?)',(i,t,'sensor_msgs/Image' if k.startswith('ir') else 'sensor_msgs/Imu'));ids[k]=i
 entries=[]
 base=10_000_000_000
 for k,ts in [('ir1',[base,base+33_333_333]),('ir2',[base,base+33_333_333])]:
  for j,t in enumerate(ts):entries.append((ids[k],t+999999999,image_msg(t,j)))
 gt=[base-10_000_000+i*5_000_000 for i in range(12)]
 at=[base-10_000_000+i*10_000_000 for i in range(7)]
 if reverse:gt=gt[:5]+list(reversed(gt[5:]))
 for t in gt:entries.append((ids['gyro'],t+888888888,imu_msg(t,(0.1,0.2,0.3),(0,0,0))))
 for t in at:entries.append((ids['accel'],t+777777777,imu_msg(t,(0,0,0),(1.,2.,3.))))
 # Per-topic message IDs preserve stream order; deliberately unrelated receive timestamps.
 for i,(topic,recv,data) in enumerate(entries,1):conn.execute('INSERT INTO messages VALUES(?,?,?,?)',(i,topic,recv,data))
 conn.commit();conn.close()
class TestN4B(unittest.TestCase):
 def qualify(self,seq,out):return subprocess.run([sys.executable,str(HERE/'qualify_euroc.py'),'--analyze-only',str(seq),'--outdir',str(out)],capture_output=True,text=True)
 def test_round_trip_qualifies_pass(self):
  with tempfile.TemporaryDirectory() as td:
   bag=Path(td)/'tiny.db3';out=Path(td)/'euroc';make_bag(bag)
   db3.convert(bag,out)
   meta=json.loads((out/'metadata.json').read_text());self.assertEqual(meta['clock_policy']['selected_stamp'],'ROS sensor_msgs header.stamp')
   self.assertEqual(meta['imu_combination_policy']['driver_method'],'UNKNOWN');self.assertEqual(meta['imu_combination_policy']['export_method'],'linear_interpolation')
   self.assertEqual(meta['imu_rows'],12)
   p=self.qualify(out,Path(td)/'qualify');self.assertEqual(p.returncode,0,p.stdout+p.stderr)
   result=json.loads((Path(td)/'qualify/result.json').read_text());self.assertEqual(result['status'],'PASS')
   self.assertTrue(all((Path(td)/'qualify'/c['evidence']).is_file() for c in result['criteria']))
   self.assertIn('qualification.txt',[r['path'] for r in result['raw_files']])
   self.assertNotEqual(int((out/'timestamps.txt').read_text().splitlines()[0]),10_999_999_999)
 def test_injected_imu_reversal_fails(self):
  with tempfile.TemporaryDirectory() as td:
   bag=Path(td)/'tiny.db3';out=Path(td)/'euroc';make_bag(bag);db3.convert(bag,out)
   p=out/'mav0/imu0/data.csv'
   with p.open() as f: rows=list(csv.reader(f))
   p.write_text('\n'.join([','.join(rows[0])]+[','.join(r) for r in reversed(rows[1:])])+'\n')
   q=self.qualify(out,Path(td)/'q');self.assertEqual(q.returncode,1,q.stdout+q.stderr)
   res=json.loads((Path(td)/'q/result.json').read_text());self.assertEqual(res['status'],'FAIL');self.assertEqual(next(c for c in res['criteria'] if c['id']=='imu_order')['status'],'FAIL')
 def test_missing_camera_is_blocked(self):
  with tempfile.TemporaryDirectory() as td:
   seq=Path(td)/'seq';(seq/'mav0/cam1').mkdir(parents=True);q=self.qualify(seq,Path(td)/'q');self.assertEqual(q.returncode,2,q.stdout+q.stderr)
   result=json.loads((Path(td)/'q/result.json').read_text());self.assertEqual(result['status'],'BLOCKED')
   self.assertTrue(all((Path(td)/'q'/c['evidence']).is_file() for c in result['criteria']))
 def test_qualifier_usage_error_uses_contract_exit_code(self):
  p=subprocess.run([sys.executable,str(HERE/'qualify_euroc.py')],capture_output=True,text=True)
  self.assertEqual(p.returncode,3,p.stdout+p.stderr)
 def test_runner_sets_orb_library_path_before_ldd(self):
  with tempfile.TemporaryDirectory() as td:
   root=Path(td); orb=root/'orb'; (orb/'Examples/Stereo-Inertial').mkdir(parents=True); (orb/'Vocabulary').mkdir()
   binary=orb/'Examples/Stereo-Inertial/stereo_inertial_euroc'; binary.write_text('#!/usr/bin/env bash\nexit 0\n'); binary.chmod(0o755)
   (orb/'Vocabulary/ORBvoc.txt').write_text('fixture'); (orb/'lib').mkdir(); (orb/'lib/libORB_SLAM3.so').write_text('fixture library')
   seq=root/'seq'; (seq/'mav0').mkdir(parents=True); (seq/'times_ns.txt').write_text('1\n'); settings=root/'settings.yaml'; settings.write_text('fixture')
   fakebin=root/'bin'; fakebin.mkdir(); ldd=fakebin/'ldd'; ldd.write_text('#!/usr/bin/env bash\nprintf "LD_LIBRARY_PATH=%s\\n" "$LD_LIBRARY_PATH"\nprintf "libfixture.so => not found\\n"\n'); ldd.chmod(0o755)
   out=root/'run'; env=os.environ.copy(); env['PATH']=str(fakebin)+os.pathsep+env['PATH']; env.pop('LD_LIBRARY_PATH',None)
   p=subprocess.run(['bash',str(HERE/'run_n4b.sh'),'--allow-unpatched',str(orb),str(seq),str(seq/'times_ns.txt'),str(settings),str(out)],capture_output=True,text=True,env=env)
   self.assertEqual(p.returncode,2,p.stdout+p.stderr)
   self.assertIn(f'LD_LIBRARY_PATH={orb}/lib:{orb}/Thirdparty/DBoW2/lib:{orb}/Thirdparty/g2o/lib',(out/'ldd.txt').read_text())
   self.assertIn(str(orb/'lib/libORB_SLAM3.so'),(out/'provenance.sha256').read_text())
 def test_saved_real_evidence_fixture_is_labeled(self):
  d=json.loads((HERE/'tests/fixtures/real_saved_stream_summary.json').read_text());self.assertEqual(d['evidence_label'],'MEASURED');self.assertEqual(d['streams']['ir_count'] if 'ir_count' in d['streams'] else d['streams']['ir1_count'],447)
if __name__=='__main__':unittest.main()
