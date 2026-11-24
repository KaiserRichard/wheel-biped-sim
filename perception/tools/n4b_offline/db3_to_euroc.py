#!/usr/bin/env python3
"""Export a librealsense ROS 2 SQLite bag to EuRoC layout (stdlib only).

Adapted from experiments/2026-10-14-db3-to-euroc/db3_to_euroc.py;
original attribution: WBR Perception Phase 1 Runtime project, 2026-10-14.
This hardened version selects ROS header.stamp exclusively. SQLite receive
 timestamps and librealsense metadata timestamps are never used for export.
"""
from __future__ import annotations
import argparse, csv, hashlib, json, math, sqlite3, struct, sys, zlib
from bisect import bisect_right
from datetime import datetime, timezone
from pathlib import Path

TOPICS = {
    "ir1": "/device_0/sensor_0/Infrared_1/image/data",
    "ir2": "/device_0/sensor_0/Infrared_2/image/data",
    "gyro": "/device_0/sensor_2/Gyro_0/imu/data",
    "accel": "/device_0/sensor_2/Accel_0/imu/data",
}
WIDTH, HEIGHT = 640, 480

class CDR:
    def __init__(self, blob):
        if len(blob) < 4 or blob[:2] != b'\x00\x01': raise ValueError('expected CDR little-endian encapsulation')
        self.b, self.p = memoryview(blob), 4
    def get(self, fmt):
        size=struct.calcsize('<'+fmt); a=min(struct.calcsize('<'+fmt[0]),8); self.p=(self.p+a-1)&~(a-1)
        if self.p+size>len(self.b): raise ValueError('truncated CDR')
        x=struct.unpack_from('<'+fmt,self.b,self.p); self.p+=size
        return x[0] if len(x)==1 else x
    def string(self):
        n=self.get('I')
        if n<1 or self.p+n>len(self.b): raise ValueError('invalid CDR string')
        raw=bytes(self.b[self.p:self.p+n]); self.p+=n
        if raw[-1:]!=b'\0': raise ValueError('unterminated CDR string')
        return raw[:-1].decode('utf8')
    def seq(self):
        n=self.get('I')
        if n>len(self.b)-self.p: raise ValueError('invalid CDR sequence')
        v=bytes(self.b[self.p:self.p+n]); self.p+=n; return v

def header(c):
    sec,nsec=c.get('iI'); c.string()
    if nsec>=1_000_000_000: raise ValueError('invalid ROS header.stamp nanoseconds')
    return sec*1_000_000_000+nsec

def image(blob):
    c=CDR(blob); ns=header(c); h,w=c.get('II'); enc=c.string(); be=c.get('B'); step=c.get('I'); pix=c.seq()
    if (w,h)!=(WIDTH,HEIGHT) or enc.lower() not in ('mono8','y8') or be!=0 or step!=w or len(pix)!=w*h:
        raise ValueError(f'unsupported image layout: {w}x{h} {enc} endian={be} step={step} bytes={len(pix)}')
    return ns,pix

def imu(blob):
    c=CDR(blob); ns=header(c); c.get('dddd'); c.get('d'*9); g=c.get('ddd'); c.get('d'*9); a=c.get('ddd'); c.get('d'*9)
    if not all(math.isfinite(x) for x in (*g,*a)): raise ValueError('non-finite IMU value')
    return ns,g,a

def png(path,pixels):
    def chunk(t,d):
        q=t+d; return struct.pack('>I',len(d))+q+struct.pack('>I',zlib.crc32(q)&0xffffffff)
    scan=b''.join(b'\0'+pixels[y*WIDTH:(y+1)*WIDTH] for y in range(HEIGHT))
    path.write_bytes(b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('>IIBBBBB',WIDTH,HEIGHT,8,0,0,0,0))+chunk(b'IDAT',zlib.compress(scan,6))+chunk(b'IEND',b''))

def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda:f.read(1<<20),b''): h.update(b)
    return h.hexdigest()

def load(conn, topic):
    t=conn.execute('SELECT id FROM topics WHERE name=?',(topic,)).fetchone()
    if not t: raise KeyError(topic)
    return [bytes(r[0]) for r in conn.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY id',(t[0],))]

def interpolate(accels, t):
    ts=[r[0] for r in accels]; i=bisect_right(ts,t)
    if i==0 or i==len(ts): return None
    t0,a0=accels[i-1]; t1,a1=accels[i]
    if t1<=t0: return None
    q=(t-t0)/(t1-t0); return tuple(x+q*(y-x) for x,y in zip(a0,a1))

def detect_policy(conn, gyro, accel):
    """Use a combined RealSense IMU topic when available to classify unite_imu_method.

    Separate gyro/accel topics alone do not encode the driver's unite method;
    in that layout the export policy is explicitly marked as exporter interpolation.
    """
    rows=conn.execute('SELECT id,name FROM topics').fetchall()
    candidates=[n for _,n in rows if n not in TOPICS.values() and n.endswith('/imu/data') and ('Imu' in n or n.endswith('/imu/data'))]
    for topic in candidates:
        try: blobs=load(conn,topic)
        except (KeyError,sqlite3.Error): continue
        matched_copy=matched_linear=0
        accel_by_t={t:a for t,a in accel}
        for b in blobs:
            try: t,g,a=imu(b)
            except ValueError: continue
            if not any(abs(t-gt)<=0 for gt,_ in gyro): continue
            exact=accel_by_t.get(t)
            if exact is not None and all(abs(x-y)<1e-6 for x,y in zip(a,exact)): matched_copy+=1
            else:
                pred=interpolate(accel,t)
                if pred is not None and all(abs(x-y)<1e-6 for x,y in zip(a,pred)): matched_linear+=1
        if matched_copy or matched_linear:
            return {'driver_method':'copy' if matched_copy>=matched_linear else 'linear_interpolation','export_method':'linear_interpolation','evidence':'combined IMU messages matched against raw gyro/accel header.stamp data','combined_topic':topic,'copy_matches':matched_copy,'linear_interpolation_matches':matched_linear}
    return {'driver_method':'UNKNOWN','export_method':'linear_interpolation','evidence':'no verifiable united IMU topic in bag; exporter interpolates raw accel to gyro header.stamp grid (driver unite_imu_method is not observable from separate topics)','combined_topic':None,'copy_matches':0,'linear_interpolation_matches':0}

def convert(db_path:Path,out:Path):
    if out.exists() and any(out.iterdir()): raise ValueError(f'output directory is not empty: {out}')
    conn=sqlite3.connect(f'file:{db_path}?mode=ro',uri=True)
    try:
        streams={}
        for k,topic in TOPICS.items():
            try: blobs=load(conn,topic)
            except KeyError as e: raise ValueError(f'missing required topic {topic}') from e
            if not blobs: raise ValueError(f'missing samples for {k}')
            if k.startswith('ir'):
                streams[k]=[(*image(b),) for b in blobs] # (stamp,pixels)
            else:
                streams[k]=[(lambda v:(v[0],v[1] if k=='gyro' else v[2]))(imu(b)) for b in blobs]
        policy=detect_policy(conn,streams['gyro'],streams['accel'])
        topic_rows=conn.execute('SELECT name,type FROM topics ORDER BY id').fetchall()
    finally: conn.close()
    for k,rs in streams.items():
        ts=[r[0] for r in rs]
        if any(b<=a for a,b in zip(ts,ts[1:])): raise ValueError(f'{k}: header.stamp must be strictly increasing')
    l={t:pix for t,pix in streams['ir1']}; r={t:pix for t,pix in streams['ir2']}; common=sorted(l.keys()&r.keys())
    if not common: raise ValueError('no exact stereo header.stamp matches')
    out.mkdir(parents=True,exist_ok=True)
    file_hashes=[]
    for cam,src in [('cam0',l),('cam1',r)]:
        d=out/'mav0'/cam; (d/'data').mkdir(parents=True)
        with (d/'data.csv').open('w',newline='') as f:
            w=csv.writer(f); w.writerow(['#timestamp [ns]','filename'])
            for t in common:
                n=f'{t}.png'; w.writerow([t,n]); png(d/'data'/n,src[t])
    (out/'mav0'/'imu0').mkdir(parents=True)
    imuro=[]
    for t,g in streams['gyro']:
        a=interpolate(streams['accel'],t)
        if a is not None: imuro.append((t,*g,*a))
    with (out/'mav0/imu0/data.csv').open('w',newline='') as f:
        w=csv.writer(f); w.writerow(['#timestamp [ns]','w_RS_S_x','w_RS_S_y','w_RS_S_z','a_RS_S_x','a_RS_S_y','a_RS_S_z']); w.writerows(imuro)
    time_text=''.join(f'{t}\n' for t in common)
    (out/'timestamps.txt').write_text(time_text)
    (out/'times_ns.txt').write_text(time_text)
    input_size=db_path.stat().st_size
    meta={'schema_version':'1.0','input_db3':db_path.name,'input_sha256':sha(db_path),'input_bytes':input_size,
      'generated_utc':datetime.now(timezone.utc).isoformat(),'clock_policy':{'selected_stamp':'ROS sensor_msgs header.stamp','clock_domain':'D435i device/global time as configured by realsense2_camera; mapping remains UNKNOWN','conversion':'ROS sec*1e9 + nanosec integer','sqlite_messages.timestamp':'bag receive timestamp; ignored','librealsense metadata timestamp':'ignored'},
      'imu_combination_policy':policy,'imu_units':'gyro rad/s; acceleration m/s^2; no axis transform','streams':{k:{'topic':TOPICS[k],'count':len(v),'first_header_stamp_ns':v[0][0],'last_header_stamp_ns':v[-1][0]} for k,v in streams.items()},'stereo_pairs':len(common),'imu_rows':len(imuro),'note':'Separate accel/gyro bags do not reveal driver unite_imu_method; when no combined stream matches raw values, converter interpolation is stated without claiming driver policy.'}
    (out/'metadata.json').write_text(json.dumps(meta,indent=2)+'\n')
    print(f'EuRoC export: {len(common)} stereo pairs, {len(imuro)} IMU rows; clock=header.stamp; accel policy={policy["driver_method"]}')

def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__); p.add_argument('db3',type=Path); p.add_argument('out_dir',type=Path); a=p.parse_args(argv)
    try: convert(a.db3,a.out_dir)
    except (ValueError,sqlite3.Error,OSError,struct.error) as e: print(f'ERROR: {e}',file=sys.stderr); return 1
    return 0
if __name__=='__main__': raise SystemExit(main())
