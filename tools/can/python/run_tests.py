"""Run independent C/cantools parity, log replay and CAN load experiments."""
import csv
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import random
import sys
import time
import bisect
import can
import cantools
from dm_codec import *
from decode_log import decode, frames
from bus_load import load

ROOT=Path(__file__).resolve().parents[1]


def write_csv(path,rows):
    with path.open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=rows[0]);w.writeheader();w.writerows(rows)


def main():
    start=time.perf_counter(); checks={}
    db=cantools.database.load_file(ROOT/'wbr_dm.dbc')
    fw=cantools.database.load_file(ROOT/'dbcs/codec_firmware_wheel.dbc')
    for p in (ROOT/'dbcs').glob('*.dbc'): cantools.database.load_file(p,strict=True)
    vectors=list(csv.DictReader((ROOT/'tests/vectors.csv').open()))
    oracle=list(csv.DictReader((ROOT/'out/c_oracle.csv').open()))
    assert len(vectors)==len(oracle)
    physical_rounding_differences=0
    max_c_decode_error=0
    for r,c in zip(vectors,oracle):
        assert r['case']==c['case'] and r['hex']==c['hex']
        profile=r['profile'];v=[float(r[k]) for k in FIELDS];q=quantize(v,profile)
        ids=[1,4,2,5] if profile=='hip' else [3,6] if profile=='wheel' else [3]
        use=fw if profile=='firmware_wheel' else db
        for identifier in ids:
            msg=use.get_message_by_frame_id(identifier)
            data=msg.encode(dict(zip(FIELDS,q)),scaling=False)
            assert data.hex().upper()==c['hex']
            raw=msg.decode(data,scaling=False,decode_choices=False)
            assert [raw[k] for k in FIELDS]==q
            decoded=msg.decode(data,decode_choices=False)
            for k,(lo,hi),nb in zip(FIELDS,limits(profile),BITS):
                target=min(hi,max(lo,f32(float(r[k]))))
                assert abs(decoded[k]-target)<=(hi-lo)/((1<<nb)-1)+2e-6
            for k in ['Position','Velocity','Torque']:
                err=abs(decoded[k]-float(c[k.lower()]))
                max_c_decode_error=max(max_c_decode_error,err)
                assert err < 2e-5, (r['case'],k,err)
        vals={k:min(hi,max(lo,float(r[k]))) for k,(lo,hi) in zip(FIELDS,limits(profile))}
        physical_rounding_differences += msg.encode(vals)!=data
    checks['C_vectors']=len(vectors)
    checks['max_C_float32_vs_DBC_double_decode_error']=max_c_decode_error
    checks['implicit_physical_rounding_differs_vectors']=physical_rounding_differences
    # Walking every command bit and random feedback payloads catch swapped nibbles.
    rng=random.Random(714)
    for axis in range(6):
        m=db.get_message_by_frame_id(axis+1)
        for field,nb in enumerate(BITS):
            for bit in range(nb):
                q=[0]*5;q[field]=1<<bit
                data=m.encode(dict(zip(FIELDS,q)),scaling=False)
                assert data==pack_codes(q)
                assert m.decode(data,scaling=False,decode_choices=False)==dict(zip(FIELDS,q))
    checks['walking_bit_vectors']=384
    fbrows=[]
    for axis in range(6):
        profile='wheel' if axis in (2,5) else 'hip'
        msg=db.get_message_by_frame_id(axis+17)
        for k in range(96):
            q=[rng.randrange(1<<nb) for nb in BITS]
            state=k%16;mos=0 if k%2 else 255;rotor=k%256
            data=feedback(q,axis+1,state,mos,rotor)
            raw=msg.decode(data,scaling=False,decode_choices=False)
            expected=dict(MotorID=axis+1,State=state,Position=q[0],Velocity=q[1],Torque=q[4],
                          MOS_Temperature=mos,Rotor_Temperature=rotor)
            assert raw==expected and msg.encode(raw,scaling=False)==data
            fbrows.append(dict(id=axis+17,hex=data.hex().upper(),**expected))
    write_csv(ROOT/'tests/feedback_vectors.csv',fbrows);checks['feedback_vectors']=len(fbrows)
    specialdb=cantools.database.load_file(ROOT/'dbcs/wbr_dm_special.dbc')
    for axis in range(6):
        for opcode in SPECIAL:
            m=specialdb.get_message_by_frame_id(axis+1)
            values={f'Prefix{k}':255 for k in range(7)};values['Opcode']=opcode
            assert m.encode(values)==special(opcode)
    checks['lifecycle_vectors']=24
    for v in [[math.nan]*5,[math.inf]*5]:
        try: quantize(v,'hip'); raise AssertionError('nonfinite accepted')
        except ValueError: pass
    assert quantize([1e100]*5,'hip')==quantize([12.5,30,500,5,10],'hip')
    try:
        command([12.5,30,500,5,10-2.5*20/4095],'hip')
        raise AssertionError('lifecycle collision accepted')
    except ValueError:pass
    checks['nonfinite_extreme_reserved_checks']=4
    # Address validation is deliberately separate from decoding payloads.
    mapping=json.loads((ROOT/'reference/motor_map.json').read_text())
    conflicts=[]
    for bus in range(1,4):
        group=[m for m in mapping if m['bus']==bus]
        for role in ['firmware_command_id','firmware_feedback_id']:
            if len({m[role] for m in group})!=len(group):
                conflicts.append(dict(bus=bus,field=role,reason='duplicate ID on S12 bus'))
    (ROOT/'out/address_conflicts.json').write_text(json.dumps(conflicts,indent=2)+'\n')
    assert len(conflicts)==5;checks['address_conflicts_detected']=len(conflicts)
    # Preserve the trace in the study; source contains aggregate channels only.
    source=ROOT/'reference/s11_trace.csv'
    if not source.exists():
        source.write_bytes((ROOT.parent/'S11_packet_tolerance/out/traces/001_s11_s9.csv').read_bytes())
    trace=list(csv.DictReader(source.open()))
    (ROOT/'reference/trace_provenance.json').write_text(json.dumps(dict(source='../S11_packet_tolerance/out/traces/001_s11_s9.csv',
        sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        mapping='Aggregate MuJoCo channels -> synthetic six-axis fixture, not original joint telemetry',
        command_torque='hips: signed hip_max_Nm; wheels: wheel_drive_mean_Nm',
        feedback_torque='command delayed by 2 ms (assumed actuator replay)',
        position='hips: pitch_deg converted to rad with role sign; wheels: integrated wheel_rate_mean_rad_s',
        velocity='hips: pitch_rate_deg_s to rad/s; wheels: wheel_rate_mean_rad_s',
        temperatures='constant synthetic 30/31 degC; State=1 enabled',
        transport='500 Hz; deterministic +/-40 us release fixture; 50 us turnaround; actual stuffing',
        interval='source t=1..3 s; output relative t=0..2 s'),indent=2)+'\n')
    times=[float(r['time_s']) for r in trace]
    def sample(t,key):
        j=max(0,min(len(times)-2,bisect.bisect_right(times,t)-1));a=(t-times[j])/(times[j+1]-times[j])
        return float(trace[j][key])*(1-a)+float(trace[j+1][key])*a
    wheel_pos=0;events=[]
    for tick in range(1000):
        t=tick*.002;wheel_pos+=sample(t+1,'wheel_rate_mean_rad_s')*.002
        for bus in range(1,4):
            group=[m for m in mapping if m['bus']==bus];pending=[]
            for m in group:
                wheel=m['profile']=='wheel';sign=1 if 'front' in m['axis'] or wheel else -1
                def values(stamp):
                    return [wheel_pos if wheel else sign*math.radians(sample(stamp,'pitch_deg')),
                            sample(stamp,'wheel_rate_mean_rad_s') if wheel else sign*math.radians(sample(stamp,'pitch_rate_deg_s')),
                            0,0,sample(stamp,'wheel_drive_mean_Nm') if wheel else sign*sample(stamp,'hip_max_Nm')]
                cmd=command(values(t+1),m['profile'])
                fb=feedback(quantize(values(max(1,t+1-.002)),m['profile']),m['study_command_id'])
                release=t+.00004+.00004*math.sin(tick*.13+m['study_command_id'])
                pending.append((release,m['study_command_id'],cmd,m,fb))
            now=t
            while pending:
                now=max(now,min(x[0] for x in pending))
                chosen=min((x for x in pending if x[0]<=now+1e-12),key=lambda x:x[1]);pending.remove(chosen)
                ready,identifier,data,m,fb=chosen
                events.append((now,bus,identifier,data));now+=frame_bits(identifier,data)/1e6
                if fb is not None: pending.append((now+.00005,m['study_feedback_id'],fb,m,None))
    # Lifecycle fixtures precede the motion and stay outside load window.
    for m in mapping:
        for k,opcode in enumerate(SPECIAL):events.append((-.010+k*.001,m['bus'],m['study_command_id'],special(opcode)))
    # Shift all times positive: CAN log readers use nonnegative relative stamps.
    events=sorted((t+.01,b,i,d) for t,b,i,d in events)
    path=ROOT/'out/synthetic.log'
    path.write_text(''.join(f'({t:.9f}) can{b} {i:03X}#{d.hex().upper()}\n' for t,b,i,d in events))
    for suffix,cls in [('asc',can.ASCWriter),('blf',can.BLFWriter)]:
        with cls(ROOT/f'out/synthetic.{suffix}') as writer:
            for t,b,i,d in events:writer(can.Message(timestamp=t,channel=b-1,arbitration_id=i,is_extended_id=False,data=d))
    reference=decode(path,ROOT/'out/decoded_python.csv')
    for suffix in ['asc','blf']:
        other=decode(ROOT/f'out/synthetic.{suffix}',ROOT/f'out/decoded_{suffix}.csv')
        assert len(other)==len(reference)
        for a,b in zip(reference,other):
            assert all(a[k]==b[k] for k in ['bus','id','hex','kind','axis'])
            assert abs(a['time_s']-b['time_s'])<1.1e-6
    checks['decoded_frames_per_format']=len(reference);checks['formats_agree']=3
    assert all(r['validation']=='ok' for r in reference)
    # Negative fixtures: invalid DLC rejected; unknown ID kept; motor ID mismatch flagged.
    bad=ROOT/'tests/bad_dlc.log';bad.write_text('(0.1) can1 001#00\n')
    try:decode(bad,ROOT/'out/unused.csv');raise AssertionError('bad DLC accepted')
    except ValueError:pass
    neg=ROOT/'tests/negative.log';neg.write_text('(0.1) can1 777#0000000000000000\n(0.2) can1 011#1200000000000000\n')
    negrows=decode(neg,ROOT/'out/negative_decode.csv')
    assert [r['validation'] for r in negrows]==['unknown_id','payload_id_mismatch']
    checks['negative_log_checks']=3
    loads=[]
    s12=list(csv.DictReader((ROOT/'reference/s12_can_rta.csv').open()))
    for buses,n in [(1,6),(2,3),(3,2)]:
        computed=load(n);lower=computed['unstuffed_pct'];upper=computed['worst_pct']
        expected=next(r for r in s12 if int(r['buses'])==buses)
        assert abs(upper-float(expected['max_stuffed_load_pct']))<1e-10
        loads.append(dict(buses=buses,motors_per_bus=n,bitrate_bps=1000000,rate_Hz=500,
            unstuffed_pct=lower,worst_pct=upper,s12_worst_pct=expected['max_stuffed_load_pct'],
            worst_one_retry_per_cycle_pct=upper+500*(31+135)/1e6*100))
    write_csv(ROOT/'out/bus_load.csv',loads)
    sensitivity=[]
    for rate,bitrate,replies,retries in [(250,1000000,1,0),(500,1000000,1,0),
            (1000,1000000,1,0),(500,500000,1,0),(500,2000000,1,0),
            (500,1000000,0,0),(500,1000000,2,0),(500,1000000,1,1)]:
        sensitivity.append(dict(rate_Hz=rate,bitrate_bps=bitrate,replies_per_command=replies,
            retries_per_cycle=retries,**load(2,rate,bitrate,replies=replies,retries_per_cycle=retries)))
    write_csv(ROOT/'out/load_sensitivity.csv',sensitivity)
    steps=[]
    for p in ['hip','wheel','firmware_wheel']:
        for field,nb,(lo,hi) in zip(FIELDS,BITS,limits(p)):
            steps.append(dict(profile=p,field=field,bits=nb,minimum=lo,maximum=hi,step=(hi-lo)/((1<<nb)-1)))
    write_csv(ROOT/'out/codec_steps.csv',steps)
    # Compare actual CRC/stuffing with S12 implementation for 1000 random frames.
    # Load just the bits function: avoid NumPy, simulations or module side effects.
    text=(ROOT/'reference/s12_bits.py').read_text()
    scope={};exec(text,scope)
    for _ in range(1000):
        i=rng.randrange(2048);data=bytes(rng.randrange(256) for _ in range(8))
        assert frame_bits(i,data)==scope['bits'](i,data)<=135
    checks['S12_stuffing_parity_frames']=1000
    actual=[]
    for b in range(1,4):
        bits=sum(r['frame_bits'] for r in reference if r['bus']==b and r['kind'] in ['command','feedback'])
        actual.append(dict(bus=b,frames=4000,interval_s=2,actual_stuffed_pct=bits/2e6*100,worst_pct=27))
    write_csv(ROOT/'out/replay_bus_load.csv',actual)
    checks['seconds']=time.perf_counter()-start
    checks['cantools_version']=cantools.__version__;checks['python_can_version']=can.__version__
    checks['result']='PASS'
    (ROOT/'out/python_summary.json').write_text(json.dumps(checks,indent=2)+'\n')
    print(json.dumps(checks,indent=2))


if __name__=='__main__':
    main()
