"""Build a Foxglove JSON MCAP from the recorded camera-day side streams.

All side streams are presentation overlays. IMU clock alignment is arbitrary;
the required --imu-offset is an explicit offset from the first IMU sample to
the first camera packet, in seconds.
"""
from __future__ import annotations

import argparse
from bisect import bisect_right
import csv
import json
import math
import re
import sys
from pathlib import Path

from mcap.writer import Writer

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
DEFAULT_PACKETS = Path.home() / "wbr-evidence-archive/lab-day-20261025/3-box13/packets.csv"
DEFAULT_IMU = Path.home() / "wbr-evidence-archive/imu-view-live-20261030/imu_view_run.csv"
DEFAULT_BUS = Path.home() / "wbr-evidence-archive/imu-can-bench-20261029/run-02/raw.log"
DEFAULT_FB = Path.home() / "wbr-evidence-archive/imu-can-bench-20261029/canfix05-home-01/motor_fb.csv"

def schema(title, props):
    return {"$schema": "http://json-schema.org/draft-07/schema#", "title": title,
            "type": "object", "properties": props, "required": list(props),
            "additionalProperties": False}

NUM = {"type": "number"}
STR = {"type": "string"}
SCHEMAS = {
    "/body/imu": schema("wbr.bench.Imu", {"timestamp_ns": {"type":"integer"}, "pitch_rad": NUM, "roll_rad": NUM, "pitch_rate_rad_s": NUM, "roll_rate_rad_s": NUM, "alignment": STR}),
    "/body/jump_phase": schema("wbr.bench.JumpPhase", {"phase": STR, "source": STR}),
    "/body/leg_length": schema("wbr.bench.LegLength", {"length_m": NUM, "source": STR}),
    "/bus/stats": schema("wbr.bench.BusStats", {"rate_hz": NUM, "errors": {"type":"integer"}, "busy": {"type":"integer"}, "esr": STR, "source": STR}),
    "/bus/motor_feedback": schema("wbr.bench.MotorFeedback", {"left_position_rad": NUM, "right_position_rad": NUM, "left_velocity_rad_s": NUM, "right_velocity_rad_s": NUM, "left_effort_nm": NUM, "right_effort_nm": NUM, "source": STR}),
    "/brain/advisory": schema("wbr.bench.Advisory", {"raw": STR, "gated": STR, "gate_confidence": NUM, "confidence_source": STR}),
    "/twin/scene": schema("foxglove.SceneUpdate", {"entities": {"type":"array", "items":{"type":"object"}}}),
}

def enc(x): return json.dumps(x, separators=(",", ":"), allow_nan=False).encode()

def rows(path):
    with Path(path).open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))

def packet_data(path):
    from tools.scene_twin.replay import read_packets
    packets = read_packets(Path(path))
    return packets

def _tilt(samples):
    from tools.imu_view.filter import TiltFilters
    filt = TiltFilters()
    prev = None
    out = []
    for s in samples:
        if prev is None: dt = .01
        else: dt = s.t - prev
        est = filt.update(s.accel, s.gyro, dt)
        from tools.imu_view.filter import euler_rates
        pd, rd = euler_rates(est.pitch, est.roll, *s.gyro)
        out.append((s.t, est.pitch, est.roll, pd, rd))
        prev = s.t
    return out

def _bus_rows(path):
    pat = re.compile(r"STAT\s+ms=(\d+).*?c1_busy=(\d+).*?c1_txok=(\d+).*?c1_txerr=(\d+).*?c1_esr=(0x[0-9a-fA-F]+).*?c2_rx=(\d+).*?c2_busy=(\d+).*?c2_esr=(0x[0-9a-fA-F]+)")
    parsed=[]
    for line in Path(path).read_text(errors="replace").splitlines():
        m=pat.search(line)
        if m: parsed.append(tuple(m.groups()))
    if not parsed: raise ValueError(f"no usable STAT records in {path}")
    out=[]; last=None
    for r in parsed:
        ms,busy,tx,err,esr,rx,busy2,esr2= r
        cur=tuple(map(int,(ms,tx,err,busy,rx,busy2)))
        if last:
            dt=(cur[0]-last[0])/1000
            rate=((cur[1]-last[1])+(cur[4]-last[4]))/(2*dt) if dt>0 else 0
            out.append((cur[0]/1000, rate, (cur[2]-last[2]), (cur[3]-last[3])+(cur[5]-last[5]), esr if esr!='0x00000000' else esr2))
        last=cur
    return out

def convert(output, *, packets=DEFAULT_PACKETS, imu=DEFAULT_IMU, imu_offset, bus=DEFAULT_BUS, feedback=DEFAULT_FB, jump_at=20.0):
    packets=Path(packets); output=Path(output)
    if output.exists(): raise FileExistsError(f"refusing to overwrite {output}")
    ps=packet_data(packets)
    if not ps: raise ValueError("no camera packet rows")
    cam0=ps[0].stamp
    from tools.imu_view.stream import from_csv
    imu_data=_tilt(from_csv(imu)); imu0=imu_data[0][0]
    # The offset is intentionally the only IMU-to-camera alignment control.
    imu_data=[(cam0+round((t-imu0+imu_offset)*1e9),p,r,pd,rd) for t,p,r,pd,rd in imu_data]
    from tools.jump_gate.timeline import build_timeline
    from tools.scene_twin.replay import Replay, bag_start, read_packets
    replay=Replay(read_packets(packets), bag_start(packets.parent), motion="static")
    timeline=build_timeline(replay, jump_at=jump_at)
    by_packet={}
    for tr in timeline.rows: by_packet.setdefault(tr.packet_index,tr)
    bus_data=_bus_rows(bus)
    fb_rows=rows(feedback)
    usable_fb=[r for r in fb_rows if r.get("elapsed_s") and r.get("left_position_rad")]
    # Bus and motor log clocks are separate bench captures: display relative to camera start, marked.
    messages=[]
    def add(topic, stamp, obj): messages.append((int(stamp),topic,obj))
    for stamp,p,r,pd,rd in imu_data:
        add("/body/imu",stamp,dict(timestamp_ns=stamp,pitch_rad=p,roll_rad=r,pitch_rate_rad_s=pd,roll_rate_rad_s=rd,alignment="ARBITRARY: separate IMU capture; explicit offset"))
    imu_stamps=[x[0] for x in imu_data]
    from tools.scene_twin.replay import NAMES
    for i,p in enumerate(ps):
        tr=by_packet[i]
        out=tr.output
        add("/brain/advisory",p.stamp,dict(raw=NAMES[p.advisory],gated=out.advisory.name,gate_confidence=out.confidence,confidence_source="SYNTHETIC gate replay; source raw confidence ASSUMED 1.0"))
        add("/body/jump_phase",p.stamp,dict(phase=tr.pose.phase,source="SYNTHETIC jump_gate timeline"))
        add("/body/leg_length",p.stamp,dict(length_m=tr.pose.leg_length_m,source="SYNTHETIC jump_gate timeline"))
        # Simple schematic, with body pitch and leg extension. Geometry is explicitly assumed.
        length=tr.pose.leg_length_m
        imu_i=max(0,min(len(imu_data)-1,bisect_right(imu_stamps,p.stamp)-1))
        pitch=imu_data[imu_i][1]
        q=[0,math.sin(pitch/2),0,math.cos(pitch/2)]
        def cube(x,y,z,sx,sy,sz): return {"pose":{"position":{"x":x,"y":y,"z":z},"orientation":{"x":q[0],"y":q[1],"z":q[2],"w":q[3]}},"size":{"x":sx,"y":sy,"z":sz},"color":{"r":.2,"g":.55,"b":.9,"a":1}}
        ent={"timestamp":{"sec":p.stamp//10**9,"nsec":p.stamp%10**9},"frame_id":"base_link","id":"bench-twin","cubes":[cube(0,0,.20,.32,.20,.12),cube(.10,.10,-length/2,.035,.035,length),cube(.10,-.10,-length/2,.035,.035,length),cube(-.10,.13,.04,.08,.04,.08),cube(-.10,-.13,.04,.08,.04,.08)]}
        add("/twin/scene",p.stamp,{"entities":[ent]})
    for t,rate,errors,busy,esr in bus_data:
        add("/bus/stats",cam0+round(t*1e9),dict(rate_hz=rate,errors=int(errors),busy=int(busy),esr=esr,source="MEASURED 2026-10-29 STAT logs; timestamp offset ARBITRARY"))
    if usable_fb:
        for r in usable_fb:
            stamp=cam0+round(float(r["elapsed_s"])*1e9)
            add("/bus/motor_feedback",stamp,{k:float(r[k]) for k in ("left_position_rad","right_position_rad","left_velocity_rad_s","right_velocity_rad_s","left_effort_nm","right_effort_nm")} | {"source":"MEASURED CSV; timestamp offset ARBITRARY"})
    else:
        # No populated feedback CSV: generate a clearly labelled illustrative stream.
        for k in range(100):
            t=k*.1; add("/bus/motor_feedback",cam0+round(t*1e9),dict(left_position_rad=0.0,right_position_rad=0.0,left_velocity_rad_s=0.0,right_velocity_rad_s=0.0,left_effort_nm=0.0,right_effort_nm=0.0,source="SYNTHETIC zero telemetry; no populated feedback rows"))
    messages.sort(key=lambda x:x[0])
    output.parent.mkdir(parents=True,exist_ok=True)
    counts={}
    with output.open("xb") as f:
        w=Writer(f); w.start(profile="",library="wbr.bench_twin")
        channels={}
        for topic,s in SCHEMAS.items():
            sid=w.register_schema(s["title"],"jsonschema",enc(s)); channels[topic]=w.register_channel(topic,"json",sid)
        seq=0
        for stamp,topic,obj in messages:
            w.add_message(channels[topic],log_time=stamp,publish_time=stamp,sequence=seq,data=enc(obj)); seq+=1; counts[topic]=counts.get(topic,0)+1
        w.finish()
    return {"output":str(output),"messages":len(messages),"counts":counts,"first_ns":messages[0][0],"last_ns":messages[-1][0],"bus_records":len(bus_data),"feedback_rows":len(usable_fb),"imu_samples":len(imu_data)}

def main():
    a=argparse.ArgumentParser(description=__doc__); a.add_argument("output",type=Path); a.add_argument("--packets",type=Path,default=DEFAULT_PACKETS); a.add_argument("--imu",type=Path,default=DEFAULT_IMU); a.add_argument("--imu-offset",type=float,required=True); a.add_argument("--bus",type=Path,default=DEFAULT_BUS); a.add_argument("--feedback",type=Path,default=DEFAULT_FB); a.add_argument("--jump-at",type=float,default=20.0); x=a.parse_args()
    print(json.dumps(convert(x.output,packets=x.packets,imu=x.imu,imu_offset=x.imu_offset,bus=x.bus,feedback=x.feedback,jump_at=x.jump_at),indent=2))
if __name__=="__main__": main()
