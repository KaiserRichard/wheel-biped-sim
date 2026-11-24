"""Align two USB logs at their first timestamp and compare 500 Hz telemetry."""
import sys,csv,subprocess,tempfile
from pathlib import Path
import numpy as np
from convert import convert

def load(p):
    tmp=Path(p).with_suffix('.csv'); convert(p,tmp)
    with open(tmp) as f: rows=[r for r in csv.DictReader(f) if r['msg_type']=='129']
    t=np.array([float(r['time_s']) for r in rows]); return t,{k:np.array([float(r[k] or 'nan') for r in rows]) for k in ('pitch_rad','wheel_speed_rad_s','wheel_torque_cmd_Nm')}
def main(a,b):
    ta,xa=load(a);tb,xb=load(b);t0=max(ta[0],tb[0]);end=min(ta[-1],tb[-1]);t=np.arange(t0,end,0.002)
    W,H=1200,1050; left,right,top=100,30,55; plot_w=W-left-right
    colors=['#1464c0','#d23b32']; names=[Path(a).stem,Path(b).stem]; labels=[('Pitch (deg)','pitch_rad'),('Wheel speed (rad/s)','wheel_speed_rad_s'),('Wheel torque command (N m)','wheel_torque_cmd_Nm')]
    svg=[f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}"><rect width="100%" height="100%" fill="white"/><style>text{{font-family:DejaVu Sans,Arial;fill:#111}} .tick{{font-size:15px}} .axis{{font-size:18px}} .title{{font-size:24px;font-weight:bold}}</style><text x="{W/2}" y="30" text-anchor="middle" class="title">Log overlay: {names[0]} vs {names[1]}</text>']
    for i,(ylab,key) in enumerate(labels):
        y0=top+i*310; pt=y0+25; ph=220; va=np.interp(t,ta,xa[key]);vb=np.interp(t,tb,xb[key]);err=va-vb
        print(f'{key}: RMSE={np.sqrt(np.mean(err**2)):.8g}, peak_abs={np.max(np.abs(err)):.8g}')
        plot_va,plot_vb=(np.degrees(va),np.degrees(vb)) if key=='pitch_rad' else (va,vb)
        lo=min(plot_va.min(),plot_vb.min());hi=max(plot_va.max(),plot_vb.max());pad=(hi-lo)*.08 or .1;lo-=pad;hi+=pad
        xp=lambda x:left+plot_w*(x-t0)/(end-t0);yp=lambda y:pt+ph*(hi-y)/(hi-lo)
        for j in range(5):
            y=lo+(hi-lo)*j/4;py=yp(y);svg.append(f'<line x1="{left}" y1="{py:.2f}" x2="{left+plot_w}" y2="{py:.2f}" stroke="#ddd"/><text x="{left-12}" y="{py+5:.2f}" text-anchor="end" class="tick">{y:.3g}</text>')
        svg.append(f'<line x1="{left}" y1="{pt}" x2="{left}" y2="{pt+ph}" stroke="#111"/><line x1="{left}" y1="{pt+ph}" x2="{left+plot_w}" y2="{pt+ph}" stroke="#111"/><text x="25" y="{pt+ph/2}" transform="rotate(-90 25 {pt+ph/2})" text-anchor="middle" class="axis">{ylab}</text>')
        for j in range(5):
            x=t0+(end-t0)*j/4;svg.append(f'<text x="{xp(x):.2f}" y="{pt+ph+23}" text-anchor="middle" class="tick">{x-t0:.1f}</text>')
        for idx,(vals,col,name) in enumerate(zip((plot_va,plot_vb),colors,names)):
            pts=' '.join(f'{xp(xx):.1f},{yp(yy):.1f}' for xx,yy in zip(t[::10],vals[::10]));svg.append(f'<polyline points="{pts}" fill="none" stroke="{col}" stroke-width="2"/>')
            lx=left+plot_w-220;ly=pt+20+idx*24;svg.append(f'<line x1="{lx}" y1="{ly-5}" x2="{lx+25}" y2="{ly-5}" stroke="{col}" stroke-width="3"/><text x="{lx+32}" y="{ly}" class="tick">{name}</text>')
        svg.append(f'<text x="{left+plot_w/2}" y="{pt+ph+48}" text-anchor="middle" class="axis">Time from sync (s)</text>')
    svg.append('</svg>'); png=Path('demo/overlay.png'); png.parent.mkdir(exist_ok=True)
    with tempfile.NamedTemporaryFile(mode='w',suffix='.svg',delete=False) as f:f.write(''.join(svg));src=f.name
    try:subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-y','-i',src,'-frames:v','1',str(png)],check=True)
    finally:Path(src).unlink(missing_ok=True)
if __name__=='__main__':main(*sys.argv[1:])
