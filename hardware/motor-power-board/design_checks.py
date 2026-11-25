"""Reduced hardware design checks based on the S13 CSV sweeps."""
from pathlib import Path
import csv
import math
import argparse
from PIL import Image, ImageDraw, ImageFont

HERE=Path(__file__).resolve().parent
S13=None
CAPS=(1.0,4.7,10.0,22.0)
PACK=25.2
RES_NOM=3.0
RES_MAX=3.15
R_BLEED=330.0
WINDOW=2**18*113e3/50e3*1e-6
OFFSET=2.5*(100e3/294e3)
TRACE_DT=.0001

def load_rows(name):
    if S13 is None:
        raise ValueError('Set --s13-dir to the external source CSV directory')
    with (S13/name).open(newline='',encoding='utf-8-sig') as fh:
        return list(enumerate(csv.DictReader(fh),2))

def max_row(rows,field,subset=lambda r:True):
    return max((float(r[field]),line,r.get('case'),r.get('phase'),r.get('scenario',r.get('protection')))
               for line,r in rows if subset(r) and r[field])

def crossing(c,r,i,delta):
    equivalent=1/(1/r+1/R_BLEED)
    final=(PACK/r-i)*equivalent
    target=PACK-delta
    return -equivalent*c*math.log1p(-target/final) if final>target else math.inf

def charge(c,r,i,t):
    equivalent=1/(1/r+1/R_BLEED)
    final=(PACK/r-i)*equivalent
    return final*-math.expm1(-t/(equivalent*c))

def write_csv(path,header,rows):
    with (HERE/path).open('w',encoding='utf-8',newline='') as fh:
        w=csv.writer(fh);w.writerow(header);w.writerows(rows)

def run():
    connected=load_rows('connected_sweep.csv')
    disconnected=load_rows('disconnected_sweep.csv')
    extremes=[]
    for field in ['peak_motor_draw_A','peak_motor_return_A','motor_dc_RMS_A','peak_pack_discharge_A']:
        for scenario in ['base','winding']:
            extremes.append(('connected_sweep.csv',field,scenario,*max_row(connected,field,lambda r:r['scenario']==scenario)))
    for field in ['max_link_V','dump_J','peak_source_W','peak_dump_W']:
        extremes.append(('disconnected_sweep.csv',field,'chopper_corner',*max_row(disconnected,field,lambda r:r['protection']=='chopper_corner')))
    write_csv('S13_rating_rows.csv',['file','metric','filter','value','source_csv_line','case','phase','scenario'],extremes)
    rows=[]
    for cm in CAPS:
        cap=cm*1e-3
        for r in [2.85,3.0,3.15,4.7,4.935]:
            for idle in [0,.075,.10,.125,.15,.20]:
                tc=crossing(cap,r,idle,OFFSET)
                # Two specified typical 33-ms pickup delays advance before charge; a third before K1 proof.
                proof_time=tc+.099
                rows.append([cm,r,idle,OFFSET,WINDOW,tc,proof_time,proof_time<WINDOW,
                             PACK-charge(cap,r,idle,WINDOW-.066)])
    write_csv('precharge_sweep.csv',['C_mF','R_ohm','drive_standby_A','delta_nominal_V','window_s',
         'charge_to_delta_s','typical_total_to_K1_proof_s','within_nominal_window','delta_at_window_V'],rows)
    vneg=2.5*10.7/11.7
    gain=1+100/9.49+100/2050
    clamp_on=vneg*gain-.15*100/2050
    clamp_off=vneg*gain-10.23*100/2050
    ov=vneg*(1+100/8.91)
    for row in extremes: print(row)
    print('window, delta, clamp-on, clamp-off, OV:',WINDOW,OFFSET,clamp_on,clamp_off,ov)
    for idle in [0,.075,.1,.125,.15,.20]: print(idle,crossing(.022,RES_MAX,idle,OFFSET))
    plot()

def plot():
    im=Image.new('RGB',(1300,800),'white')
    draw=ImageDraw.Draw(im)
    normal=ImageFont.truetype('arial.ttf',24)
    bold=ImageFont.truetype('arialbd.ttf',34)
    small=ImageFont.truetype('arial.ttf',21)
    color=['#0179a4','#12654a','#795029','#93347a']
    draw.text((82,36),'Precharge sensitivity at 25.2 V and high resistor tolerance',font=bold,fill='#172625')
    draw.text((85,82),'3.15 Ω, 330 Ω bleeder, constant drive standby load',font=normal,fill='#344843')
    x0,y0,x1,y1=130,626,1200,140
    def point(t,u): return (x0+t/.70*(x1-x0),y0-u/26*(y0-y1))
    for i in range(8):
        x,_=point(i*.1,0)
        draw.line((x,y1,x,y0),fill='#dbe2df',width=2)
        draw.text((x,y0+15),str(i*100),font=normal,fill='#253932',anchor='mt')
    for v in [0,5,10,15,20,25]:
        _,y=point(0,v)
        draw.line((x0,y,x1,y),fill='#dbe2df',width=2)
        draw.text((x0-12,y),str(v),font=normal,fill='#253932',anchor='rm')
    draw.line((x0,y1,x0,y0,x1,y0),fill='#22302b',width=3)
    draw.text(((x0+x1)//2,684),'Time since K2 command (ms)',font=normal,fill='#253932',anchor='mt')
    draw.text((84,115),'Link voltage (V)',font=small,fill='#253932')
    threshold=PACK-OFFSET
    draw.line((*point(0,threshold),*point(.70,threshold)),fill='#546560',width=2)
    draw.text((840,111),'Nominal ΔV gate is 24.35 V',font=small,fill='#35463f')
    draw.line((*point(WINDOW,0),*point(WINDOW,26)),fill='#4e5450',width=3)
    draw.text(point(WINDOW+.008,19),'Timer end',font=small,fill='#35463f')
    for k,(cap,idle,label) in enumerate([(1.,0,'1 mF, 0 A'),(10.,0,'10 mF, 0 A'),(22.,.075,'22 mF, 0.075 A'),(22.,.15,'22 mF, 0.15 A')]):
        points=[point(i/1000,charge(cap/1000,RES_MAX,idle,max(0,i/1000-.066))) for i in range(round(WINDOW*1000)+1)]
        draw.line(points,fill=color[k],width=4)
        lx=126+k*270
        draw.line((lx,746,lx+35,746),fill=color[k],width=4)
        draw.text((lx+44,746),label,font=small,fill='#253932',anchor='lm')
    im.save(HERE/'precharge_sensitivity.png')

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--s13-dir', type=Path, required=True)
    S13=parser.parse_args().s13_dir
    run()


