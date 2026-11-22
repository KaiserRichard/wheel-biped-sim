"""Deterministic pixel overlays; Pillow avoids host font-cache dependencies."""
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw
out=Path(__file__).resolve().parent
for run in ('S1-empty','B-H13'):
    d=np.load(out/f'{run}-first.npz'); dep=d['depth']; pix=d['pixels']; pts=d['points_map']; obs=d['obstacle']; near=d['nearest']
    gray=(255*(1-np.clip(dep/3,0,1))).astype(np.uint8); gray[dep==0]=0
    canvas=Image.new('RGB',(1280,530),'white'); canvas.paste(Image.fromarray(gray).convert('RGB'),(0,30))
    draw=ImageDraw.Draw(canvas)
    for row,col in pix[obs]: draw.point((int(col),int(row)+30),fill='red')
    for row,col in pix[near]: draw.ellipse((int(col)-4,int(row)+26,int(col)+4,int(row)+34),outline='cyan',width=2)
    draw.text((5,5),run+': depth 0-3m (white-black); red = sampled height > 3cm',fill='black')
    draw.text((650,5),'Map: x forward 0..3m; y left -1..1m; red obstacle',fill='black')
    def xy(p): return (int(680+p[0]*180),int(270-p[1]*220))
    for lateral in [-1,-.5,0,.5,1]:
        yy=xy((0,lateral))[1]; draw.line((680,yy,1220,yy),fill='lightgray'); draw.text((645,yy),str(lateral),fill='black')
    for forward in [0,1,2,3]:
        xx=xy((forward,0))[0]; draw.line((xx,50,xx,490),fill='lightgray'); draw.text((xx,495),str(forward)+'m',fill='black')
    for p,is_obs in zip(pts,obs):
        if 0<=p[0]<=3 and abs(p[1])<=1: draw.point(xy(p),fill='red' if is_obs else 'gray')
    for lateral in [-.25,.25]: draw.line((680,xy((0,lateral))[1],1220,xy((0,lateral))[1]),fill='green',width=1)
    draw.text((650,510),'Green = straight footprint +/-0.25m; cyan = nearest cell point',fill='black')
    for p in pts[near]:
        x,y=xy(p); draw.ellipse((x-4,y-4,x+4,y+4),outline='cyan',width=2)
    canvas.save(out/f'{run}-obstacles.png',optimize=True)
