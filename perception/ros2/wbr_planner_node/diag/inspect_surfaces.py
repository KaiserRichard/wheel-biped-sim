import json,sqlite3,sys
from pathlib import Path
from collections import Counter
import numpy as np
from replay_empty import ARCHIVE,OUT,TS,messages
sys.path.insert(0,str(OUT.parents[2]))
from ros2.wbr_planner_node.wbr_planner_node.core import PlannerCore
from tools.local_map.grid import ElevationGrid

def surface(run):
    d=np.load(OUT/f'{run}-first.npz'); p=d['points_map']; pix=d['pixels']; obs=d['obstacle']
    c=PlannerCore(); c.grid=ElevationGrid(6,.05); c.grid.update(p,time_s=0)
    o=c.grid.outputs(time_s=0,obstacle_threshold_m=.03)
    row,col=np.where(o['obstacle']); x=-3+(col+.5)*.05; y=3-(row+.5)*.05
    lanes=[]
    for target in np.round(np.arange(-.6,.6001,.05),2):
        r0=max(0,int(np.floor((3-(target+.25))/.05))); r1=min(c.grid.n,int(np.ceil((3-(target-.25))/.05)))
        c0=int(np.floor((.5+3)/.05)); c1=c.grid.n
        mask=(row>=r0)&(row<r1)&(col>=c0)&(col<c1)
        lanes.append(dict(target=float(target),cells=int(mask.sum()),x_min=float(x[mask].min()),x_max=float(x[mask].max())))
    counterfactual={}
    for limit in (1.,1.3,1.5,1.7,2.,3.):
        altered=dict(o); altered['obstacle']=o['obstacle'].copy(); altered['obstacle'][:,np.arange(c.grid.n)*.05-3>=limit]=False
        counterfactual[str(limit)]=c._choose(np.array([0.,0.,.29]),altered)
    nearest=p[obs][np.argmin(np.linalg.norm(p[obs,:2],axis=1))]
    summary=dict(lanes=lanes,remove_obstacles_beyond_x=counterfactual,nearest_point=nearest.tolist())
    # Color read directly from archive with immutable read-only SQLite.
    path=ARCHIVE/run/'rgbd-bag/rgbd-bag_0.db3'
    with sqlite3.connect(path.as_uri()+'?mode=ro&immutable=1',uri=True) as conn:
        topics=list(conn.execute('SELECT id,name,type FROM topics')); summary['topics']=topics
        for tid,name,typ in topics:
            if name.endswith('/color/image_raw'):
                data=conn.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp LIMIT 1',(tid,)).fetchone()[0]
                m=TS.deserialize_cdr(data,typ)
                from PIL import Image
                rgb=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
                if m.encoding=='bgr8': rgb=rgb[:,:,::-1]
                Image.fromarray(rgb).save(OUT/f'{run}-color.jpg',quality=85)
    return summary
r={run:surface(run) for run in ('S1-empty','B-H13')}
counts={}; examples={}
for topic,stamp,msg in messages('LIVE'):
    if 'advisory' not in topic: continue
    value=msg.data
    if isinstance(value,str):
        value=json.loads(value); key=value.get('advisory',value.get('action','unknown'))
    else: key=str(value.tolist() if hasattr(value,'tolist') else value)
    counts.setdefault(topic,Counter())[key]+=1
    examples.setdefault(topic,value)
r['LIVE']=dict(counts={k:dict(v) for k,v in counts.items()},examples=examples)
(OUT/'surfaces.json').write_text(json.dumps(r,indent=2))
print(json.dumps(r,indent=2))
