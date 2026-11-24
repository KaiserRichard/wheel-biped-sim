"""Synthetic parameter sweep; writes metrics and two diagnostic plots."""
import json, pathlib, sys
import numpy as np
import matplotlib.pyplot as plt
ROOT=pathlib.Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path: sys.path.insert(0,str(ROOT))
from tools.ground_plane.synth import render,Box
from tools.obstacle_decision.detect import process_frame,Params
from tools.obstacle_decision.decide import update,DecisionParams,Advisory

def main():
    out=pathlib.Path(__file__).parent/'results'; out.mkdir(exist_ok=True)
    K=np.array([[386.953,0,321.034],[0,386.953,241.040],[0,0,1.]])
    rows=[]; confusion={a.name:{b.name:0 for b in Advisory} for a in Advisory}
    for h in (.03,.05,.08,.12,.20):
      for d in (.5,1.,1.5,2.):
       for pitch in (0.,3.):
        for seed in range(2):
         box=Box((0,d+.06,h/2),(.30,.12,h))
         depth,labels,_=render(height=.30,pitch=pitch,boxes=(box,),sigma_d=.1,seed=seed)
         state={}; decstate={}; frames=[]
         for i in range(5):
          fr,state=process_frame(depth,K,Params(),state); frames.append(fr)
          decision,decstate=update(fr,decstate,DecisionParams())
         obs=frames[-1].nearest
         gt_adv=Advisory.STOP_CANDIDATE if h>.12 else Advisory.JUMP_CANDIDATE
         confusion[gt_adv.name][decision.advisory.name]+=1
         rows.append(dict(true_height=h,true_distance=d,true_width=.30,pitch_deg=pitch,seed=seed,
           height_error_m=None if obs is None else obs.height_m-h,
           distance_error_m=None if obs is None else obs.distance_m-d,
           width_error_m=None if obs is None else obs.width_m-.30,advisory=decision.advisory.name,
           ms_per_frame=float(np.mean([f.timing_ms for f in frames]))))
    # Negative controls
    controls=[]
    for label,boxes in [('empty',()),('outside',(Box((.7,1.06,.025),(.30,.12,.05)),))]:
      depth,_,_=render(height=.30,boxes=boxes,sigma_d=.1)
      st={}; ds={}
      for _ in range(5):
       fr,st=process_frame(depth,K,Params(),st); dec,ds=update(fr,ds,DecisionParams())
      controls.append({'case':label,'advisory':dec.advisory.name,'detected':fr.nearest is not None})
    def stats(k):
      vals=[r[k] for r in rows if r[k] is not None]
      return {'count':len(vals),'mean_abs':float(np.mean(np.abs(vals))),'max_abs':float(np.max(np.abs(vals)))}
    metrics={'cases':len(rows),'height_error':stats('height_error_m'),'distance_error':stats('distance_error_m'),'width_error':stats('width_error_m'),
      'confusion_matrix':confusion,'mean_ms_per_frame':float(np.mean([r['ms_per_frame'] for r in rows])),'controls':controls,'details':rows}
    (out/'metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
    fig,ax=plt.subplots();
    for h in (.03,.05,.08,.12,.20):
      rs=[r for r in rows if r['true_height']==h and r['height_error_m'] is not None]
      ax.scatter([r['true_distance'] for r in rs],[r['height_error_m'] for r in rs],label=f'{h:.2f} m')
    ax.set(xlabel='Front-face distance (m)',ylabel='Height error (m)',title='Synthetic height error'); ax.legend(); fig.tight_layout(); fig.savefig(out/'height_error.png',dpi=150); plt.close(fig)
    fig,ax=plt.subplots();
    names=list(Advisory); mat=np.array([[confusion[a.name][b.name] for b in names] for a in names]); im=ax.imshow(mat,cmap='Blues');
    ax.set_xticks(range(4),[x.name for x in names],rotation=35,ha='right'); ax.set_yticks(range(4),[x.name for x in names]); ax.set(xlabel='Predicted',ylabel='Truth',title='Advisory confusion (synthetic)'); fig.colorbar(im,ax=ax); fig.tight_layout(); fig.savefig(out/'confusion.png',dpi=150); plt.close(fig)
    print(json.dumps({k:v for k,v in metrics.items() if k!='details'},indent=2))
if __name__=='__main__': main()
