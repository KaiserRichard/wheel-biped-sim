"""Synthetic camera tilt sweep for the existing obstacle detector."""
import csv
import json
import pathlib
import sys
import math
import numpy as np
import matplotlib.pyplot as plt

ROOT = pathlib.Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from tools.ground_plane.synth import render, Box
from tools.ground_plane.obstacles import height_image, fixed_mount, angular_height_error
from tools.obstacle_decision.detect import process_frame, Params, FrameResult, Cluster
from tools.obstacle_decision.decide import update, DecisionParams, Advisory

K = np.array([[386.953, 0, 321.034], [0, 386.953, 241.040], [0, 0, 1.]])
PITCHES = (-15, -10, -5, -2, 0, 2, 5, 10, 15)
ROLLS = (0, 5, 10)
DISTANCES = (.5, 1., 1.3)
HEIGHTS = (.046, .132)
SEEDS = (0, 1, 2)


def advisory_for(frame):
    state = {}; ds = {}
    for _ in range(5):
        frame, state = (frame, state)
        decision, ds = update(frame, ds, DecisionParams())
    return decision


def fixed_frame(depth, labels, true_h, distance, box_h):
    """Fixed pose height scoring on known box returns, same advisory policy."""
    mask = (labels == 2) & np.isfinite(true_h)
    if not mask.any():
        return FrameResult(False, False, 0, .4, 0, 0, None)
    hm = height_image(depth, np.array([0., -1., 0.]), .4)
    # Select top-face returns by world height to estimate the visible top band.
    top = mask & (true_h >= box_h - .004)
    vals = hm[top] if top.any() else hm[mask]
    est_h = float(np.median(vals))
    # The known-render label is used only to isolate obstacle pixels; axial depth
    # converted to forward is close to truth and avoids attributing clustering error.
    z = depth[mask]
    est_d = max(.3, float(np.percentile(z, 5)) - .06)
    ob = Cluster(est_d, est_h, .30, .12, int(mask.sum()))
    return FrameResult(True, False, 0, .4, 0, 0, ob)


def run_case(pitch, roll, d, h, seed):
    box = Box((0., d + .06, h/2), (.30, .12, h))
    depth, labels, true_h = render(height=.40, pitch=pitch, roll=roll,
        boxes=(box,), sigma_d=.1, seed=seed)
    state = {}; frames=[]
    for _ in range(5):
        fr, state = process_frame(depth, K, Params(), state)
        frames.append(fr)
    obs = frames[-1].nearest
    plane_dec = advisory_for(frames[-1])
    fixed = fixed_frame(depth, labels, true_h, d, h)
    fixed_dec = advisory_for(fixed)
    expected = Advisory.JUMP_CANDIDATE if h <= .12 else Advisory.STOP_CANDIDATE
    rows=[]
    for method, ob, dec, valid in (
        ('plane_fit', obs, plane_dec, frames[-1].plane_valid),
        ('fixed_mount', fixed.nearest, fixed_dec, True)):
        rows.append(dict(pitch_deg=pitch, roll_deg=roll, true_distance_m=d,
            true_height_m=h, seed=seed, method=method,
            estimated_height_m=None if ob is None else ob.height_m,
            height_error_m=None if ob is None else ob.height_m-h,
            estimated_distance_m=None if ob is None else ob.distance_m,
            distance_error_m=None if ob is None else ob.distance_m-d,
            advisory=dec.advisory.name, expected_advisory=expected.name,
            advisory_correct=dec.advisory == expected, plane_valid=bool(valid),
            plane_pitch_deg=float(np.rad2deg(frames[-1].plane_pitch_rad)) if frames[-1].plane_valid else None,
            plane_roll_deg=float(np.rad2deg(frames[-1].plane_roll_rad)) if frames[-1].plane_valid else None,
            obstacle_pixels=int(np.count_nonzero(labels == 2))))
    return rows


def main():
    out=pathlib.Path(__file__).parent/'results'; out.mkdir(exist_ok=True)
    rows=[]
    for p in PITCHES:
      for r in ROLLS:
       for d in DISTANCES:
        for h in HEIGHTS:
         for seed in SEEDS:
          rows.extend(run_case(p,r,d,h,seed))
    with (out/'metrics.csv').open('w', newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
    summary={}
    for method in ('plane_fit','fixed_mount'):
      rr=[x for x in rows if x['method']==method]
      for key in ('height_error_m','distance_error_m'):
        vals=[x[key] for x in rr if x[key] is not None]
        summary.setdefault(method,{})[key]={'n':len(vals),'mean_abs':float(np.mean(np.abs(vals))) if vals else None,'max_abs':float(np.max(np.abs(vals))) if vals else None}
      summary[method]['plane_invalid_n']=sum(not x['plane_valid'] for x in rr)
      summary[method]['advisory_correct_n']=sum(x['advisory_correct'] for x in rr)
      summary[method]['n']=len(rr)
    summary['analytic_d_sin_5deg_m']={str(d):float(d*np.sin(np.deg2rad(5))) for d in DISTANCES}
    summary['cases_per_method']=len(rows)//2
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    fig,ax=plt.subplots(figsize=(8,5))
    for method in ('plane_fit','fixed_mount'):
      for h in HEIGHTS:
       means=[]
       for p in PITCHES:
        v=[x['height_error_m'] for x in rows if x['method']==method and x['true_height_m']==h and x['pitch_deg']==p and x['height_error_m'] is not None]
        means.append(np.mean(v) if v else np.nan)
       ax.plot(PITCHES,means,marker='o',label=f'{method}, h={h:.3f}m')
    ax.axhline(0,color='black',lw=.6); ax.set(xlabel='Camera pitch (deg)',ylabel='Mean height error (m)',title='Obstacle height error vs pitch'); ax.legend(fontsize=8); fig.tight_layout(); fig.savefig(out/'height_error_vs_pitch.png',dpi=160); plt.close(fig)
    fig,axs=plt.subplots(1,2,figsize=(10,4),sharey=True)
    for ax,method in zip(axs,('plane_fit','fixed_mount')):
      mat=np.full((len(PITCHES),len(DISTANCES)),np.nan)
      for i,p in enumerate(PITCHES):
       for j,d in enumerate(DISTANCES):
        v=[x['advisory_correct'] for x in rows if x['method']==method and x['pitch_deg']==p and x['true_distance_m']==d]
        mat[i,j]=np.mean(v)
      im=ax.imshow(mat,vmin=0,vmax=1,aspect='auto',cmap='RdYlGn',origin='lower'); ax.set(title=method,xlabel='Box distance (m)',xticks=range(3),xticklabels=DISTANCES,yticks=range(len(PITCHES)),yticklabels=PITCHES)
      for i in range(len(PITCHES)):
       for j in range(len(DISTANCES)): ax.text(j,i,f'{mat[i,j]:.0%}',ha='center',va='center',fontsize=7)
    axs[0].set_ylabel('Camera pitch (deg)'); fig.colorbar(im,ax=axs.ravel().tolist(),label='Advisory correctness'); fig.suptitle('Advisory correctness by pitch and distance (pooled roll, height, seed)'); fig.savefig(out/'advisory_correctness.png',dpi=160,bbox_inches='tight'); plt.close(fig)
    # floor support proxy: observed rendered floor pixels, per pitch and distance
    fig,ax=plt.subplots(figsize=(8,4))
    for r in ROLLS:
      vals=[]
      for p in PITCHES:
       subset=[x['obstacle_pixels'] for x in rows if x['method']=='plane_fit' and x['roll_deg']==r and x['pitch_deg']==p]
       vals.append(np.mean(subset))
      ax.plot(PITCHES,vals,marker='o',label=f'roll={r}°')
    ax.set(xlabel='Pitch (deg)',ylabel='Obstacle pixels (mean)',title='Rendered obstacle support'); ax.legend(); fig.tight_layout(); fig.savefig(out/'obstacle_support.png',dpi=150); plt.close(fig)
    print(json.dumps(summary,indent=2))
if __name__=='__main__': main()
