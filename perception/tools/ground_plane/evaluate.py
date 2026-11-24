#!/usr/bin/env python3
"""Synthetic sweep comparing per-frame plane fitting to a fixed-mount model."""
import argparse
import json
import time
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

try:
    from .synth import render, Box
    from .plane import estimate_plane
    from .obstacles import label_obstacles, angular_height_error
except ImportError:  # direct script invocation
    from synth import render, Box
    from plane import estimate_plane
    from obstacles import label_obstacles, angular_height_error

ROOT = Path(__file__).resolve().parent


def _pr(tp, fp, fn):
    return (tp/(tp+fp) if tp+fp else None, tp/(tp+fn) if tp+fn else None)


def _scores(pred, gt, depth, true_h, threshold=0.05):
    """Pixel scores. Positive: box pixel whose true height >= threshold.
    Negative: floor pixel. Box pixels below threshold (box bottoms, short boxes)
    are ignored because no threshold rule can be asked to flag them.
    Distance bins use axial depth Z (camera frame), not ground distance."""
    valid = (gt != 0) & (depth > 0)
    pos = valid & (gt == 2) & (np.nan_to_num(true_h, nan=-1.0) >= threshold)
    neg = valid & (gt == 1)
    def counts(m):
        return (int(np.sum(m & pred & pos)), int(np.sum(m & pred & neg)), int(np.sum(m & ~pred & pos)))
    tp, fp, fn = counts(valid)
    precision, recall = _pr(tp, fp, fn)
    any_box = valid & (gt == 2)
    recall_any_box = (int(np.sum(pred & any_box)) / int(np.sum(any_box))) if np.any(any_box) else None
    bins=[]
    for lo,hi in ((0,1),(1,2),(2,3)):
        m = valid & (depth>=lo) & (depth<hi)
        bt, bf, bn = counts(m); bp, br = _pr(bt, bf, bn)
        bins.append({"range_m":[lo,hi],"precision":bp,"recall":br,"tp":bt,"fp":bf,"fn":bn,
                     "gt_obstacle_pixels":bt+bn})
    return {"precision":precision,"recall":recall,"tp":tp,"fp":fp,"fn":fn,
            "recall_any_box_pixel":recall_any_box,"positive_definition":f"box pixel with true height >= {threshold} m",
            "distance_bins":bins}


def run(quick=False):
    pitches=[0,3,6] if quick else [0,1.5,3,4.5,6]
    sigmas=[0.05,0.1,0.2]
    seeds=[7] if quick else [7,19,41]
    rows=[]; rng=np.random.default_rng(2026)
    for pitch in pitches:
        for sigma in sigmas:
            for seed in seeds:
                # Boxes span the requested ranges, with fixed dimensions generated reproducibly.
                boxes=[]
                for _ in range(3 if quick else 4):
                    dist=float(rng.uniform(.6,3.0)); width=float(rng.uniform(.10,.28)); depth=float(rng.uniform(.10,.25)); h=float(rng.uniform(.03,.15))
                    lateral=float(rng.uniform(-.45,.45))
                    boxes.append(Box((lateral,dist, h/2),(width,depth,h)))
                d,gt,gt_h=render(height=.40,pitch=pitch,boxes=boxes,sigma_d=sigma,
                                 invalid_fraction=.01,seed=seed)
                p=np.deg2rad(pitch)
                true_n=np.array([0.,-np.cos(p),-np.sin(p)])
                # Metrics at stride 4; timings measure full-resolution plane fitting.
                est=estimate_plane(d,stride=4,iterations=100 if quick else 160,
                                   threshold=.012,seed=seed,min_inliers=50)
                for method in ("per_frame","fixed_mount"):
                    if method=="per_frame":
                        if est.valid:
                            pred=label_obstacles(d,est.normal,est.camera_height)
                            angle=float(np.rad2deg(np.arccos(np.clip(est.normal@true_n,-1,1))))
                            herr=abs(est.camera_height-.40)
                        else:
                            pred=np.zeros_like(gt,dtype=bool); angle=None; herr=None
                    else:
                        pred=label_obstacles(d,np.array([0.,-1.,0.]),.40)
                        angle=float(pitch)
                        herr=float(np.mean([angular_height_error(b.position[1],p) for b in boxes]))
                    rows.append({"pitch_deg":pitch,"sigma_d_px":sigma,"seed":seed,"method":method,
                        "plane_valid":bool(est.valid) if method=="per_frame" else True,
                        "normal_angle_error_deg":angle,"height_error_m":herr,
                        "obstacles":_scores(pred,gt,d,gt_h)})
    # Runtime is separately measured with and without pixel subsampling, using a representative frame.
    bench_depth,_,_=render(height=.40,pitch=4.5,boxes=[Box((0,1.4,.06),(.25,.2,.12))],
                           sigma_d=.1,seed=99)
    timing={}
    for stride in (1,4):
        reps=1 if quick else 3
        ts=[]
        for j in range(reps):
            t0=time.perf_counter(); estimate_plane(bench_depth,stride=stride,iterations=80,
                    threshold=.012,seed=j,min_inliers=100); ts.append((time.perf_counter()-t0)*1000)
        timing[str(stride)]={"mean_ms_per_frame":float(np.mean(ts)),"samples_ms":ts,"repetitions":reps}
    return {"metadata":{"claim_labels":{"intrinsics":"MEASURED factory values supplied by task",
             "stereo_baseline_m":"SOURCE-SUPPORTED datasheet parameter supplied by task",
             "nominal_camera_height_m":"ASSUMED 0.40 m","scenes":"SYNTHETIC / CALCULATED ray casting",
             "noise_model":"CALCULATED from supplied stereo disparity equation"},
             "quick":quick,"pitches_deg":pitches,"sigma_d_px":sigmas,"seeds":seeds,
             "intrinsics":{"width":640,"height":480,"fx":386.953,"fy":386.953,"cx":321.034,"cy":241.040},
             "baseline_m":.05,"max_range_m":6,"obstacle_threshold_m":.05},
            "timing":timing,"runs":rows}


def make_plots(data, outdir):
    runs=data["runs"]
    methods=("per_frame","fixed_mount")
    # Aggregate across noise and seeds.
    pitches=sorted(set(r["pitch_deg"] for r in runs))
    def mean(method,key,pitch):
        a=[r[key] for r in runs if r["method"]==method and r["pitch_deg"]==pitch and r[key] is not None]
        return float(np.mean(a)) if a else np.nan
    for key,title,ylabel,name in (("normal_angle_error_deg","Plane normal error","Angle error (deg)","normal_error.png"),
                                  ("height_error_m","Camera height error","Absolute error (m)","height_error.png")):
        fig,ax=plt.subplots(figsize=(6,4))
        for m in methods: ax.plot(pitches,[mean(m,key,p) for p in pitches],"o-",label=m.replace("_"," "))
        ax.set(title=title,xlabel="True camera pitch (deg)",ylabel=ylabel);ax.grid(True,alpha=.3);ax.legend();fig.tight_layout();fig.savefig(outdir/name,dpi=150);plt.close(fig)
    fig,ax=plt.subplots(figsize=(7,4))
    for method in methods:
        vals=[]
        for lo in (0,1,2):
            a=[b["recall"] for r in runs if r["method"]==method for b in r["obstacles"]["distance_bins"] if b["range_m"][0]==lo and b["recall"] is not None]
            vals.append(np.mean(a) if a else np.nan)
        ax.plot(["0–1 m","1–2 m","2–3 m"],vals,"o-",label=method.replace("_"," "))
    ax.set(title="Obstacle recall by range",ylabel="Recall",ylim=(0,1.05));ax.grid(True,alpha=.3);ax.legend();fig.tight_layout();fig.savefig(outdir/"recall_by_range.png",dpi=150);plt.close(fig)
    fig,ax=plt.subplots(figsize=(6,4)); strides=list(data["timing"])
    ax.bar(["stride 1","stride 4"],[data["timing"][s]["mean_ms_per_frame"] for s in strides])
    ax.set(title="Plane estimation runtime",ylabel="ms/frame (host, synthetic input)");ax.grid(axis="y",alpha=.3);fig.tight_layout();fig.savefig(outdir/"timing.png",dpi=150);plt.close(fig)


def main():
    parser=argparse.ArgumentParser();parser.add_argument("--quick",action="store_true");parser.add_argument("--output",type=Path,default=ROOT/"results")
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    data=run(args.quick);(args.output/"metrics.json").write_text(json.dumps(data,indent=2,allow_nan=False)+"\n")
    make_plots(data,args.output)
    print(json.dumps({"quick":args.quick,"runs":len(data["runs"]),"timing":data["timing"],"metrics":str(args.output/"metrics.json")},indent=2))

if __name__=="__main__": main()
