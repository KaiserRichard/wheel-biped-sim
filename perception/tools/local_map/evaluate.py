"""Run deterministic synthetic local-map evaluation and write plots/JSON."""
from pathlib import Path
import json
import os
import sys
import time
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from tools.ground_plane.plane import backproject
from tools.local_map.fuse import fuse_sequence, transform_points
from tools.local_map.grid import StereoNoise
from tools.local_map.synth_sequence import make_sequence

os.environ.setdefault("MPLBACKEND", "Agg")
os.environ.setdefault("MPLCONFIGDIR", "/private/tmp/wbr-mpl-cache")
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def ground_truth_grid(seq, T_map_world, center_xy, size_m=4., res=.05):
    n = int(round(size_m/res)); center = np.asarray(center_xy)
    truth = np.full((n,n), np.nan)
    for depth, pose, labels, heights in zip(seq["frames"], seq["poses"], seq["labels"], seq["true_height"]):
        pts, ij = backproject(depth, *seq["intrinsics"], stride=4)
        rr, cc = ij[:,0], ij[:,1]
        keep = labels[rr,cc] > 0
        pts = pts[keep]; rr=rr[keep]; cc=cc[keep]
        pmap = transform_points(T_map_world, transform_points(pose, pts))
        rows=np.floor(((center[1]+size_m/2)-pmap[:,1])/res).astype(int)
        cols=np.floor((pmap[:,0]-(center[0]-size_m/2))/res).astype(int)
        ok=(rows>=0)&(rows<n)&(cols>=0)&(cols<n)
        for r,c,h in zip(rows[ok],cols[ok],pmap[ok,2]):
            truth[r,c] = h if not np.isfinite(truth[r,c]) else max(truth[r,c], h)
    return truth


def metrics(grid, truth):
    out = grid.outputs(time_s=(len(seq_time) - 1) / 10, unknown_after_s=1000)
    valid = out["known"] & np.isfinite(truth)
    err = out["height"] - truth
    rmse = float(np.sqrt(np.mean(err[valid]**2))) if valid.any() else float("nan")
    gt = np.isfinite(truth) & (truth > .05)
    pred = out["obstacle"]
    tp=int(np.sum(gt & pred)); fp=int(np.sum(~gt & pred & out["known"])); fn=int(np.sum(gt & ~pred & out["known"]))
    prec=tp/(tp+fp) if tp+fp else 0.; rec=tp/(tp+fn) if tp+fn else 0.
    return {"height_rmse_m":rmse,"observed_cells":int(valid.sum()),"obstacle_precision":prec,
            "obstacle_recall":rec,"tp":tp,"fp":fp,"fn":fn}, out, err, valid


def main():
    global seq_time
    seq=make_sequence(); seq_time=seq["frames"]
    opts={"intrinsics":seq["intrinsics"],"stride":4,"size_m":4.,"resolution_m":.05,
          "noise_model":StereoNoise(sigma_d_px=0.0)}
    start=time.perf_counter(); base,T,origin,plane,updates=fuse_sequence(seq["frames"],seq["poses"],**opts)
    elapsed=time.perf_counter()-start
    truth=ground_truth_grid(seq,T,base.center_xy)
    baseline, out, err, valid=metrics(base,truth)
    cases={"gravity_aligned":baseline}
    for name,bias in (("pose_bias_1cm",{"translation":(0.,.01,0.)}),
                      ("pose_bias_3cm",{"translation":(0.,.03,0.)}),
                      ("pose_bias_1deg",{"rotation_rad":(0.,np.deg2rad(1.),0.)})):
        g, tmap, *_=fuse_sequence(seq["frames"],seq["poses"],pose_bias=bias,**opts)
        # Compare on its own frame using the unbiased truth frame to expose bias.
        cases[name],_,_,_=metrics(g,truth)
    raw,_,_,_,_=fuse_sequence(seq["frames"],seq["poses"],gravity_align=False,**opts)
    cases["raw_pose_no_gravity_alignment"],_,_,_=metrics(raw,truth)
    results={"sequence":{"frames":len(seq["frames"]),"travel_m":1.5,"height_m":.30,
                         "pitch_peak_deg":3.,"stride":4,"points_per_frame_mean":float(np.mean(updates))},
             "floor_plane":{"valid":bool(plane.valid),"inliers":int(plane.inlier_count),
                            "camera_height_m":float(plane.camera_height),"pitch_deg":float(np.rad2deg(plane.pitch))},
             "timing":{"total_ms":elapsed*1000,"ms_per_frame":elapsed*1000/len(seq["frames"])},
             "cases":cases}
    results_path=HERE/"results"/"metrics.json"
    results_path.write_text(json.dumps(results,indent=2,allow_nan=False)+"\n")
    extent=[-2,2,-2,2]
    fig,ax=plt.subplots(figsize=(7,5)); im=ax.imshow(out["height"],origin="upper",extent=extent,cmap="terrain",vmin=0,vmax=.14); fig.colorbar(im,ax=ax,label="height above floor (m)"); ax.set_title("Fused elevation grid"); ax.set_xlabel("right (m)"); ax.set_ylabel("forward (m)"); fig.tight_layout(); fig.savefig(HERE/"results"/"height_map.png",dpi=150); plt.close(fig)
    fig,ax=plt.subplots(figsize=(7,5)); data=np.where(valid,err,np.nan); im=ax.imshow(data,origin="upper",extent=extent,cmap="coolwarm",vmin=-.06,vmax=.06); fig.colorbar(im,ax=ax,label="map − truth (m)"); ax.set_title("Height error on observed cells"); ax.set_xlabel("right (m)"); ax.set_ylabel("forward (m)"); fig.tight_layout(); fig.savefig(HERE/"results"/"error_map.png",dpi=150); plt.close(fig)
    fig,ax=plt.subplots(figsize=(7,5)); classes=out["classes"]; cmap=matplotlib.colors.ListedColormap(["#222222","#c7d7bd","#d84a3a"]); im=ax.imshow(classes,origin="upper",extent=extent,cmap=cmap,vmin=0,vmax=2); ax.set_title("Traversability: unknown / free / obstacle"); ax.set_xlabel("right (m)"); ax.set_ylabel("forward (m)"); fig.tight_layout(); fig.savefig(HERE/"results"/"obstacle_classes.png",dpi=150); plt.close(fig)
    print(json.dumps(results,indent=2))


if __name__=="__main__": main()
