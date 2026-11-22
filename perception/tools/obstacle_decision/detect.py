"""Offline depth obstacle extraction. No transport or actuation authority."""
from dataclasses import dataclass, field
import time
import numpy as np
from tools.ground_plane import plane as gp
from tools.ground_plane import obstacles as go


@dataclass
class Params:
    min_obstacle_height: float = 0.02  # ASSUMED
    corridor_half_width: float = 0.25  # ASSUMED robot half-width plus margin
    min_forward: float = 0.30
    max_forward: float = 3.0
    stride: int = 2
    cell_size: float = 0.02
    min_cells: int = 3
    plane_max_age: int = 3
    plane_gate_deg: float = 30.0
    edge_rel_jump: float = 0.03   # drop pixels next to a depth step > 3% of range (flying pixels)
    top_normal_deg: float = 25.0  # local normal within this angle of the floor normal = top surface
    top_min_points: int = 15
    top_band_m: float = 0.015


@dataclass
class Cluster:
    distance_m: float
    height_m: float
    width_m: float
    depth_m: float
    point_count: int


@dataclass
class FrameResult:
    plane_valid: bool
    plane_reused: bool
    plane_age: int
    plane_height_m: float
    plane_pitch_rad: float
    plane_roll_rad: float
    nearest: object
    clusters: list = field(default_factory=list)
    timing_ms: float = 0.0


def _components(x, y, cell):
    """Group points into connected 2D cells (cells up to two apart connect).

    Labels a small occupancy grid instead of the points: a few hundred cells
    per frame, so the Python flood fill is cheap. Linking cells two apart
    bridges one-cell gaps left by the edge filter and sparse far returns,
    which otherwise split a box into fragments and let a 5-point fragment
    become the "nearest".
    """
    ix=np.floor(x/cell).astype(int); iy=np.floor(y/cell).astype(int)
    ix-=ix.min(); iy-=iy.min()
    W=iy.max()+1; flat=ix*W+iy
    occupied=np.unique(flat); label={int(c):-1 for c in occupied}; nlab=0
    for c in label:
        if label[c]>=0: continue
        label[c]=nlab; stack=[c]
        while stack:
            q=stack.pop(); a,b=divmod(q,W)
            for i in range(-2,3):
                for j in range(-2,3):
                    if 0<=b+j<W:
                        r=(a+i)*W+b+j
                        if label.get(r,0)==-1: label[r]=nlab; stack.append(r)
        nlab+=1
    lut=np.array([label[int(c)] for c in occupied])
    labels=lut[np.searchsorted(occupied,flat)]
    order=np.argsort(labels,kind="stable"); bounds=np.searchsorted(labels[order],np.arange(nlab+1))
    return [order[bounds[k]:bounds[k+1]] for k in range(nlab)]


def ground_coords(depth_m, K, plane, stride=1):
    """Subsampled depth, camera points and ground-frame (height, forward, lateral).

    Ground frame from the fitted plane: up = n, forward = optical Z projected on
    the plane, lateral = n x forward (left). Using optical Z as "forward" made
    the front-face distance read long by (H-h)*sin(pitch), ~+3 cm at 10 deg.
    """
    fx,fy,cx,cy=K[0,0],K[1,1],K[0,2],K[1,2]; s=max(1,int(stride))
    dep=np.asarray(depth_m,dtype=float)[::s,::s]
    rows,cols=np.mgrid[0:depth_m.shape[0]:s,0:depth_m.shape[1]:s]
    dep=np.where(np.isfinite(dep)&(dep>0),dep,np.nan)
    P=np.stack(((cols-cx)*dep/fx,(rows-cy)*dep/fy,dep),-1)
    n=np.asarray(plane.normal,dtype=float); fw=np.array([0.,0.,1.])-n[2]*n; fw/=np.linalg.norm(fw); lt=np.cross(n,fw)
    return dep,P,P@n+plane.camera_height,P@fw,P@lt


def process_frame(depth_m, K, params=None, state=None):
    """Return (FrameResult,state); K is 3x3 pinhole intrinsics.

    Ground frame coordinates are forward=optical Z, lateral=-optical X (left),
    up=n·p+d. A +camera-right optical X therefore maps to negative lateral.
    """
    t0=time.perf_counter(); p=params or Params(); state={} if state is None else dict(state)
    fx,fy,cx,cy=K[0,0],K[1,1],K[0,2],K[1,2]
    prev=state.get('plane'); age=state.get('plane_age',0)
    prior=(prev.normal,prev.camera_height) if prev is not None else (np.array([0.,-1.,0.]),0.30)
    est=gp.estimate_plane(depth_m,fx=fx,fy=fy,cx=cx,cy=cy,stride=max(1,p.stride),
        iterations=160,threshold=.012,prior=prior,max_deviation=(np.deg2rad(p.plane_gate_deg),.12),
        min_inliers=40,min_height=.05)
    reused=False
    if est.valid:
        plane=est; age=0; state.update(plane=plane,plane_age=0)
    elif prev is not None and age < p.plane_max_age:
        plane=prev; age+=1; reused=True; state.update(plane_age=age)
    else:
        plane=None; state.pop('plane',None); state['plane_age']=age+1
    if plane is None:
        result=FrameResult(False,False,age,float('nan'),float('nan'),float('nan'),None,[],(time.perf_counter()-t0)*1000)
        return result,state
    s=max(1,p.stride)
    dep,P,h,forward,lateral=ground_coords(depth_m,K,plane,s); n=np.asarray(plane.normal,dtype=float)
    # Edge filter: a pixel more than edge_rel_jump*range BEHIND its left or right
    # neighbour sits just outside a side edge, where stereo produces mixed-depth
    # flying pixels that widen clusters. The nearer (true) edge pixel is kept.
    # Only the row direction is checked: down a column, a far floor or box top
    # seen at a grazing angle has large depth steps without being an edge.
    pad=np.pad(dep,((0,0),(1,1)),constant_values=np.nan)
    jump=dep-np.fmin(pad[:,:-2],pad[:,2:])
    edge=jump>p.edge_rel_jump*dep
    # Local surface normal from central differences; near-vertical normal = top surface.
    du=np.zeros_like(P); dv=np.zeros_like(P)
    du[:,1:-1]=P[:,2:]-P[:,:-2]; dv[1:-1]=P[2:]-P[:-2]
    ln=np.cross(du,dv); ln_norm=np.linalg.norm(ln,axis=-1)
    with np.errstate(invalid='ignore',divide='ignore'):
        horiz=np.abs((ln@n)/ln_norm)>=np.cos(np.deg2rad(p.top_normal_deg))
    mask=np.isfinite(h)&~edge&(h>p.min_obstacle_height)&(np.abs(lateral)<=p.corridor_half_width)&(forward>=p.min_forward)&(forward<=p.max_forward)
    x=forward[mask]; y=lateral[mask]; hh=h[mask]; hz=horiz[mask]
    clusters=[]
    if len(x):
        for ids in _components(x,y,p.cell_size):
            if len(ids)<p.min_cells: continue
            top=hh[ids][hz[ids]]
            if len(top)>=p.top_min_points:
                # Median of the upper horizontal band: the top face, not its noise tail.
                height=float(np.median(top[top>=np.percentile(top,95)-p.top_band_m]))
            else:
                height=float(np.percentile(hh[ids],95))
            # Nearest face: k-th smallest forward value, so a small object in front of a
            # large one it touches (books against a lamp pole) sets the distance.
            k=min(len(ids)-1,max(10,len(ids)//100))
            clusters.append(Cluster(float(np.partition(x[ids],k)[k]),height,
                float(np.percentile(y[ids],99.5)-np.percentile(y[ids],0.5)),float(np.ptp(x[ids])),int(len(ids))))
    nearest=min(clusters,key=lambda c:c.distance_m) if clusters else None
    result=FrameResult(bool(est.valid),reused,age,plane.camera_height,plane.pitch,plane.roll,nearest,clusters,(time.perf_counter()-t0)*1000)
    return result,state
