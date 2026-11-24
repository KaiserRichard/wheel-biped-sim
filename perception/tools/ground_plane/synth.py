"""Analytic pinhole ray caster for a pitched/rolled camera over a box scene."""
from dataclasses import dataclass
import numpy as np

FX, FY, CX, CY = 386.953, 386.953, 321.034, 241.040


@dataclass
class Box:
    """World box; center=(right, forward, bottom-height), size=(width, depth, height)."""
    position: tuple
    size: tuple


def _rotation(pitch, roll):
    # Camera-to-world with world x right, y forward, z up. Optical y points down.
    cp, sp = np.cos(-pitch), np.sin(-pitch)
    cr, sr = np.cos(roll), np.sin(roll)
    Rx = np.array([[1,0,0],[0,cp,-sp],[0,sp,cp]])
    Rz = np.array([[cr,-sr,0],[sr,cr,0],[0,0,1]])
    base = np.array([[1,0,0],[0,0,1],[0,-1,0]])
    return base @ Rx @ Rz


def render(*, height=0.40, pitch=0.0, roll=0.0, boxes=(), width=640, height_px=480,
           fx=FX, fy=FY, cx=CX, cy=CY, sigma_d=0.0, baseline=0.050,
           max_range=6.0, invalid_fraction=0.0, seed=0):
    """Return (depth_m, labels, true_height_m), labels 0 none, 1 floor, 2 obstacle.

    Noise follows sigma_Z=Z^2*sigma_d/(f*B). Box x/y/z axes are right/forward/up;
    boxes rest on z=0. Invalid pixels are independently sampled after rendering.
    """
    if height <= 0 or baseline <= 0 or not 0 <= invalid_fraction < 1:
        raise ValueError("height/baseline must be positive and invalid_fraction in [0,1)")
    v,u = np.indices((height_px,width), dtype=float)
    rays_cam = np.stack(((u-cx)/fx, (v-cy)/fy, np.ones_like(u)), axis=-1)
    R = _rotation(np.deg2rad(pitch), np.deg2rad(roll))
    rays = rays_cam @ R.T
    origin = np.array([0.0,0.0,height])
    # Ground z=0.
    t_floor = np.where(rays[...,2] < -1e-10, -height/rays[...,2], np.inf)
    depth = t_floor.copy(); labels = np.where(np.isfinite(t_floor),1,0).astype(np.uint8)
    true_h = np.zeros_like(depth)
    # Slab intersection with axis-aligned boxes; parameter t is camera Z depth.
    for box in boxes:
        center = np.asarray(box.position, dtype=float); size = np.asarray(box.size, dtype=float)
        if center.shape != (3,) or size.shape != (3,) or np.any(size <= 0):
            raise ValueError("box position and size must be 3-vectors; sizes positive")
        lo = center-size/2; hi = center+size/2
        t1=(lo-origin)/rays; t2=(hi-origin)/rays
        near=np.max(np.minimum(t1,t2),axis=-1); far=np.min(np.maximum(t1,t2),axis=-1)
        hit=(far>=np.maximum(near,0)) & (near>0) & (near<depth)
        depth[hit]=near[hit]; labels[hit]=2
        # Height is world z of the visible ray hit, above z=0.
        hit_world=origin+rays*near[...,None]
        true_h[hit]=hit_world[...,2][hit]
    valid=np.isfinite(depth)&(depth>0)&(depth<=max_range)
    labels[~valid]=0; depth[~valid]=0; true_h[~valid]=np.nan
    rng=np.random.default_rng(seed)
    if sigma_d > 0:
        sigma=depth**2*sigma_d/(fx*baseline)
        noise=rng.normal(0,sigma)
        noisy=depth+noise
        valid &= (noisy>0)&(noisy<=max_range)
        depth=np.where(valid,noisy,0)
        labels[~valid]=0; true_h[~valid]=np.nan
    if invalid_fraction:
        invalid=rng.random(depth.shape)<invalid_fraction
        depth[invalid]=0; labels[invalid]=0; true_h[invalid]=np.nan
    return depth.astype(np.float32), labels, true_h.astype(np.float32)
