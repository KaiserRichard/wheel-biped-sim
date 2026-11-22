"""Offline diagnosis. SQLite is opened mode=ro, immutable=1; archive never written."""
import json
import sqlite3
import sys
from collections import Counter
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from rosbags.typesys import Stores, get_typestore
from ros2.wbr_planner_node.wbr_planner_node.core import PlannerCore
from tools.ground_plane.plane import backproject, estimate_plane
from tools.local_map.fuse import transform_points
from tools.obstacle_decision.detect import process_frame
from tools.obstacle_decision.decide import update

OUT = Path(__file__).resolve().parent
ARCHIVE = Path.home() / 'wbr-evidence-archive/mobile-demo-20261024-01'
TS = get_typestore(Stores.ROS2_JAZZY)
DEPTH = '/camera/camera/aligned_depth_to_color/image_raw'
INFO = '/camera/camera/color/camera_info'

def messages(run):
    db = ARCHIVE / run / 'rgbd-bag/rgbd-bag_0.db3'
    with sqlite3.connect(db.as_uri() + '?mode=ro&immutable=1', uri=True) as conn:
        topics = {i: (name, typ) for i, name, typ in conn.execute('SELECT id,name,type FROM topics')}
        for topic_id, stamp, data in conn.execute('SELECT topic_id,timestamp,data FROM messages ORDER BY timestamp,id'):
            name, typ = topics[topic_id]
            if name in (DEPTH, INFO) or 'advisory' in name:
                yield name, stamp, TS.deserialize_cdr(data, typ)

def plane_json(p):
    return dict(valid=bool(p.valid), normal=p.normal.tolist(), height=p.camera_height,
                pitch_deg=float(np.rad2deg(p.pitch)), roll_deg=float(np.rad2deg(p.roll)), inliers=p.inlier_count)

def run_bag(run):
    core = PlannerCore()
    K = None; first_stamp = None; last = -10**30
    actions = Counter(); decisions = Counter(); state = {}; decision_state = {}; frames = []; detail = None
    recorded = {}; depth_count = 0; skipped_info = 0
    for topic, _, msg in messages(run):
        if 'advisory' in topic:
            val = getattr(msg, 'data', None)
            key = json.dumps(val.tolist() if hasattr(val, 'tolist') else val)
            recorded.setdefault(topic, Counter())[key] += 1
            continue
        if topic == INFO:
            K = np.array(msg.k).reshape(3,3); continue
        depth_count += 1
        stamp = msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec
        if first_stamp is None: first_stamp = stamp
        if K is None: skipped_info += 1; continue
        if stamp-last < 333_333_333: continue
        last = stamp
        depth = np.frombuffer(msg.data, dtype='>u2' if msg.is_bigendian else '<u2').reshape(msg.height,msg.step//2)[:,:msg.width].astype(np.float32)*0.001
        advice = core.update(depth, K, np.eye(4), stamp)
        actions[advice['action']] += 1
        fr, state = process_frame(depth, K, state=state)
        dec, decision_state = update(fr, decision_state)
        decisions[dec.advisory.name] += 1
        frames.append(dict(stamp_ns=stamp, action=advice['action'], nearest=advice['nearest_obstacle_cells'][:3], obstacle_node=dec.advisory.name,
                           obstacle_nearest=vars(fr.nearest) if fr.nearest else None))
        if detail is None and core.grid is not None:
            intr = core._intrinsics(K)
            plane = estimate_plane(depth, fx=intr[0],fy=intr[1],cx=intr[2],cy=intr[3],stride=4,min_inliers=60,seed=17)
            points, pixels = backproject(depth,*intr,stride=4)
            mapped = transform_points(core.T_map_world, points)
            rr, cc = core.grid._index(mapped[:,:2]); inside=(rr>=0)&(rr<core.grid.n)&(cc>=0)&(cc<core.grid.n)
            obstacle = inside & (mapped[:,2]>.03)
            nearest = advice['nearest_obstacle_cells'][0]
            nr, nc = core.grid._index(np.array([[nearest['x'],nearest['y']]]))
            selected = obstacle & (rr==nr[0]) & (cc==nc[0])
            np.savetxt(OUT/f'{run}-nearest-points.csv',np.column_stack((pixels[selected],points[selected],mapped[selected])),delimiter=',',header='row,col,optical_x,optical_y,optical_z,map_x,map_y,map_z',comments='')
            detail = dict(stamp_ns=stamp,K=K.tolist(),plane=plane_json(plane),obstacle_plane=plane_json(state['plane']),T_map_world=core.T_map_world.tolist(),
                          nearest_cell=nearest, nearest_cell_points=int(selected.sum()), obstacle_point_count=int(obstacle.sum()),
                          obstacle_pixel_bounds=[pixels[obstacle].min(axis=0).tolist(),pixels[obstacle].max(axis=0).tolist()],
                          obstacle_map_bounds=[mapped[obstacle].min(axis=0).tolist(),mapped[obstacle].max(axis=0).tolist()],
                          occupied_cells=int(core.grid.outputs(time_s=stamp*1e-9,obstacle_threshold_m=.03)['obstacle'].sum()))
            np.savez_compressed(OUT/f'{run}-first.npz',depth=depth,K=K,pixels=pixels,points_map=mapped,obstacle=obstacle,nearest=selected)
    return dict(depth_frames=depth_count,skipped_no_info=skipped_info,actions=dict(actions),obstacle_decisions=dict(decisions),detail=detail,frames=frames,
                recorded={k:dict(v) for k,v in recorded.items()})

if __name__ == '__main__':
    results = {run:run_bag(run) for run in sys.argv[1:] or ['S1-empty','B-H13']}
    (OUT/'results.json').write_text(json.dumps(results,indent=2))
    for run,r in results.items(): print(run,json.dumps({k:v for k,v in r.items() if k not in ('frames','recorded')}))
