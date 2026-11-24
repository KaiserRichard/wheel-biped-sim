import csv
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np
from mcap_ros2.writer import Writer
from rosbags.highlevel import AnyReader
from rosbags.rosbag2 import Writer as RosbagWriter, StoragePlugin
from rosbags.typesys import Stores, get_typestore

from tools.ground_plane.synth import Box, render
from tools.obstacle_decision.replay_bag import replay_bag
from tools.obstacle_decision.eval_runs import evaluate_runs, read_run_sheet

IMAGE_DEF = '''std_msgs/Header header
uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
'''
INFO_DEF = '''std_msgs/Header header
uint32 height
uint32 width
string distortion_model
float64[] d
float64[9] k
float64[9] r
float64[12] p
uint32 binning_x
uint32 binning_y
sensor_msgs/RegionOfInterest roi
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
================================================================================
MSG: sensor_msgs/RegionOfInterest
uint32 x_offset
uint32 y_offset
uint32 height
uint32 width
bool do_rectify
'''


def make_bag(path, *, scene="box"):
    fx, fy, cx, cy = 386.953, 386.953, 321.034, 241.040
    boxes = {"box": (Box((0., 1.06, .025), (.30, .12, .05)),),
             # Wall: a 3 m wide, 1 m tall face at 1.00 m with the floor still visible below it.
             "wall": (Box((0., 1.05, .5), (3.0, .10, 1.0)),)}.get(scene, ())
    depth, _, _ = render(height=.30, boxes=boxes,
                         width=640, height_px=480, fx=fx, fy=fy, cx=cx, cy=cy)
    with open(path, 'wb') as f:
        w = Writer(f, compression=0)
        image_s = w.register_msgdef('sensor_msgs/msg/Image', IMAGE_DEF)
        info_s = w.register_msgdef('sensor_msgs/msg/CameraInfo', INFO_DEF)
        info = {'header': {'stamp': {'sec': 1, 'nanosec': 0}, 'frame_id': 'camera_color_optical_frame'},
                'height': 480, 'width': 640, 'distortion_model': 'plumb_bob', 'd': [],
                'k': [fx,0,cx,0,fy,cy,0,0,1], 'r': [1,0,0,0,1,0,0,0,1],
                'p': [fx,0,cx,0,0,fy,cy,0,0,0,1,0], 'binning_x': 0, 'binning_y': 0,
                'roi': {'x_offset':0,'y_offset':0,'height':0,'width':0,'do_rectify':False}}
        w.write_message('/camera/camera/color/camera_info', info_s, info, log_time=1_000_000_000)
        data = np.rint(depth * 1000).astype('<u2').tobytes()
        for i in range(6):
            sec, ns = divmod(1_000_000_000 + i * 33_000_000, 1_000_000_000)
            image = {'header': {'stamp': {'sec': sec, 'nanosec': ns}, 'frame_id':'camera_color_optical_frame'},
                     'height':480,'width':640,'encoding':'16UC1','is_bigendian':0,'step':1280,'data':list(data)}
            w.write_message('/camera/camera/aligned_depth_to_color/image_raw', image_s, image,
                             log_time=1_000_000_000 + i * 33_000_000)
        w.finish()


def make_rosbag_directory(source_mcap, target, storage_plugin):
    """Copy CDR records into a rosbag2 directory using the requested storage."""
    typestore = get_typestore(Stores.ROS2_JAZZY)
    with AnyReader([source_mcap], default_typestore=typestore) as reader:
        with RosbagWriter(target, version=9, storage_plugin=storage_plugin) as writer:
            conns = {}
            for c in reader.connections:
                conns[c.id] = writer.add_connection(c.topic, c.msgtype, typestore=typestore)
            for conn, timestamp, rawdata in reader.messages():
                writer.write(conns[conn.id], timestamp, rawdata)


def make_db3_file(source_mcap, target):
    """Make a standalone .db3 from the rosbag2 writer's sqlite file."""
    tempdir = target.parent / "db3-source"
    make_rosbag_directory(source_mcap, tempdir, StoragePlugin.SQLITE3)
    next(tempdir.glob("*.db3")).replace(target)


class ReplayBagTests(unittest.TestCase):
    def test_synthetic_mcap_replay(self):
        with tempfile.TemporaryDirectory() as td:
            bag = Path(td) / 'box.mcap'; out = Path(td) / 'out'; make_bag(bag)
            summary = replay_bag(bag, out=out)
            self.assertEqual(summary['frame_count'], 6)
            self.assertGreater(summary['plane_valid_rate'], .9)
            self.assertAlmostEqual(summary['camera_height_m']['median'], .30, delta=.02)
            self.assertAlmostEqual(summary['obstacle_height_m']['median'], .05, delta=.015)
            self.assertAlmostEqual(summary['obstacle_distance_m']['median'], 1.0, delta=.07)
            with (out / 'box_frames.csv').open() as f:
                rows = list(csv.DictReader(f))
            self.assertEqual(len(rows), 6)
            self.assertEqual(rows[0]['advisory'], 'UNCERTAIN')
            self.assertTrue((out / 'box_summary.json').is_file())
            self.assertEqual(json.loads((out / 'box_summary.json').read_text())['frame_count'], 6)

    def test_rosbag2_sqlite_and_mcap_directories_and_db3_file(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td); source = root / 'source.mcap'; make_bag(source)
            for name, plugin in (("sqlite", StoragePlugin.SQLITE3), ("mcap", StoragePlugin.MCAP)):
                bagdir = root / name; make_rosbag_directory(source, bagdir, plugin)
                summary = replay_bag(bagdir, out=root / f"out-{name}")
                self.assertEqual(summary["frame_count"], 6)
                self.assertAlmostEqual(summary["obstacle_distance_m"]["median"], 1.0, delta=.07)
            db3 = root / "single.db3"; make_db3_file(source, db3)
            self.assertEqual(replay_bag(db3, out=root / "out-db3")["frame_count"], 6)

    def test_eval_runs_end_to_end(self):
        with tempfile.TemporaryDirectory() as td:
            bag = Path(td) / 'box.mcap'; make_bag(bag)
            sheet = Path(td) / 'runs.csv'
            sheet.write_text('run_id,bag_path,box_height_m,box_front_distance_m,box_width_m,lateral_offset_m,camera_height_m,camera_tilt_deg,floor_type,notes\n'
                             'box1,box.mcap,0.05,1.0,0.30,0.0,0.30,0.0,vinyl,synthetic\n')
            self.assertEqual(len(read_run_sheet(sheet)), 1)
            out = Path(td) / 'eval'
            rows = evaluate_runs(sheet, out)
            self.assertAlmostEqual(rows[0]['height_error_m'], 0., delta=.015)
            self.assertAlmostEqual(rows[0]['distance_error_m'], 0., delta=.07)
            self.assertTrue(rows[0]['advisory_correct'])
            self.assertIn('| box1 |', (out / 'evaluation.md').read_text())
            self.assertTrue((out / 'evaluation.json').is_file())
            self.assertTrue((out / 'height_estimated_vs_measured.png').is_file())
            self.assertTrue((out / 'distance_error_vs_distance.png').is_file())

    def test_box_empty_floor_and_wall_sections(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            for name, scene in (("box", "box"), ("empty", "empty"), ("wall", "wall")):
                make_bag(root / f"{name}.mcap", scene=scene)
            sheet = root / "runs.csv"
            sheet.write_text(
                'run_id,bag_path,capture_type,mode,box_height_m,box_front_distance_m,box_width_m,lateral_offset_m,camera_height_m,camera_tilt_deg,wall_distance_m,floor_type,lighting,usb_disconnects,notes,extra\n'
                'box,box.mcap,static,box,0.05,1.0,0.30,0,0.30,0,,vinyl,bright,0,box,kept\n'
                'empty,empty.mcap,static,empty,,,,,0.30,0,,vinyl,bright,0,floor,extra-value\n'
                'wall,wall.mcap,static,wall,,,,,0.30,0,1.0,vinyl,bright,0,wall,kept-too\n')
            rows = evaluate_runs(sheet, root / "eval")
            by_id = {r["run_id"]: r for r in rows}
            self.assertEqual({r["run_type"] for r in rows}, {"box", "empty_floor", "wall"})
            self.assertTrue(by_id["box"]["pass"])
            self.assertEqual(by_id["empty"]["estimated_advisory"], "NONE")
            self.assertEqual(by_id["empty"]["false_detection_rate"], 0.0)
            self.assertLessEqual(by_id["empty"]["plane_height_std_m"], .005)
            self.assertLessEqual(by_id["empty"]["plane_pitch_std_deg"], .3)
            self.assertAlmostEqual(by_id["wall"]["estimated_wall_distance_m"], 1.0, delta=.001)
            self.assertAlmostEqual(by_id["wall"]["wall_error_m"], 0.0, delta=.001)
            self.assertTrue(by_id["wall"]["pass"])
            report = (root / "eval" / "evaluation.md").read_text()
            self.assertIn("## Box runs", report)
            self.assertIn("## Empty-floor runs", report)
            self.assertIn("## Wall-target runs", report)
            self.assertEqual(by_id["empty"]["run_sheet"]["extra"], "extra-value")

    def test_run_sheet_only_requires_identifiers(self):
        with tempfile.TemporaryDirectory() as td:
            p = Path(td) / 'bad.csv'; p.write_text('run_id,bag_path\na,b.mcap\n')
            rows = read_run_sheet(p)
            self.assertEqual(rows[0]["run_id"], "a")
            self.assertIsNone(rows[0]["box_height_m"])

    def test_missing_topic_error_lists_available(self):
        with tempfile.TemporaryDirectory() as td:
            bag=Path(td)/'box.mcap'; make_bag(bag)
            with self.assertRaisesRegex(ValueError, 'missing depth topic.*available topics'):
                replay_bag(bag, depth_topic='/not/depth')

if __name__ == '__main__': unittest.main()
