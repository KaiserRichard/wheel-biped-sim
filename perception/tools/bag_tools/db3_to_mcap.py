"""Copy a rosbag2 SQLite bag to MCAP for Foxglove, skipping topics whose type
is not in the ROS 2 Jazzy typestore (e.g. realsense2_camera_msgs/Metadata).
Usage: db3_to_mcap.py BAG_DIR OUT.mcap"""
import sys
from mcap.writer import Writer
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

src, dst = sys.argv[1], sys.argv[2]
ts = get_typestore(Stores.ROS2_JAZZY)
with Reader(src) as reader, open(dst, "xb") as f:
    w = Writer(f); w.start(profile="ros2")
    chan, skipped = {}, set()
    for c in reader.connections:
        if c.msgtype not in ts.types:
            skipped.add(c.topic); continue
        text, _ = ts.generate_msgdef(c.msgtype, ros_version=2)
        sid = w.register_schema(name=c.msgtype, encoding="ros2msg", data=text.encode())
        chan[c.id] = w.register_channel(topic=c.topic, message_encoding="cdr", schema_id=sid)
    n = 0
    for c, t, raw in reader.messages(connections=[c for c in reader.connections if c.id in chan]):
        w.add_message(chan[c.id], log_time=t, publish_time=t, data=raw); n += 1
    w.finish()
print(f"wrote {n} messages to {dst}; skipped topics: {sorted(skipped)}")
