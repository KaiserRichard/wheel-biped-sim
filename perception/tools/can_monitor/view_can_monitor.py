#!/usr/bin/env python3
"""Live table and optional CSV recorder for the passive STM32 CAN monitor."""
import argparse
import csv
import datetime as dt
import struct
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt32MultiArray


FIELDS = ("frames_s", "feedback_s", "commands_s", "commanded_speed_x1000",
          "tec", "rec", "lec", "bus_off", "fifo_overruns", "decode_failures",
          "fault_bits", "mode", "temperature_x10")
FIELDS = FIELDS + ("commanded_run_mode", "fault_frame_raw_word")


def signed32(value):
    return struct.unpack("<i", struct.pack("<I", value & 0xFFFFFFFF))[0]


class Viewer(Node):
    def __init__(self, csv_path):
        super().__init__("can_monitor_view")
        self.joints = {name: {"position": float("nan"), "velocity": float("nan"), "effort": float("nan")}
                       for name in ("left", "right")}
        self.stats = [[0] * len(FIELDS) for _ in range(2)]
        self.last_print = self.get_clock().now()
        self.csv_file = open(csv_path, "w", newline="", encoding="utf-8") if csv_path else None
        self.writer = None
        if self.csv_file:
            cols = ["timestamp"] + [f"{side}_{field}" for side in ("left", "right") for field in FIELDS]
            cols += [f"{side}_{field}" for side in ("left", "right") for field in ("position_rad", "velocity_rad_s", "effort_nm")]
            self.writer = csv.DictWriter(self.csv_file, fieldnames=cols)
            self.writer.writeheader()
        self.create_subscription(JointState, "/can_monitor/motors", self.on_joint, 10)
        self.create_subscription(UInt32MultiArray, "/can_monitor/stats", self.on_stats, 10)
        self.create_timer(0.2, self.print_table)

    def on_joint(self, msg):
        for i, name in enumerate(msg.name):
            if name in self.joints and i < len(msg.position) and i < len(msg.velocity) and i < len(msg.effort):
                self.joints[name] = {"position": msg.position[i], "velocity": msg.velocity[i], "effort": msg.effort[i]}

    def on_stats(self, msg):
        if len(msg.data) != 2 * len(FIELDS):
            self.get_logger().warning(f"Unexpected stats length {len(msg.data)}; expected {2 * len(FIELDS)}")
            return
        for bus in range(2):
            self.stats[bus] = list(msg.data[bus * len(FIELDS):(bus + 1) * len(FIELDS)])
        if self.writer:
            now = dt.datetime.now(dt.timezone.utc).isoformat()
            row = {"timestamp": now}
            for bus, side in enumerate(("left", "right")):
                for j, field in enumerate(FIELDS):
                    row[f"{side}_{field}"] = signed32(self.stats[bus][j]) if field == "commanded_speed_x1000" else self.stats[bus][j]
                for key in ("position", "velocity", "effort"):
                    row[f"{side}_{key}_{'rad' if key == 'position' else 'rad_s' if key == 'velocity' else 'nm'}"] = self.joints[side][key]
            self.writer.writerow(row)
            self.csv_file.flush()

    def print_table(self):
        now = self.get_clock().now()
        if (now - self.last_print).nanoseconds < 190_000_000:
            return
        self.last_print = now
        print("\033[2J\033[H", end="")
        print("Passive CAN monitor | rates in frames/s | speed in rad/s | temperature in °C")
        print("BUS   frames fb/s cmd/s  cmd_speed  pos(rad)  vel(rad/s) effort(Nm) temp  TEC REC LEC BO OVR decode fault mode run fault_frame")
        for i, side in enumerate(("left", "right")):
            s = self.stats[i]
            j = self.joints[side]
            print(f"{side:5} {s[0]:6} {s[1]:4} {s[2]:5} {signed32(s[3])/1000:10.3f} "
                  f"{j['position']:9.3f} {j['velocity']:10.3f} {j['effort']:10.3f} {s[12]/10:5.1f} "
                  f"{s[4]:3} {s[5]:3} {s[6]:3} {s[7]:2} {s[8]:3} {s[9]:6} 0x{s[10]:02x} {s[11]} {s[13]} 0x{s[14]:08x}")
        if self.csv_file:
            print(f"CSV: {self.csv_file.name}")
        sys.stdout.flush()

    def close(self):
        if self.csv_file:
            self.csv_file.close()
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", metavar="PATH", help="record a CSV row per 5 Hz stats update")
    args = parser.parse_args()
    rclpy.init()
    node = Viewer(args.csv)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
