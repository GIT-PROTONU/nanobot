#!/usr/bin/env python3
"""Record nav-chain topic timing to a JSONL file for stutter analysis.

Run ON THE BOARD with the pixi env activated (see unit_exec.sh's incantation).
Passive: only subscriptions, no publishers, no params, no services.
Records per message: wall clock + monotonic + the fields that matter for
discriminating nav-level gaps (cmd_vel/cmd_vel_nav) from wheel-level ripple
(wheel_ticks/odom) and slam cadence (scan).

    python3 stutter_rec.py --secs 180 --out /tmp/stutter.jsonl

Ctrl-C also stops and closes the file.
"""
import argparse
import json
import math
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int64MultiArray
from tf2_msgs.msg import TFMessage


class Recorder(Node):
    def __init__(self, out_path, secs):
        super().__init__("nav_stutter_rec")
        self.out = open(out_path, "w", buffering=1)
        self.t0_wall = time.time()
        self.t0_mono = time.monotonic()
        self.n = 0
        self.secs = secs
        self.done = False

        self.subs = []
        self.subs.append(self.create_subscription(Twist, "/cmd_vel", lambda m: self.emit("cmd_vel", v=m.linear.x, ang=m.angular.z), 10))
        self.subs.append(self.create_subscription(Twist, "/cmd_vel_nav", lambda m: self.emit("cmd_vel_nav", v=m.linear.x, ang=m.angular.z), 10))
        self.subs.append(self.create_subscription(Odometry, "/odom", self.on_odom, 10))
        self.subs.append(self.create_subscription(Int64MultiArray, "/wheel_ticks", self.on_ticks, 10))
        self.subs.append(self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data))
        self.subs.append(self.create_subscription(TFMessage, "/tf", self.on_tf, qos_profile_sensor_data))

    def emit(self, topic, **kw):
        now_wall = time.time()
        now_mono = time.monotonic()
        rec = {"dt": round(now_mono - self.t0_mono, 4), "w": round(now_wall, 4), "t": topic}
        rec.update(kw)
        self.out.write(json.dumps(rec, separators=(",", ":")) + "\n")
        self.n += 1

    def on_tf(self, m):
        # slam republishes map->odom once per processed scan — the lidar-based
        # pose reference that does NOT depend on the wheel encoders
        for tr in m.transforms:
            a, b = tr.header.frame_id, tr.child_frame_id
            if a == "map" and b == "odom":
                q = tr.transform.rotation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                self.emit("map_odom",
                          stamp=tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9,
                          x=round(tr.transform.translation.x, 4),
                          y=round(tr.transform.translation.y, 4),
                          yaw=round(yaw, 4))

    def emit(self, topic, **kw):
        now_wall = time.time()
        now_mono = time.monotonic()
        rec = {"dt": round(now_mono - self.t0_mono, 4), "w": round(now_wall, 4), "t": topic}
        rec.update(kw)
        self.out.write(json.dumps(rec, separators=(",", ":")) + "\n")
        self.n += 1

    def on_odom(self, m):
        p = m.pose.pose
        q = p.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.emit("odom", x=round(p.position.x, 4), y=round(p.position.y, 4),
                  yaw=round(yaw, 4), stamp=m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)

    def on_ticks(self, m):
        d = list(m.data) if m.data else [None, None]
        self.emit("ticks", L=d[0], R=d[1])

    def on_scan(self, m):
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.emit("scan", stamp=stamp, n=len(m.ranges))

    def elapsed(self):
        return time.monotonic() - self.t0_mono

    def close(self):
        try:
            self.out.close()
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--secs", type=int, default=180)
    ap.add_argument("--out", default="/tmp/stutter.jsonl")
    args = ap.parse_args()

    rclpy.init()
    node = Recorder(args.out, args.secs)

    def stop(*_):
        node.done = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    try:
        while rclpy.ok() and not node.done and node.elapsed() < node.secs:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.close()
        n = node.n
        rclpy.shutdown()
        print(f"recorded {n} messages -> {args.out}")


if __name__ == "__main__":
    main()