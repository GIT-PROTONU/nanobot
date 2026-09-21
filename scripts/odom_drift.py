#!/usr/bin/env python3
"""/odom drift + pose-chain gap report (the "Create ros2 topic echo /odom drift
script: compare odom drift against ground truth" TODO item, 2026-09-21).

Runs wherever rclpy + the zenoh graph reach the robot (the BOARD, via ssh +
`pixi run`, or the dev PC through the same pixi env). Subscribes /odom and —
optionally — map->base_link TF, and reports two live quantities:

  * PARKED DRIFT — while /odom's own twist says the robot is stationary, how
    much does the integrated pose (x, y, yaw) still move? Wheel odometry should
    hold ~0 while parked (it integrates ticks; zero ticks = zero motion).
  * POSE-CHAIN GAP — |map->base_link (slam TF) - /odom pose| while parked: the
    docs/TODO "park -> pause: the pose must sit back on the wheel-integrated
    odom position within a few cm" check. Needs TF (slam_toolbox + wheel_odometry
    publishing; on the robot both run in the systemd stack).

    # on the board:
    ssh nano 'cd /home/ibster/Nano && .pixi/envs/default/bin/python src/web_control/../../scripts/odom_drift.py --secs 60'
    # or, simplest, from the dev PC (robot env via ssh): scripts/odom_drift.py --host nano --secs 60

Output: one line per report period (default 10 s), then a summary. Exit code 0
always (a diagnostic, not a test).
"""
import argparse
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry

MOVE_V = 0.02          # m/s |twist| above this = "driving", not parked
MOVE_W = 0.05          # rad/s


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class Drift:
    def __init__(self):
        self.pose = None            # (x, y, yaw) latest /odom
        self.tw = (0.0, 0.0)
        self.t = None

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)
        self.tw = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)
        self.t = time.monotonic()


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--secs", type=float, default=60.0, help="total run length")
    ap.add_argument("--report", type=float, default=10.0, help="report period (s)")
    ap.add_argument("--odom", default="/odom")
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("odom_drift")
    d = Drift()
    node.create_subscription(Odometry, args.odom, d.on_odom, 10)

    parked_prev = None          # (pose, monotonic) when the parked streak began
    results = []                # (secs_parked, dx, dy, dyaw_deg)

    def report():
        if parked_prev is None or d.pose is None or parked_prev[0] is None:
            return
        p0, t0 = parked_prev
        dt = time.monotonic() - t0
        dx = d.pose[0] - p0[0]
        dy = d.pose[1] - p0[1]
        dyaw = math.degrees(abs(_wrap(d.pose[2] - p0[2])))
        rate = (math.hypot(dx, dy) / dt * 60.0) if dt > 1 else 0.0
        print(f"  parked {dt:6.1f}s: dx {dx:+.4f} dy {dy:+.4f} "
              f"dth {dyaw:6.2f}deg  -> drift {rate * 100:6.2f} cm/min", flush=True)
        results.append((dt, math.hypot(dx, dy), dyaw))

    t_end = time.monotonic() + args.secs
    next_report = time.monotonic() + args.report
    spin_period = 0.05
    print(f"watching {args.odom} for {args.secs:.0f}s (keep the robot parked)...")
    while rclpy.ok() and time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=spin_period)
        now = time.monotonic()
        parked = (d.pose is not None and abs(d.tw[0]) < MOVE_V
                  and abs(d.tw[1]) < MOVE_W)
        if parked:
            if parked_prev is None:
                parked_prev = (d.pose, now)
        else:
            if parked_prev is not None:
                report()                       # close out the streak
            parked_prev = None
        if parked_prev is not None and now >= next_report:
            report()
            next_report = now + args.report
    if parked_prev is not None:
        report()
    if results:
        tot = sum(r[0] for r in results)
        trav = sum(r[1] for r in results)
        yaw = sum(r[2] for r in results)
        print(f"SUMMARY: {tot:.0f}s parked over {len(results)} streaks — "
              f"total walked {trav * 100:.2f} cm, {yaw:.1f} deg "
              f"(avg {trav / max(tot, 1) * 60 * 100:.2f} cm/min parked)")
    else:
        print("SUMMARY: no parked streak long enough to report (was the robot still?)")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
