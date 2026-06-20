"""Super-light 2D SLAM for the Nano robot (Stage 1 of 3).

Estimates the robot pose by fusing wheel-odometry translation (/odom) with IMU yaw
(/imu/euler) as the motion prior, then refines it each scan with a correlative
scan-to-map matcher (see occupancy.py). Builds an occupancy grid from /scan and dumps
it — plus the live pose — to a RAM file (/dev/shm/nano_map.bin) that web_control serves
to the browser map panel. Also republishes the corrected pose on /slam_pose for debug.

No motion is commanded here. Stage 2 adds the planner + click-to-goal; Stage 3 adds the
pure-pursuit controller (gated behind enable_motion).

Map file format (atomic via os.replace): one JSON metadata line, '\n', then the raw
int8 occupancy bytes (row-major, row 0 = origin_y). The browser parses the header and
draws the rest straight into an ImageData.
"""
import json
import math
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from .occupancy import GridMap

MAP_FILE = "/dev/shm/nano_map.bin"


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class NavNode(Node):
    def __init__(self):
        super().__init__("slam_nav")
        self.declare_parameters("", [
            ("scan_topic", "scan"),
            ("odom_topic", "odom"),
            ("euler_topic", "imu/euler"),
            ("map_size_m", 24.0),
            ("map_resolution", 0.05),
            ("range_min", 0.12),
            ("range_max", 6.0),
            ("match_lin", 0.10),         # scan-match search half-window, metres
            ("match_ang", 0.12),         # scan-match search half-window, radians
            ("match_points", 90),        # scan points used for matching (decimated)
            ("min_match_score", 1.0),    # below this, keep the prior (no good overlap)
            ("use_imu_yaw", True),       # IMU yaw delta for rotation (else wheel odom)
            ("map_write_rate", 2.0),     # Hz to (re)write the /dev/shm map file
        ])
        g = self.get_parameter
        self.grid = GridMap(
            size_m=float(g("map_size_m").value), res=float(g("map_resolution").value),
            rmin=float(g("range_min").value), rmax=float(g("range_max").value))
        self.match_lin = float(g("match_lin").value)
        self.match_ang = float(g("match_ang").value)
        self.match_pts = int(g("match_points").value)
        self.min_score = float(g("min_match_score").value)
        self.use_imu = bool(g("use_imu_yaw").value)
        self._write_period = 1.0 / max(0.2, float(g("map_write_rate").value))

        # SLAM pose in the map frame.
        self.px = self.py = self.pth = 0.0
        self._have_map = False
        # Motion-prior trackers (last odom pose + last IMU yaw consumed by a scan).
        self._odom = None
        self._imu_yaw = None
        self._prev_odom = None
        self._prev_imu = None
        self._last_write = 0.0

        self.pose_pub = self.create_publisher(PoseStamped, "slam_pose", 10)
        self.create_subscription(Odometry, g("odom_topic").value, self._on_odom, 20)
        self.create_subscription(Vector3Stamped, g("euler_topic").value, self._on_euler, 10)
        self.create_subscription(
            LaserScan, g("scan_topic").value, self._on_scan, qos_profile_sensor_data)

        self.get_logger().info(
            f"slam_nav up: {self.grid.n}x{self.grid.n} grid @ {self.grid.res:.3f} m "
            f"({self.grid.n * self.grid.res:.1f} m square), match {self.match_pts} pts")

    # --- motion-prior inputs -------------------------------------------------
    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        th = math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))
        self._odom = (msg.pose.pose.position.x, msg.pose.pose.position.y, th)

    def _on_euler(self, msg):
        self._imu_yaw = math.radians(msg.vector.z)   # /imu/euler vector.z = yaw (deg)

    # --- the SLAM step (per scan) -------------------------------------------
    def _on_scan(self, msg):
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        n = len(ranges)
        if n == 0:
            return
        angles = msg.angle_min + np.arange(n, dtype=np.float32) * msg.angle_increment

        if not self._have_map:
            # Seed: drop the first scan straight in at the origin and prime trackers.
            self.grid.integrate((0.0, 0.0, 0.0), angles, ranges)
            self._have_map = True
            self._prev_odom, self._prev_imu = self._odom, self._imu_yaw
            self._write_map()
            return

        px, py, pth = self._predict(self.px, self.py, self.pth)

        # Refine against the map with a decimated set of valid beams.
        v = (np.isfinite(ranges) & (ranges >= self.grid.rmin) & (ranges <= self.grid.rmax))
        va, vr = angles[v], ranges[v]
        if len(vr) > self.match_pts:
            idx = np.linspace(0, len(vr) - 1, self.match_pts).astype(int)
            va, vr = va[idx], vr[idx]
        if len(vr) > 10:
            cand = self.grid.match((px, py, pth), va, vr,
                                   lin=self.match_lin, ang=self.match_ang)
            # Reject a match with no real overlap (e.g. wide-open space) — trust the prior.
            if self.grid.score(cand, va, vr) >= self.min_score:
                px, py, pth = cand

        self.px, self.py, self.pth = px, py, _wrap(pth)
        self.grid.integrate((self.px, self.py, self.pth), angles, ranges)
        self._publish_pose()

        now = time.monotonic()
        if now - self._last_write >= self._write_period:
            self._last_write = now
            self._write_map()

    def _predict(self, px, py, pth):
        """Apply the odom/IMU motion since the last scan as the scan-match prior."""
        if self._odom is None or self._prev_odom is None:
            self._prev_odom, self._prev_imu = self._odom, self._imu_yaw
            return px, py, pth
        ox, oy, oth = self._odom
        pox, poy, poth = self._prev_odom
        # forward distance travelled in the odom frame (projected on its heading)
        ds = (ox - pox) * math.cos(poth) + (oy - poy) * math.sin(poth)
        if self.use_imu and self._imu_yaw is not None and self._prev_imu is not None:
            dth = _wrap(self._imu_yaw - self._prev_imu)
        else:
            dth = _wrap(oth - poth)
        self._prev_odom, self._prev_imu = self._odom, self._imu_yaw
        pth = _wrap(pth + dth)
        return px + ds * math.cos(pth), py + ds * math.sin(pth), pth

    # --- outputs -------------------------------------------------------------
    def _publish_pose(self):
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = "map"
        ps.pose.position.x = self.px
        ps.pose.position.y = self.py
        ps.pose.orientation.z = math.sin(self.pth * 0.5)
        ps.pose.orientation.w = math.cos(self.pth * 0.5)
        self.pose_pub.publish(ps)

    def _write_map(self):
        occ = self.grid.occupancy_int8()
        meta = {
            "w": self.grid.n, "h": self.grid.n, "res": self.grid.res,
            "ox": self.grid.origin, "oy": self.grid.origin,
            "px": self.px, "py": self.py, "pth": self.pth,
        }
        header = (json.dumps(meta) + "\n").encode()
        tmp = MAP_FILE + ".tmp"
        try:
            with open(tmp, "wb") as f:
                f.write(header)
                f.write(occ.tobytes())
            os.replace(tmp, MAP_FILE)        # atomic: the server never reads a torn file
        except OSError as exc:
            self.get_logger().warning(f"map write failed: {exc}", throttle_duration_sec=10.0)


def main():
    rclpy.init()
    node = NavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
