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
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import PoseStamped, Twist, Vector3Stamped
from nav_msgs.msg import Odometry, Path
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
            # --- navigation (Stages 2/3) ---
            ("enable_motion", False),    # SAFETY: when false, plan+show path but DON'T drive
            ("robot_radius", 0.16),      # obstacle inflation for the planner (m)
            ("plan_downsample", 4),      # plan on a 1/N grid (CPU/RAM); 4 -> 0.20 m cells
            ("allow_unknown", True),     # let the global plan cross unmapped cells
            ("control_rate", 10.0),      # Hz controller / pursuit loop
            ("replan_period", 1.0),      # s between global replans while a goal is active
            ("max_lin", 0.15),           # m/s pure-pursuit speed cap
            ("max_ang", 1.0),            # rad/s turn rate cap
            ("lookahead", 0.30),         # m pure-pursuit lookahead
            ("goal_tol", 0.12),          # m: within this of the goal = arrived
            ("stop_distance", 0.25),     # m: obstacle closer than this ahead = stop+replan
            ("front_angle", 0.6),        # rad: half-width of the reactive front cone
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

        # navigation params
        self.enable_motion = bool(g("enable_motion").value)
        self.robot_radius = float(g("robot_radius").value)
        self.plan_downsample = int(g("plan_downsample").value)
        self.allow_unknown = bool(g("allow_unknown").value)
        self.replan_period = float(g("replan_period").value)
        self.max_lin = float(g("max_lin").value)
        self.max_ang = float(g("max_ang").value)
        self.lookahead = float(g("lookahead").value)
        self.goal_tol = float(g("goal_tol").value)
        self.stop_distance = float(g("stop_distance").value)
        self.front_angle = float(g("front_angle").value)

        # SLAM pose in the map frame.
        self.px = self.py = self.pth = 0.0
        self._have_map = False
        # Motion-prior trackers (last odom pose + last IMU yaw consumed by a scan).
        self._odom = None
        self._imu_yaw = None
        self._prev_odom = None
        self._prev_imu = None
        self._last_write = 0.0

        # navigation state (all callbacks + the control timer run on the one spin
        # thread, so plain attributes are safe — no locks needed).
        self._goal = None            # (x, y) world, or None
        self._path = []              # [(x, y)] world waypoints
        self._last_scan = None       # (angles, ranges) for the reactive layer
        self._next_replan = 0.0

        self.pose_pub = self.create_publisher(PoseStamped, "slam_pose", 10)
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.path_pub = self.create_publisher(Path, "plan", 5)
        self.create_subscription(Odometry, g("odom_topic").value, self._on_odom, 20)
        self.create_subscription(Vector3Stamped, g("euler_topic").value, self._on_euler, 10)
        self.create_subscription(
            LaserScan, g("scan_topic").value, self._on_scan, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "goal_pose", self._on_goal, 5)
        self.create_timer(1.0 / max(1.0, float(g("control_rate").value)), self._control)
        self.add_on_set_parameters_callback(self._on_params)

        self.get_logger().info(
            f"slam_nav up: {self.grid.n}x{self.grid.n} grid @ {self.grid.res:.3f} m "
            f"({self.grid.n * self.grid.res:.1f} m square), match {self.match_pts} pts; "
            f"motion {'ENABLED' if self.enable_motion else 'disabled (view/plan only)'}")

    def _on_params(self, params):
        # let the web UI flip enable_motion / retune speeds live via set_parameters
        for p in params:
            if p.name == "enable_motion":
                self.enable_motion = bool(p.value)
                if not self.enable_motion:
                    self._send(0.0, 0.0)     # drop to a stop the moment it's disabled
            elif p.name == "max_lin":
                self.max_lin = float(p.value)
            elif p.name == "max_ang":
                self.max_ang = float(p.value)
        return SetParametersResult(successful=True)

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
        self._last_scan = (angles, ranges)        # for the reactive front-stop layer

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

    # --- navigation (Stages 2/3) --------------------------------------------
    def _on_goal(self, msg):
        self._goal = (msg.pose.position.x, msg.pose.position.y)
        self._next_replan = 0.0                  # plan immediately on the next tick
        self.get_logger().info(f"goal set: ({self._goal[0]:.2f}, {self._goal[1]:.2f})")

    def _control(self):
        if self._goal is None or not self._have_map:
            return
        now = time.monotonic()
        if now >= self._next_replan:
            self._next_replan = now + self.replan_period
            path = self.grid.plan(
                (self.px, self.py), self._goal, radius_m=self.robot_radius,
                downsample=self.plan_downsample, allow_unknown=self.allow_unknown)
            self._path = path or []
            self._publish_path()                 # always publish so the UI shows the plan
            if not self._path:
                self.get_logger().warning("no path to goal", throttle_duration_sec=3.0)

        gx, gy = self._goal
        if math.hypot(gx - self.px, gy - self.py) < self.goal_tol:
            self.get_logger().info("goal reached")
            self._goal, self._path = None, []
            self._publish_path()
            self._send(0.0, 0.0)
            return

        # reactive safety: an obstacle in the forward cone -> stop and replan around it
        if self._front_blocked():
            self._send(0.0, 0.0)
            self._next_replan = min(self._next_replan, now + 0.2)
            return

        v, w = self._pursuit()
        self._send(v, w)

    def _front_blocked(self):
        if self._last_scan is None:
            return False
        ang, rng = self._last_scan
        fwd = np.abs(np.arctan2(np.sin(ang), np.cos(ang))) < self.front_angle
        r = rng[fwd]
        r = r[np.isfinite(r) & (r > 0.05)]
        return r.size > 0 and float(r.min()) < self.stop_distance

    def _pursuit(self):
        if not self._path:
            return 0.0, 0.0
        # lookahead target = first waypoint at least `lookahead` away (else the last)
        tx, ty = self._path[-1]
        for (x, y) in self._path:
            if math.hypot(x - self.px, y - self.py) >= self.lookahead:
                tx, ty = x, y
                break
        err = _wrap(math.atan2(ty - self.py, tx - self.px) - self.pth)
        dgoal = math.hypot(self._goal[0] - self.px, self._goal[1] - self.py)
        v = 0.0 if abs(err) > 0.6 else self.max_lin    # rotate in place if facing away
        v = min(v, self.max_lin * max(0.25, dgoal / 0.5))   # ease off near the goal
        w = max(-self.max_ang, min(self.max_ang, 1.5 * err))
        return v, w

    def _send(self, v, w):
        if not self.enable_motion:               # view/plan-only mode: never drive
            return
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(w)
        self.cmd_pub.publish(t)

    def _publish_path(self):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "map"
        for (x, y) in self._path:
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)

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
