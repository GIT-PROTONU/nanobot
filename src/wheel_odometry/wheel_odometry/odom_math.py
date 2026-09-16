"""Pure differential-drive odometry math (stdlib only — no rclpy/robot_msgs).

Extracted from encoder_node.py so the integration the whole SLAM/Nav2 chain rides on
(/odom + the odom->base_link TF) is unit-testable offline. Conventions match the node:

    * ticks are signed cumulative counts (forward +, reverse -), signed by the
      firmware by commanded wheel direction
    * dl/dr are metres per wheel since the previous sample
    * midpoint integration: the translation step uses the heading at the
      midpoint of the arc (th + dth/2) — the standard two-wheel approximation
    * heading wraps via atan2 so it stays in (-pi, pi]
"""
import math


def meters_per_tick(wheel_radius: float, ticks_per_rev: float) -> float:
    """Metres travelled per encoder tick: wheel circumference / ticks per rev."""
    return (2.0 * math.pi * wheel_radius) / ticks_per_rev


def integrate_pose(x: float, y: float, th: float, dl: float, dr: float,
                   wheel_sep: float):
    """One midpoint-integration step. (x, y, th) -> (x, y, th, ds, dth): the new
    pose plus the step's path length (ds, m) and heading change (dth, rad)."""
    ds = 0.5 * (dl + dr)
    dth = (dr - dl) / wheel_sep
    x += ds * math.cos(th + 0.5 * dth)
    y += ds * math.sin(th + 0.5 * dth)
    th = math.atan2(math.sin(th + dth), math.cos(th + dth))
    return x, y, th, ds, dth


def yaw_to_quat(yaw: float):
    """Heading (rad) -> (x, y, z, w). Planar rotation only (matches the node's
    Quaternion fill: z = sin(yaw/2), w = cos(yaw/2))."""
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))
