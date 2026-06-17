"""Quadrature wheel-encoder odometry.

Reads four GPIO lines (A/B per wheel) as edge events via libgpiod v2, decodes
quadrature into signed tick counts on a background thread, then integrates a
differential-drive model and publishes:

    /odom            nav_msgs/Odometry
    /joint_states    sensor_msgs/JointState   (left_wheel_joint, right_wheel_joint)
    /wheel_encoders  robot_msgs/WheelEncoders  (raw counts, for debugging)
    TF: odom -> base_link

The GPIO numbers in robot.yaml are GLOBAL libgpiod offsets (bank*32 + pin) — see
nanopi-neo-plus2-pinmap.md. Make sure none collide with lines the pinmap lists
as already claimed.
"""
import math
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from robot_msgs.msg import WheelEncoders

try:
    import gpiod
    from gpiod.line import Edge
    from gpiod import EdgeEvent
    _RISING = EdgeEvent.Type.RISING_EDGE
    HAVE_GPIOD = True
except Exception as exc:  # pragma: no cover - hardware lib
    HAVE_GPIOD = False
    _GPIOD_ERR = exc

# Quadrature state machine: index = (prev_state << 2) | new_state, where
# state = (A << 1) | B. +1 = forward, -1 = reverse, 0 = invalid/no-move.
_QUAD_LUT = (0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0)


def _yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class EncoderNode(Node):
    def __init__(self):
        super().__init__("wheel_odometry")

        p = self.declare_parameters("", [
            ("gpio_chip", "/dev/gpiochip0"),
            ("left_pin_a", 7), ("left_pin_b", 8),
            ("right_pin_a", 198), ("right_pin_b", 199),
            ("ticks_per_rev", 1440),
            ("wheel_radius", 0.0335),
            ("wheel_separation", 0.16),
            ("invert_left", False), ("invert_right", False),
            ("publish_rate", 30.0),
            ("publish_tf", True),
            ("odom_frame", "odom"),
            ("base_frame", "base_link"),
        ])
        g = self.get_parameter
        self.chip_path = g("gpio_chip").value
        self.la, self.lb = g("left_pin_a").value, g("left_pin_b").value
        self.ra, self.rb = g("right_pin_a").value, g("right_pin_b").value
        self.ticks_per_rev = g("ticks_per_rev").value
        self.wheel_radius = g("wheel_radius").value
        self.wheel_sep = g("wheel_separation").value
        self.inv_l = -1 if g("invert_left").value else 1
        self.inv_r = -1 if g("invert_right").value else 1
        self.publish_tf = g("publish_tf").value
        self.odom_frame = g("odom_frame").value
        self.base_frame = g("base_frame").value
        rate = g("publish_rate").value

        # metres travelled per encoder tick
        self.m_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev

        # Shared tick counters (updated by the reader thread).
        self._lock = threading.Lock()
        self.left_ticks = 0
        self.right_ticks = 0
        self._state_l = 0
        self._state_r = 0

        # Integrated pose + last sampled counts (timer thread only).
        self.x = self.y = self.th = 0.0
        self._prev_l = 0
        self._prev_r = 0
        self._prev_time = self.get_clock().now()

        self.odom_pub = self.create_publisher(Odometry, "odom", 20)
        self.js_pub = self.create_publisher(JointState, "joint_states", 20)
        self.enc_pub = self.create_publisher(WheelEncoders, "wheel_encoders", 20)
        self.tf_bc = TransformBroadcaster(self)

        if not HAVE_GPIOD:
            self.get_logger().error(
                f"python 'gpiod' (libgpiod v2) not available: {_GPIOD_ERR}. "
                "Encoder counts will stay at zero. `pixi add --pypi gpiod`.")
        else:
            self._stop = threading.Event()
            self._reader = threading.Thread(target=self._read_loop, daemon=True)
            self._reader.start()

        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            f"wheel_odometry up: chip={self.chip_path} L=({self.la},{self.lb}) "
            f"R=({self.ra},{self.rb}) {self.ticks_per_rev} ticks/rev")

    # --- GPIO reader thread --------------------------------------------------
    def _read_loop(self):
        offsets = [self.la, self.lb, self.ra, self.rb]
        settings = gpiod.LineSettings(edge_detection=Edge.BOTH)
        try:
            request = gpiod.request_lines(
                self.chip_path, consumer="wheel_odometry",
                config={tuple(offsets): settings})
        except Exception as exc:  # pragma: no cover
            self.get_logger().error(f"failed to request GPIO lines: {exc}")
            return

        with request:
            vals = {o: request.get_value(o) for o in offsets}
            self._state_l = (_v(vals[self.la]) << 1) | _v(vals[self.lb])
            self._state_r = (_v(vals[self.ra]) << 1) | _v(vals[self.rb])
            while not self._stop.is_set():
                if not request.wait_edge_events(0.2):
                    continue
                for ev in request.read_edge_events():
                    vals[ev.line_offset] = 1 if ev.event_type == _RISING else 0
                    if ev.line_offset in (self.la, self.lb):
                        new = (_v(vals[self.la]) << 1) | _v(vals[self.lb])
                        delta = _QUAD_LUT[(self._state_l << 2) | new]
                        self._state_l = new
                        with self._lock:
                            self.left_ticks += delta * self.inv_l
                    else:
                        new = (_v(vals[self.ra]) << 1) | _v(vals[self.rb])
                        delta = _QUAD_LUT[(self._state_r << 2) | new]
                        self._state_r = new
                        with self._lock:
                            self.right_ticks += delta * self.inv_r

    # --- odometry integration / publishing -----------------------------------
    def _publish(self):
        now = self.get_clock().now()
        dt = (now - self._prev_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        with self._lock:
            l, r = self.left_ticks, self.right_ticks
        dl = (l - self._prev_l) * self.m_per_tick
        dr = (r - self._prev_r) * self.m_per_tick
        self._prev_l, self._prev_r, self._prev_time = l, r, now

        ds = 0.5 * (dl + dr)
        dth = (dr - dl) / self.wheel_sep
        # midpoint integration
        self.x += ds * math.cos(self.th + 0.5 * dth)
        self.y += ds * math.sin(self.th + 0.5 * dth)
        self.th = math.atan2(math.sin(self.th + dth), math.cos(self.th + dth))
        vx, wz = ds / dt, dth / dt
        stamp = now.to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = _yaw_to_quat(self.th)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation = _yaw_to_quat(self.th)
            self.tf_bc.sendTransform(t)

        js = JointState()
        js.header.stamp = stamp
        js.name = ["left_wheel_joint", "right_wheel_joint"]
        js.position = [l * self.m_per_tick / self.wheel_radius,
                       r * self.m_per_tick / self.wheel_radius]
        js.velocity = [(dl / self.wheel_radius) / dt, (dr / self.wheel_radius) / dt]
        self.js_pub.publish(js)

        enc = WheelEncoders()
        enc.header.stamp = stamp
        enc.left_ticks = int(l)
        enc.right_ticks = int(r)
        enc.left_velocity = (dl / self.wheel_radius) / dt
        enc.right_velocity = (dr / self.wheel_radius) / dt
        self.enc_pub.publish(enc)

    def destroy_node(self):
        if HAVE_GPIOD and hasattr(self, "_stop"):
            self._stop.set()
            self._reader.join(timeout=1.0)
        super().destroy_node()


def _v(value) -> int:
    """Normalise a gpiod Value (enum or int) to 0/1."""
    return 1 if int(getattr(value, "value", value)) else 0


def main():
    rclpy.init()
    node = EncoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
