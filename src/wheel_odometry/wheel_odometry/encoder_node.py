"""Wheel-encoder odometry from the ESP32 coprocessor.

The ESP32 (native zenoh-pico, no micro-ROS) counts single-channel rising edges on
each wheel and publishes raw cumulative counts on:

    /wheel_ticks     std_msgs/Int64MultiArray   data = [left, right]

The encoders have no second channel, so direction isn't sensed in hardware; the
firmware signs each count by the commanded wheel direction before publishing, so the
counts are already signed (forward +, reverse -). This node samples them on its own
timer and integrates a differential-drive model:

    /odom            nav_msgs/Odometry
    /joint_states    sensor_msgs/JointState   (left_wheel_joint, right_wheel_joint)
    /wheel_encoders  robot_msgs/WheelEncoders  (raw counts, for debugging)
    TF: odom -> base_link

/joint_states and /wheel_encoders are only published when something subscribes (the
map/UI use /odom + /wheel_ticks). The invert_* params are an SBC-side sign fallback.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Int64MultiArray
from tf2_ros import TransformBroadcaster

from robot_msgs.msg import WheelEncoders


def _yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class EncoderNode(Node):
    def __init__(self):
        super().__init__("wheel_odometry")

        self.declare_parameters("", [
            ("ticks_topic", "wheel_ticks"),
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
        self.ticks_per_rev = g("ticks_per_rev").value
        self.wheel_radius = g("wheel_radius").value
        self.wheel_sep = g("wheel_separation").value
        self.inv_l = -1 if g("invert_left").value else 1
        self.inv_r = -1 if g("invert_right").value else 1
        self.publish_tf = g("publish_tf").value
        self.odom_frame = g("odom_frame").value
        self.base_frame = g("base_frame").value
        ticks_topic = g("ticks_topic").value
        rate = g("publish_rate").value

        # metres travelled per encoder tick
        self.m_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev

        # Latest counts received from the coprocessor (None until first message).
        self.left_ticks = 0
        self.right_ticks = 0
        self._have_ticks = False

        # Integrated pose + last sampled counts (timer thread only).
        self.x = self.y = self.th = 0.0
        self._prev_l = 0
        self._prev_r = 0
        self._prev_time = self.get_clock().now()

        self.odom_pub = self.create_publisher(Odometry, "odom", 20)
        self.js_pub = self.create_publisher(JointState, "joint_states", 20)
        self.enc_pub = self.create_publisher(WheelEncoders, "wheel_encoders", 20)
        self.tf_bc = TransformBroadcaster(self)

        # Best-effort to match the ESP32's high-rate sensor publisher.
        ticks_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(
            Int64MultiArray, ticks_topic, self._on_ticks, ticks_qos)

        self.publish_rate = max(1.0, float(rate))
        self._timer = self.create_timer(1.0 / self.publish_rate, self._publish)
        # let the web UI slider retune the odom/TF rate live via set_parameters
        self.add_on_set_parameters_callback(self._on_params)
        self.get_logger().info(
            f"wheel_odometry up: integrating {ticks_topic} "
            f"({self.ticks_per_rev} ticks/rev) at {self.publish_rate:.0f} Hz")

    def _on_params(self, params):
        for p in params:
            if p.name == "publish_rate":
                self.publish_rate = max(1.0, float(p.value))
                self.destroy_timer(self._timer)
                self._timer = self.create_timer(1.0 / self.publish_rate, self._publish)
        return SetParametersResult(successful=True)

    def _on_ticks(self, msg: Int64MultiArray):
        if len(msg.data) < 2:
            return
        self.left_ticks = int(msg.data[0]) * self.inv_l
        self.right_ticks = int(msg.data[1]) * self.inv_r
        if not self._have_ticks:
            # Seed the deltas so the first integration step doesn't lurch.
            self._prev_l, self._prev_r = self.left_ticks, self.right_ticks
            self._have_ticks = True

    # --- odometry integration / publishing -----------------------------------
    def _publish(self):
        if not self._have_ticks:
            return
        now = self.get_clock().now()
        dt = (now - self._prev_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return
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

        # /joint_states + /wheel_encoders are debug/RViz aids — only build them when
        # something subscribes (the map, OLED and web UI all use /odom + /wheel_ticks).
        if self.js_pub.get_subscription_count() > 0:
            js = JointState()
            js.header.stamp = stamp
            js.name = ["left_wheel_joint", "right_wheel_joint"]
            js.position = [l * self.m_per_tick / self.wheel_radius,
                           r * self.m_per_tick / self.wheel_radius]
            js.velocity = [(dl / self.wheel_radius) / dt, (dr / self.wheel_radius) / dt]
            self.js_pub.publish(js)

        if self.enc_pub.get_subscription_count() > 0:
            enc = WheelEncoders()
            enc.header.stamp = stamp
            enc.left_ticks = int(l)
            enc.right_ticks = int(r)
            enc.left_velocity = (dl / self.wheel_radius) / dt
            enc.right_velocity = (dr / self.wheel_radius) / dt
            self.enc_pub.publish(enc)


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
