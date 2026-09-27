"""Wheel-encoder odometry from the ESP32 coprocessor.

The ESP32 (native zenoh-pico, no micro-ROS) counts single-channel rising edges on
each wheel and publishes raw cumulative counts on:

    /wheel_ticks     std_msgs/Int64MultiArray   data = [left, right]

The encoders have no second channel, so direction isn't sensed in hardware; the
firmware signs each count by the commanded wheel direction before publishing, so the
counts are already signed (forward +, reverse -). This node samples them on its own
publish thread and integrates a differential-drive model:

    /odom            nav_msgs/Odometry
    /joint_states    sensor_msgs/JointState   (left_wheel_joint, right_wheel_joint)
    /wheel_encoders  robot_msgs/WheelEncoders  (raw counts, for debugging)
    TF: odom -> base_link

STRUCTURAL INVARIANT (2026-09-27): /odom + the odom->base_link TF are the nav-critical
sensor feed (slam_toolbox's map->odom chain and Nav2's RPP both consume them), so they
are published from a DEDICATED THREAD, never from an executor timer/callback — the
same pattern the LDS and IMU reader threads already use. Executor load (sys_monitor
timers, param callbacks, a wedged subscription) can therefore never stall /odom or
freeze the slam TF chain. Only the tiny /wheel_ticks + /reset_ticks callbacks (store
+ generation bump) and the 2 s liveness check run on the executor.

/joint_states and /wheel_encoders are only published when something subscribes (the
map/UI use /odom + /wheel_ticks). The invert_* params are an SBC-side sign fallback.
"""
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int64MultiArray
from tf2_ros import TransformBroadcaster

from robot_msgs.msg import WheelEncoders

from .odom_math import integrate_pose, meters_per_tick, yaw_to_quat


def _yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z, q.w = yaw_to_quat(yaw)[2:]
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
            ("publish_rate", 15.0),
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

        # metres travelled per encoder tick
        self.m_per_tick = meters_per_tick(self.wheel_radius, self.ticks_per_rev)

        # ---- shared state (executor callbacks -> publisher thread) -------------
        # The callbacks only STORE; all integration + publishing lives in the
        # dedicated thread below. The lock makes the (left, right) pair atomic so a
        # torn read can't skew the differential integration.
        self._tick_lock = threading.Lock()
        self._latest_l = 0
        self._latest_r = 0
        self._seeded = False            # True once at least one /wheel_ticks arrived
        self._reset_gen = 0             # bumped by /reset_ticks; publisher re-seeds on change
        self._last_tick_at = time.monotonic()
        self._tick_lost_warn = False

        # Publisher-thread-only state (integration).
        self.x = self.y = self.th = 0.0
        self._pub_l = 0
        self._pub_r = 0
        self._pub_gen = -1              # -1: force a seed on the first sample
        self._prev_time = self.get_clock().now()

        self.odom_pub = self.create_publisher(Odometry, "odom", 20)
        self.js_pub = self.create_publisher(JointState, "joint_states", 20)
        self.enc_pub = self.create_publisher(WheelEncoders, "wheel_encoders", 20)
        self.tf_bc = TransformBroadcaster(self)

        # ESP32 /wheel_ticks liveness: if the coprocessor link dies, /odom keeps
        # publishing the last integrated pose and the slam_toolbox odom chain
        # silently freezes.
        # Timeout high enough that a slow boot or transient gap isn't a false alarm.
        self._started = time.monotonic()   # boot grace for the liveness check
        self.create_timer(2.0, self._check_ticks_alive)

        # Best-effort to match the ESP32's high-rate sensor publisher.
        ticks_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(
            Int64MultiArray, ticks_topic, self._on_ticks, ticks_qos)
        # The ESP32's /reset_ticks (bench calibration / clearing a stray-tick count)
        # zeros its raw counters — without a re-seed, the next integration step would
        # see the raw count fall from its old cumulative value to 0 and integrate a
        # huge phantom reverse motion. The generation bump makes the PUBLISHER thread
        # re-seed on the first post-reset sample (no cross-thread race).
        self.create_subscription(Bool, "reset_ticks", self._on_reset_ticks, 5)

        self.publish_rate = max(1.0, float(g("publish_rate").value))
        # let the web UI slider retune the odom/TF rate live via set_parameters
        self.add_on_set_parameters_callback(self._on_params)

        # THE nav-critical path: a dedicated publish thread (mirrors the LDS/IMU
        # reader-thread pattern). Lives or dies with the node, never with the
        # executor's callback queue.
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._publish_loop, daemon=True,
                                        name="odom_publish")
        self._thread.start()

        self.get_logger().info(
            f"wheel_odometry up: integrating {ticks_topic} "
            f"({self.ticks_per_rev} ticks/rev) at {self.publish_rate:.0f} Hz "
            f"(dedicated publish thread)")

    def _on_params(self, params):
        for p in params:
            if p.name == "publish_rate":
                # The publish loop re-reads the rate every iteration — no timer to
                # rebuild, and the change takes effect within one tick.
                self.publish_rate = max(1.0, float(p.value))
        return SetParametersResult(successful=True)

    def _on_ticks(self, msg: Int64MultiArray):
        if len(msg.data) < 2:
            return
        now = time.monotonic()
        l = int(msg.data[0]) * self.inv_l
        r = int(msg.data[1]) * self.inv_r
        with self._tick_lock:
            self._latest_l, self._latest_r = l, r
            self._seeded = True
            self._last_tick_at = now
        if self._tick_lost_warn:
            self._tick_lost_warn = False
            self.get_logger().warning("/wheel_ticks resumed after a gap")

    def _check_ticks_alive(self):
        """Troubleshooting aid: if /wheel_ticks goes silent, /odom freezes silently
        (it keeps republishing the last pose) and the SLAM chain stalls. Warn once
        per outage with an actionable cause instead of leaving the operator wondering
        why the robot's position stopped moving."""
        age = time.monotonic() - self._last_tick_at
        if time.monotonic() - self._started < 15.0:
            return   # boot grace: the ESP32/zenoh link may take a moment to stream
        if age > 5.0 and not self._tick_lost_warn:
            self._tick_lost_warn = True
            cause = ("no /wheel_ticks yet (ESP32 not streaming?)" if not self._seeded
                     else "link to ESP32 stall — heartbeat/ticks stopped")
            self.get_logger().error(
                f"wheel_ticks SILENT {age:.0f}s — {cause}. /odom is now frozen; "
                f"check the ESP32 coprocessor + zenoh link.")

    def _on_reset_ticks(self, msg: Bool):
        if msg.data:
            with self._tick_lock:
                self._reset_gen += 1
            self.get_logger().info("wheel_odometry: raw ticks reset (re-seeding)")

    # --- dedicated publish thread ---------------------------------------------
    def _publish_loop(self):
        """Integrate + publish /odom and the TF on a private clock. The only
        shared reads are the latest counts (under the tick lock); every other
        field here is thread-local, so executor load can never delay a publish."""
        while rclpy.ok() and not self._stop.is_set():
            period = 1.0 / self.publish_rate
            t0 = time.monotonic()
            try:
                self._publish_once()
            except Exception as exc:    # never let the thread die silently
                self.get_logger().error(f"odom publish error: {exc}")
            # Steady period (not sleep(period) — drift-free vs callback work).
            elapsed = time.monotonic() - t0
            self._stop.wait(max(0.001, period - elapsed))

    def _publish_once(self):
        with self._tick_lock:
            l, r = self._latest_l, self._latest_r
            gen, seeded = self._reset_gen, self._seeded
        if not seeded:
            return
        if gen != self._pub_gen:
            # First sample ever, or a /reset_ticks landed: re-seed the delta
            # baseline so the next real sample is the new zero point.
            self._pub_gen, self._pub_l, self._pub_r = gen, l, r
            return
        now = self.get_clock().now()
        dt = (now - self._prev_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        dl = (l - self._pub_l) * self.m_per_tick
        dr = (r - self._pub_r) * self.m_per_tick
        self._pub_l, self._pub_r = l, r
        self._prev_time = now

        self.x, self.y, self.th, ds, dth = integrate_pose(
            self.x, self.y, self.th, dl, dr, self.wheel_sep)
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

    def destroy_node(self):
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        return super().destroy_node()


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
