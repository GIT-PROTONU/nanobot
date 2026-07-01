"""Dev-PC-only Gazebo hardware stand-in.

On the real robot, three things live below the shared node graph: the LDS02RR lidar
(lds_driver_py), the BWT901CL IMU (imu_driver), and the ESP32 coprocessor (motors,
single-channel wheel encoders, board telemetry — see firmware/nanobot_coprocessor).
Gazebo Sim (ros_gz_sim) + ros_gz_bridge stand in for the physics/sensors; THIS node is
the only new logic, and its entire job is to re-publish exactly the topic contracts
those three real sources publish, so every real consumer (wheel_odometry, slam_nav,
oled_display, web_control, behavior) runs completely unmodified:

    /joint_states (bridged, sensor_msgs/JointState, wheel angles)
        -> integrate to cumulative signed counts -> /wheel_ticks
           (std_msgs/Int64MultiArray [left,right]) -- the REAL wheel_odometry node
           consumes this and produces /odom + TF, exactly as on the robot. (Gazebo's
           own diff-drive odometry is deliberately NOT used here.)
    /imu (bridged, sensor_msgs/Imu)
        -> /imu/euler (geometry_msgs/Vector3Stamped, roll/pitch/yaw degrees)
        -> /imu/web   (geometry_msgs/Vector3Stamped, |accel| m/s^2, |gyro| rad/s, rate Hz)
           -- matches imu_driver's exact output (see imu_driver/imu_node.py)
    /scan (bridged, sensor_msgs/LaserScan)
        -> /dev/shm/nano_scan.bin (same writer lds_driver_py uses -- see scan_blob.py)

Plus synthetic-but-plausible ESP32 board telemetry with no real hardware behind it
(temp/hall/heartbeat/wheel-suspended/lds rpm+hz), and log-only no-ops on the topics the
ESP32 would have consumed (/fan_pwm, /led, /lds_target_rpm).
"""
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32, Int64MultiArray
from sensor_msgs.msg import Imu, JointState, LaserScan
from geometry_msgs.msg import Vector3Stamped

from lds_driver_py.scan_blob import write_scan_blob


def _quat_to_euler_deg(x, y, z, w):
    """ZYX (yaw-pitch-roll) Euler angles in degrees from a quaternion."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    r = 180.0 / math.pi
    return roll * r, pitch * r, yaw * r


class SimBridgeNode(Node):
    def __init__(self):
        super().__init__("sim_bridge")
        self.declare_parameters("", [
            ("ticks_per_rev", 1440),          # must match wheel_odometry's ticks_per_rev
            ("left_wheel_joint", "left_wheel_joint"),
            ("right_wheel_joint", "right_wheel_joint"),
            ("invert_left", False),
            ("invert_right", False),
            ("esp_temp", 34.0),               # constant "healthy" board telemetry
            ("lds_rpm", 300.0),
            ("lds_hz", 5.0),
            ("heartbeat_rate", 1.0),          # Hz
        ])
        g = self.get_parameter
        self.ticks_per_rev = int(g("ticks_per_rev").value)
        self.left_joint = g("left_wheel_joint").value
        self.right_joint = g("right_wheel_joint").value
        self.inv_l = -1 if g("invert_left").value else 1
        self.inv_r = -1 if g("invert_right").value else 1
        self.esp_temp = float(g("esp_temp").value)
        self.lds_rpm = float(g("lds_rpm").value)
        self.lds_hz = float(g("lds_hz").value)

        # --- wheel ticks (from bridged /joint_states) --------------------------------
        self.ticks_pub = self.create_publisher(Int64MultiArray, "wheel_ticks", 20)
        self.create_subscription(JointState, "joint_states_sim", self._on_joint_states, 20)

        # --- IMU (from bridged /imu) ---------------------------------------------------
        self.eul_pub = self.create_publisher(Vector3Stamped, "imu/euler", 10)
        self.web_pub = self.create_publisher(Vector3Stamped, "imu/web", 10)
        self.create_subscription(Imu, "imu", self._on_imu, 10)

        # --- lidar scan blob (from bridged /scan) --------------------------------------
        self._scan_seq = 0
        self.create_subscription(LaserScan, "scan", self._on_scan, 10)

        # --- synthetic ESP32 board telemetry --------------------------------------------
        self.temp_pub = self.create_publisher(Float32, "esp32_temp", 10)
        self.hall_pub = self.create_publisher(Int32, "esp32_hall", 10)
        self.heartbeat_pub = self.create_publisher(Int32, "esp32_heartbeat", 10)
        self.susp_l_pub = self.create_publisher(Bool, "left_wheel_suspended", 10)
        self.susp_r_pub = self.create_publisher(Bool, "right_wheel_suspended", 10)
        self.lds_rpm_pub = self.create_publisher(Float32, "lds_rpm", 10)
        self.lds_hz_pub = self.create_publisher(Float32, "lds_hz", 10)
        self._beat = 0
        rate = max(0.1, float(g("heartbeat_rate").value))
        self.create_timer(1.0 / rate, self._on_heartbeat)

        # --- no-op sinks for what the ESP32 would have consumed -------------------------
        self.create_subscription(Float32, "fan_pwm", self._log_noop("fan_pwm"), 10)
        self.create_subscription(Bool, "led", self._log_noop("led"), 10)
        self.create_subscription(Float32, "lds_target_rpm", self._log_noop("lds_target_rpm"), 10)

        self.get_logger().info(
            f"sim_bridge up: {self.left_joint}/{self.right_joint} -> /wheel_ticks "
            f"({self.ticks_per_rev} ticks/rev), /imu -> /imu/euler+/imu/web, "
            f"/scan -> /dev/shm/nano_scan.bin")

    def _log_noop(self, name):
        def cb(_msg):
            self.get_logger().debug(f"sim_bridge: ignoring /{name} (no simulated actuator)")
        return cb

    # --- wheel ticks -------------------------------------------------------------------
    def _on_joint_states(self, msg: JointState):
        try:
            li = msg.name.index(self.left_joint)
            ri = msg.name.index(self.right_joint)
        except ValueError:
            return
        per_tick = self.ticks_per_rev / (2.0 * math.pi)
        left = int(round(msg.position[li] * per_tick)) * self.inv_l
        right = int(round(msg.position[ri] * per_tick)) * self.inv_r
        out = Int64MultiArray()
        out.data = [left, right]
        self.ticks_pub.publish(out)

    # --- IMU -----------------------------------------------------------------------------
    def _on_imu(self, msg: Imu):
        stamp = msg.header.stamp
        roll, pitch, yaw = _quat_to_euler_deg(
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        eul = Vector3Stamped()
        eul.header.stamp = stamp
        eul.vector.x, eul.vector.y, eul.vector.z = roll, pitch, yaw
        self.eul_pub.publish(eul)

        ax, ay, az = (msg.linear_acceleration.x, msg.linear_acceleration.y,
                      msg.linear_acceleration.z)
        gx, gy, gz = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        web = Vector3Stamped()
        web.header.stamp = stamp
        web.vector.x = math.sqrt(ax * ax + ay * ay + az * az)
        web.vector.y = math.sqrt(gx * gx + gy * gy + gz * gz)
        web.vector.z = 100.0   # nominal rate; Gazebo's IMU sensor rate isn't measured here
        self.web_pub.publish(web)

    # --- lidar scan blob -----------------------------------------------------------------
    def _on_scan(self, msg: LaserScan):
        self._scan_seq += 1
        write_scan_blob(self._scan_seq, msg.angle_min, msg.angle_increment, msg.ranges)

    # --- synthetic ESP32 telemetry ---------------------------------------------------------
    def _on_heartbeat(self):
        self._beat += 1
        self.temp_pub.publish(Float32(data=self.esp_temp))
        self.hall_pub.publish(Int32(data=0))
        self.heartbeat_pub.publish(Int32(data=self._beat))
        self.susp_l_pub.publish(Bool(data=False))
        self.susp_r_pub.publish(Bool(data=False))
        self.lds_rpm_pub.publish(Float32(data=self.lds_rpm))
        self.lds_hz_pub.publish(Float32(data=self.lds_hz))


def main():
    rclpy.init()
    node = SimBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
