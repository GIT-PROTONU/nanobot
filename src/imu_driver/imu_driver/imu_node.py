"""BWT901CL 9-axis IMU driver — WitMotion 0x55 serial protocol over USB (CH340).

The sensor streams 11-byte frames: [0x55][type][4x int16 little-endian][checksum],
checksum = sum(first 10 bytes) & 0xFF. The BWT901CL is fixed at 115200 baud and by
default emits time(0x50) / accel(0x51) / gyro(0x52) / angle(0x53) / mag(0x54) — no
quaternion — so orientation is derived from the reported Euler angles.

Publishes a coherent set once per cycle (on the angle frame):
    /imu/data   sensor_msgs/Imu             accel (m/s^2), gyro (rad/s), orientation
    /imu/mag    sensor_msgs/MagneticField   raw magnetometer counts
    /imu/euler  geometry_msgs/Vector3Stamped  roll/pitch/yaw in degrees (web UI)

Best-effort: if pyserial or the port is missing the node still spins (logs once)
so the rest of the stack is unaffected.
"""
import math
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Vector3Stamped

try:
    import serial
    HAVE_SERIAL = True
except Exception as exc:  # pragma: no cover - hardware lib
    HAVE_SERIAL = False
    _SERIAL_ERR = exc

G = 9.80665
DEG2RAD = math.pi / 180.0
ACC_SCALE = 16.0 / 32768.0 * G              # raw int16 -> m/s^2  (+/-16 g range)
GYRO_SCALE = 2000.0 / 32768.0 * DEG2RAD     # raw int16 -> rad/s  (+/-2000 deg/s)
ANG_SCALE = 180.0 / 32768.0                 # raw int16 -> degrees (+/-180)

HEADER = 0x55
FRAME_LEN = 11
T_ACC, T_GYRO, T_ANGLE, T_MAG = 0x51, 0x52, 0x53, 0x54


def euler_to_quat(roll, pitch, yaw):
    """ZYX (yaw-pitch-roll) Euler angles in radians -> (x, y, z, w)."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class ImuNode(Node):
    def __init__(self):
        super().__init__("imu_driver")
        self.declare_parameters("", [
            ("port", "/dev/ttyUSB0"),
            ("baud", 115200),
            ("frame_id", "imu_link"),
            ("publish_rate", 50.0),
        ])
        g = self.get_parameter
        self.frame_id = g("frame_id").value
        # The sensor streams ~200 Hz; cap published rate to keep CPU/bus load down
        # (nothing here needs more than this, and the web UI only shows ~10 Hz).
        rate = max(1.0, g("publish_rate").value)
        self._pub_period = 1.0 / rate
        self._next_pub = 0.0
        # let the web UI slider retune the rate live via /imu_driver/set_parameters
        self.add_on_set_parameters_callback(self._on_params)

        self.pub_imu = self.create_publisher(Imu, "imu/data", 10)
        self.pub_mag = self.create_publisher(MagneticField, "imu/mag", 10)
        self.pub_eul = self.create_publisher(Vector3Stamped, "imu/euler", 10)

        # Latest decoded values, filled in frame-type order each cycle.
        self.acc = (0.0, 0.0, 0.0)
        self.gyro = (0.0, 0.0, 0.0)
        self.mag = (0.0, 0.0, 0.0)
        self.euler_deg = (0.0, 0.0, 0.0)

        self.ser = None
        self._stop = threading.Event()
        if not HAVE_SERIAL:
            self.get_logger().error(f"pyserial unavailable: {_SERIAL_ERR}")
            return
        try:
            self.ser = serial.Serial(g("port").value, g("baud").value, timeout=0.2)
            self.get_logger().info(
                f"BWT901CL open on {g('port').value} @{g('baud').value}")
        except Exception as exc:
            self.get_logger().error(f"IMU open failed: {exc}")
            return
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def _on_params(self, params):
        for p in params:
            if p.name == "publish_rate":
                self._pub_period = 1.0 / max(1.0, float(p.value))
        return SetParametersResult(successful=True)

    def _reader(self):
        """Blocking read loop: accumulate bytes, emit checksum-valid frames."""
        buf = bytearray()
        while not self._stop.is_set():
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
            except Exception as exc:
                if not self._stop.is_set():
                    self.get_logger().error(f"serial read error: {exc}")
                return
            if not chunk:
                continue
            buf.extend(chunk)
            i, n = 0, len(buf)
            while n - i >= FRAME_LEN:
                if buf[i] != HEADER:
                    i += 1
                    continue
                fr = buf[i:i + FRAME_LEN]
                if (sum(fr[:10]) & 0xFF) != fr[10]:
                    i += 1            # bad checksum -> resync one byte at a time
                    continue
                self._handle(fr)
                i += FRAME_LEN
            del buf[:i]               # keep the trailing partial frame

    def _handle(self, fr):
        t = fr[1]
        a, b, c, _ = struct.unpack("<hhhh", fr[2:10])
        if t == T_ACC:
            self.acc = (a * ACC_SCALE, b * ACC_SCALE, c * ACC_SCALE)
        elif t == T_GYRO:
            self.gyro = (a * GYRO_SCALE, b * GYRO_SCALE, c * GYRO_SCALE)
        elif t == T_MAG:
            self.mag = (float(a), float(b), float(c))
        elif t == T_ANGLE:
            self.euler_deg = (a * ANG_SCALE, b * ANG_SCALE, c * ANG_SCALE)
            # angle is the cycle's anchor frame; publish a coherent set, throttled
            now = time.monotonic()
            if now >= self._next_pub:
                self._next_pub = now + self._pub_period
                self._publish()

    def _publish(self):
        now = self.get_clock().now().to_msg()
        roll, pitch, yaw = (v * DEG2RAD for v in self.euler_deg)
        qx, qy, qz, qw = euler_to_quat(roll, pitch, yaw)

        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = self.frame_id
        imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w = qx, qy, qz, qw
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = self.gyro
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = self.acc
        # Rough fixed diagonals — the BWT901CL doesn't report per-axis variance.
        imu.orientation_covariance[0] = imu.orientation_covariance[4] = imu.orientation_covariance[8] = 0.01
        imu.angular_velocity_covariance[0] = imu.angular_velocity_covariance[4] = imu.angular_velocity_covariance[8] = 0.001
        imu.linear_acceleration_covariance[0] = imu.linear_acceleration_covariance[4] = imu.linear_acceleration_covariance[8] = 0.04
        self.pub_imu.publish(imu)

        mag = MagneticField()
        mag.header.stamp = now
        mag.header.frame_id = self.frame_id
        mag.magnetic_field.x, mag.magnetic_field.y, mag.magnetic_field.z = self.mag
        self.pub_mag.publish(mag)

        eul = Vector3Stamped()
        eul.header.stamp = now
        eul.header.frame_id = self.frame_id
        eul.vector.x, eul.vector.y, eul.vector.z = self.euler_deg
        self.pub_eul.publish(eul)

    def destroy_node(self):
        self._stop.set()
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        super().destroy_node()


def main():
    rclpy.init()
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
