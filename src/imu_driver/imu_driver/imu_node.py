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

Mounting offset: if the IMU isn't at the robot's centre of rotation (base_link
origin), its accelerometer also picks up lever-arm acceleration from the robot's
own rotation -- centripetal (omega x (omega x r)) and tangential (alpha x r), on
top of the true body acceleration. `offset_{x,y,z}_mm` (REP-103: x fwd, y left,
z up, mm from base_link) lets the web UI record where the sensor actually sits;
`lever_arm_correction` subtracts that lever-arm term every cycle so /imu/data
and /imu/web report the acceleration an IMU AT the centre would have seen. Gyro
needs no correction -- angular velocity is the same everywhere on a rigid body.
"""
import math
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import String

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
_WANTED = (T_ACC, T_GYRO, T_ANGLE, T_MAG)
# Precompiled little-endian 4x int16 decode — avoids re-parsing the format string
# on every frame (this runs hundreds of times a second).
_UNPACK_FROM = struct.Struct("<hhhh").unpack_from
# WitMotion output-rate register (RRATE 0x03) codes — used to slow the device
# down from its 200 Hz default so the node has far fewer frames to parse.
RATE_CODES = {1: 0x03, 2: 0x04, 5: 0x05, 10: 0x06, 20: 0x07,
              50: 0x08, 100: 0x09, 200: 0x0b}


def _dev_rate_for(hz):
    """Smallest supported device stream rate that still covers `hz` (capped 200)."""
    for r in (1, 2, 5, 10, 20, 50, 100, 200):
        if r >= hz:
            return r
    return 200


def _cross(u, v):
    ux, uy, uz = u
    vx, vy, vz = v
    return (uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx)


def lever_arm_correction(acc, gyro, alpha, offset_m):
    """Subtract the lever-arm acceleration an IMU at `offset_m` (metres, from the
    rotation centre) picks up from the body's own rotation, returning the
    acceleration an IMU AT the centre would read. `gyro` is angular velocity
    (rad/s), `alpha` its rate of change (rad/s^2, finite-differenced between
    samples). No-op (returns `acc` unchanged) when the offset is zero."""
    if offset_m == (0.0, 0.0, 0.0):
        return acc
    centripetal = _cross(gyro, _cross(gyro, offset_m))
    tangential = _cross(alpha, offset_m)
    return tuple(a - t - c for a, t, c in zip(acc, tangential, centripetal))


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
            ("port", "/dev/imu"),
            ("baud", 115200),
            ("frame_id", "imu_link"),
            ("publish_rate", 100.0),
            ("euler_rate", 25.0),       # /imu/euler only drives the web angle readout
            ("mag_rate", 10.0),         # /imu/mag is slow-moving + unused by the UI
            ("web_rate", 15.0),         # /imu/web (accel|/|gyro| summary for the UI)
            ("output_rate_hz", 0),      # device stream rate; 0 = auto-follow publish
            ("offset_x_mm", 0.0),       # IMU mounting offset from base_link centre
            ("offset_y_mm", 0.0),       # (REP-103: x fwd, y left, z up), mm --
            ("offset_z_mm", 0.0),       # see lever_arm_correction above
        ])
        g = self.get_parameter
        self.frame_id = g("frame_id").value
        # The device streams continuously; the reader parses EVERY frame, so the
        # cheapest lever for CPU is to make the device stream no faster than we
        # publish. `output_rate_hz` > 0 pins the device rate; 0 = auto-follow.
        self.force_rate = int(g("output_rate_hz").value)
        self.publish_rate = max(1.0, g("publish_rate").value)
        self._pub_period = 1.0 / self.publish_rate
        # /imu/euler + /imu/mag have no high-rate consumer (the standard /imu/data
        # already carries orientation) — cap each well below /imu/data.
        self.euler_rate = max(0.0, float(g("euler_rate").value))
        self._eul_period = (1.0 / self.euler_rate) if self.euler_rate > 0 else None
        self.mag_rate = max(0.0, float(g("mag_rate").value))
        self._mag_period = (1.0 / self.mag_rate) if self.mag_rate > 0 else None
        # /imu/web is a tiny Vector3Stamped (|accel|, |gyro|, actual /imu/data Hz) that
        # feeds the web readout, so rosbridge bridges THIS low-rate summary instead of
        # deserializing the full 50 Hz Imu (covariances and all) just for two numbers.
        self.web_rate = max(0.0, float(g("web_rate").value))
        self._web_period = (1.0 / self.web_rate) if self.web_rate > 0 else None
        self._offset_m = (g("offset_x_mm").value / 1000.0,
                           g("offset_y_mm").value / 1000.0,
                           g("offset_z_mm").value / 1000.0)
        self._prev_gyro = None          # for finite-differencing angular acceleration
        self._prev_gyro_t = None        # (only exercised while an offset is set)
        self._alpha = (0.0, 0.0, 0.0)   # last angular acceleration, rad/s^2
        self._next_pub = 0.0
        self._next_eul = 0.0
        self._next_mag = 0.0
        self._next_web = 0.0
        self._imu_hz = 0.0             # measured /imu/data publish rate (windowed avg)
        self._rate_t0 = None           # current measurement window start (monotonic)
        self._rate_n = 0               # publishes counted in the current window
        self._dev_hz = 0                # last rate actually programmed into device
        self._need_reconfig = threading.Event()
        # let the web UI slider retune the rate live via /imu_driver/set_parameters
        self.add_on_set_parameters_callback(self._on_params)
        # One-off calibration trigger from the web UI (WitMotion 5-byte hex protocol,
        # see _do_calibrate). Mirrors _need_reconfig: the callback (executor thread)
        # only sets a flag + the pending command; the actual serial write happens in
        # the reader thread, since self.ser must only ever be touched there.
        self._cal_pending = threading.Event()
        self._cal_cmd = None
        self.create_subscription(String, "imu_calibrate", self._on_calibrate_cmd, 5)
        self.pub_cal_status = self.create_publisher(
            String, "imu_calibrate_status",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.pub_imu = self.create_publisher(Imu, "imu/data", 10)
        self.pub_mag = self.create_publisher(MagneticField, "imu/mag", 10)
        self.pub_eul = self.create_publisher(Vector3Stamped, "imu/euler", 10)
        self.pub_web = self.create_publisher(Vector3Stamped, "imu/web", 10)

        # Pre-allocate the messages and set every constant field once; the hot path
        # only mutates the live values + stamp and re-publishes. Avoids building
        # three messages (and re-writing the covariances) on every cycle.
        self._imu = Imu()
        self._imu.header.frame_id = self.frame_id
        self._imu.orientation_covariance[0] = self._imu.orientation_covariance[4] = \
            self._imu.orientation_covariance[8] = 0.01
        self._imu.angular_velocity_covariance[0] = self._imu.angular_velocity_covariance[4] = \
            self._imu.angular_velocity_covariance[8] = 0.001
        self._imu.linear_acceleration_covariance[0] = self._imu.linear_acceleration_covariance[4] = \
            self._imu.linear_acceleration_covariance[8] = 0.04
        self._mag_msg = MagneticField()
        self._mag_msg.header.frame_id = self.frame_id
        self._eul_msg = Vector3Stamped()
        self._eul_msg.header.frame_id = self.frame_id
        self._web_msg = Vector3Stamped()
        self._web_msg.header.frame_id = self.frame_id

        # Latest decoded values, filled in frame-type order each cycle.
        self.acc = (0.0, 0.0, 0.0)
        self.gyro = (0.0, 0.0, 0.0)
        self.mag = (0.0, 0.0, 0.0)
        self.euler_deg = (0.0, 0.0, 0.0)

        self.port = g("port").value
        self.baud = g("baud").value
        self.ser = None
        self._stop = threading.Event()
        if not HAVE_SERIAL:
            self.get_logger().error(f"pyserial unavailable: {_SERIAL_ERR}")
            return
        # The IMU is on a USB-serial adapter that can be unplugged/re-enumerated;
        # the reader thread (re)opens the port and reconnects on its own, so the
        # node recovers automatically without a restart.
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def _configure_device(self):
        """Program the BWT901CL stream rate (WitMotion unlock + set RRATE) to match
        what we actually publish, so the reader parses no more frames than needed.
        Not saved to flash — re-sent on every (re)connect, so no wear and it resets
        to default on power-cycle."""
        want = self.force_rate if self.force_rate > 0 else self.publish_rate
        self._dev_hz = _dev_rate_for(want)
        code = RATE_CODES[self._dev_hz]
        # RSW (0x02) is a bitmask of WHICH frame types the device emits each cycle.
        # By default it streams time+accel+gyro+angle+mag (5 frames). We only use
        # accel+gyro+angle (+mag if enabled) — dropping the rest cuts serial bytes
        # AND parse work by ~40-60%, the biggest single CPU lever at high rates.
        rsw = 0x02 | 0x04 | 0x08                 # accel | gyro | angle
        if self.mag_rate > 0:
            rsw |= 0x10                          # mag
        try:
            self.ser.write(b"\xff\xaa\x69\x88\xb5")              # unlock
            time.sleep(0.05)
            self.ser.write(bytes((0xff, 0xaa, 0x02, rsw, 0x00)))  # set RSW (content)
            time.sleep(0.05)
            self.ser.write(bytes((0xff, 0xaa, 0x03, code, 0x00)))  # set RRATE (rate)
            time.sleep(0.05)
            self.ser.reset_input_buffer()
        except Exception as exc:
            self.get_logger().warning(f"IMU rate config failed: {exc}")

    _CAL_CMDS = ("accel", "mag_start", "mag_stop", "save")

    def _on_calibrate_cmd(self, msg):
        cmd = str(msg.data or "").strip().lower()
        if cmd not in self._CAL_CMDS:
            self._publish_cal_status(f"unknown command: {cmd}")
            return
        if not HAVE_SERIAL or self.ser is None:
            self._publish_cal_status("IMU port not connected — try again once it's up")
            return
        self._cal_cmd = cmd
        self._cal_pending.set()

    def _publish_cal_status(self, text):
        try:
            self.pub_cal_status.publish(String(data=text[:120]))
        except Exception:
            pass

    def _do_calibrate(self, cmd):
        """WitMotion 5-byte hex calibration commands. Runs in the reader thread (the
        only place self.ser is touched) since _reader() dispatches it. Accel cal is a
        blocking ~3s ("keep the sensor still and level" per the datasheet); mag cal is
        a start/stop pair around the user physically rotating the robot, so those two
        return immediately and rely on a follow-up button press."""
        try:
            self.ser.write(b"\xff\xaa\x69\x88\xb5")               # unlock registers
            time.sleep(0.05)
            if cmd == "accel":
                self._publish_cal_status("accel: keep the robot still & level — calibrating (3s)...")
                self.ser.write(bytes((0xff, 0xaa, 0x01, 0x01, 0x00)))
                time.sleep(3.0)
                self.ser.write(bytes((0xff, 0xaa, 0x01, 0x00, 0x00)))  # stop
                self._publish_cal_status("accel: done — press Save to keep it")
            elif cmd == "mag_start":
                self.ser.write(bytes((0xff, 0xaa, 0x01, 0x02, 0x00)))
                self._publish_cal_status(
                    "mag: rotating — slowly turn the robot through all orientations, then press Stop")
            elif cmd == "mag_stop":
                self.ser.write(bytes((0xff, 0xaa, 0x01, 0x00, 0x00)))
                self._publish_cal_status("mag: stopped — press Save to keep it")
            elif cmd == "save":
                self.ser.write(bytes((0xff, 0xaa, 0x00, 0x00, 0x00)))
                self._publish_cal_status("calibration saved to flash")
            self.ser.reset_input_buffer()
        except Exception as exc:
            self._publish_cal_status(f"error: {exc}")
            self.get_logger().warning(f"IMU calibration '{cmd}' failed: {exc}")

    def _on_params(self, params):
        for p in params:
            if p.name == "publish_rate":
                self.publish_rate = max(1.0, float(p.value))
                self._pub_period = 1.0 / self.publish_rate
                self._need_reconfig.set()   # auto-follow: re-tune the device rate
            elif p.name == "euler_rate":
                self.euler_rate = max(0.0, float(p.value))
                self._eul_period = (1.0 / self.euler_rate) if self.euler_rate > 0 else None
            elif p.name == "mag_rate":
                self.mag_rate = max(0.0, float(p.value))
                self._mag_period = (1.0 / self.mag_rate) if self.mag_rate > 0 else None
            elif p.name == "web_rate":
                self.web_rate = max(0.0, float(p.value))
                self._web_period = (1.0 / self.web_rate) if self.web_rate > 0 else None
            elif p.name == "output_rate_hz":
                self.force_rate = int(p.value)
                self._need_reconfig.set()
            elif p.name in ("offset_x_mm", "offset_y_mm", "offset_z_mm"):
                x, y, z = self._offset_m
                if p.name == "offset_x_mm":
                    x = float(p.value) / 1000.0
                elif p.name == "offset_y_mm":
                    y = float(p.value) / 1000.0
                else:
                    z = float(p.value) / 1000.0
                self._offset_m = (x, y, z)
        return SetParametersResult(successful=True)

    def _reader(self):
        """Open (with retry) and read the port; reconnect if it goes away."""
        buf = bytearray()
        while not self._stop.is_set():
            if self.ser is None:
                try:
                    self.ser = serial.Serial(self.port, self.baud, timeout=0.2)
                    self._need_reconfig.clear()
                    self._configure_device()
                    # The very first configure right after opening the port can be lost
                    # (suspected: the CH340 adapter's DTR toggle on open resets the
                    # BWT901CL, which is still finishing its own power-on cycle when the
                    # unlock/RRATE bytes land) -- the device then silently keeps
                    # streaming its 200 Hz factory default while `_dev_hz` (and the
                    # publish-gate fast path in `_handle`) already trust the requested
                    # rate, so /imu/data publishes at the raw 200 Hz instead of
                    # `publish_rate`. Confirmed on hardware: simply re-sending the exact
                    # same config (e.g. re-touching the web UI rate slider) always fixes
                    # it once the port is stable -- so repeat it once here after a brief
                    # settle instead of waiting on the user to notice and retune.
                    time.sleep(0.5)
                    self._configure_device()
                    self.get_logger().info(
                        f"BWT901CL open on {self.port} @{self.baud} "
                        f"(stream {self._dev_hz} Hz, publish {self.publish_rate:g} Hz)")
                    buf.clear()
                except Exception as exc:
                    self.get_logger().warning(
                        f"IMU port {self.port} unavailable ({exc}); retrying",
                        throttle_duration_sec=10.0)
                    self._stop.wait(2.0)        # back off before retrying
                    continue
            # A live publish_rate change retunes the device stream rate (done here,
            # in the reader thread, so the serial port is only ever touched here).
            if self._need_reconfig.is_set():
                self._need_reconfig.clear()
                self._configure_device()
            if self._cal_pending.is_set():
                self._cal_pending.clear()
                self._do_calibrate(self._cal_cmd)
            try:
                # Read in batches (blocks until 64 bytes or the 0.2 s timeout)
                # instead of draining to 1 byte at a time — that per-byte spin was
                # the bulk of the CPU. 64 B ≈ one full output cycle.
                chunk = self.ser.read(64)
            except Exception as exc:
                if not self._stop.is_set():
                    self.get_logger().warning(f"serial error ({exc}); reconnecting")
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None                 # trigger reopen on next loop
                continue
            if not chunk:
                continue
            buf.extend(chunk)
            i, n = 0, len(buf)
            try:
                while n - i >= FRAME_LEN:
                    if buf[i] != HEADER:
                        i += 1
                        continue
                    if (sum(buf[i:i + 10]) & 0xFF) != buf[i + 10]:
                        i += 1        # bad checksum -> resync one byte at a time
                        continue
                    self._handle(buf[i + 1], buf, i + 2)   # type, buffer, data off
                    i += FRAME_LEN
            except Exception as exc:
                # Never let a parse/publish error kill the reader thread (that would
                # silence the IMU until a full restart). Log once and keep going.
                if not self._stop.is_set() and rclpy.ok():
                    self.get_logger().warning(f"IMU parse error ({exc})",
                                              throttle_duration_sec=5.0)
            del buf[:i]               # keep the trailing partial frame

    def _handle(self, t, buf, off):
        if t not in _WANTED:            # skip time/port/etc. frames -> no unpack
            return
        a, b, c, _ = _UNPACK_FROM(buf, off)     # decode straight from the buffer
        if t == T_ACC:
            self.acc = (a * ACC_SCALE, b * ACC_SCALE, c * ACC_SCALE)
        elif t == T_GYRO:
            gyro = (a * GYRO_SCALE, b * GYRO_SCALE, c * GYRO_SCALE)
            if self._offset_m != (0.0, 0.0, 0.0):
                now = time.monotonic()
                if self._prev_gyro is not None:
                    dt = now - self._prev_gyro_t
                    if dt > 1e-4:
                        self._alpha = tuple((g - p) / dt for g, p in zip(gyro, self._prev_gyro))
                self._prev_gyro, self._prev_gyro_t = gyro, now
            self.gyro = gyro
        elif t == T_MAG:
            self.mag = (float(a), float(b), float(c))
        elif t == T_ANGLE:
            self.euler_deg = (a * ANG_SCALE, b * ANG_SCALE, c * ANG_SCALE)
            # angle is the cycle's anchor frame -> publish a coherent set. The device
            # stream auto-follows publish_rate, so when it's already at/below our target
            # we publish EVERY angle frame. The wall-clock gate is only for the in-between
            # case (e.g. publish 30 Hz off a 50 Hz stream); applying it when dev<=publish
            # would drop frames that arrive bunched in one batched read (two frames share
            # ~one timestamp -> the 2nd is gated out), which capped the effective rate at
            # roughly half the requested one (ask 100 -> get ~60).
            now = time.monotonic()
            if self._dev_hz <= self.publish_rate or now >= self._next_pub:
                self._next_pub = now + self._pub_period
                self._publish(now)

    def _publish(self, mono):
        # The reader is a daemon thread; during shutdown/restart the rclpy context
        # can be torn down out from under it. Bail rather than raise (an unhandled
        # exception here would kill the reader thread and silence the IMU).
        if self._stop.is_set() or not rclpy.ok():
            return
        # Track the real /imu/data publish rate (this method runs once per published
        # frame) so the UI can show it without subscribing to the topic itself. Count
        # publishes over a sliding window rather than a per-sample 1/dt EMA: angle
        # frames arrive bunched in one batched serial read (several publishes a few
        # microseconds apart, then a gap), so an instantaneous-dt estimate spikes to
        # many times the true rate (a 200 Hz stream read as ~800 Hz).
        if self._rate_t0 is None:
            self._rate_t0 = mono
        self._rate_n += 1
        win = mono - self._rate_t0
        if win >= 0.5:
            self._imu_hz = self._rate_n / win
            self._rate_t0 = mono
            self._rate_n = 0
        stamp = self.get_clock().now().to_msg()

        # /imu/data — full Imu (orientation + covariances). The web UI reads /imu/web
        # instead, and nothing else on the board consumes /imu/data by default, so only
        # build + serialize it when something actually subscribes. On an idle/autonomous
        # robot this skips ~50 Hz of pointless serialization (the biggest idle CPU lever).
        acc = lever_arm_correction(self.acc, self.gyro, self._alpha, self._offset_m)
        if self.pub_imu.get_subscription_count() > 0:
            roll, pitch, yaw = (v * DEG2RAD for v in self.euler_deg)
            qx, qy, qz, qw = euler_to_quat(roll, pitch, yaw)
            imu = self._imu               # reuse pre-built msg (constants set in __init__)
            imu.header.stamp = stamp
            imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w = qx, qy, qz, qw
            imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = self.gyro
            imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = acc
            self.pub_imu.publish(imu)

        # /imu/euler — drives the web UI angle readout; its own (lower) rate.
        if self._eul_period is not None and mono >= self._next_eul:
            self._next_eul = mono + self._eul_period
            eul = self._eul_msg
            eul.header.stamp = stamp
            eul.vector.x, eul.vector.y, eul.vector.z = self.euler_deg
            self.pub_eul.publish(eul)

        # /imu/mag — slow-moving and unused by the UI; published at its own low rate,
        # and only when subscribed (nothing on the board consumes it by default).
        if (self._mag_period is not None and mono >= self._next_mag):
            self._next_mag = mono + self._mag_period
            if self.pub_mag.get_subscription_count() > 0:
                mag = self._mag_msg
                mag.header.stamp = stamp
                mag.magnetic_field.x, mag.magnetic_field.y, mag.magnetic_field.z = self.mag
                self.pub_mag.publish(mag)

        # /imu/web — the web UI's whole IMU readout in one tiny low-rate message:
        # x=|accel| (m/s^2), y=|gyro| (rad/s), z=actual /imu/data rate (Hz). Lets the
        # browser drop its 50 Hz /imu/data subscription, cutting rosbridge's load.
        if self._web_period is not None and mono >= self._next_web:
            self._next_web = mono + self._web_period
            ax, ay, az = acc
            gx, gy, gz = self.gyro
            web = self._web_msg
            web.header.stamp = stamp
            web.vector.x = math.sqrt(ax * ax + ay * ay + az * az)
            web.vector.y = math.sqrt(gx * gx + gy * gy + gz * gz)
            web.vector.z = self._imu_hz
            self.pub_web.publish(web)

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
