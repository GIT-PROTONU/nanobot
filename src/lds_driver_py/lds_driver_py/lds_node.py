"""Roborock LDS02RR / Neato XV-11 lidar driver (Python).

Pragmatic stand-in for the Rust `lds_driver`: r2r won't compile against this
RoboStack Humble build (r2r_msg_gen vs the rosidl bindings), and the data rate is
low (~450 packets/s, ~10 KB/s) so Python keeps up easily on the board.

Binary protocol (verified on the wire): 22-byte packets
    FA idx spdL spdM [4x(distL distM sigL sigM)] crcL crcM
90 packets (idx 0xA0..0xF9) x 4 readings = 360 deg. RPM = speed / 64. A distance
MSB bit7 = invalid, bit6 = low-signal; distance is the low 14 bits in mm. A 15-bit
checksum over the first 20 bytes is compared to bytes 20..21.

Publishes one sensor_msgs/LaserScan per revolution on /scan. Node name is
"lds_driver" so it reads the existing `lds_driver:` block in robot.yaml.
"""
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import LaserScan

try:
    import serial
    HAVE_SERIAL = True
except Exception as exc:  # pragma: no cover - hardware lib
    HAVE_SERIAL = False
    _SERIAL_ERR = exc

PKT_LEN = 22
CMD = 0xFA
IDX_LO = 0xA0
BAD_MASK = 0xC0      # invalid (0x80) | low-signal (0x40)
RAYS = 360


class LdsParser:
    """Byte-stream parser. feed() yields a full revolution (list of points)."""

    def __init__(self):
        self.buf = bytearray()
        self.building = []          # [(angle, dist_mm, quality)]
        self.rpm = 0.0

    def feed(self, data: bytes):
        scans = []
        b = self.buf
        b.extend(data)
        i, n = 0, len(b)
        while n - i >= PKT_LEN:
            if b[i] != CMD:
                i += 1
                continue
            pkt = b[i:i + PKT_LEN]
            if self._valid(pkt):
                done = self._parse(pkt)
                if done is not None:
                    scans.append(done)
                i += PKT_LEN
            else:
                i += 1              # bad checksum -> resync a byte at a time
        del b[:i]
        return scans

    @staticmethod
    def _valid(p) -> bool:
        chk = 0
        for ix in range(0, 20, 2):
            chk = (chk * 2 + p[ix] + (p[ix + 1] << 8)) & 0xFFFFFFFF
        cs = (chk & 0x7FFF) + (chk >> 15)
        cs &= 0x7FFF
        return (cs & 0xFF) == p[20] and ((cs >> 8) & 0xFF) == p[21]

    def _parse(self, p):
        start = ((p[1] - IDX_LO) & 0xFF) * 4
        self.rpm = ((p[3] << 8) | p[2]) / 64.0
        completed = None
        for q in range(4):
            o = 4 + q * 4
            dm = p[o + 1]
            bad = dm & BAD_MASK
            dist = 0 if bad else (p[o] | ((dm & 0x3F) << 8))
            quality = 0 if bad else (p[o + 2] | (p[o + 3] << 8))
            angle = start + q
            if angle == 0 and self.building:
                completed = self.building
                self.building = []
            self.building.append((angle, dist, quality))
        return completed


class LdsNode(Node):
    def __init__(self):
        super().__init__("lds_driver")
        self.declare_parameters("", [
            ("port", "/dev/ttyS1"), ("baud", 115200), ("frame_id", "laser"),
            ("clockwise", True), ("angle_offset_deg", 0.0),
            ("range_min", 0.12), ("range_max", 6.0), ("publish_rate", 10.0),
        ])
        g = self.get_parameter
        self.frame_id = g("frame_id").value
        self.clockwise = g("clockwise").value
        self.offset = int(round(g("angle_offset_deg").value))
        self.range_min = float(g("range_min").value)
        self.range_max = float(g("range_max").value)
        # Max /scan publish rate. The motor caps the real rate (~5 Hz); a lower
        # value here decimates scans. Retunable live from the web UI slider.
        self._pub_period = 1.0 / max(0.1, g("publish_rate").value)
        self._next_pub = 0.0
        self.add_on_set_parameters_callback(self._on_params)

        self.pub = self.create_publisher(LaserScan, "scan", qos_profile_sensor_data)
        self.parser = LdsParser()
        self.ser = None
        self._stop = threading.Event()

        if not HAVE_SERIAL:
            self.get_logger().error(f"pyserial unavailable: {_SERIAL_ERR}")
            return
        try:
            self.ser = serial.Serial(g("port").value, g("baud").value, timeout=0.2)
            self.get_logger().info(f"LDS open on {g('port').value} @{g('baud').value}")
        except Exception as exc:
            self.get_logger().error(f"LDS open failed: {exc}")
            return
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def _reader(self):
        while not self._stop.is_set():
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
            except Exception as exc:
                if not self._stop.is_set():
                    self.get_logger().error(f"serial read error: {exc}")
                return
            if not chunk:
                continue
            for scan in self.parser.feed(chunk):
                self._publish(scan)

    def _on_params(self, params):
        for p in params:
            if p.name == "publish_rate":
                self._pub_period = 1.0 / max(0.1, float(p.value))
        return SetParametersResult(successful=True)

    def _publish(self, points):
        now = time.monotonic()
        if now < self._next_pub:        # rate cap (motor sets the natural max)
            return
        self._next_pub = now + self._pub_period
        ranges = [math.inf] * RAYS
        intensities = [0.0] * RAYS
        for angle, dist_mm, quality in points:
            if dist_mm == 0:
                continue
            dist = dist_mm / 1000.0
            if dist < self.range_min or dist > self.range_max:
                continue
            base = (RAYS - angle) % RAYS if self.clockwise else angle
            idx = (base + self.offset) % RAYS
            if dist < ranges[idx]:        # keep closest hit per degree bin
                ranges[idx] = dist
                intensities[idx] = float(quality)

        rpm = self.parser.rpm
        scan_time = 60.0 / rpm if rpm > 1.0 else 0.2

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi * (RAYS - 1) / RAYS
        msg.angle_increment = 2.0 * math.pi / RAYS
        msg.time_increment = scan_time / RAYS
        msg.scan_time = scan_time
        msg.range_min = self.range_min
        msg.range_max = self.range_max
        msg.ranges = ranges
        msg.intensities = intensities
        self.pub.publish(msg)

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
    node = LdsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
