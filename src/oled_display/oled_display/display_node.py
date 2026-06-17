"""Status OLED. Renders a small dashboard on an I2C SSD1306 via luma.oled:

    line 1: current time HH:MM:SS (local clock, ticks every refresh)
    line 2: custom text          (from /oled_text, set via the web UI)
    line 3: hostname + IP        (so you can find the web UI without a screen)
    line 4: pose  x / y / theta   (from /odom)
    line 5: speed v / w           (from /odom twist)
    line 6: LDS scan rate (Hz)    (from /scan timestamps)

Only the lines that fit the panel height are drawn (rows are ROW_PX tall), so the
clock and custom text stay visible while lower lines drop off on short panels.

Subscribe-only and best-effort: if the panel or luma isn't present the node still
runs (so the rest of the stack is unaffected) and just logs once. The render timer
does no I/O or allocation beyond the frame buffer — host/IP are cached.
"""
import math
import socket
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

try:
    from luma.core.interface.serial import i2c
    from luma.core.render import canvas
    from luma.oled.device import ssd1306
    from PIL import ImageFont
    HAVE_LUMA = True
except Exception as exc:  # pragma: no cover - hardware lib
    HAVE_LUMA = False
    _LUMA_ERR = exc

ROW_PX = 12           # vertical pitch of one text row, in pixels
IP_REFRESH_S = 30.0   # re-resolve the outbound IP at most this often


def _primary_ip() -> str:
    """Best-effort outbound IP without actually sending anything."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except Exception:
        return "0.0.0.0"
    finally:
        s.close()


class DisplayNode(Node):
    def __init__(self):
        super().__init__("oled_display")
        self.declare_parameters("", [
            ("i2c_bus", 1), ("i2c_address", 0x3C),
            ("width", 128), ("height", 64),
            ("refresh_rate", 2.0), ("show_ip", True),
        ])
        g = self.get_parameter
        self.show_ip = g("show_ip").value
        self.max_lines = max(1, g("height").value // ROW_PX)

        # Latest telemetry — written by callbacks, read by the render timer.
        self.pose = (0.0, 0.0, 0.0)
        self.speed = (0.0, 0.0)
        self.scan_hz = 0.0
        self._last_scan = None
        self.text = ""                       # custom message from the web UI

        # Cache host/IP so the render loop never does name resolution per frame.
        self.host = socket.gethostname()[:8]
        self._ip = "0.0.0.0"
        self._ip_due = 0.0

        self.device = None
        self.font = None
        if HAVE_LUMA:
            try:
                serial = i2c(port=g("i2c_bus").value, address=g("i2c_address").value)
                self.device = ssd1306(serial, width=g("width").value,
                                      height=g("height").value)
                self.font = ImageFont.load_default()
                self.get_logger().info(
                    f"SSD1306 ready on /dev/i2c-{g('i2c_bus').value} "
                    f"@0x{g('i2c_address').value:02x}")
            except Exception as exc:
                self.get_logger().error(f"OLED init failed: {exc}")
        else:
            self.get_logger().error(f"luma.oled unavailable: {_LUMA_ERR}")

        self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self.create_subscription(LaserScan, "scan", self._on_scan, 10)
        self.create_subscription(String, "oled_text", self._on_text, 10)
        self.create_timer(1.0 / g("refresh_rate").value, self._render)

    def _on_text(self, msg: String):
        self.text = msg.data
        self.get_logger().info(f"OLED text set to {msg.data!r}")

    def _on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        self.speed = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def _on_scan(self, msg: LaserScan):
        now = self.get_clock().now()
        if self._last_scan is not None:
            dt = (now - self._last_scan).nanoseconds * 1e-9
            if dt > 0:
                self.scan_hz = 0.8 * self.scan_hz + 0.2 / dt
        self._last_scan = now

    def _cached_ip(self) -> str:
        now = time.monotonic()
        if now >= self._ip_due:
            self._ip = _primary_ip()
            self._ip_due = now + IP_REFRESH_S
        return self._ip

    def _render(self):
        if not self.device:
            return
        x, y, th = self.pose
        v, w = self.speed
        lines = [time.strftime("%H:%M:%S")]
        if self.text:
            lines.append(self.text)
        if self.show_ip:
            lines.append(f"{self.host} {self._cached_ip()}")
        lines.append(f"x{x:+.2f} y{y:+.2f}")
        lines.append(f"th{math.degrees(th):+4.0f} v{v:+.2f} w{w:+.2f}")
        lines.append(f"lidar {self.scan_hz:4.1f} Hz")

        with canvas(self.device) as draw:
            for i, line in enumerate(lines[:self.max_lines]):
                draw.text((0, i * ROW_PX), line, font=self.font, fill=255)

    def destroy_node(self):
        if self.device:
            try:
                self.device.clear()
            except Exception:
                pass
        super().destroy_node()


def main():
    rclpy.init()
    node = DisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
