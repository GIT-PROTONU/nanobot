"""Status OLED. Renders a small dashboard on an I2C SSD1306 via luma.oled:

    line 1: hostname + IP (so you can find the web UI without a screen on the bot)
    line 2: pose  x / y / theta   (from /odom)
    line 3: speed v / w           (from /odom twist)
    line 4: LDS scan rate (Hz)    (from /scan timestamps)

Subscribe-only and best-effort: if the panel or luma isn't present the node
still runs (so the rest of the stack is unaffected) and just logs once.
"""
import math
import socket

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

try:
    from luma.core.interface.serial import i2c
    from luma.oled.device import ssd1306
    from PIL import ImageDraw, ImageFont
    HAVE_LUMA = True
except Exception as exc:  # pragma: no cover - hardware lib
    HAVE_LUMA = False
    _LUMA_ERR = exc


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

        self.pose = (0.0, 0.0, 0.0)
        self.speed = (0.0, 0.0)
        self._last_scan = None
        self.scan_hz = 0.0

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
        self.create_timer(1.0 / g("refresh_rate").value, self._render)

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
                self.scan_hz = 0.8 * self.scan_hz + 0.2 * (1.0 / dt)
        self._last_scan = now

    def _render(self):
        if not self.device:
            return
        x, y, th = self.pose
        v, w = self.speed
        lines = []
        if self.show_ip:
            lines.append(f"{socket.gethostname()[:8]} {_primary_ip()}")
        lines.append(f"x{ x:+.2f} y{ y:+.2f}")
        lines.append(f"th{math.degrees(th):+4.0f} v{v:+.2f} w{w:+.2f}")
        lines.append(f"lidar {self.scan_hz:4.1f} Hz")

        from luma.core.render import canvas
        with canvas(self.device) as draw:  # type: ImageDraw.ImageDraw
            for i, line in enumerate(lines):
                draw.text((0, i * 12), line, font=self.font, fill=255)

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
