"""Board health monitor. Publishes diagnostic_msgs/DiagnosticArray on /diagnostics
at a fixed rate, read straight from /proc and sysfs — no psutil, no allocation
churn, negligible cost on the 1 GB H5.

A single DiagnosticStatus named "system" carries KeyValue fields:
    cpu_percent, load1, mem_used_mb, mem_total_mb, mem_percent,
    cpu_temp_c, gpu_temp_c, disk_percent, uptime_s
and a level (OK/WARN) flagged when temp/mem/disk cross soft thresholds. The web
UI renders these in a System panel.
"""
import os
import socket

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

# Soft thresholds -> status WARN (purely advisory, shown in the UI).
TEMP_WARN_C = 75.0
MEM_WARN_PCT = 90.0
DISK_WARN_PCT = 90.0


def _read(path: str) -> str:
    try:
        with open(path) as f:
            return f.read()
    except Exception:
        return ""


def _thermal_zones():
    """Map sysfs thermal-zone 'type' -> millidegree temp path, discovered once."""
    base = "/sys/class/thermal"
    zones = {}
    try:
        for z in os.listdir(base):
            if z.startswith("thermal_zone"):
                t = _read(f"{base}/{z}/type").strip()
                if t:
                    zones[t] = f"{base}/{z}/temp"
    except Exception:
        pass
    return zones


class MonitorNode(Node):
    def __init__(self):
        super().__init__("sys_monitor")
        self.declare_parameter("publish_rate", 1.0)
        rate = self.get_parameter("publish_rate").value

        self.pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.host = socket.gethostname()
        self.zones = _thermal_zones()
        self._prev = self._cpu_times()      # (idle, total) for delta-based CPU%
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"sys_monitor publishing /diagnostics at {rate} Hz "
            f"(thermal: {', '.join(self.zones) or 'none'})")

    @staticmethod
    def _cpu_times():
        try:
            parts = [int(x) for x in _read("/proc/stat").split("\n")[0].split()[1:]]
            idle = parts[3] + (parts[4] if len(parts) > 4 else 0)  # idle + iowait
            return idle, sum(parts)
        except Exception:
            return 0, 0

    def _cpu_percent(self) -> float:
        idle, total = self._cpu_times()
        di, dt = idle - self._prev[0], total - self._prev[1]
        self._prev = (idle, total)
        return 100.0 * (1.0 - di / dt) if dt > 0 else 0.0

    def _temp(self, zone_type: str) -> float:
        raw = _read(self.zones.get(zone_type, "")).strip()
        return int(raw) / 1000.0 if raw else float("nan")

    def _tick(self):
        # memory
        mem = {}
        for line in _read("/proc/meminfo").splitlines():
            k, _, rest = line.partition(":")
            mem[k] = int(rest.split()[0]) if rest.split() else 0  # kB
        total_mb = mem.get("MemTotal", 0) / 1024.0
        avail_mb = mem.get("MemAvailable", 0) / 1024.0
        used_mb = total_mb - avail_mb
        mem_pct = 100.0 * used_mb / total_mb if total_mb else 0.0

        # disk (rootfs), like df: used / capacity
        try:
            s = os.statvfs("/")
            disk_pct = 100.0 * (1.0 - s.f_bavail / s.f_blocks) if s.f_blocks else 0.0
        except Exception:
            disk_pct = 0.0

        load1 = os.getloadavg()[0]
        cpu_pct = self._cpu_percent()
        cpu_t = self._temp("cpu-thermal")
        gpu_t = self._temp("gpu-thermal")
        uptime = float((_read("/proc/uptime").split() or ["0"])[0] or 0)

        fields = {
            "cpu_percent": f"{cpu_pct:.1f}",
            "load1": f"{load1:.2f}",
            "mem_used_mb": f"{used_mb:.0f}",
            "mem_total_mb": f"{total_mb:.0f}",
            "mem_percent": f"{mem_pct:.0f}",
            "cpu_temp_c": f"{cpu_t:.1f}",
            "gpu_temp_c": f"{gpu_t:.1f}",
            "disk_percent": f"{disk_pct:.0f}",
            "uptime_s": f"{uptime:.0f}",
        }

        warn = (cpu_t == cpu_t and cpu_t >= TEMP_WARN_C) \
            or mem_pct >= MEM_WARN_PCT or disk_pct >= DISK_WARN_PCT

        st = DiagnosticStatus()
        st.level = DiagnosticStatus.WARN if warn else DiagnosticStatus.OK
        st.name = "system"
        st.hardware_id = self.host
        st.message = "elevated" if warn else "ok"
        st.values = [KeyValue(key=k, value=v) for k, v in fields.items()]

        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [st]
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = MonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
