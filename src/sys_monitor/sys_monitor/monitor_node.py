"""Board health monitor. Publishes diagnostic_msgs/DiagnosticArray on /diagnostics
at a fixed rate, read straight from /proc and sysfs — no psutil, no allocation
churn, negligible cost on the 1 GB H5.

A single DiagnosticStatus named "system" carries KeyValue fields:
    cpu_percent, cpu_cores (per-core busy%, "12,4,6,8"), load1,
    mem_used_mb, mem_total_mb, mem_percent,
    cpu_temp_c, gpu_temp_c, disk_percent, uptime_s,
    wifi_iface, wifi_ssid, wifi_signal_dbm, wifi_quality_pct
and a level (OK/WARN) flagged when temp/mem/disk cross soft thresholds. The web
UI renders these in a System panel.
"""
import os
import socket
import subprocess
import time

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Float32

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

        # Cooling fan: publish a PWM duty (0..1) on /fan_pwm for the ESP32 to actuate.
        # Auto curve ramps the fan with CPU temperature between fan_temp_min..fan_temp_max
        # (mapped fan_min_duty..fan_max_duty). The web UI can force a fixed duty by setting
        # fan_override >= 0 (a -1 sentinel = auto). All settable live via /set_parameters.
        self.declare_parameter("fan_temp_min", 45.0)   # °C: fan starts ramping above this
        self.declare_parameter("fan_temp_max", 70.0)   # °C: fan at full above this
        self.declare_parameter("fan_min_duty", 0.0)    # duty at/below fan_temp_min
        self.declare_parameter("fan_max_duty", 1.0)    # duty at/above fan_temp_max
        self.declare_parameter("fan_override", -1.0)   # <0 = auto; 0..1 = forced duty
        self.fan_pub = self.create_publisher(Float32, "/fan_pwm", 10)

        self.pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.host = socket.gethostname()
        self.zones = _thermal_zones()
        self._prev = self._cpu_times()      # (idle, total) for delta-based CPU%
        self._ssid = ""                     # cached; refreshed at most every 5 s (subprocess)
        self._ssid_at = -1e9
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"sys_monitor publishing /diagnostics at {rate} Hz "
            f"(thermal: {', '.join(self.zones) or 'none'})")

    @staticmethod
    def _cpu_times():
        """{'cpu':(idle,total), 'cpu0':(idle,total), ...} — aggregate + per core."""
        out = {}
        for line in _read("/proc/stat").splitlines():
            if not line.startswith("cpu"):
                break                       # cpu* lines are first in /proc/stat
            f = line.split()
            try:
                parts = [int(x) for x in f[1:]]
            except ValueError:
                continue
            idle = parts[3] + (parts[4] if len(parts) > 4 else 0)  # idle + iowait
            out[f[0]] = (idle, sum(parts))
        return out

    def _cpu_percents(self):
        """Delta-based busy% for every cpu line since the last call."""
        cur = self._cpu_times()
        res = {}
        for name, (idle, total) in cur.items():
            pi, pt = self._prev.get(name, (idle, total))
            di, dt = idle - pi, total - pt
            res[name] = 100.0 * (1.0 - di / dt) if dt > 0 else 0.0
        self._prev = cur
        return res

    def _temp(self, zone_type: str) -> float:
        raw = _read(self.zones.get(zone_type, "")).strip()
        return int(raw) / 1000.0 if raw else float("nan")

    @staticmethod
    def _wifi_link():
        """(iface, quality, signal_dbm) from /proc/net/wireless — a pure read, no tools.
        Returns ('', nan, nan) when no wireless link is up."""
        for line in _read("/proc/net/wireless").splitlines():
            name, sep, rest = line.partition(":")
            if not sep or name.strip() in ("Inter-| sta", "face | tus"):
                continue                      # skip the two header rows
            f = rest.split()
            if len(f) >= 3:
                try:
                    return name.strip(), float(f[1].rstrip(".")), float(f[2].rstrip("."))
                except ValueError:
                    return name.strip(), float("nan"), float("nan")
        return "", float("nan"), float("nan")

    def _wifi_ssid(self, iface, now):
        """SSID of `iface`, cached for 5 s (the only subprocess here; ~ms, 0.2 Hz)."""
        if iface and now - self._ssid_at > 5.0:
            self._ssid_at = now
            self._ssid = ""
            for cmd in (["iwgetid", "-r", iface], ["iw", "dev", iface, "link"]):
                try:
                    out = subprocess.run(cmd, capture_output=True, text=True,
                                         timeout=1.0).stdout
                except (OSError, subprocess.SubprocessError):
                    continue
                if cmd[0] == "iwgetid" and out.strip():
                    self._ssid = out.strip(); break
                for ln in out.splitlines():
                    if ln.strip().startswith("SSID:"):
                        self._ssid = ln.split("SSID:", 1)[1].strip(); break
                if self._ssid:
                    break
        return self._ssid if iface else ""

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
        cpu = self._cpu_percents()
        cpu_pct = cpu.get("cpu", 0.0)
        # per-core busy%, in core order (cpu0, cpu1, ...) -> "12,4,6,8"
        cores, i = [], 0
        while f"cpu{i}" in cpu:
            cores.append(f"{cpu[f'cpu{i}']:.0f}")
            i += 1
        cpu_t = self._temp("cpu-thermal")
        gpu_t = self._temp("gpu-thermal")
        uptime = float((_read("/proc/uptime").split() or ["0"])[0] or 0)

        # WiFi: signal/quality (pure /proc read) + SSID (cached subprocess, 0.2 Hz)
        wifi_if, wifi_q, wifi_dbm = self._wifi_link()
        wifi_ssid = self._wifi_ssid(wifi_if, time.monotonic())
        wifi_pct = max(0.0, min(100.0, wifi_q / 70.0 * 100.0)) if wifi_q == wifi_q else float("nan")

        fields = {
            "cpu_percent": f"{cpu_pct:.1f}",
            "cpu_cores": ",".join(cores),     # per-core busy%, core order
            "load1": f"{load1:.2f}",
            "mem_used_mb": f"{used_mb:.0f}",
            "mem_total_mb": f"{total_mb:.0f}",
            "mem_percent": f"{mem_pct:.0f}",
            "cpu_temp_c": f"{cpu_t:.1f}",
            "gpu_temp_c": f"{gpu_t:.1f}",
            "disk_percent": f"{disk_pct:.0f}",
            "uptime_s": f"{uptime:.0f}",
            "wifi_iface": wifi_if,
            "wifi_ssid": wifi_ssid,
            "wifi_signal_dbm": "" if wifi_dbm != wifi_dbm else f"{wifi_dbm:.0f}",
            "wifi_quality_pct": "" if wifi_pct != wifi_pct else f"{wifi_pct:.0f}",
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

        self._publish_fan(cpu_t)

    def _publish_fan(self, cpu_t):
        """Publish the cooling-fan duty (0..1) on /fan_pwm: web override if set, else a
        linear ramp on CPU temperature. If temp is unreadable, fail safe to fan_max_duty."""
        override = self.get_parameter("fan_override").value
        lo_d = self.get_parameter("fan_min_duty").value
        hi_d = self.get_parameter("fan_max_duty").value
        if override is not None and override >= 0.0:
            duty = override
        elif cpu_t != cpu_t:                       # NaN: no thermal zone -> cool at full
            duty = hi_d
        else:
            lo_t = self.get_parameter("fan_temp_min").value
            hi_t = self.get_parameter("fan_temp_max").value
            frac = 0.0 if hi_t <= lo_t else (cpu_t - lo_t) / (hi_t - lo_t)
            duty = lo_d + max(0.0, min(1.0, frac)) * (hi_d - lo_d)
        self.fan_pub.publish(Float32(data=float(max(0.0, min(1.0, duty)))))


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
