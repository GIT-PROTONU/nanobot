"""Cheap /proc + thermal-zone body reads (ROS-free), shared by web_server (robot) and
scripts/dev_webui.py (dev harness) — one source of truth for the spoken-stats sampling
and the sensor-snapshot CPU/RAM/temp numbers. Everything degrades to NaN/None off-Linux
(the dev harness on Windows just says "No data")."""

STAT_PATH = "/proc/stat"
MEMINFO_PATH = "/proc/meminfo"
THERMAL_PATH = "/sys/class/thermal/thermal_zone0/temp"


def cpu_sample():
    """(idle_jiffies, total_jiffies) from /proc/stat, or None when unreadable."""
    try:
        with open(STAT_PATH) as f:
            parts = [int(x) for x in f.readline().split()[1:]]
        idle = parts[3] + (parts[4] if len(parts) > 4 else 0)  # idle + iowait
        return idle, sum(parts)
    except Exception:
        return None


def cpu_percent(prev):
    """Busy % since `prev` (an earlier cpu_sample()). Returns (pct_or_nan, new_sample)
    so the caller owns the between-calls state."""
    cur = cpu_sample()
    pct = float("nan")
    if cur and prev:
        di, dt = cur[0] - prev[0], cur[1] - prev[1]
        if dt > 0:
            pct = 100.0 * (1.0 - di / dt)
    return pct, cur


def mem_percent():
    try:
        tot = avail = 0
        with open(MEMINFO_PATH) as f:
            for line in f:
                if line.startswith("MemTotal:"):
                    tot = int(line.split()[1])
                elif line.startswith("MemAvailable:"):
                    avail = int(line.split()[1])
                if tot and avail:
                    break
        return 100.0 * (tot - avail) / tot if tot else float("nan")
    except Exception:
        return float("nan")


def cpu_temp():
    try:
        with open(THERMAL_PATH) as f:
            return int(f.read().strip()) / 1000.0
    except Exception:
        return float("nan")


def compose_stats(cpu, mem, temp):
    """The spoken system-stats line; NaN readings are skipped (x == x is the NaN test)."""
    parts = []
    if cpu == cpu:
        parts.append(f"C P U {cpu:.0f} percent")
    if mem == mem:
        parts.append(f"RAM {mem:.0f} percent")
    if temp == temp:
        parts.append(f"Temperature {temp:.0f} degrees")
    if not parts:
        return "No data"
    return ". ".join(parts)
