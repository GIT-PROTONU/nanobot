"""Offline tests for the health-event watch (no ROS).

Run: pixi run python -m pytest src/sys_monitor/test
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from sys_monitor.health_log import (  # noqa: E402
    HealthWatch, ESP32_TIMEOUT, START_GRACE, PROGRESS_SECS, MAX_BYTES,
)

RPM, HZ = 300.0, 450.0


def blob(rx=1000, err=0, stale=None, age=0.5, open_=1):
    h = {"seq": 1, "rx": rx, "err": err, "age": age, "open": open_}
    if stale:
        h["stale"] = 1
    return h


def watch(tmp_path):
    return HealthWatch(str(tmp_path / "health.log"), now=0.0)


def run_healthy(w, t0, t1, rx0=0):
    """Drive healthy ticks from t0..t1; returns all emitted lines."""
    out = []
    for t in range(int(t0), int(t1)):
        out += w.update(float(t), 1.0, RPM, HZ, blob(rx=rx0 + t * 10000))
    return out


def test_boot_up_lines(tmp_path):
    w = watch(tmp_path)
    lines = run_healthy(w, 0, 10)
    assert any("esp32 UP" in ln for ln in lines)
    assert any("lds UP" in ln for ln in lines)
    assert len(lines) == 2                       # transitions only, no spam


def test_never_up_logged_after_grace(tmp_path):
    w = watch(tmp_path)
    by_tick = [w.update(float(t), float("inf"), float("nan"), float("nan"), None)
               for t in range(0, int(START_GRACE) + 5)]
    joined = "\n".join(ln for tick in by_tick for ln in tick)
    assert "esp32 DOWN: no heartbeat ever received" in joined
    assert "down since start" in joined
    assert "no scan blob file" in joined
    # nothing was declared DOWN inside the grace window
    assert not any(tick for tick in by_tick[: int(START_GRACE) - 1])


def test_esp32_drop_and_recover(tmp_path):
    w = watch(tmp_path)
    run_healthy(w, 0, 60)
    lines = []
    for t in range(60, 70):                      # heartbeat stops at t=60
        lines += w.update(float(t), t - 60.0, RPM, HZ, blob(rx=t * 10000))
    down = [ln for ln in lines if "esp32 DOWN" in ln]
    assert len(down) == 1 and "was up" in down[0]
    lines = [ln for t in range(70, 75)
             for ln in w.update(float(t), 0.5, RPM, HZ, blob(rx=t * 10000))]
    up = [ln for ln in lines if "esp32 UP after" in ln]
    assert len(up) == 1


def test_lds_classify_sbc_branch_dead(tmp_path):
    w = watch(tmp_path)
    run_healthy(w, 0, 60)
    lines = []
    for t in range(60, 75):                      # rx freezes, esp32 still fine
        lines += w.update(float(t), 1.0, RPM, HZ, blob(rx=590000, stale=1))
    down = [ln for ln in lines if "lds DOWN" in ln]
    assert len(down) == 1
    assert "upstream fine" in down[0] and "esp32 up" in down[0]
    verdict = [ln for ln in lines if "outage counters" in ln]
    assert len(verdict) == 1 and "PA1 branch dead" in verdict[0]


def test_lds_classify_not_spinning(tmp_path):
    w = watch(tmp_path)
    run_healthy(w, 0, 60)
    lines = [ln for t in range(60, 75)
             for ln in w.update(float(t), 1.0, 0.0, 0.0, blob(rx=590000, stale=1))]
    down = [ln for ln in lines if "lds DOWN" in ln]
    assert "not spinning" in down[0]
    # upstream cause -> no misleading SBC-branch counter verdict
    assert not any("outage counters" in ln for ln in lines)


def test_lds_classify_garbled(tmp_path):
    w = watch(tmp_path)
    run_healthy(w, 0, 60)
    lines = []
    for t in range(60, 75):                      # rx + err both climbing
        lines += w.update(float(t), 1.0, RPM, HZ,
                          blob(rx=590000 + (t - 59) * 1000, err=(t - 59) * 500, stale=1))
    verdict = [ln for ln in lines if "outage counters" in ln]
    assert len(verdict) == 1
    assert "degraded/garbled" in verdict[0] and "err +" in verdict[0]


def test_lds_classify_driver_silent(tmp_path):
    w = watch(tmp_path)
    run_healthy(w, 0, 60)
    lines = [ln for t in range(60, 70)
             for ln in w.update(float(t), 1.0, RPM, HZ, blob(rx=590000, age=30.0))]
    down = [ln for ln in lines if "lds DOWN" in ln]
    assert "driver silent" in down[0]


def test_progress_and_recovery_deltas(tmp_path):
    w = watch(tmp_path)
    run_healthy(w, 0, 60)
    lines = []
    for t in range(60, 60 + int(PROGRESS_SECS) + 10):
        lines += w.update(float(t), 1.0, RPM, HZ, blob(rx=590000, err=7, stale=1))
    assert any("lds still down" in ln for ln in lines)
    lines = [ln for t in range(200, 205)
             for ln in w.update(float(t), 1.0, RPM, HZ, blob(rx=600000, err=7))]
    up = [ln for ln in lines if "lds UP after" in ln]
    assert len(up) == 1 and "rx +" in up[0]


def test_write_and_rotate(tmp_path):
    w = watch(tmp_path)
    w.write(["hello world"])
    text = (tmp_path / "health.log").read_text()
    assert "hello world" in text and text[:4].isdigit()
    (tmp_path / "health.log").write_text("x" * (MAX_BYTES + 1))
    w.write(["after rotate"])
    assert "after rotate" in (tmp_path / "health.log").read_text()
    assert (tmp_path / "health.log.1").exists()


def test_esp32_timeout_constant_sane():
    assert 2.0 < ESP32_TIMEOUT < 15.0
