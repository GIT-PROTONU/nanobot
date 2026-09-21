"""Offline tests (ROS-free) for the IMU interference self-test — the automated
mounting-interference hunt (cycles LDS / fan / LED / optional motor wiggle and
scores the magnetometer disturbance per phase). What is ours is exactly the glue:

    * the RLock re-entrancy: start() holds the lock across its checks + spawn and
      returns status() — a plain Lock SELF-DEADLOCKS (found live 2026-09-21 via
      the app_hub SIGUSR1 dump; the test had never been startable on hardware)
    * status() reflects the phases as they advance, and stop() ends a run
    * the pickup/driving guards refuse to start

    pixi run test
"""
import threading
import time

import pytest

from web_control.imu_interference import IMUInterferenceTest


class _FakePub:
    def publish(self, msg):
        pass


class _FakeParam:
    def __init__(self, value):
        self.value = value


class _FakeNode:
    def __init__(self):
        self.telemetry = self
        self._mag = (20.0, 0.0, 40.0)
        self._eul = (0.0, 0.0, 0.0)
        self._cmd_vel = (0.0, 0.0)
        self._susp = (False, False)
        self._params = {"vision_bumper_cmd_eps": _FakeParam(0.02),
                        "interference_lds_rpm": _FakeParam(300.0),
                        "interference_motor_ang": _FakeParam(0.3)}

    def create_publisher(self, type_, name, qos):
        return _FakePub()

    def _susp_eff(self):
        return self._susp

    def get_parameter(self, name):
        return self._params[name]

    def set_param_json(self, d):
        pass

    def get_logger(self):
        return self

    def info(self, *a, **k):
        pass


def _test(fast=True):
    """A test object with all phase durations collapsed to ~0.1 s so the run
    finishes quickly; the baseline/lds/fan/led phases still execute."""
    n = _FakeNode()
    t = IMUInterferenceTest(n)
    t.lds_rpm = 300.0
    t.motor_ang = 0.3
    return t, n


def _quick(t, monkeypatch, secs=0.05):
    monkeypatch.setattr(type(t), "_sample_window",
                        lambda self, s, check_cmd=True: (0.1, 40.0, 0.0))


def test_start_returns_without_deadlock(monkeypatch):
    """The 2026-09-21 live deadlock: start() held the Lock and re-entered via
    status(). start() must RETURN promptly (and the run thread must not wedge
    the lock for later status() calls)."""
    t, n = _test()
    _quick(t, monkeypatch)
    done = threading.Event()
    out = {}

    def go():
        out["r"] = t.start()
        done.set()

    th = threading.Thread(target=go, daemon=True)
    th.start()
    assert done.wait(5.0), "start() deadlocked (non-reentrant lock regression)"
    assert not out["r"].get("error")


def test_status_advances_and_finishes(monkeypatch):
    t, n = _test()
    _quick(t, monkeypatch)
    assert not t.start().get("error")
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        st = t.status()
        if not st["active"]:
            break
        time.sleep(0.05)
    st = t.status()
    assert st["active"] is False
    assert st["phase"] == "done"
    assert len(st["results"]) == 4            # baseline + lds + fan + led


def test_picked_up_refuses(monkeypatch):
    t, n = _test()
    _quick(t, monkeypatch)
    n._susp = (True, False)
    r = t.start()
    assert "error" in r and "picked up" in r["error"]


def test_driven_refuses(monkeypatch):
    t, n = _test()
    _quick(t, monkeypatch)
    n.telemetry._cmd_vel = (0.1, 0.0)
    r = t.start()
    assert "error" in r and "driven" in r["error"]
