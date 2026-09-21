"""clock_step detector (sys_monitor's NTP clock-step watcher) — ROS-free unit tests.

The detector guards the no-RTC clock-step failure that wrecked live NAV on
2026-09-21 (future-dated TF stamps -> failed pose lookups -> Nav2 "collision
ahead" -> the recovery-spin loop). See the pure `clock_step` in health_log.py.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import pytest  # noqa: E402

from sys_monitor.health_log import (  # noqa: E402
    clock_step, CLOCK_STEP_THRESH,
)


def test_first_tick_never_flags():
    # prev_drift None = seeding tick — cannot know a step from one sample
    assert clock_step(None, 1234.0) is False


def test_stable_clock_no_step():
    # epoch-minus-monotonic constant across ticks (typical NTP-synced state)
    assert clock_step(500.0, 500.0) is False
    assert clock_step(500.0, 500.001) is False


def test_small_jitter_under_threshold_no_step():
    # normal scheduler/NTP-sleek jitter must not trip the watcher
    assert clock_step(500.0, 501.0) is False        # 1 s < thresh 2.0


def test_forward_step_flags():
    assert clock_step(500.0, 502.5) is True


def test_backward_step_flags():
    # a backward correction (the 2026-09-21 pm case: future-dated TF stamps)
    assert clock_step(500.0, 497.0) is True


def test_threshold_boundary():
    t = CLOCK_STEP_THRESH
    assert clock_step(0.0, t) is False              # exactly at thresh = no
    assert clock_step(0.0, t + 0.01) is True


def test_drift_none_after_step_seeds_false():
    assert clock_step(500.0, None) is False
