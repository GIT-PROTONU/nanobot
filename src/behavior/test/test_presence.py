"""Offline tests for the presence statechart — no ROS, no hardware.

Run on the board or the dev host:  pixi run python -m pytest src/behavior/test

Drives the chart with a manual clock and a recording `face` stub, so it proves the
"feel alive" behaviour (greeting -> rest -> periodic liveliness -> stand down/resume)
without rclpy. This is the safety-of-logic test the behaviour layer was meant to have.
"""
import os
import sys

# Make `behavior` importable whether or not the colcon overlay is sourced.
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import pytest

pytest.importorskip("sismic")  # skip cleanly if sismic isn't installed
from sismic.clock import SimulatedClock  # noqa: E402
from sismic.model import Event  # noqa: E402
from behavior.presence import build_interpreter  # noqa: E402

GREET, IDLE, PERFORM = 1.0, 2.0, 1.0


def _build():
    faces = []
    clock = SimulatedClock()
    interp, _ = build_interpreter(
        faces.append, greet_secs=GREET, idle_secs=IDLE, perform_secs=PERFORM,
        clock=clock)
    return interp, clock, faces


def _step(interp, clock, t):
    clock.time = t
    interp.execute()


def test_boot_greeting_shows_happy():
    interp, _clock, faces = _build()
    assert "greeting" in interp.configuration
    assert faces == ["happy"]            # boot "hello" face


def test_settles_to_dashboard_after_greeting():
    interp, clock, faces = _build()
    _step(interp, clock, GREET + 0.1)
    assert "resting" in interp.configuration
    assert faces[-1] == ""               # dashboard (face cleared)


def test_liveliness_beat_then_back_to_rest():
    interp, clock, faces = _build()
    _step(interp, clock, GREET + 0.1)            # -> resting
    rest_at = clock.time
    _step(interp, clock, rest_at + IDLE + 0.1)   # idle long enough -> performing
    assert "performing" in interp.configuration
    assert faces[-1] == "happy"
    perform_at = clock.time
    _step(interp, clock, perform_at + PERFORM + 0.1)  # beat over -> resting
    assert "resting" in interp.configuration
    assert faces[-1] == ""


def test_standdown_yields_and_resume_returns():
    interp, clock, faces = _build()
    _step(interp, clock, GREET + 0.1)            # -> resting
    n = len(faces)
    interp.queue(Event("standdown"))
    interp.execute()
    assert "dormant" in interp.configuration
    assert len(faces) == n               # dormant must not touch the panel: no new face

    interp.queue(Event("resume"))
    interp.execute()
    assert "resting" in interp.configuration
    assert faces[-1] == ""               # resume hands the panel back to the dashboard


def test_standdown_during_liveliness_is_preempted():
    interp, clock, faces = _build()
    _step(interp, clock, GREET + 0.1)
    _step(interp, clock, clock.time + IDLE + 0.1)   # performing (happy)
    assert "performing" in interp.configuration
    interp.queue(Event("standdown"))
    interp.execute()
    assert "dormant" in interp.configuration        # parent transition preempts the beat
