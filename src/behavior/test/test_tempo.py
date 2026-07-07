"""Offline tests for the chart's time-of-day tempo (ROS-free):

  - `tempo()` (a live callable injected by the node) multiplies the idle cadence,
    so a night factor of N makes beats fire N x farther apart;
  - it's re-read on every guard evaluation, so flipping day<->night takes effect
    on the NEXT cycle without rebuilding the interpreter;
  - omitted, the cadence is exactly the pre-tempo behaviour (factor 1.0).

    pixi run python -m pytest src/behavior/test
"""
import random

import pytest

pytest.importorskip("sismic")
from sismic.clock import SimulatedClock  # noqa: E402
from behavior.presence import build_interpreter  # noqa: E402

GREET, IDLE, PERFORM = 1.0, 2.0, 1.0


def _build(tempo=None, seed=0):
    faces, beats = [], []
    clock = SimulatedClock()
    interp, _ = build_interpreter(
        faces.append, do_beat=beats.append, greet_secs=GREET, idle_secs=IDLE,
        perform_secs=PERFORM, clock=clock, rng=random.Random(seed), tempo=tempo)
    return interp, clock, beats


def _step(interp, clock, t):
    clock.time = t
    interp.execute()


def test_default_cadence_is_unchanged_without_tempo():
    interp, clock, beats = _build()
    _step(interp, clock, GREET + 0.1)                 # -> resting
    _step(interp, clock, GREET + IDLE + 0.2)          # one idle period later
    assert beats, "beat should fire at the plain idle cadence"


def test_night_tempo_stretches_the_idle_cadence():
    interp, clock, beats = _build(tempo=lambda: 3.0)
    _step(interp, clock, GREET + 0.1)                 # -> resting
    _step(interp, clock, GREET + IDLE + 0.2)          # 1x idle: NOT yet at 3x tempo
    assert not beats
    _step(interp, clock, GREET + 3 * IDLE + 0.3)      # 3x idle: now it fires
    assert beats


def test_tempo_is_live_day_night_flip():
    """The callable is re-read each evaluation: leaving 'night' restores the day cadence
    on the next cycle without rebuilding the chart."""
    night = {"on": True}
    interp, clock, beats = _build(tempo=lambda: 4.0 if night["on"] else 1.0)
    _step(interp, clock, GREET + 0.1)                 # -> resting (night: 4x cadence)
    _step(interp, clock, GREET + IDLE + 0.2)
    assert not beats                                  # too early for night tempo
    night["on"] = False                               # morning comes
    _step(interp, clock, GREET + IDLE + 0.4)
    assert beats                                      # day cadence applies immediately
