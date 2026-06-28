"""Offline tests for the presence statechart — no ROS, no hardware.

Run on the board or the dev host:  pixi run python -m pytest src/behavior/test

Drives the chart with a manual clock + recording `face`/`do_beat` stubs, so it proves
the deterministic baseline (greeting -> rest -> stand down/resume) AND the beat rotation
(sensor `musing` every cycle, camera `looking` every look_every-th) without rclpy or any
network. The LLM enrichment is layered on top of these beats by the node; the chart only
fires `do_beat(name)` — exactly what this test records.
"""
import os
import sys

# Make `behavior` importable whether or not the colcon overlay is sourced.
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import pytest

pytest.importorskip("sismic")  # skip cleanly if sismic isn't installed
from sismic.clock import SimulatedClock  # noqa: E402
from sismic.model import Event  # noqa: E402
from behavior.presence import build_interpreter, BEATS, DEFAULT_TRAITS  # noqa: E402

GREET, IDLE, PERFORM = 1.0, 2.0, 1.0


def _build(camera_beats=True, look_every=4, traits=None, registry=None, alpha=0.1):
    faces, beats = [], []
    clock = SimulatedClock()
    interp, _ = build_interpreter(
        faces.append, do_beat=beats.append, greet_secs=GREET, idle_secs=IDLE,
        perform_secs=PERFORM, camera_beats=camera_beats, look_every=look_every,
        traits=traits, registry=registry, alpha=alpha, clock=clock)
    return interp, clock, faces, beats


def _step(interp, clock, t):
    clock.time = t
    interp.execute()


def _run_cycles(interp, clock, n):
    """Advance through n idle->beat->rest cycles; return the beat-state entered each (or
    '(none)' when nothing was eligible). Assumes default extraversion (cadence factor 1.0)."""
    seq, t = [], clock.time
    for _ in range(n):
        t += IDLE + 0.1
        _step(interp, clock, t)                  # resting -> a beat (or a skip)
        leaf = [s for s in interp.configuration if s in ("musing", "looking")]
        seq.append(leaf[0] if leaf else "(none)")
        t += PERFORM + 0.1
        _step(interp, clock, t)                  # beat -> resting
    return seq


def test_boot_greeting_shows_happy():
    interp, _clock, faces, _beats = _build()
    assert "greeting" in interp.configuration
    assert faces == ["happy"]            # boot "hello" face


def test_settles_to_dashboard_after_greeting():
    interp, clock, faces, _beats = _build()
    _step(interp, clock, GREET + 0.1)
    assert "resting" in interp.configuration
    assert faces[-1] == ""               # dashboard (face cleared)


def test_beats_rotate_sensor_then_camera():
    interp, clock, _faces, beats = _build(look_every=4)
    _step(interp, clock, GREET + 0.1)            # -> resting
    seq = _run_cycles(interp, clock, 4)
    assert seq == ["musing", "musing", "musing", "looking"]
    assert beats == seq                          # do_beat fired with the right names
    # The chart rotates musing/looking; `pursuing` is a node-side upgrade of the musing
    # beat (delivered by mood_node when the planner has a task), not a separate chart state.
    assert set(BEATS) == {"musing", "looking", "pursuing"}


def test_meditate_pauses_beats_until_wake():
    interp, clock, faces, beats = _build(look_every=4)
    _step(interp, clock, GREET + 0.1)            # -> resting
    interp.queue(Event("meditate"))
    _step(interp, clock, GREET + 0.2)
    assert "meditating" in interp.configuration
    assert faces[-1] == "focused"                # the calm meditate face
    before = list(beats)
    _run_cycles(interp, clock, 3)                # no beats should fire while meditating
    assert beats == before
    interp.queue(Event("wake"))
    _step(interp, clock, clock.time + 0.1)
    assert "resting" in interp.configuration     # back to normal presence


def test_camera_beats_off_only_musing():
    interp, clock, _faces, beats = _build(camera_beats=False, look_every=4)
    _step(interp, clock, GREET + 0.1)
    _run_cycles(interp, clock, 5)
    assert beats == ["musing"] * 5               # no autonomous camera when disabled


def test_standdown_yields_and_resume_returns():
    interp, clock, faces, _beats = _build()
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


def test_standdown_during_beat_is_preempted():
    interp, clock, _faces, _beats = _build()
    _step(interp, clock, GREET + 0.1)
    _step(interp, clock, clock.time + IDLE + 0.1)   # a beat (musing/looking)
    assert "musing" in interp.configuration or "looking" in interp.configuration
    interp.queue(Event("standdown"))
    interp.execute()
    assert "dormant" in interp.configuration         # parent transition preempts the beat


# ---- personality / evolution ----

def test_low_curiosity_gates_out_looking():
    # curiosity below the looking 'needs' gate -> the camera beat never fires.
    interp, clock, _f, _b = _build(traits={"curiosity": 0.1})
    _step(interp, clock, GREET + 0.1)
    assert _run_cycles(interp, clock, 4) == ["musing"] * 4


def test_registry_disabled_musing_still_reaches_looking():
    # musing off: the skip self-transition keeps the counter advancing so the 4th cycle
    # still reaches the (enabled) looking beat.
    interp, clock, _f, _b = _build(registry={"musing": {"enabled": False}})
    _step(interp, clock, GREET + 0.1)
    seq = _run_cycles(interp, clock, 4)
    assert seq.count("musing") == 0 and seq[-1] == "looking"


def test_evolve_smooths_toward_target():
    interp, _clk, _f, _b = _build(alpha=0.5)              # caution starts at default 0.6
    interp.queue(Event("evolve", traits={"caution": 1.0}, registry={}))
    interp.execute()
    assert abs(interp.context["traits"]["caution"] - 0.8) < 1e-6   # 0.6 -> 0.8
    interp.queue(Event("evolve", traits={"caution": 1.0}, registry={}))
    interp.execute()
    assert abs(interp.context["traits"]["caution"] - 0.9) < 1e-6   # 0.8 -> 0.9


def test_brain_lost_reverts_to_seeded_baseline():
    interp, _clk, _f, _b = _build(traits={"curiosity": 0.9, "caution": 0.2}, alpha=0.5)
    interp.queue(Event("evolve", traits={"caution": 1.0},
                       registry={"looking": {"enabled": False}}))
    interp.execute()
    assert interp.context["registry"]["looking"]["enabled"] is False
    interp.queue(Event("brain_lost"))                    # cognitive layer unreachable
    interp.execute()
    assert interp.context["traits"]["curiosity"] == 0.9  # back to the SEEDED baseline,
    assert interp.context["traits"]["caution"] == 0.2    # not the generic defaults
    assert interp.context["registry"]["looking"]["enabled"] is True
