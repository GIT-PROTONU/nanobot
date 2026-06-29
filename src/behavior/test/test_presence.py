"""Offline tests for the presence statechart — no ROS, no hardware.

Run on the board or the dev host:  pixi run python -m pytest src/behavior/test

Two layers are tested:
  * the **chart** — driven with a manual clock + recording `face`/`do_beat` stubs — proves the
    deterministic reflexes (greeting -> rest -> stand down/resume, reflect pause) and that each
    idle cycle fires exactly one chosen beat;
  * the **chooser** (`choose_beat`, pure) — proves the priority-weighted, novelty-aware,
    trait-gated idle-beat lottery that makes the behaviour dynamic + self-learning.

No rclpy or network either way.
"""
import os
import random
import sys

# Make `behavior` importable whether or not the colcon overlay is sourced.
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import pytest

pytest.importorskip("sismic")  # skip cleanly if sismic isn't installed
from sismic.clock import SimulatedClock  # noqa: E402
from sismic.model import Event  # noqa: E402
from behavior.presence import (build_interpreter, choose_beat, BEATS,  # noqa: E402
                               DEFAULT_TRAITS, DEFAULT_REGISTRY)

GREET, IDLE, PERFORM = 1.0, 2.0, 1.0


def _build(camera_beats=True, traits=None, registry=None, alpha=0.1, seed=0):
    faces, beats = [], []
    clock = SimulatedClock()
    interp, _ = build_interpreter(
        faces.append, do_beat=beats.append, greet_secs=GREET, idle_secs=IDLE,
        perform_secs=PERFORM, camera_beats=camera_beats,
        traits=traits, registry=registry, alpha=alpha, clock=clock,
        rng=random.Random(seed))
    return interp, clock, faces, beats


def _step(interp, clock, t):
    clock.time = t
    interp.execute()


def _run_cycles(interp, clock, n):
    """Advance through n idle->perform->rest cycles (default extraversion -> cadence 1.0)."""
    t = clock.time
    for _ in range(n):
        t += IDLE + 0.1
        _step(interp, clock, t)                  # resting -> performing (fires do_beat)
        t += PERFORM + 0.1
        _step(interp, clock, t)                  # performing -> resting


def _reg(**overrides):
    """A full registry = the defaults with per-beat patches merged on top."""
    out = {n: dict(cfg) for n, cfg in DEFAULT_REGISTRY.items()}
    for name, patch in overrides.items():
        out.setdefault(name, {}).update(patch)
    return out


# ---- chart reflexes ---------------------------------------------------------

def test_boot_greeting_shows_happy():
    interp, _clock, faces, _beats = _build()
    assert "greeting" in interp.configuration
    assert faces == ["happy"]            # boot "hello" face


def test_settles_to_dashboard_after_greeting():
    interp, clock, faces, _beats = _build()
    _step(interp, clock, GREET + 0.1)
    assert "resting" in interp.configuration
    assert faces[-1] == ""               # dashboard (face cleared)


def test_beats_table_has_all_kinds():
    # The discretionary chart beats + the two node-side upgrades (pursuing/skill).
    assert set(BEATS) == {"musing", "looking", "wondering", "listening", "pursuing", "skill"}


def test_each_cycle_fires_one_enabled_beat():
    interp, clock, _faces, beats = _build()
    _step(interp, clock, GREET + 0.1)            # -> resting
    _run_cycles(interp, clock, 8)
    assert len(beats) == 8                        # exactly one beat per cycle
    assert all(b in ("musing", "looking", "wondering", "listening") for b in beats)
    assert "musing" in beats                      # the highest-priority default shows up


def test_camera_beats_off_never_looks():
    interp, clock, _faces, beats = _build(camera_beats=False)
    _step(interp, clock, GREET + 0.1)
    _run_cycles(interp, clock, 12)
    assert beats and "looking" not in beats       # no autonomous camera when disabled


def test_reflect_pauses_beats_until_wake():
    interp, clock, faces, beats = _build()
    _step(interp, clock, GREET + 0.1)            # -> resting
    interp.queue(Event("reflect"))
    _step(interp, clock, GREET + 0.2)
    assert "reflecting" in interp.configuration
    assert faces[-1] == "focused"                # the calm reflect face
    before = list(beats)
    _run_cycles(interp, clock, 3)                # no beats should fire while reflecting
    assert beats == before
    interp.queue(Event("wake"))
    _step(interp, clock, clock.time + 0.1)
    assert "resting" in interp.configuration     # back to normal presence


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
    _step(interp, clock, clock.time + IDLE + 0.1)    # -> performing (a beat)
    assert "performing" in interp.configuration
    interp.queue(Event("standdown"))
    interp.execute()
    assert "dormant" in interp.configuration          # parent transition preempts the beat


# ---- chooser: the dynamic, self-learning idle-beat lottery ------------------

def test_choose_only_eligible_when_others_disabled():
    reg = _reg(looking={"enabled": False}, wondering={"enabled": False},
               listening={"enabled": False})
    rng = random.Random(0)
    assert all(choose_beat(DEFAULT_TRAITS, reg, rng) == "musing" for _ in range(20))


def test_choose_empty_when_all_disabled():
    reg = _reg(musing={"enabled": False}, looking={"enabled": False},
               wondering={"enabled": False}, listening={"enabled": False})
    assert choose_beat(DEFAULT_TRAITS, reg, random.Random(0)) == ""


def test_choose_camera_off_excludes_looking():
    got = {choose_beat(DEFAULT_TRAITS, DEFAULT_REGISTRY, random.Random(s), camera_beats=False)
           for s in range(60)}
    assert "looking" not in got


def test_choose_low_curiosity_gates_looking_and_wondering():
    traits = {**DEFAULT_TRAITS, "curiosity": 0.1}
    got = {choose_beat(traits, DEFAULT_REGISTRY, random.Random(s)) for s in range(80)}
    assert "looking" not in got and "wondering" not in got


def test_choose_low_extraversion_gates_listening():
    traits = {**DEFAULT_TRAITS, "extraversion": 0.1}
    got = {choose_beat(traits, DEFAULT_REGISTRY, random.Random(s)) for s in range(80)}
    assert "listening" not in got


def test_choose_priority_biases_distribution():
    # Higher base priority should win the lottery more often (the learnable lever).
    reg = {"musing": {"priority": 0.9, "enabled": True},
           "wondering": {"priority": 0.1, "enabled": True}}
    draws = [choose_beat(DEFAULT_TRAITS, reg, random.Random(s)) for s in range(300)]
    assert draws.count("musing") > 3 * draws.count("wondering")


def test_choose_novelty_downweights_last():
    # Two equally-weighted beats: the one that just ran is penalised, so the OTHER dominates.
    reg = {"musing": {"priority": 1.0, "enabled": True},
           "wondering": {"priority": 1.0, "enabled": True}}
    draws = [choose_beat(DEFAULT_TRAITS, reg, random.Random(s), last="musing")
             for s in range(200)]
    assert draws.count("wondering") > draws.count("musing")


def test_choose_trait_scales_weight():
    # `looking` is trait-scaled by curiosity; a very curious robot picks it far more than a
    # barely-curious one (same seeds, only the trait differs).
    hi = {**DEFAULT_TRAITS, "curiosity": 1.0}
    lo = {**DEFAULT_TRAITS, "curiosity": 0.35}        # still above the 0.3 needs gate
    n_hi = sum(choose_beat(hi, DEFAULT_REGISTRY, random.Random(s)) == "looking"
               for s in range(300))
    n_lo = sum(choose_beat(lo, DEFAULT_REGISTRY, random.Random(s)) == "looking"
               for s in range(300))
    assert n_hi > n_lo


# ---- personality / evolution ----

def test_evolve_smooths_toward_target():
    interp, _clk, _f, _b = _build(alpha=0.5)              # caution starts at default 0.6
    interp.queue(Event("evolve", traits={"caution": 1.0}, registry={}))
    interp.execute()
    assert abs(interp.context["traits"]["caution"] - 0.8) < 1e-6   # 0.6 -> 0.8
    interp.queue(Event("evolve", traits={"caution": 1.0}, registry={}))
    interp.execute()
    assert abs(interp.context["traits"]["caution"] - 0.9) < 1e-6   # 0.8 -> 0.9


def test_evolve_can_relearn_beat_priority():
    # LLM reflection nudges a beat's priority via the registry patch -> the live registry moves,
    # so the idle mix is genuinely learnable.
    interp, _clk, _f, _b = _build()
    interp.queue(Event("evolve", traits={}, registry={"wondering": {"priority": 0.95}}))
    interp.execute()
    assert interp.context["registry"]["wondering"]["priority"] == 0.95


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
