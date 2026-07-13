"""Offline tests for the vision-driven behaviour reflexes (ROS-free):

    pixi run python -m pytest src/behavior/test

Covers the beat-boost hook in choose_beat/pick_beat (novelty -> a likelier "looking"
beat), the ambient-mood tint (feel_face / decide_next), and Personality's vision fast
rules (looming startle + the clutter caution hold-and-release that doubles as the
velocity throttle when slam_nav's trait_motion clamp is on).
"""
import random

from behavior.brain import Personality
from behavior.presence import build_interpreter, choose_beat, DEFAULT_REGISTRY, DEFAULT_TRAITS


# ---- choose_beat: transient boosts --------------------------------------------
def test_boost_zero_excludes_a_beat():
    rng = random.Random(0)
    for _ in range(50):
        name = choose_beat(dict(DEFAULT_TRAITS), DEFAULT_REGISTRY, rng,
                           boosts={"musing": 0.0, "wondering": 0.0, "listening": 0.0})
        assert name == "looking"                    # the only beat left with weight


def test_boost_scales_up_a_beat():
    rng = random.Random(0)
    picks = [choose_beat(dict(DEFAULT_TRAITS), DEFAULT_REGISTRY, rng,
                         boosts={"looking": 1000.0})
             for _ in range(50)]
    assert picks.count("looking") > 45              # overwhelmingly favoured


def test_bad_boost_values_are_ignored():
    rng = random.Random(0)
    name = choose_beat(dict(DEFAULT_TRAITS), DEFAULT_REGISTRY, rng,
                       boosts={"musing": "junk"})
    assert name in DEFAULT_REGISTRY


# ---- ambient mood: the feeling-state tint --------------------------------------
def test_feel_face_prefers_llm_mood_over_ambient():
    ambient = {"face": "happy"}
    interp, _ = build_interpreter(face=lambda _m: None, clock=None,
                                  ambient_mood=lambda: ambient["face"])
    ctx = interp.context
    assert ctx["feel_face"]() == "happy"            # no LLM mood -> the ambient tint
    ctx["drives"]["mood"] = "focused"
    assert ctx["feel_face"]() == "focused"          # a deliberate LLM mood wins
    ctx["drives"]["mood"] = ""
    ambient["face"] = ""
    assert ctx["feel_face"]() == ""                 # neither -> plain dashboard return


def test_ambient_mood_triggers_the_feel_step():
    interp, _ = build_interpreter(face=lambda _m: None, clock=None,
                                  ambient_mood=lambda: "happy")
    ctx = interp.context
    assert ctx["drives"]["mood"] == ""              # the LLM has set nothing
    ctx["decide_next"]()                            # energy at 0.5 -> never a burst
    assert ctx["next_step"]() == "feel"             # ambient alone brings out `feeling`


def test_no_ambient_no_mood_rests():
    interp, _ = build_interpreter(face=lambda _m: None, clock=None)
    ctx = interp.context
    ctx["decide_next"]()
    assert ctx["next_step"]() == "rest"


# ---- Personality: vision fast rules --------------------------------------------
def build_personality(tmp_path, **kw):
    pers = Personality(path=str(tmp_path / "personality.json"),
                       publish=lambda _s: None, heartbeat_enable=False, **kw)
    interp, _ = build_interpreter(face=lambda _m: None, alpha=1.0,
                                  traits={"caution": 0.6}, clock=None)
    pers.attach(interp)
    return pers, interp


def test_looming_startle_is_edge_triggered(tmp_path):
    pers, interp = build_personality(tmp_path, nudge_looming_caution=1.0)
    pers.tick_events(0.0, picked=False, vision={"looming": True})
    interp.execute()                                 # alpha=1.0 -> caution snaps to target
    assert interp.context["traits"]["caution"] == 1.0
    # Still looming -> no re-queue (edge, not level); manually lower and confirm.
    interp.context["traits"]["caution"] = 0.6
    pers.tick_events(1.0, picked=False, vision={"looming": True})
    interp.execute()
    assert interp.context["traits"]["caution"] == 0.6


def test_clutter_holds_then_releases_caution(tmp_path):
    pers, interp = build_personality(tmp_path, clutter_caution=0.9,
                                     vision_rule_period=0.0)
    pers.tick_events(0.0, picked=False, vision={"clutter": True})
    interp.execute()
    assert interp.context["traits"]["caution"] == 0.9      # held at the wary target
    assert pers._clutter_baseline == 0.6                    # pre-clutter value remembered
    pers.tick_events(1.0, picked=False, vision={"clutter": False})
    interp.execute()
    assert interp.context["traits"]["caution"] == 0.6      # released back
    pers.tick_events(2.0, picked=False, vision={"clutter": False})
    assert pers._clutter_baseline is None                   # release complete


def test_clutter_never_lowers_an_already_wary_robot(tmp_path):
    pers, interp = build_personality(tmp_path, clutter_caution=0.7,
                                     vision_rule_period=0.0)
    interp.context["traits"]["caution"] = 0.95              # already warier than the target
    pers.tick_events(0.0, picked=False, vision={"clutter": True})
    interp.execute()
    assert interp.context["traits"]["caution"] == 0.95     # max(baseline, target) held


def test_vision_rules_can_be_disabled(tmp_path):
    pers, interp = build_personality(tmp_path, vision_caution_enable=False,
                                     nudge_looming_caution=1.0, vision_rule_period=0.0)
    pers.tick_events(0.0, picked=False, vision={"looming": True, "clutter": True})
    interp.execute()
    assert interp.context["traits"]["caution"] == 0.6      # untouched


def test_stale_vision_is_a_noop(tmp_path):
    pers, interp = build_personality(tmp_path, vision_rule_period=0.0)
    pers.tick_events(0.0, picked=False, vision=None)        # camera off / stale feed
    interp.execute()
    assert interp.context["traits"]["caution"] == 0.6
