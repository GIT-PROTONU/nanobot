"""Offline tests for the ROS-free brain orchestration (behavior.brain): PurposeBrain (goal /
reward / A-B / beat-upgrade / reflection mode) + Personality (chart-context evolution). All state
IO is redirected to a tmp dir, so these never touch ~/.local/state.

Run: pixi run python -m pytest src/behavior/test"""
import random
import time

from behavior.brain import PurposeBrain, Personality, Schedule, load_personality, load_json
from behavior.presence import build_interpreter, DEFAULT_TRAITS


def make_brain(tmp_path, **kw):
    """A PurposeBrain wired to recording publish adapters + tmp state files."""
    pub = {"purpose": [], "task": [], "exp": []}
    defaults = dict(name="Nano", enable=True, rng=random.Random(0), epsilon=0.0,
                    pursue_min_interval=0.0, skills_enable=True, skill_every=3,
                    read_cog_log=lambda: [])
    defaults.update(kw)
    b = PurposeBrain(
        purpose_path=str(tmp_path / "purpose.json"),
        experiments_path=str(tmp_path / "experiments.json"),
        cog_log_path=str(tmp_path / "cognition.log"),
        publish_purpose=lambda o: pub["purpose"].append(o),
        publish_task=lambda p: pub["task"].append(p),
        publish_experiments=lambda s: pub["exp"].append(s),
        **defaults)
    return b, pub


# ---- PurposeBrain: beat upgrades ----------------------------------------------
def test_skill_beat_cadence(tmp_path):
    b, _ = make_brain(tmp_path, skill_every=3)
    assert [b.take_skill_beat() for _ in range(6)] == [False, False, True, False, False, True]


def test_skill_beat_paused_while_reflecting(tmp_path):
    b, _ = make_brain(tmp_path, skill_every=2)
    b.reflecting = True
    assert b.take_skill_beat() is False          # paused: no count advances
    b.reflecting = False
    assert [b.take_skill_beat() for _ in range(2)] == [False, True]   # counter started at 0


def test_skill_beat_off_when_disabled(tmp_path):
    b, _ = make_brain(tmp_path, skills_enable=False)
    assert all(b.take_skill_beat() is False for _ in range(5))


def test_next_pursuing_returns_spec_and_announces(tmp_path):
    b, pub = make_brain(tmp_path)
    spec = b.next_pursuing(now=100.0)
    assert spec is not None and spec["task"] == "get_acquainted"
    assert b.task["task"] == "get_acquainted"
    assert pub["task"] and pub["task"][-1]["task"] == "get_acquainted"
    assert pub["exp"]                            # moved A/B stats announced


def test_next_pursuing_none_when_reflecting_or_disabled(tmp_path):
    b, _ = make_brain(tmp_path)
    b.reflecting = True
    assert b.next_pursuing(now=100.0) is None
    off, _ = make_brain(tmp_path, enable=False)
    assert off.next_pursuing(now=100.0) is None
    assert off.summary() == {"experiments": {}}


def test_next_pursuing_blocked_by_world_precondition(tmp_path):
    # While held off the ground the observe task must not verify -> no pursuing beat.
    b, _ = make_brain(tmp_path, picked=lambda: True)
    assert b.world_state()["picked"] is True
    assert b.next_pursuing(now=100.0) is None


# ---- PurposeBrain: reward + reflection mode ----------------------------------------
def test_apply_reward_credits_last_arm(tmp_path):
    b, _ = make_brain(tmp_path)
    b.next_pursuing(now=100.0)                    # assigns an A/B arm
    assert b.apply_reward(1.0, None, scope="contextual") is True
    assert b.apply_reward(1.0, None, scope="global") is False    # global is reward-shaping only


def test_set_reflecting_consolidates(tmp_path):
    b, pub = make_brain(tmp_path)
    assert b.set_reflecting(True) is True         # changed
    assert b.set_reflecting(True) is False        # idempotent
    assert b.reflecting is True
    assert pub["purpose"]                         # forced reflection announced the purpose
    assert b.set_reflecting(False) is True


# ---- PurposeBrain: persistence ------------------------------------------------
def test_save_roundtrip(tmp_path):
    b, _ = make_brain(tmp_path)
    b.next_pursuing(now=100.0)
    b.apply_reward(1.0, None)
    b.save()
    assert load_json(str(tmp_path / "purpose.json")) is not None
    assert load_json(str(tmp_path / "experiments.json")) is not None
    # A fresh brain restores the persisted A/B stats.
    b2, _ = make_brain(tmp_path)
    assert b2.summary()["experiments"]


# ---- load_personality ---------------------------------------------------------
def test_load_personality_defaults(tmp_path):
    full = load_personality(str(tmp_path / "nope.json"), with_defaults=True)
    assert full["name"] == "Nano"
    assert set(full["traits"]) == set(DEFAULT_TRAITS)
    bare = load_personality(str(tmp_path / "nope.json"), with_defaults=False)
    assert bare["traits"] == {}


def test_load_personality_merges_file(tmp_path):
    p = tmp_path / "personality.json"
    p.write_text('{"name": "Pip", "traits": {"curiosity": 0.9}}', encoding="utf-8")
    data = load_personality(str(p), with_defaults=True)
    assert data["name"] == "Pip"
    assert data["traits"]["curiosity"] == 0.9
    assert data["traits"]["caution"] == DEFAULT_TRAITS["caution"]    # untouched default kept


# ---- Personality: chart-context evolution -------------------------------------
def build_personality(tmp_path, **kw):
    pub = []
    pers = Personality(path=str(tmp_path / "personality.json"),
                       publish=lambda s: pub.append(s), **kw)
    interp, _ = build_interpreter(face=lambda _m: None, alpha=1.0,
                                  traits={"caution": 0.6}, clock=None)
    pers.attach(interp)
    return pers, interp, pub


def test_personality_publishes_on_change(tmp_path):
    pers, _interp, pub = build_personality(tmp_path)
    pers.publish_and_persist(0.0)                 # initial snapshot differs from None
    assert pub and "traits" in pub[-1] and "registry" in pub[-1]


def test_personality_pickup_nudge_applied(tmp_path):
    pers, interp, _ = build_personality(tmp_path, nudge_pickup_caution=1.0,
                                        nudge_pickup_playful=0.3)
    assert pers.tick_events(0.0, picked=True) is None      # first pickup queues the nudge
    interp.execute()                                       # alpha=1.0 -> caution snaps to target
    assert interp.context["traits"]["caution"] == 1.0


def test_personality_heartbeat_reverts(tmp_path):
    pers, _interp, _ = build_personality(tmp_path, heartbeat_enable=True, brain_timeout=0.0)
    pers._last_brain = 0.0                                   # pin the "last alive" reference
    assert pers.tick_events(1.0, picked=False) == "lost"    # no evolve within timeout
    assert pers.tick_events(2.0, picked=False) is None      # only fires once


# ---- Schedule: scheduled routines (fire a named skill once a day at HH:MM) ----
def _lt(hour, minute, yday=191):
    return time.struct_time((2026, 7, 10, hour, minute, 0, 4, yday, 0))


def test_schedule_drops_malformed_and_unpaired_entries():
    logs = []
    s = Schedule([{"time": "09:00", "skill": "patrol"}, {"time": "bad", "skill": "x"},
                 {"time": "25:00", "skill": "y"}, {"time": "18:30", "skill": ""}],
                logger=logs.append)
    assert [e["skill"] for e in s._entries] == ["patrol"]    # 18:30 has no skill
    assert len(logs) == 3                                    # "bad", "25:00", unpaired "18:30"


def test_schedule_blank_entries_are_silently_ignored():
    logs = []
    s = Schedule([{"time": "", "skill": ""}], logger=logs.append)
    assert s._entries == []
    assert logs == []


def test_schedule_due_fires_once_then_waits_for_tomorrow():
    s = Schedule([{"time": "09:00", "skill": "patrol"}])
    assert s.due(_lt(8, 59)) is None                # not yet
    assert s.due(_lt(9, 0)) == "patrol"             # due
    assert s.due(_lt(9, 5)) is None                 # already fired today
    assert s.due(_lt(9, 0, yday=192)) == "patrol"   # fires again tomorrow


def test_schedule_due_returns_one_entry_per_call():
    s = Schedule([{"time": "09:00", "skill": "a"}, {"time": "09:01", "skill": "b"}])
    now = _lt(9, 5)
    first = s.due(now)
    second = s.due(now)
    assert {first, second} == {"a", "b"}      # both eventually due, one per call


def test_schedule_to_list_round_trips_normalized():
    s = Schedule([{"time": "9:5", "skill": "patrol"}])
    assert s.to_list() == [{"time": "09:05", "skill": "patrol"}]
