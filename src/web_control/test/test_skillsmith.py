"""Offline unit tests for the skill-workshop core (ROS-free, no LLM):

    pixi run python -m pytest src/web_control/test

Covers the deterministic pieces of reflection mode's skill-synthesis loop: candidate render +
round-trip, the validation gate, and the trial ledger's adopt/retire decisions.
"""
import os
import time

from web_control.skills import parse_skill_text
from web_control.skillsmith import (ADOPTED, RETIRED, TRIAL, WorkshopState,
                                    render_skill_md, validate_candidate)


# ---- render + round-trip ----------------------------------------------------
def test_render_roundtrips_through_parser():
    spec = {"mode": "new", "name": "Greet By Name",
            "description": "Greet a person warmly.", "trigger": "when someone appears",
            "action": {"kind": "observe", "sources": ["sensors"]},
            "body": "Say a warm hello using what the body feels."}
    text = render_skill_md(spec)
    sk = parse_skill_text(text)
    assert sk is not None
    assert sk.name == "greet-by-name"
    assert sk.kind == "observe"
    assert "sensors" in sk.sources
    assert sk.description == "Greet a person warmly."


def test_render_forces_action_skill_disabled():
    spec = {"name": "spin-up", "action": {"kind": "topic", "topic": "/lds_target_rpm",
            "enabled": True}, "body": "Spin the lidar."}
    sk = parse_skill_text(render_skill_md(spec))
    assert sk.is_action
    assert sk.enabled is False          # born gated regardless of what the model asked for


def test_render_no_name_is_empty():
    assert render_skill_md({"name": "  !!  ", "body": "x"}) == ""


# ---- validation -------------------------------------------------------------
def test_validate_new_ok():
    ok, why = validate_candidate(
        {"mode": "new", "name": "weather-quip", "action": {"kind": "say"},
         "body": "Quip about the room."}, existing_names=["read-lidar"])
    assert ok, why


def test_validate_new_collision():
    ok, why = validate_candidate(
        {"mode": "new", "name": "read-lidar", "action": {"kind": "say"}},
        existing_names=["read-lidar"])
    assert not ok and "exists" in why


def test_validate_adapt_requires_existing_target():
    ok, why = validate_candidate(
        {"mode": "adapt", "name": "read-lidar-v2", "target": "ghost",
         "action": {"kind": "observe"}}, existing_names=["read-lidar"])
    assert not ok and "target" in why


def test_validate_adapt_must_use_fresh_name():
    # adapting must produce a NEW variant name, so a rollback never clobbers the parent
    ok, why = validate_candidate(
        {"mode": "adapt", "name": "read-lidar", "target": "read-lidar",
         "action": {"kind": "observe"}}, existing_names=["read-lidar"])
    assert not ok and "exists" in why


def test_validate_rejects_action_when_disabled():
    spec = {"mode": "new", "name": "go-forward",
            "action": {"kind": "topic", "topic": "/cmd_vel"}}
    ok, _ = validate_candidate(spec, existing_names=[], allow_actions=False)
    assert not ok
    ok, _ = validate_candidate(spec, existing_names=[], allow_actions=True)
    assert ok


def test_validate_unknown_kind():
    ok, why = validate_candidate(
        {"mode": "new", "name": "x", "action": {"kind": "teleport"}}, existing_names=[])
    assert not ok and "kind" in why


# ---- the ledger / gate ------------------------------------------------------
def _state(tmp_path, **kw):
    return WorkshopState(os.path.join(str(tmp_path), "workshop.json"), **kw)


def test_track_and_persist(tmp_path):
    st = _state(tmp_path)
    st.track("quip", origin="new", rationale="filled a gap")
    assert st.is_trial("quip")
    # a fresh instance reads it back from disk
    st2 = _state(tmp_path)
    assert st2.status_of("quip") == TRIAL
    assert st2.skills["quip"]["origin"] == "new"


def test_gate_adopts_on_good_evidence(tmp_path):
    st = _state(tmp_path, min_runs=3)
    st.track("quip")
    for _ in range(3):
        st.record_run("quip", ok=True)
    st.record_reward("quip", 1.0)
    assert st.gate("quip") == "adopt"


def test_gate_holds_without_enough_runs(tmp_path):
    st = _state(tmp_path, min_runs=3)
    st.track("quip")
    st.record_run("quip", ok=True)
    st.record_reward("quip", 1.0)
    assert st.gate("quip") is None


def test_gate_retires_on_errors(tmp_path):
    st = _state(tmp_path, retire_errors=2)
    st.track("quip")
    st.record_run("quip", ok=False)
    st.record_run("quip", ok=False)
    assert st.gate("quip") == "retire"


def test_gate_retires_on_negative_reward(tmp_path):
    st = _state(tmp_path, retire_net_neg=2)
    st.track("quip")
    st.record_run("quip", ok=True)
    st.record_reward("quip", -1.0)
    st.record_reward("quip", -1.0)
    assert st.gate("quip") == "retire"


def test_positive_reward_outweighs_one_thumbsdown(tmp_path):
    st = _state(tmp_path, min_runs=2, retire_net_neg=2)
    st.track("quip")
    st.record_run("quip", ok=True)
    st.record_run("quip", ok=True)
    st.record_reward("quip", 1.0)
    st.record_reward("quip", 1.0)
    st.record_reward("quip", -1.0)
    assert st.gate("quip") == "adopt"


def test_gate_quiet_adopts_without_any_reward(tmp_path):
    # An autonomous robot rarely gets an explicit 👍: a clean, uncomplained-about track record
    # of enough runs must adopt on its own, or trials would pile up forever.
    st = _state(tmp_path, min_runs=3, adopt_quiet_runs=5)
    st.track("quip")
    for _ in range(4):
        st.record_run("quip", ok=True)
    assert st.gate("quip") is None                  # clean but not enough runs yet
    st.record_run("quip", ok=True)
    assert st.gate("quip") == "adopt"               # 5 clean runs, no 👎 -> graduates


def test_quiet_adopt_blocked_by_a_thumbsdown(tmp_path):
    st = _state(tmp_path, min_runs=3, adopt_quiet_runs=4, retire_net_neg=2)
    st.track("quip")
    for _ in range(5):
        st.record_run("quip", ok=True)
    st.record_reward("quip", -1.0)                  # one complaint
    assert st.gate("quip") is None                  # not adopted (neg>0) and not yet net-retire


def test_quiet_runs_floor_is_min_runs(tmp_path):
    st = _state(tmp_path, min_runs=6, adopt_quiet_runs=3)
    assert st.adopt_quiet_runs == 6                 # can't quiet-adopt below min_runs


def test_gate_retires_stale_trial_past_ttl(tmp_path):
    st = _state(tmp_path, trial_ttl=100.0)
    st.track("dud")
    st.skills["dud"]["created"] = time.time() - 200.0   # aged past the TTL
    assert st.gate("dud") == "retire"
    # a fresh trial with no TTL configured never goes stale
    st2 = _state(tmp_path, trial_ttl=0.0)
    st2.track("ok")
    st2.skills["ok"]["created"] = time.time() - 10 ** 7
    assert st2.gate("ok") is None


def test_due_trials_least_run_first(tmp_path):
    st = _state(tmp_path, adopt_quiet_runs=5)
    st.track("a")
    st.track("b")
    st.track("c")
    for _ in range(2):
        st.record_run("a", ok=True)
    st.record_run("b", ok=True)
    for _ in range(5):
        st.record_run("c", ok=True)                 # c has hit the bar -> not "due"
    assert st.due_trials() == ["b", "a"]            # fewest runs first, c excluded


def test_manual_keep_and_kill(tmp_path):
    st = _state(tmp_path)
    st.track("a")
    st.track("b")
    assert st.keep("a") and st.status_of("a") == ADOPTED
    assert st.kill("b") and st.status_of("b") == RETIRED
    # non-trial records are inert to run/reward tracking + the gate
    assert st.record_run("a") is False
    assert st.gate("a") is None


def test_gate_all_and_forget(tmp_path):
    st = _state(tmp_path, min_runs=1, retire_errors=1)
    st.track("good")
    st.record_run("good", ok=True)
    st.record_reward("good", 1.0)
    st.track("bad")
    st.record_run("bad", ok=False)
    decisions = dict(st.gate_all())
    assert decisions == {"good": "adopt", "bad": "retire"}
    st.forget("bad")
    assert st.status_of("bad") is None
