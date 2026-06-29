"""Offline tests for the Purpose Engine (ROS-free; folded into behavior.brain).
Run: pixi run python -m pytest src/behavior/test"""
from behavior.brain import (default_purpose, merge_purpose, summarize_experience,
                            reflect_purpose, REWARD_AXES, DEFAULT_OBJECTIVE)


def test_default_purpose_schema():
    p = default_purpose("Nano")
    assert p["objective"]["id"] == DEFAULT_OBJECTIVE
    assert set(p["intrinsic_reward"]) == set(REWARD_AXES)
    assert p["identity"]["name"] == "Nano"
    assert p["identity"]["deep_questions"]              # non-empty


def test_merge_tolerates_partial_and_foreign():
    merged = merge_purpose({"intrinsic_reward": {"curiosity": 0.9, "junk": 5},
                            "objective": {"id": "nonexistent"}}, name="Bot")
    assert merged["intrinsic_reward"]["curiosity"] == 0.9
    assert merged["objective"]["id"] == DEFAULT_OBJECTIVE   # unknown id rejected
    assert merge_purpose("not a dict")["objective"]["id"] == DEFAULT_OBJECTIVE


def test_summarize_counts_observe_and_reward():
    s = summarize_experience([
        {"trigger": "beat:looking", "status": "spoke"},
        {"trigger": "beat:musing", "status": "bank"},
        {"trigger": "beat:musing", "status": "llm-unavailable"},
        {"trigger": "reward", "status": "up"},
        {"trigger": "reward", "status": "down"},
        "garbage", 42,
    ])
    assert s["observe"] == 3 and s["observe_ok"] == 2
    assert s["reward_up"] == 1 and s["reward_down"] == 1


def test_sparse_observation_raises_curiosity():
    p = default_purpose()
    p["intrinsic_reward"]["curiosity"] = 0.4
    exp = summarize_experience([])                  # nothing observed -> want to explore
    new, changed = reflect_purpose(p, exp, {"caution": 0.5}, alpha=1.0)
    assert changed
    assert new["intrinsic_reward"]["curiosity"] >= 0.7


def test_human_reward_shapes_primary_axis():
    p = default_purpose()                            # objective primary = curiosity
    base = p["intrinsic_reward"]["curiosity"]
    up = reflect_purpose(p, {"observe": 9, "reward_up": 3, "reward_down": 0},
                         {"caution": 0.5}, alpha=1.0)[0]["intrinsic_reward"]["curiosity"]
    down = reflect_purpose(p, {"observe": 9, "reward_up": 0, "reward_down": 3},
                           {"caution": 0.5}, alpha=1.0)[0]["intrinsic_reward"]["curiosity"]
    assert up > base > down


def test_reflect_is_deterministic_and_clamped():
    p = default_purpose()
    a = reflect_purpose(p, {"observe": 0, "reward_up": 99}, {"caution": 1.0})[0]
    b = reflect_purpose(p, {"observe": 0, "reward_up": 99}, {"caution": 1.0})[0]
    assert a == b
    assert all(0.0 <= v <= 1.0 for v in a["intrinsic_reward"].values())
