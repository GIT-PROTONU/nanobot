"""Offline tests for the Horizon Planner + bandit (ROS-free; folded into behavior.brain).
Run via pytest."""
import random

from behavior.brain import decompose, verify, Bandit, Planner


def test_decompose_and_verify():
    assert decompose("get_acquainted") == ["observe_surroundings"]
    assert decompose("unknown") == []
    assert verify("observe_surroundings", {"picked": False})[0] is True
    assert verify("observe_surroundings", {"picked": True})[0] is False    # being carried
    assert verify("observe_surroundings", {"sensors_fresh": False})[0] is False
    assert verify("nope", {})[0] is False


def test_planner_rate_limits():
    p = Planner(min_interval=120.0)
    rng = random.Random(0)
    world = {"picked": False}
    assert p.next_task(world, rng, now=0.0) is not None      # first is allowed
    assert p.next_task(world, rng, now=10.0) is None         # too soon -> rate-limited
    assert p.next_task(world, rng, now=200.0) is not None    # interval elapsed


def test_planner_skips_unverifiable_task():
    p = Planner(min_interval=0.0)
    rng = random.Random(0)
    assert p.next_task({"picked": True}, rng, now=1.0) is None     # nothing verifies
    assert p.next_task({"picked": False}, rng, now=2.0) is not None


def test_bandit_learns_winner_and_exploits():
    b = Bandit(epsilon=0.0)                                   # pure exploit
    rng = random.Random(1)
    # Teach it that "playful" is better than "terse".
    b.record("pursuing_style", "playful", 1.0)
    b.record("pursuing_style", "terse", -1.0)
    assert b.winner("pursuing_style") == "playful"
    assert b.assign("pursuing_style", rng) == "playful"      # epsilon=0 -> always best


def test_reward_credits_the_assigned_variant():
    p = Planner(min_interval=0.0, epsilon=0.0)
    rng = random.Random(2)
    spec = p.next_task({"picked": False}, rng, now=1.0)
    assert spec["exp"] == "pursuing_style"
    # Contextual reward with an explicit target pins the arm.
    assert p.on_reward(1.0, {"exp": "pursuing_style", "variant": spec["variant"]})
    assert p.bandit.stats["pursuing_style"][spec["variant"]]["n"] == 1
    # No target -> falls back to the last assignment.
    assert p.on_reward(-1.0)
    assert p.bandit.stats["pursuing_style"][spec["variant"]]["n"] == 2


def test_state_roundtrip():
    p = Planner(min_interval=0.0)
    rng = random.Random(3)
    p.next_task({"picked": False}, rng, now=1.0)
    p.on_reward(0.5)
    state = p.to_state()
    p2 = Planner(state=state)
    assert p2.objective == p.objective
    assert p2.bandit.stats == p.bandit.stats
