"""Offline tests for the Pursuit driver + A/B bandit (ROS-free; folded into behavior.brain).
Run via pytest."""
import random

from behavior.brain import Bandit, Pursuit, precond_ok, OBJECTIVES, DEFAULT_OBJECTIVE


def test_precond_ok():
    obj = OBJECTIVES[DEFAULT_OBJECTIVE]
    assert precond_ok(obj, {"picked": False}) is True
    assert precond_ok(obj, {"picked": True}) is False           # being carried
    assert precond_ok(obj, {"sensors_fresh": False}) is False
    assert precond_ok({}, {}) is True                           # no predicate -> always ok
    assert precond_ok({"precond": lambda w: 1 / 0}, {}) is False  # a bad predicate never crashes


def test_pursuit_rate_limits():
    p = Pursuit(min_interval=120.0)
    rng = random.Random(0)
    world = {"picked": False}
    assert p.next_task(world, rng, now=0.0) is not None      # first is allowed
    assert p.next_task(world, rng, now=10.0) is None         # too soon -> rate-limited
    assert p.next_task(world, rng, now=200.0) is not None    # interval elapsed


def test_pursuit_skips_when_precondition_fails():
    p = Pursuit(min_interval=0.0)
    rng = random.Random(0)
    assert p.next_task({"picked": True}, rng, now=1.0) is None     # carried -> precond fails
    assert p.next_task({"picked": False}, rng, now=2.0) is not None


def test_bandit_learns_winner_and_exploits():
    b = Bandit(epsilon=0.0)                                   # pure exploit
    rng = random.Random(1)
    # Teach it that "playful" is better than "terse" for the default objective.
    b.record(DEFAULT_OBJECTIVE, "playful", 1.0)
    b.record(DEFAULT_OBJECTIVE, "terse", -1.0)
    assert b.winner(DEFAULT_OBJECTIVE) == "playful"
    assert b.assign(DEFAULT_OBJECTIVE, rng) == "playful"     # epsilon=0 -> always best


def test_reward_credits_the_assigned_variant():
    p = Pursuit(min_interval=0.0, epsilon=0.0)
    rng = random.Random(2)
    spec = p.next_task({"picked": False}, rng, now=1.0)
    assert spec["exp"] == DEFAULT_OBJECTIVE                   # exp echoes the objective id
    # Contextual reward with an explicit target pins the arm.
    assert p.on_reward(1.0, {"exp": DEFAULT_OBJECTIVE, "variant": spec["variant"]})
    assert p.bandit.stats[DEFAULT_OBJECTIVE][spec["variant"]]["n"] == 1
    # No target -> falls back to the last assignment.
    assert p.on_reward(-1.0)
    assert p.bandit.stats[DEFAULT_OBJECTIVE][spec["variant"]]["n"] == 2


def test_state_roundtrip():
    p = Pursuit(min_interval=0.0)
    rng = random.Random(3)
    p.next_task({"picked": False}, rng, now=1.0)
    p.on_reward(0.5)
    state = p.to_state()
    p2 = Pursuit(state=state)
    assert p2.objective == p.objective
    assert p2.bandit.stats == p.bandit.stats
