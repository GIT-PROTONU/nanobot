"""Purpose Engine — the slow "identity / teleological" layer (ROS-free, unit-tested offline:

    pixi run python -m pytest src/behavior/test

Sits ABOVE the presence statechart. It owns the robot's current *objective* and its
*intrinsic-reward* weights, and updates them by reflecting on accumulated experience (the
shared decision log, written by web_control) through the lens of its personality traits.

It is **deterministic and fully local** — no LLM on the critical path. By design it is kept
*separate* from the existing LLM reflection: that path owns *trait* drift; this owns *goals
and reward weights*. Everything here is **narrative-only** — it never moves the robot; the
reward weights only bias which idle beat the planner narrates.

Mirrors the presence.py / mood_node.py split: pure logic here, ROS glue in mood_node.
"""
import copy
import time

# The robot's standing "deep questions" — its identity prompt, folded into reflection.
DEEP_QUESTIONS = [
    "What in my surroundings has changed since I last looked?",
    "Am I taking care of my body — heat, balance, rest?",
]

# Intrinsic-reward axes (0..1): how much the robot currently values each drive. The planner
# reads these to decide what to narrate; reflect_purpose nudges them from experience.
REWARD_AXES = ("curiosity", "social", "order", "rest")
DEFAULT_REWARD = {"curiosity": 0.55, "social": 0.3, "order": 0.2, "rest": 0.4}

# Objective catalogue (narrative-only). Each objective declares the reward axis it serves,
# so human reward can shape the right drive. Start with one; add more as recipes grow.
OBJECTIVES = {
    "get_acquainted": {"text": "Get to know the space around me", "primary": "curiosity"},
    # future: "stay_well" -> rest, "be_sociable" -> social, "keep_order" -> order
}
DEFAULT_OBJECTIVE = "get_acquainted"


def clamp01(v, default=0.5):
    try:
        return max(0.0, min(1.0, float(v)))
    except (TypeError, ValueError):
        return default


def default_purpose(name="Nano", now=None):
    """The seed purpose state (also the JSON schema persisted to purpose.json)."""
    now = time.time() if now is None else now
    obj = OBJECTIVES[DEFAULT_OBJECTIVE]
    return {
        "identity": {"name": name, "deep_questions": list(DEEP_QUESTIONS)},
        "objective": {"id": DEFAULT_OBJECTIVE, "text": obj["text"],
                      "set_at": now, "horizon_secs": 86400.0},
        "intrinsic_reward": dict(DEFAULT_REWARD),
        "traits_signature": {},
    }


def merge_purpose(saved, name="Nano"):
    """Merge a loaded purpose.json over the seed, tolerating partial/foreign files."""
    base = default_purpose(name)
    if not isinstance(saved, dict):
        return base
    if isinstance(saved.get("objective"), dict):
        oid = saved["objective"].get("id")
        if oid in OBJECTIVES:
            base["objective"]["id"] = oid
            base["objective"]["text"] = OBJECTIVES[oid]["text"]
        for k in ("set_at", "horizon_secs"):
            if isinstance(saved["objective"].get(k), (int, float)):
                base["objective"][k] = float(saved["objective"][k])
    if isinstance(saved.get("intrinsic_reward"), dict):
        for k in REWARD_AXES:
            if k in saved["intrinsic_reward"]:
                base["intrinsic_reward"][k] = clamp01(saved["intrinsic_reward"][k],
                                                      base["intrinsic_reward"][k])
    if isinstance(saved.get("traits_signature"), dict):
        base["traits_signature"] = dict(saved["traits_signature"])
    return base


def summarize_experience(entries):
    """Roll up decision-log entries (list of dicts) into the counts the heuristic reads.
    Tolerant of partial/foreign entries — anything unparseable is skipped.

    Counts: how often the robot recently looked/observed and whether it 'landed' (spoke or
    used a cached line), and the up/down human reward tally (logged by web_control)."""
    s = {"observe": 0, "observe_ok": 0, "reward_up": 0, "reward_down": 0, "n": 0}
    for e in entries or []:
        if not isinstance(e, dict):
            continue
        s["n"] += 1
        trig = str(e.get("trigger", ""))
        status = str(e.get("status", ""))
        if trig in ("beat:pursuing", "beat:looking", "look", "observe", "beat:musing"):
            s["observe"] += 1
            if status in ("spoke", "bank"):
                s["observe_ok"] += 1
        elif trig == "reward":
            if status == "up":
                s["reward_up"] += 1
            elif status == "down":
                s["reward_down"] += 1
    return s


def reflect_purpose(purpose, experience, traits, alpha=0.15, now=None):
    """Pure, deterministic heuristic. Returns (new_purpose, changed).

    Eases the intrinsic-reward weights toward targets derived from recent experience and
    personality, using the same exponential smoothing the presence chart uses for traits, so
    the robot's "values" drift gently rather than jumping. Narrative-only."""
    now = time.time() if now is None else now
    new = copy.deepcopy(purpose)
    rew = new["intrinsic_reward"]
    targets = {}

    # Sparse looking -> want MORE curiosity (go explore the room a bit more).
    if experience.get("observe", 0) < 3:
        targets["curiosity"] = max(rew.get("curiosity", 0.5), 0.7)

    # Human reward shapes the drive the current objective serves (its "primary" axis).
    up, down = experience.get("reward_up", 0), experience.get("reward_down", 0)
    if up or down:
        net = (up - down) / float(up + down)               # -1..1
        prim = OBJECTIVES.get(new["objective"]["id"], {}).get("primary", "curiosity")
        targets[prim] = clamp01(rew.get(prim, 0.5) + 0.3 * net)

    # Personality colours rest: a more cautious robot values rest/quiet a little more.
    targets["rest"] = clamp01(0.25 + 0.4 * traits.get("caution", 0.5))

    changed = False
    for k, tgt in targets.items():
        old = rew.get(k, 0.5)
        nv = round((1 - alpha) * old + alpha * clamp01(tgt), 4)
        if abs(nv - old) > 1e-4:
            rew[k] = nv
            changed = True

    sig = {k: round(clamp01(traits.get(k, 0.5)), 3) for k in ("curiosity", "caution")}
    if sig != new.get("traits_signature"):
        new["traits_signature"] = sig
        changed = True
    return new, changed
