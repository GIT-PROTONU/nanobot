"""Horizon Planner + online A/B bandit — the "strategy" layer (ROS-free, unit-tested offline:

    pixi run python -m pytest src/behavior/test

Turns the Purpose Engine's objective into a small task queue (a flat list for now; a DAG
later), verifies each task's preconditions against the world before narrating it — a light
"predictive shadow" (no physics sim, since this layer is narrative-only) — and re-plans
tasks that don't verify, with their reason.

It also runs a local **epsilon-greedy multi-armed bandit** that A/B-tests *how* a task is
narrated (line phrasing) and learns from human reward. Everything is deterministic (the rng
and clock are injected), so it's offline-testable exactly like presence.py.

Mirrors the presence.py / mood_node.py split: pure logic here, ROS glue in mood_node.
"""
import copy

# objective id -> ordered sub-tasks (the DAG; a flat list for the first slice).
RECIPES = {
    "get_acquainted": ["observe_surroundings"],
}

# task id -> how it's narrated (which beat / whether it wants the camera / a phrase to slot
# into the prompt) + its precondition: a pure predicate over a world-state dict.
TASKS = {
    "observe_surroundings": {
        "beat": "pursuing",
        "camera": True,
        "text": "look around and notice what's in the space",
        # Don't narrate "seeing the room" while being carried, or with stale sensors.
        "precond": lambda w: (not w.get("picked")) and bool(w.get("sensors_fresh", True)),
    },
}

# A/B experiments. Each binds a task to a set of narration variants the bandit chooses among.
# First experiment: terse vs playful phrasing of the observe line.
DEFAULT_EXPERIMENTS = {
    "pursuing_style": {
        "task": "observe_surroundings",
        "variants": {
            "terse":   {"style_hint": "Keep it to a few plain, matter-of-fact words."},
            "playful": {"style_hint": "Be playful and openly curious about it."},
        },
    },
}


def decompose(objective_id):
    """Objective -> ordered task ids (the strategy layer's plan)."""
    return list(RECIPES.get(objective_id, []))


def verify(task_id, world):
    """The 'predictive shadow': can this task be narrated right now? Returns (ok, reason)."""
    t = TASKS.get(task_id)
    if t is None:
        return False, "unknown-task"
    try:
        ok = bool(t["precond"](world))
    except Exception as exc:                       # a bad predicate must never crash the brain
        return False, "precond-error:%s" % (exc,)
    return (ok, "" if ok else "precondition-not-met")


class Bandit:
    """Epsilon-greedy multi-armed bandit over named experiments. Reward is in [-1, 1];
    each arm tracks a running mean. State is plain JSON (persist/restore friendly)."""

    def __init__(self, experiments=None, state=None, epsilon=0.2):
        self.exp = copy.deepcopy(experiments or DEFAULT_EXPERIMENTS)
        self.epsilon = float(epsilon)
        # stats[exp_id][variant] = {"n": int, "mean": float}
        self.stats = {eid: {v: {"n": 0, "mean": 0.0} for v in e["variants"]}
                      for eid, e in self.exp.items()}
        if isinstance(state, dict):
            for eid, vs in (state.get("stats") or {}).items():
                if eid in self.stats and isinstance(vs, dict):
                    for v, st in vs.items():
                        if v in self.stats[eid] and isinstance(st, dict):
                            self.stats[eid][v] = {"n": int(st.get("n", 0)),
                                                  "mean": float(st.get("mean", 0.0))}
            if isinstance(state.get("epsilon"), (int, float)):
                self.epsilon = float(state["epsilon"])

    def assign(self, exp_id, rng):
        """Pick a variant: explore (random) with prob epsilon, else exploit the best mean.
        Ties broken deterministically (fewer trials, then name) so tests are stable."""
        e = self.exp.get(exp_id)
        if not e:
            return None
        variants = list(e["variants"])
        if rng.random() < self.epsilon:
            return rng.choice(variants)
        st = self.stats[exp_id]
        return max(variants, key=lambda v: (st[v]["mean"], -st[v]["n"], v))

    def record(self, exp_id, variant, reward):
        st = self.stats.get(exp_id, {}).get(variant)
        if st is None:
            return False
        reward = max(-1.0, min(1.0, float(reward)))
        st["n"] += 1
        st["mean"] = round(st["mean"] + (reward - st["mean"]) / st["n"], 4)
        return True

    def winner(self, exp_id):
        vs = self.stats.get(exp_id)
        if not vs:
            return None
        return max(vs, key=lambda v: (vs[v]["mean"], vs[v]["n"], v))

    def to_state(self):
        return {"stats": copy.deepcopy(self.stats), "epsilon": self.epsilon}

    def summary(self):
        return {eid: {"variants": {v: dict(st) for v, st in vs.items()},
                      "winner": self.winner(eid)}
                for eid, vs in self.stats.items()}


class Planner:
    """The receding-horizon planner: holds the current objective's task queue + the bandit,
    hands out the next verified task to narrate (rate-limited), and credits human reward back
    to the A/B arm that produced the line."""

    def __init__(self, objective_id="get_acquainted", experiments=None, state=None,
                 epsilon=0.2, min_interval=120.0):
        st = state or {}
        self.objective = st.get("objective") or objective_id
        self.bandit = Bandit(experiments, st.get("bandit"), epsilon=epsilon)
        self.queue = list(st.get("queue") or decompose(self.objective))
        self.min_interval = float(min_interval)
        self._last_t = -1e9
        self._i = 0
        self.last_assignment = None        # {exp, variant, task} of the most recent narration

    def set_objective(self, objective_id):
        if objective_id and objective_id != self.objective:
            self.objective = objective_id
            self.queue = decompose(objective_id)
            self._i = 0

    def _exp_for(self, task_id):
        for eid, e in self.bandit.exp.items():
            if e.get("task") == task_id:
                return eid
        return None

    def next_task(self, world, rng, now=0.0):
        """Pick the next verified task to narrate, assign it an A/B variant, and return its
        narration spec — or None when nothing's due (rate-limited) or eligible (no task
        verifies). Tasks that fail verification are left in the queue (a real DAG would log +
        reorder); the rotation still advances so a later cycle can try a different one."""
        if not self.queue:
            self.queue = decompose(self.objective)
        if not self.queue or (now - self._last_t) < self.min_interval:
            return None
        n = len(self.queue)
        for _ in range(n):
            task_id = self.queue[self._i % n]
            self._i += 1
            ok, _reason = verify(task_id, world)
            if not ok:
                continue
            self._last_t = now
            t = TASKS[task_id]
            exp_id = self._exp_for(task_id)
            variant = self.bandit.assign(exp_id, rng) if exp_id else None
            hint = ""
            if variant is not None:
                hint = self.bandit.exp[exp_id]["variants"][variant].get("style_hint", "")
            self.last_assignment = {"exp": exp_id, "variant": variant, "task": task_id}
            return {"task": task_id, "beat": t["beat"], "camera": bool(t["camera"]),
                    "text": t["text"], "exp": exp_id, "variant": variant, "style_hint": hint}
        return None

    def on_reward(self, value, target=None):
        """Credit a human reward to the A/B arm that produced the narrated line. `target`
        (echoed from /task_current) pins the exact exp+variant; else use the last one."""
        tgt = target if isinstance(target, dict) else self.last_assignment
        if not tgt:
            return False
        return self.bandit.record(tgt.get("exp"), tgt.get("variant"), value)

    def to_state(self):
        return {"objective": self.objective, "queue": self.queue,
                "bandit": self.bandit.to_state()}

    def summary(self):
        return {"objective": self.objective, "queue": self.queue,
                "experiments": self.bandit.summary(), "last": self.last_assignment}
