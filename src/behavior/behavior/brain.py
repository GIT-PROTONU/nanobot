"""The brain — the behaviour layer's ROS-free thinking, in ONE module.

This is the whole "what should I do" stack, kept out of the ROS node so it unit-tests offline:

    pixi run python -m pytest src/behavior/test

It holds, in order:
  * the **Purpose Engine** — the slow teleological layer: the current *objective* + the
    *intrinsic-reward* weights, drifted deterministically by reflecting on experience;
  * the **Pursuit** driver + an online A/B **bandit** — narrates the active objective when its
    precondition holds (rate-limited), and learns which phrasing earns reward;
  * **PurposeBrain** — the orchestration of the two above (beat upgrades, reflection, reward,
    reflection-mode consolidation, persistence);
  * **Personality** — the Sismic-chart-context personality glue (seed load, evolve/heartbeat
    events, latched publish + throttled persist).

(The Purpose Engine + Pursuit used to live in their own `purpose.py` / `planner.py`; they were
folded in here to make the behaviour layer fewer moving parts. The presence statechart stays in
`presence.py` and the ROS glue in `mood_node.py`.)

Like `web_control.cognition.CognitionCore`, the same orchestration runs on the robot node
(`behavior.mood_node`) and the dev web harness (`scripts/dev_webui.py`); everything
platform-specific is injected as a tiny **adapter**:

    PurposeBrain adapters (all optional):
      read_cog_log()          -> the recent decision-log entries (list of dicts)
      picked()                -> True while the robot is held off the ground (bool)
      traits_snapshot()       -> the live personality traits dict (for reflection)
      publish_purpose(obj)    -> announce the current purpose          (robot: latched topic)
      publish_task(payload)   -> announce the current pursued task     (dev: store for HTTP)
      publish_experiments(s)  -> announce the A/B experiment summary
      logger(msg)             -> log a line

Everything here is deterministic + narrative-only: it never moves the robot.
"""
import copy
import json
import os
import time

from .presence import DEFAULT_TRAITS, DEFAULT_REGISTRY, DEFAULT_DRIVES

# Sismic Event is only needed by Personality (interpreter-coupled). Imported defensively so
# importing this module on a board without sismic still works (e.g. for PurposeBrain alone).
try:
    from sismic.model import Event
except Exception:  # pragma: no cover - depends on env
    Event = None


def clamp01(v, default=0.5):
    try:
        return max(0.0, min(1.0, float(v)))
    except (TypeError, ValueError):
        return default


# =============================================================================
# Purpose Engine — the slow "identity / teleological" layer (was purpose.py).
# Sits ABOVE the presence statechart. Owns the current objective + intrinsic-reward weights,
# drifted by reflecting on the shared decision log through the lens of the personality traits.
# Deterministic + fully local (no LLM on the critical path) and narrative-only — the reward
# weights only bias which objective the robot pursues; it never moves the robot. Kept
# separate from the LLM reflection: that owns *trait* drift, this owns *goals + reward weights*.
# =============================================================================

# The robot's standing "deep questions" — its identity prompt, folded into reflection.
DEEP_QUESTIONS = [
    "What in my surroundings has changed since I last looked?",
    "Am I taking care of my body — heat, balance, rest?",
]

# Intrinsic-reward axes (0..1): how much the robot currently values each drive. reflect_purpose
# nudges them from experience; an objective's `axis` says which drive its reward shapes.
REWARD_AXES = ("curiosity", "social", "order", "rest")
DEFAULT_REWARD = {"curiosity": 0.55, "social": 0.3, "order": 0.2, "rest": 0.4}

# Objective catalogue (narrative-only). Each objective is self-contained — its human-facing
# text, the intrinsic-reward `axis` it serves (so human reward shapes the right drive), and how
# it's pursued: the `beat` it narrates through, whether it wants the `camera`, the `narrate`
# line phrase, a `precond` predicate over the light world snapshot (so e.g. it won't narrate
# "seeing the room" while being carried), and the A/B phrasing `variants` (name -> style hint)
# the bandit learns from reward. One objective for now; add a row to grow the repertoire.
OBJECTIVES = {
    "get_acquainted": {
        "text": "Get to know the space around me",
        "axis": "curiosity",
        "beat": "pursuing",
        "camera": True,
        "narrate": "look around and notice what's in the space",
        "precond": lambda w: (not w.get("picked")) and bool(w.get("sensors_fresh", True)),
        "variants": {
            "terse":   "Keep it to a few plain, matter-of-fact words.",
            "playful": "Be playful and openly curious about it.",
        },
    },
    # future: "stay_well" -> rest, "be_sociable" -> social, "keep_order" -> order
}
DEFAULT_OBJECTIVE = "get_acquainted"


def precond_ok(objective, world):
    """The objective's narrative-only precondition over the light world snapshot ("can I pursue
    this right now?"). Missing predicate = always ok; a bad predicate must never crash the
    brain, so any exception reads as not-ok."""
    fn = (objective or {}).get("precond")
    if fn is None:
        return True
    try:
        return bool(fn(world))
    except Exception:
        return False


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
        prim = OBJECTIVES.get(new["objective"]["id"], {}).get("axis", "curiosity")
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


# =============================================================================
# Pursuit driver + online A/B bandit — the "strategy" layer (was planner.py).
# Narrates the active objective when its precondition holds (rate-limited), and runs a local
# epsilon-greedy bandit that A/B-tests *how* it's phrased and learns from human reward.
# Deterministic (rng + clock injected), narrative-only. (This used to be a receding-horizon
# "Horizon Planner" with a per-objective task DAG — decompose/verify/queue — but with the
# objectives flat and single-task there was nothing to schedule, so it collapsed to this.)
# =============================================================================


class Bandit:
    """Epsilon-greedy multi-armed bandit over each objective's A/B phrasing variants. Reward is
    in [-1, 1]; each arm tracks a running mean. State is plain JSON (persist/restore friendly)."""

    def __init__(self, objectives=None, state=None, epsilon=0.2):
        cat = objectives or OBJECTIVES
        # variants[obj_id] = [variant names]; stats[obj_id][variant] = {"n": int, "mean": float}
        self.variants = {oid: list((o.get("variants") or {})) for oid, o in cat.items()}
        self.epsilon = float(epsilon)
        self.stats = {oid: {v: {"n": 0, "mean": 0.0} for v in vs}
                      for oid, vs in self.variants.items()}
        if isinstance(state, dict):
            for oid, vs in (state.get("stats") or {}).items():
                if oid in self.stats and isinstance(vs, dict):
                    for v, st in vs.items():
                        if v in self.stats[oid] and isinstance(st, dict):
                            self.stats[oid][v] = {"n": int(st.get("n", 0)),
                                                  "mean": float(st.get("mean", 0.0))}
            if isinstance(state.get("epsilon"), (int, float)):
                self.epsilon = float(state["epsilon"])

    def assign(self, obj_id, rng):
        """Pick a variant: explore (random) with prob epsilon, else exploit the best mean.
        Ties broken deterministically (fewer trials, then name) so tests are stable."""
        variants = self.variants.get(obj_id)
        if not variants:
            return None
        if rng.random() < self.epsilon:
            return rng.choice(variants)
        st = self.stats[obj_id]
        return max(variants, key=lambda v: (st[v]["mean"], -st[v]["n"], v))

    def record(self, obj_id, variant, reward):
        st = self.stats.get(obj_id, {}).get(variant)
        if st is None:
            return False
        reward = max(-1.0, min(1.0, float(reward)))
        st["n"] += 1
        st["mean"] = round(st["mean"] + (reward - st["mean"]) / st["n"], 4)
        return True

    def winner(self, obj_id):
        vs = self.stats.get(obj_id)
        if not vs:
            return None
        return max(vs, key=lambda v: (vs[v]["mean"], vs[v]["n"], v))

    def to_state(self):
        return {"stats": copy.deepcopy(self.stats), "epsilon": self.epsilon}

    def summary(self):
        return {oid: {"variants": {v: dict(st) for v, st in vs.items()},
                      "winner": self.winner(oid)}
                for oid, vs in self.stats.items()}


class Pursuit:
    """Drives the single active objective: when it's due (rate-limited) and its precondition
    holds, return its narration spec — choosing an A/B phrasing variant the bandit learns from
    human reward — and credit reward back to the arm that produced a line."""

    def __init__(self, objective_id=DEFAULT_OBJECTIVE, objectives=None, state=None,
                 epsilon=0.2, min_interval=120.0):
        st = state or {}
        self.cat = objectives or OBJECTIVES
        self.objective = st.get("objective") or objective_id
        self.bandit = Bandit(self.cat, st.get("bandit"), epsilon=epsilon)
        self.min_interval = float(min_interval)
        self._last_t = -1e9
        self.last_assignment = None        # {exp, variant, task} of the most recent narration

    def set_objective(self, objective_id):
        if objective_id and objective_id != self.objective:
            self.objective = objective_id

    def next_task(self, world, rng, now=0.0):
        """If the active objective is due and its precondition holds, assign an A/B variant and
        return its narration spec; else None (rate-limited, unknown objective, or precondition
        not met — e.g. being carried). `exp`/`task` echo the objective id so the web reward
        target round-trips back to the right arm."""
        obj = self.cat.get(self.objective)
        if obj is None or (now - self._last_t) < self.min_interval:
            return None
        if not precond_ok(obj, world):
            return None
        self._last_t = now
        variant = self.bandit.assign(self.objective, rng)
        hint = (obj.get("variants") or {}).get(variant, "") if variant is not None else ""
        self.last_assignment = {"exp": self.objective, "variant": variant,
                                "task": self.objective}
        return {"task": self.objective, "beat": obj["beat"], "camera": bool(obj["camera"]),
                "text": obj["narrate"], "exp": self.objective, "variant": variant,
                "style_hint": hint}

    def on_reward(self, value, target=None):
        """Credit a human reward to the A/B arm that produced the narrated line. `target`
        (echoed from /task_current) pins the exact exp+variant; else use the last one."""
        tgt = target if isinstance(target, dict) else self.last_assignment
        if not tgt:
            return False
        return self.bandit.record(tgt.get("exp"), tgt.get("variant"), value)

    def to_state(self):
        return {"objective": self.objective, "bandit": self.bandit.to_state()}

    def summary(self):
        return {"objective": self.objective,
                "experiments": self.bandit.summary(), "last": self.last_assignment}


def _state_path(explicit, default_name):
    """Resolve a state-file path: an explicit override, else ~/.local/state/nanobot/<name>."""
    return explicit or os.path.expanduser(f"~/.local/state/nanobot/{default_name}")


def load_json(path, logger=None):
    """Best-effort read of a JSON file. None if absent/unreadable (never raises)."""
    try:
        with open(path, encoding="utf-8") as f:
            return json.load(f)
    except FileNotFoundError:
        return None
    except Exception as exc:
        if logger:
            logger(f"could not read {path} ({exc})")
        return None


def save_json(path, obj, logger=None):
    """Atomically persist `obj` as JSON (tmp + os.replace). Best-effort (never raises)."""
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        tmp = path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(obj, f, indent=2, ensure_ascii=False)
        os.replace(tmp, path)
    except Exception as exc:
        if logger:
            logger(f"could not persist {path} ({exc})")


def read_cog_log_tail(path, n=40, logger=None):
    """Tail the last `n` JSON-lines of the shared decision log (the Purpose Engine's
    experience input). Best-effort: [] if absent/unreadable."""
    try:
        with open(path, encoding="utf-8") as f:
            lines = f.readlines()[-n:]
    except Exception:
        return []
    out = []
    for ln in lines:
        ln = ln.strip()
        if ln:
            try:
                out.append(json.loads(ln))
            except Exception:
                pass
    return out


def load_personality(path, *, with_defaults=True, logger=None):
    """Seed the personality from personality.json (written by the creator / persisted as it
    drifts). With `with_defaults`, traits/registry are merged over the frozen presence
    defaults (the robot wants a full, known-good seed); without, only what's in the file is
    returned (the dev harness lets build_interpreter fill the gaps). Best-effort."""
    if with_defaults:
        base = {"name": "Nano", "persona": "", "traits": dict(DEFAULT_TRAITS),
                "registry": copy.deepcopy(DEFAULT_REGISTRY), "drives": dict(DEFAULT_DRIVES)}
    else:
        base = {"name": "Nano", "persona": "", "traits": {}, "registry": {}, "drives": {}}
    saved = load_json(path, logger=logger)
    if isinstance(saved, dict):
        for k in ("name", "persona"):
            if isinstance(saved.get(k), str):
                base[k] = saved[k]
        if isinstance(saved.get("traits"), dict):
            base["traits"].update(saved["traits"])
        if isinstance(saved.get("drives"), dict):
            base["drives"].update(saved["drives"])
        if isinstance(saved.get("registry"), dict):
            if with_defaults:
                for n, patch in saved["registry"].items():
                    base["registry"].setdefault(n, {}).update(patch or {})
            else:
                base["registry"] = saved["registry"]
    return base


class PurposeBrain:
    """The slow "identity / strategy" layer: owns the Purpose Engine state + the Pursuit
    driver, decides when an idle beat upgrades to a goal-pursuit or a skill, reflects on
    experience, credits human reward to the A/B bandit, and consolidates while reflecting.

    Pure + deterministic (rng/clock injected). It never executes a beat — it only DECIDES
    (returns specs) and announces state through the injected publish adapters; the node turns
    that into a `/cognition/request` (robot) or a direct CognitionCore call (dev)."""

    def __init__(self, *, name="Nano", enable=True, rng, epsilon=0.2,
                 pursue_min_interval=180.0, reflect_period=600.0,
                 skills_enable=True, skill_every=6,
                 purpose_path="", experiments_path="", cog_log_path="",
                 read_cog_log=None, picked=None, traits_snapshot=None,
                 publish_purpose=None, publish_task=None, publish_experiments=None,
                 logger=None):
        self.enable = bool(enable)
        self.skills_enable = bool(skills_enable)
        self._skill_every = max(1, int(skill_every))
        self._reflect_period = max(30.0, float(reflect_period))
        self._rng = rng
        self._log = logger or (lambda *_: None)
        # state-file paths (resolved once)
        self._purpose_path = _state_path(purpose_path, "purpose.json")
        self._experiments_path = _state_path(experiments_path, "experiments.json")
        self._cog_log_path = _state_path(cog_log_path, "cognition.log")
        # adapters
        self._read_cog_log = read_cog_log or (
            lambda: read_cog_log_tail(self._cog_log_path, logger=self._log))
        self._picked = picked or (lambda: False)
        self._traits_snapshot = traits_snapshot or (lambda: {})
        self._publish_purpose = publish_purpose or (lambda _o: None)
        self._publish_task = publish_task or (lambda _p: None)
        self._publish_experiments = publish_experiments or (lambda _s: None)
        # runtime state
        self.reflecting = False
        self.task = {}                 # last pursued-task payload (the dev /task_current readout)
        self._body_beat_n = 0          # counts body (musing) beats, for the skill cadence
        self._reflect_last = 0.0       # monotonic of the last purpose reflection
        self._pub_hb = 0.0             # monotonic of the last latched republish
        self._purpose = merge_purpose(load_json(self._purpose_path, logger=self._log),
                                      name=name)
        self._pursuit = None
        if self.enable:
            self._pursuit = Pursuit(
                objective_id=self._purpose["objective"]["id"],
                state=load_json(self._experiments_path, logger=self._log),
                epsilon=float(epsilon), min_interval=float(pursue_min_interval))

    # ---- readouts (the dev harness serves these over HTTP) ------------------
    @property
    def purpose(self):
        return self._purpose

    def summary(self):
        return self._pursuit.summary() if self._pursuit is not None else {"experiments": {}}

    def world_state(self):
        """The light world snapshot the pursuit checks its precondition against (narrative-only)."""
        return {"picked": bool(self._picked()), "sensors_fresh": True,
                "reflecting": self.reflecting}

    # ---- beat upgrades (decisions; the node executes them) ------------------
    def next_pursuing(self, now):
        """If a goal-pursuit beat is due, return its narration spec (and announce the task +
        moved A/B stats); else None. Eligible only while enabled and not reflecting."""
        if not (self.enable and self._pursuit is not None) or self.reflecting:
            return None
        spec = self._pursuit.next_task(self.world_state(), self._rng, now=now)
        if spec is None:
            return None
        self.task = {"task": spec["task"], "exp": spec["exp"], "variant": spec["variant"],
                     "text": spec["text"], "t": time.time()}
        self._publish_task(self.task)
        self._publish_experiments(self.summary())          # stats moved (new assignment)
        return spec

    def take_skill_beat(self):
        """Advance the skill-beat cadence and report whether THIS body beat is a skill beat.
        Only counts while skills are enabled and we're not reflecting (matches the chart)."""
        if not (self.skills_enable and not self.reflecting):
            return False
        self._body_beat_n += 1
        return self._body_beat_n % self._skill_every == 0

    @staticmethod
    def pursuing_prompt(spec, base_prompt):
        """Fill a `pursuing` beat's prompt with the task phrase (+ any A/B style hint)."""
        prompt = base_prompt.format(task=spec["text"])
        if spec.get("style_hint"):
            prompt = prompt + " " + spec["style_hint"]
        return prompt

    # ---- purpose reflection / reward / reflection mode ----------------------
    def run_reflection(self, traits=None, force=False):
        """Reflect on recent experience (the shared log) + traits -> drift the intrinsic-reward
        weights + keep the pursuit's objective in sync. Deterministic, local. Returns whether
        the purpose changed; persists + announces it when it did (or on `force`)."""
        if self._pursuit is None:
            return False
        traits = traits if traits is not None else dict(self._traits_snapshot())
        exp = summarize_experience(self._read_cog_log())
        new, changed = reflect_purpose(self._purpose, exp, traits)
        self._purpose = new
        self._pursuit.set_objective(new["objective"]["id"])
        if changed or force:
            self._publish_purpose(self._purpose)
            self.save_purpose()
        return changed

    def apply_reward(self, value, target=None, scope="contextual"):
        """Credit a human reward to the A/B arm that produced the narrated line (contextual).
        Global reward shapes the intrinsic-reward weights via the log on the next reflection.
        Returns whether an arm was credited; persists + announces the moved stats when so."""
        if self._pursuit is None or str(scope) == "global":
            return False
        tgt = target if isinstance(target, dict) else None
        if self._pursuit.on_reward(value, tgt):
            self.save_experiments()
            self._publish_experiments(self.summary())
            return True
        return False

    def set_reflecting(self, on, traits=None):
        """Enter/leave reflection. On entry, consolidate the local brain (reflect on purpose +
        finalize the A/B winners). Returns True iff the flag changed."""
        on = bool(on)
        if on == self.reflecting:
            return False
        self.reflecting = on
        if on:
            self.run_reflection(traits, force=True)
            self.finalize_experiments()
        return True

    def finalize_experiments(self):
        """Log the current A/B winners and persist + announce the experiment stats."""
        if self._pursuit is None:
            return
        summ = self._pursuit.summary()
        winners = ", ".join(f"{eid}->{e['winner']}"
                            for eid, e in summ["experiments"].items())
        self._log(f"A/B winners: {winners or '(none)'}")
        self.save_experiments()
        self._publish_experiments(summ)

    def tick(self, now):
        """Slow loop (driven off the node's tick): reflect on the period (faster while
        reflecting) and republish the latched readouts on a heartbeat for late subscribers."""
        if self.enable and self._pursuit is not None:
            period = 30.0 if self.reflecting else self._reflect_period
            if (now - self._reflect_last) >= period:
                self._reflect_last = now
                self.run_reflection()
        if (now - self._pub_hb) >= 5.0:                    # heartbeat republish (cheap, small)
            self._pub_hb = now
            self._publish_purpose(self._purpose)
            self._publish_experiments(self.summary())

    # ---- persistence --------------------------------------------------------
    def save_purpose(self):
        save_json(self._purpose_path, self._purpose, logger=self._log)

    def save_experiments(self):
        if self._pursuit is not None:
            save_json(self._experiments_path, self._pursuit.to_state(), logger=self._log)

    def save(self):
        self.save_purpose()
        self.save_experiments()


class Personality:
    """The chart-context personality glue (robot-side): the seed, the `evolve` / `brain_lost`
    events that drive the presence statechart's live `traits`/`registry`, the fast pick-up
    nudge, and the latched publish + throttled persist of the current personality.

    The chart owns the *live* values (smoothed in `presence.apply_evolve`); this wraps the
    ROS/event plumbing around it. The pure smoothing/revert logic stays in `presence.py`."""

    def __init__(self, *, path="", logger=None, heartbeat_enable=True, brain_timeout=90.0,
                 nudge_pickup_caution=0.92, nudge_pickup_playful=0.3,
                 publish=None, save_period=15.0):
        self.path = path or os.path.expanduser("~/.local/state/nanobot/personality.json")
        self._log = logger or (lambda *_: None)
        self.data = load_personality(self.path, with_defaults=True, logger=self._log)
        self._heartbeat_enable = bool(heartbeat_enable)
        self._brain_timeout = float(brain_timeout)
        self._nudge_caution = float(nudge_pickup_caution)
        self._nudge_playful = float(nudge_pickup_playful)
        self._publish = publish or (lambda _s: None)
        self._save_period = float(save_period)
        self._interp = None
        # evolution / heartbeat state
        self._last_brain = time.monotonic()
        self._brain_lost = False
        self._was_picked = False
        # publish/persist bookkeeping
        self._last_pub = None
        self._dirty = False
        self._last_save = 0.0

    def attach(self, interp):
        """Bind the live Sismic interpreter once it's built (its context holds the live dicts)."""
        self._interp = interp

    # convenience accessors for the seed
    @property
    def name(self):
        return self.data.get("name", "Nano")

    @property
    def persona(self):
        return self.data.get("persona", "")

    @property
    def traits(self):
        return self.data["traits"]

    @property
    def registry(self):
        return self.data["registry"]

    @property
    def drives(self):
        return self.data["drives"]

    def live_traits(self):
        """The current live traits from the chart context (the seed before the chart is up)."""
        if self._interp is not None:
            return dict(self._interp.context["traits"])
        return dict(self.data["traits"])

    def live_drives(self):
        """The current live drives (energy/focus/introspection/mood) from the chart context."""
        if self._interp is not None:
            return dict(self._interp.context["drives"])
        return dict(self.data["drives"])

    # ---- evolution events ---------------------------------------------------
    def on_evolve(self, payload):
        """A trait/registry proposal from the cognitive layer -> a Sismic `evolve` event
        (smoothed in the chart, unless `payload["hard"]` — a deliberate web-UI edit — sets it
        exactly and re-baselines). Also feeds the heartbeat (the brain is alive). `payload` is
        the decoded /cognition/evolve dict. Returns True iff the brain just came back."""
        if self._interp is None or Event is None:
            return False
        self._interp.queue(Event("evolve", traits=payload.get("traits") or {},
                                 registry=payload.get("registry") or {},
                                 drives=payload.get("drives") or {},
                                 hard=bool(payload.get("hard", False))))
        self._last_brain = time.monotonic()
        came_back = self._brain_lost
        self._brain_lost = False
        return came_back

    def tick_events(self, now, picked):
        """Queue the fast pick-up nudge + the heartbeat revert (both Sismic events, applied on
        the node's next chart execute). Returns one of None / "lost" for the node to log."""
        if self._interp is None or Event is None:
            return None
        if picked and not self._was_picked:                # being handled -> warier, less playful
            self._interp.queue(Event("evolve", registry={}, drives={}, traits={
                "caution": self._nudge_caution, "playfulness": self._nudge_playful}))
        self._was_picked = picked
        if (self._heartbeat_enable and not self._brain_lost
                and (now - self._last_brain) > self._brain_timeout):
            self._interp.queue(Event("brain_lost"))
            self._brain_lost = True
            return "lost"
        return None

    # ---- publish + persist --------------------------------------------------
    def publish_and_persist(self, now):
        """Publish (latched) the current personality on change; persist it, throttled."""
        if self._interp is None:
            return
        ctx = self._interp.context
        snap = (dict(ctx["traits"]), copy.deepcopy(ctx["registry"]), dict(ctx["drives"]))
        if snap != self._last_pub:
            self._publish({"traits": snap[0], "registry": snap[1], "drives": snap[2]})
            self._last_pub = snap
            self._dirty = True
        if self._dirty and (now - self._last_save) > self._save_period:
            self.save()
            self._dirty = False
            self._last_save = now

    def save(self):
        """Persist the current (drifted) personality back to personality.json. Best-effort."""
        if self._interp is not None:
            ctx = self._interp.context
            self.data["traits"] = dict(ctx["traits"])
            self.data["registry"] = copy.deepcopy(ctx["registry"])
            self.data["drives"] = dict(ctx["drives"])
        save_json(self.path, self.data, logger=self._log)
