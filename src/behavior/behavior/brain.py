"""The brain's ROS-free orchestration layer — shared by the robot and the dev harness.

This finishes the pattern `web_control.cognition.CognitionCore` started for the LLM side:
the platform-agnostic *orchestration* of the Purpose Engine + Horizon Planner and of the
chart-context personality lives here, in ONE place, so the robot behaviour node
(`behavior.mood_node`) and the dev web harness (`scripts/dev_webui.py`) compose the same
code instead of each re-implementing it (they used to, and were already drifting).

Like `CognitionCore`, everything platform-specific is injected as a tiny **adapter**:

    PurposeBrain adapters (all optional):
      read_cog_log()          -> the recent decision-log entries (list of dicts)
      picked()                -> True while the robot is held off the ground (bool)
      traits_snapshot()       -> the live personality traits dict (for reflection)
      publish_purpose(obj)    -> announce the current purpose          (robot: latched topic)
      publish_task(payload)   -> announce the current pursued task     (dev: store for HTTP)
      publish_experiments(s)  -> announce the A/B experiment summary
      logger(msg)             -> log a line

`Personality` wraps the Sismic-chart-context personality glue (seed load, evolve/heartbeat
events, latched publish + throttled persist) used by the robot node. The pure modules it
builds on — `presence` / `purpose` / `planner` — are untouched and still unit-tested offline:

    pixi run python -m pytest src/behavior/test

Everything here is deterministic + narrative-only: it never moves the robot.
"""
import copy
import json
import os
import time

from .presence import DEFAULT_TRAITS, DEFAULT_REGISTRY
from .purpose import merge_purpose, reflect_purpose, summarize_experience
from .planner import Planner

# Sismic Event is only needed by Personality (interpreter-coupled). Imported defensively so
# importing this module on a board without sismic still works (e.g. for PurposeBrain alone).
try:
    from sismic.model import Event
except Exception:  # pragma: no cover - depends on env
    Event = None


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
                "registry": copy.deepcopy(DEFAULT_REGISTRY)}
    else:
        base = {"name": "Nano", "persona": "", "traits": {}, "registry": {}}
    saved = load_json(path, logger=logger)
    if isinstance(saved, dict):
        for k in ("name", "persona"):
            if isinstance(saved.get(k), str):
                base[k] = saved[k]
        if isinstance(saved.get("traits"), dict):
            base["traits"].update(saved["traits"])
        if isinstance(saved.get("registry"), dict):
            if with_defaults:
                for n, patch in saved["registry"].items():
                    base["registry"].setdefault(n, {}).update(patch or {})
            else:
                base["registry"] = saved["registry"]
    return base


class PurposeBrain:
    """The slow "identity / strategy" layer: owns the Purpose Engine state + the Horizon
    Planner, decides when an idle beat upgrades to a goal-pursuit or a skill, reflects on
    experience, credits human reward to the A/B bandit, and consolidates while meditating.

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
        self.meditating = False
        self.task = {}                 # last pursued-task payload (the dev /task_current readout)
        self._body_beat_n = 0          # counts body (musing) beats, for the skill cadence
        self._reflect_last = 0.0       # monotonic of the last purpose reflection
        self._pub_hb = 0.0             # monotonic of the last latched republish
        self._purpose = merge_purpose(load_json(self._purpose_path, logger=self._log),
                                      name=name)
        self._planner = None
        if self.enable:
            self._planner = Planner(
                objective_id=self._purpose["objective"]["id"],
                state=load_json(self._experiments_path, logger=self._log),
                epsilon=float(epsilon), min_interval=float(pursue_min_interval))

    # ---- readouts (the dev harness serves these over HTTP) ------------------
    @property
    def purpose(self):
        return self._purpose

    def summary(self):
        return self._planner.summary() if self._planner is not None else {"experiments": {}}

    def world_state(self):
        """The light world snapshot the planner verifies a task against (narrative-only)."""
        return {"picked": bool(self._picked()), "sensors_fresh": True,
                "meditating": self.meditating}

    # ---- beat upgrades (decisions; the node executes them) ------------------
    def next_pursuing(self, now):
        """If a goal-pursuit beat is due, return its narration spec (and announce the task +
        moved A/B stats); else None. Eligible only while enabled and not meditating."""
        if not (self.enable and self._planner is not None) or self.meditating:
            return None
        spec = self._planner.next_task(self.world_state(), self._rng, now=now)
        if spec is None:
            return None
        self.task = {"task": spec["task"], "exp": spec["exp"], "variant": spec["variant"],
                     "text": spec["text"], "t": time.time()}
        self._publish_task(self.task)
        self._publish_experiments(self.summary())          # stats moved (new assignment)
        return spec

    def take_skill_beat(self):
        """Advance the skill-beat cadence and report whether THIS body beat is a skill beat.
        Only counts while skills are enabled and we're not meditating (matches the chart)."""
        if not (self.skills_enable and not self.meditating):
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

    # ---- reflection / reward / meditation -----------------------------------
    def run_reflection(self, traits=None, force=False):
        """Reflect on recent experience (the shared log) + traits -> drift the intrinsic-reward
        weights + keep the planner's objective in sync. Deterministic, local. Returns whether
        the purpose changed; persists + announces it when it did (or on `force`)."""
        if self._planner is None:
            return False
        traits = traits if traits is not None else dict(self._traits_snapshot())
        exp = summarize_experience(self._read_cog_log())
        new, changed = reflect_purpose(self._purpose, exp, traits)
        self._purpose = new
        self._planner.set_objective(new["objective"]["id"])
        if changed or force:
            self._publish_purpose(self._purpose)
            self.save_purpose()
        return changed

    def apply_reward(self, value, target=None, scope="contextual"):
        """Credit a human reward to the A/B arm that produced the narrated line (contextual).
        Global reward shapes the intrinsic-reward weights via the log on the next reflection.
        Returns whether an arm was credited; persists + announces the moved stats when so."""
        if self._planner is None or str(scope) == "global":
            return False
        tgt = target if isinstance(target, dict) else None
        if self._planner.on_reward(value, tgt):
            self.save_experiments()
            self._publish_experiments(self.summary())
            return True
        return False

    def set_meditating(self, on, traits=None):
        """Enter/leave meditation. On entry, consolidate the local brain (reflect on purpose +
        finalize the A/B winners). Returns True iff the flag changed."""
        on = bool(on)
        if on == self.meditating:
            return False
        self.meditating = on
        if on:
            self.run_reflection(traits, force=True)
            self.finalize_experiments()
        return True

    def finalize_experiments(self):
        """Log the current A/B winners and persist + announce the experiment stats."""
        if self._planner is None:
            return
        summ = self._planner.summary()
        winners = ", ".join(f"{eid}->{e['winner']}"
                            for eid, e in summ["experiments"].items())
        self._log(f"A/B winners: {winners or '(none)'}")
        self.save_experiments()
        self._publish_experiments(summ)

    def tick(self, now):
        """Slow loop (driven off the node's tick): reflect on the period (faster while
        meditating) and republish the latched readouts on a heartbeat for late subscribers."""
        if self.enable and self._planner is not None:
            period = 30.0 if self.meditating else self._reflect_period
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
        if self._planner is not None:
            save_json(self._experiments_path, self._planner.to_state(), logger=self._log)

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

    def live_traits(self):
        """The current live traits from the chart context (the seed before the chart is up)."""
        if self._interp is not None:
            return dict(self._interp.context["traits"])
        return dict(self.data["traits"])

    # ---- evolution events ---------------------------------------------------
    def on_evolve(self, payload):
        """A trait/registry proposal from the cognitive layer -> a Sismic `evolve` event
        (smoothed in the chart). Also feeds the heartbeat (the brain is alive). `payload` is the
        decoded /cognition/evolve dict. Returns True iff the brain just came back."""
        if self._interp is None or Event is None:
            return False
        self._interp.queue(Event("evolve", traits=payload.get("traits") or {},
                                 registry=payload.get("registry") or {}))
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
            self._interp.queue(Event("evolve", registry={}, traits={
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
        snap = (dict(ctx["traits"]), copy.deepcopy(ctx["registry"]))
        if snap != self._last_pub:
            self._publish({"traits": snap[0], "registry": snap[1]})
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
        save_json(self.path, self.data, logger=self._log)
