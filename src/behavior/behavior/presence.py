"""The "presence" statechart (Sismic) + a builder — kept free of any ROS import so
it can be unit-tested offline:

    pixi run python -m pytest src/behavior/test

`mood_node` (the ROS node) feeds it real topics; the test feeds it a fake clock and
recording stubs. Same chart, so the test proves the behaviour offline.

The chart is the slow, **deterministic** scaffold (states + timed transitions via Sismic's
``after(...)`` guard); its side effects are the injected ``face(mood)`` / ``do_beat(name)``
callbacks. **Personality lives in the chart context** as three mutable dicts — ``traits``
(0..1: curiosity / extraversion / caution / playfulness), ``registry`` (which discretionary
beats exist + their priority/enable/gates), and ``drives`` (NEW expressive axes the LLM can
steer beyond the traits — ``energy`` / ``focus`` / ``introspection`` 0..1 + a categorical
``mood`` face). The chart reads all three, so the same chart behaves differently as the
personality evolves.

**The drives add new expressive *states*, not just weights** (so the LLM has genuinely more
influence): ``energy`` paces the idle cadence and can trigger an *energetic burst* (the
``performing`` state loops to chain a 2nd beat); ``focus`` gates a brief alert ``attending``
perk-up before a beat; a set ``mood`` makes the robot wear that face in a short ``feeling``
state between beats; ``introspection`` (read by ``mood_node``) biases how soon it drifts into
``reflecting``. All four are nudged by the SAME ``evolve`` event as traits, with the SAME
guardrails (clamped, smoothed, reverted on ``brain_lost``) and stay expression-only. At the
0.5 default every new state is OFF, so default behaviour is unchanged until the LLM pushes a
drive up (see ``drive_prob``).

**Dynamic, self-learning idle behaviour.** Instead of a hard-wired "musing every cycle,
looking every Nth" cadence, each idle cycle the chart enters one ``performing`` state that
asks the injected ``pick_beat()`` to choose ONE beat. The chooser (``choose_beat``, pure +
unit-tested) is a **priority-weighted, novelty-aware** lottery over the *enabled* registry
beats whose trait ``needs`` are met:

  * each beat's ``priority`` is its base weight — and these weights are **evolvable**
    (``evolve`` events from LLM reflection nudge them), so the robot *learns* which beats it
    prefers over time and the idle mix drifts with reward;
  * an optional ``trait`` scales the weight by a live personality axis (e.g. a more curious
    robot looks/wonders more), so the mix shifts dynamically with the mood;
  * the most-recent beat is down-weighted (``HABITUATION``) so it doesn't repeat — keeping
    the behaviour varied rather than droning the same beat.

Adding a beat = one ``BEATS`` row + one ``DEFAULT_REGISTRY`` row; no chart surgery.

**Evolution is event-driven + smoothed, and degrades safely:**
  * An ``evolve`` event (from fast rules OR slow LLM reflection) carries *target* trait
    values + registry patches; an **internal** transition (no ``target:`` → stays in the
    current state) eases the live values toward the targets by ``alpha`` (exponential
    smoothing → no jitter).
  * A ``brain_lost`` event (the node's heartbeat fired) reverts traits+registry to the
    **seeded baseline** — "safe & stupid" when the cognitive layer is unreachable. The reflex
    states never depend on the brain, so the chart keeps running regardless.

Reflexes (``greeting`` / ``resting`` / ``dormant`` + stand-down + ``reflecting``) are
deterministic and deliberately NOT in the registry, so the cognitive layer can never demote
or disable them. The discretionary beats are in BEATS + the registry — see below.
"""
import copy
import random
from collections import namedtuple

# Convention table: each beat's predefined default face (shown offline, immediately), whether
# it wants the camera and/or the microphone, and the prompt that steers its LLM enrichment.
# Faces must be OLED moods (oled_display KNOWN_MOODS): happy / focused / angry / stress / sleepy /
# looking (the wide, scanning "peeking" face shown while the camera is in use).
Beat = namedtuple("Beat", "face camera audio prompt")
BEATS = {
    "musing": Beat(
        face="happy", camera=False, audio=False,
        prompt=("React in one short spoken line to how your body and sensors feel "
                "right now.")),
    "looking": Beat(
        face="looking", camera=True, audio=False,
        prompt=("Say one short spoken line about what you can see in front of you "
                "right now.")),
    # New: a reflective "deep question" beat — the robot wonders about itself / the room.
    "wondering": Beat(
        face="focused", camera=False, audio=False,
        prompt=("Wonder aloud in one short spoken line about yourself or what might have "
                "changed around you — a small, curious thought.")),
    # New: an attentive beat that reacts to what the microphone currently hears.
    "listening": Beat(
        face="focused", camera=False, audio=True,
        prompt=("React in one short spoken line to what you can hear around you right now.")),
    # Goal-pursuit beat: delivered in place of the chosen body beat when the Horizon Planner
    # has a verified task to narrate. `{task}` is filled by mood_node._deliver_pursuing. Not in
    # the registry — it's a node-side upgrade, not a discretionary chart beat.
    "pursuing": Beat(
        face="looking", camera=True, audio=False,
        prompt="Say one short spoken line as you {task} right now."),
    # Skill beat: a node-side upgrade where web_control PICKS a capability and performs it.
    "skill": Beat(face="focused", camera=False, audio=False,
                  prompt="Choose and perform a fitting capability right now."),
}

# Personality schema + the frozen fail-safe baseline the heartbeat reverts to.
TRAIT_KEYS = ("curiosity", "extraversion", "caution", "playfulness")
DEFAULT_TRAITS = {"curiosity": 0.5, "extraversion": 0.5, "caution": 0.6, "playfulness": 0.5}
# The discretionary idle beats + their LEARNABLE weights. `priority` is the base lottery
# weight (evolvable); `needs` gates the beat on trait thresholds; `trait` scales the weight by
# a live personality axis. pursuing/skill are NOT here (node-side upgrades, not discretionary).
DEFAULT_REGISTRY = {
    "musing":    {"priority": 0.5, "enabled": True},
    "looking":   {"priority": 0.4, "enabled": True,
                  "needs": {"curiosity": 0.3}, "trait": "curiosity"},
    "wondering": {"priority": 0.3, "enabled": True,
                  "needs": {"curiosity": 0.35}, "trait": "curiosity"},
    "listening": {"priority": 0.3, "enabled": True,
                  "needs": {"extraversion": 0.3}, "trait": "extraversion"},
}

# How hard the most-recent beat is down-weighted next cycle (0..1; lower = stronger novelty
# drive). Keeps the idle behaviour from droning the same beat without ever forbidding a repeat.
HABITUATION = 0.4

# The LLM-steerable "drives" — NEW expressive axes beyond the four traits. Like traits they live
# as a mutable dict in the chart context, are nudged by the slow LLM reflection through the same
# `evolve` event (clamped + exponentially smoothed), revert to the seed on a brain-loss heartbeat,
# and are EXPRESSION-ONLY (they never move the robot). Three 0..1 scalars + one categorical face:
#   energy        — pacing/restlessness: scales the idle cadence + drives an "energetic burst"
#                   (chain a 2nd beat) on top of extraversion.
#   focus         — attention: gates a brief alert "attending" perk-up (scan) before a beat.
#   introspection — how readily the robot drifts into reflection mode on a long idle (read by
#                   mood_node to scale reflect_auto_idle; not a chart transition).
#   mood          — a baseline emotional face worn briefly (the "feeling" state) between beats;
#                   "" = no tint (return straight to the dashboard).
DRIVE_KEYS = ("energy", "focus", "introspection")          # the smoothed 0..1 scalars
DEFAULT_DRIVES = {"energy": 0.5, "focus": 0.5, "introspection": 0.5, "mood": ""}
# Faces the LLM may set as the idle `mood` tint (OLED moods + "" for none). Anything else is
# ignored so a stray value can't blank or corrupt the panel.
MOOD_FACES = ("", "happy", "angry", "focused", "stress", "neutral", "looking", "sleepy")
# Max probability of an energetic burst / an attentive perk-up at FULL energy / focus. 0.5 is the
# neutral "off" point: at the default 0.5 the new states NEVER fire (default behaviour is exactly
# unchanged), and the LLM must push a drive ABOVE 0.5 to bring them out — linearly up to the max.
BURST_MAX = 0.5
ATTEND_MAX = 0.6


def drive_prob(value, ceiling):
    """Map a 0..1 drive to a 0..ceiling probability: 0 at/below 0.5, `ceiling` at 1.0 (linear)."""
    return ceiling * clamp01((value - 0.5) / 0.5)


def clamp01(v, default=0.5):
    try:
        return max(0.0, min(1.0, float(v)))
    except (TypeError, ValueError):
        return default


def choose_beat(traits, registry, rng, camera_beats=True, last=None, beats=BEATS):
    """Pure, deterministic (rng injected) idle-beat lottery — the heart of the dynamic,
    self-learning idle behaviour. Returns a beat name (a key of `beats`) or "" for "nothing
    eligible".

    Eligibility: the beat is in `beats`, ``enabled`` in the registry, its camera is allowed by
    ``camera_beats``, and every trait in its ``needs`` meets the threshold. Weight: the
    registry ``priority`` (the learnable base), scaled by the live ``trait`` axis if given
    (0.5..1.5), and down-weighted by ``HABITUATION`` if it's the beat that just ran. A weighted
    random draw picks among the survivors — so higher-priority / on-trait beats fire more often
    but the mix stays varied and shifts as the personality evolves."""
    cands = []
    for name, cfg in (registry or {}).items():
        if name not in beats or not isinstance(cfg, dict):
            continue
        if not cfg.get("enabled", True):
            continue
        if beats[name].camera and not camera_beats:
            continue
        if any(clamp01(traits.get(k, 0.5)) < float(v)
               for k, v in (cfg.get("needs") or {}).items()):
            continue
        w = max(0.0, float(cfg.get("priority", 0.5)))
        axis = cfg.get("trait")
        if axis:
            w *= 0.5 + clamp01(traits.get(axis, 0.5))          # 0.5..1.5 by the live trait
        if name == last:
            w *= HABITUATION                                    # boredom: avoid repeating
        if w > 0:
            cands.append((name, w))
    if not cands:
        return ""
    total = sum(w for _, w in cands)
    r = rng.random() * total
    upto = 0.0
    for name, w in cands:
        upto += w
        if r <= upto:
            return name
    return cands[-1][0]


# `after(...)` is Sismic's elapsed-time guard; `face`/`do_beat`/`pick_beat`/`apply_evolve`/
# `revert` and the *_secs / traits values are injected via initial_context. Each idle cycle the
# chart waits (scaled by extraversion — an outgoing robot comes alive sooner; default 0.5 ->
# exactly idle_secs), enters `performing`, and `pick_beat()` chooses ONE beat to fire.
PRESENCE_YAML = """
statechart:
  name: Nano presence
  description: Idle "feel alive" OLED-face supervisor (expression only).
  root state:
    name: presence
    initial: greeting
    transitions:
      - event: evolve
        action: apply_evolve(event.traits, event.registry, getattr(event, 'drives', {}))
      - event: brain_lost
        action: revert()
      # Reflection mode: from anywhere, drop into a calm "reflecting" state that pauses the
      # idle beats (the node consolidates the brain + forges skills in the background) until `wake`.
      - event: reflect
        target: reflecting
    states:
      - name: greeting
        on entry: face(greet_face)
        transitions:
          - target: idle_life
            guard: after(greet_secs)
          - event: standdown
            target: dormant
      - name: idle_life
        initial: resting
        transitions:
          - event: standdown
            target: dormant
        states:
          # Cadence: idle_secs, lengthened by calmness (low extraversion) AND low energy, so a
          # restless/outgoing robot comes alive sooner. On entry we DECIDE (once, with the rng)
          # whether this cycle perks up into the focus-gated "attending" state; the two outgoing
          # guards then read that flag so they stay mutually exclusive (Sismic forbids two
          # simultaneously-enabled eventless transitions from one state).
          - name: resting
            on entry: face(''); decide_attend()
            transitions:
              - target: attending
                guard: after(idle_secs * (1.4 - 0.8 * traits['extraversion']) * (1.3 - 0.6 * drives['energy'])) and attend_next()
              - target: performing
                guard: after(idle_secs * (1.4 - 0.8 * traits['extraversion']) * (1.3 - 0.6 * drives['energy'])) and not attend_next()
          # Attention: briefly perk up with the alert face before acting (focus-driven). A
          # short, expressive "I noticed something" pause that precedes the beat.
          - name: attending
            on entry: face(attend_face)
            transitions:
              - target: performing
                guard: after(attend_secs)
          # One beat per cycle: the chooser picks WHICH (priority-weighted, novelty-aware,
          # trait-gated). do_beat('') (nothing eligible) is a harmless no-op in the node. On entry
          # we also DECIDE the next step once (burst / feel / rest) so the three guards below are
          # mutually exclusive: an energetic burst chains another beat, else briefly wear the
          # baseline mood (if set), else rest.
          - name: performing
            on entry: do_beat(pick_beat()); decide_next()
            transitions:
              - target: performing
                guard: after(perform_secs) and next_step() == 'burst'
              - target: feeling
                guard: after(perform_secs) and next_step() == 'feel'
              - target: resting
                guard: after(perform_secs) and next_step() == 'rest'
          # Wear the LLM's chosen baseline mood for a moment, then return to the dashboard.
          - name: feeling
            on entry: face(drives['mood'])
            transitions:
              - target: resting
                guard: after(feel_secs)
      - name: dormant
        transitions:
          - event: resume
            target: idle_life
      - name: reflecting
        on entry: face(reflect_face)
        transitions:
          - event: wake
            target: idle_life
"""


def build_interpreter(face, do_beat=None, greet_secs=3.0, idle_secs=90.0,
                      perform_secs=4.0, camera_beats=True, look_every=4,
                      traits=None, registry=None, drives=None, alpha=0.1, clock=None,
                      reflect_face="focused", greet_face="happy", attend_face="looking",
                      attend_secs=2.0, feel_secs=2.5, rng=None):
    """Parse + validate the chart and return (interpreter, clock), already advanced into
    `greeting`. `traits`/`registry`/`drives` seed the live personality (merged over the frozen
    defaults); `alpha` is the exponential-smoothing rate for `evolve`. `rng` (injected for
    deterministic tests) drives the idle-beat lottery + the burst/attend gates. The live dicts
    are `interpreter.context['traits' | 'registry' | 'drives']` — the node reads them to colour
    prompts, persist, and publish. Sismic is imported lazily so importing this module never needs
    it.

    `look_every` is accepted for backward-compat but no longer used — the camera cadence is now
    driven by the `looking` beat's learnable priority/trait in the registry (see choose_beat)."""
    from sismic.io import import_from_yaml
    from sismic.interpreter import Interpreter
    from sismic.clock import SimulatedClock

    # Live dicts (deep-copied so they never alias the frozen defaults).
    live_traits = copy.deepcopy(DEFAULT_TRAITS)
    live_traits.update({k: clamp01(v) for k, v in (traits or {}).items() if k in DEFAULT_TRAITS})
    live_registry = copy.deepcopy(DEFAULT_REGISTRY)
    for name, patch in (registry or {}).items():
        live_registry.setdefault(name, {}).update(patch or {})
    live_drives = copy.deepcopy(DEFAULT_DRIVES)
    for k, v in (drives or {}).items():
        if k in DRIVE_KEYS:
            live_drives[k] = clamp01(v)
        elif k == "mood" and v in MOOD_FACES:
            live_drives["mood"] = v
    a = clamp01(alpha, 0.1)
    rng = rng if rng is not None else random.Random()
    # The safe baseline the heartbeat reverts to = the CONFIGURED personality (which is
    # known-good), not the generic defaults — so a brief brain outage drops only the drift
    # accumulated on top, never a deliberately-created character.
    base_traits = copy.deepcopy(live_traits)
    base_registry = copy.deepcopy(live_registry)
    base_drives = copy.deepcopy(live_drives)
    last_beat = {"name": None}                  # novelty memory for the chooser

    def apply_evolve(traits_patch, registry_patch, drives_patch=None):
        """Ease live traits + drive scalars toward the target values (exponential smoothing),
        merge the registry patch, and set the categorical `mood` face directly. All untrusted
        inputs are clamped / whitelisted."""
        for k, target in (traits_patch or {}).items():
            if k in live_traits:
                live_traits[k] = round((1 - a) * live_traits[k] + a * clamp01(target), 4)
        for name, patch in (registry_patch or {}).items():
            live_registry.setdefault(name, {}).update(patch or {})
        for k, target in (drives_patch or {}).items():
            if k in DRIVE_KEYS:
                live_drives[k] = round((1 - a) * live_drives[k] + a * clamp01(target), 4)
            elif k == "mood" and target in MOOD_FACES:
                live_drives["mood"] = target        # categorical: set directly (still revertible)

    def revert():
        live_traits.clear(); live_traits.update(copy.deepcopy(base_traits))
        live_registry.clear(); live_registry.update(copy.deepcopy(base_registry))
        live_drives.clear(); live_drives.update(copy.deepcopy(base_drives))

    # Per-state decisions made ONCE on entry (where the rng is rolled) and then read by the
    # outgoing guards, so simultaneously-eligible eventless transitions stay mutually exclusive.
    rest_decision = {"attend": False}           # this rest cycle perks up into `attending`?
    perform_decision = {"next": "rest"}         # after this beat: 'burst' | 'feel' | 'rest'

    def decide_attend():
        """Roll whether to perk up alert this cycle — likelier the higher `focus` is (0 at 0.5)."""
        rest_decision["attend"] = rng.random() < drive_prob(live_drives["focus"], ATTEND_MAX)

    def attend_next():
        return rest_decision["attend"]

    def decide_next():
        """Roll what happens after a beat: an energetic 'burst' (chain another, likelier at high
        `energy`), else 'feel' (wear the baseline mood, if one is set), else 'rest'."""
        if rng.random() < drive_prob(live_drives["energy"], BURST_MAX):
            perform_decision["next"] = "burst"
        elif live_drives["mood"]:
            perform_decision["next"] = "feel"
        else:
            perform_decision["next"] = "rest"

    def next_step():
        return perform_decision["next"]

    def pick_beat():
        """The chart's per-cycle beat chooser: a priority-weighted, novelty-aware draw over the
        live (evolving) registry. Remembers what it picked so the next cycle can avoid it."""
        name = choose_beat(live_traits, live_registry, rng, camera_beats=camera_beats,
                           last=last_beat["name"])
        if name:
            last_beat["name"] = name
        return name

    statechart = import_from_yaml(PRESENCE_YAML)
    statechart.validate()
    clock = clock if clock is not None else SimulatedClock()
    interpreter = Interpreter(statechart, clock=clock, initial_context={
        "face": face,
        "do_beat": do_beat or (lambda _name: None),
        "pick_beat": pick_beat,
        "decide_attend": decide_attend,
        "attend_next": attend_next,
        "decide_next": decide_next,
        "next_step": next_step,
        "apply_evolve": apply_evolve,
        "revert": revert,
        "greet_secs": float(greet_secs),
        "idle_secs": float(idle_secs),
        "perform_secs": float(perform_secs),
        "attend_secs": float(attend_secs),
        "feel_secs": float(feel_secs),
        "traits": live_traits,
        "registry": live_registry,
        "drives": live_drives,
        "reflect_face": str(reflect_face),
        "greet_face": str(greet_face),
        "attend_face": str(attend_face),
    })
    interpreter.execute()        # run the initial step -> enter `greeting`
    return interpreter, clock
