"""The "presence" statechart (Sismic) + a builder — kept free of any ROS import so
it can be unit-tested offline:

    pixi run python -m pytest src/behavior/test

`mood_node` (the ROS node) feeds it real topics; the test feeds it a fake clock and
recording stubs. Same chart, so the test proves the behaviour offline.

The chart is the slow, **deterministic** "brain": states + timed transitions via Sismic's
``after(...)`` guard. Its side effects are the injected ``face(mood)`` / ``do_beat(name)``
callbacks. **Personality lives in the chart context** as two mutable dicts — ``traits``
(0..1: curiosity / extraversion / caution / playfulness) and ``registry`` (which
discretionary beats exist + their priority/enable + gates). Guards read them, so the same
chart behaves differently as the personality evolves.

**Evolution is event-driven + smoothed, and degrades safely:**
  * An ``evolve`` event (from fast rules OR slow LLM reflection) carries *target* trait
    values + registry patches; an **internal** transition (no ``target:`` → stays in the
    current state) eases the live values toward the targets by ``alpha`` (exponential
    smoothing → no jitter).
  * A ``brain_lost`` event (the node's heartbeat fired) reverts traits+registry to the
    frozen ``DEFAULT_TRAITS`` / ``DEFAULT_REGISTRY`` — "safe & stupid" when the cognitive
    layer is unreachable. The reflex states never depend on the brain, so the chart keeps
    running regardless.

Reflexes (``greeting`` / ``resting`` / ``dormant`` + stand-down) are deterministic and are
deliberately NOT in the registry, so the cognitive layer can never demote or disable them.
The enrichable beats are ``musing`` (sensors) and ``looking`` (camera) — see BEATS.
"""
import copy
from collections import namedtuple

# Convention table: each enrichable beat's predefined default face (shown offline,
# immediately), the prompt that steers its LLM enrichment, and whether it wants a camera.
Beat = namedtuple("Beat", "face camera prompt")
BEATS = {
    "musing": Beat(
        face="happy", camera=False,
        prompt=("React in one short spoken line to how your body and sensors feel "
                "right now.")),
    "looking": Beat(
        face="focused", camera=True,
        prompt=("Say one short spoken line about what you can see in front of you "
                "right now.")),
}

# Personality schema + the frozen fail-safe baseline the heartbeat reverts to.
TRAIT_KEYS = ("curiosity", "extraversion", "caution", "playfulness")
DEFAULT_TRAITS = {"curiosity": 0.5, "extraversion": 0.5, "caution": 0.6, "playfulness": 0.5}
DEFAULT_REGISTRY = {
    "musing":  {"priority": 0.5, "enabled": True},
    "looking": {"priority": 0.4, "enabled": True, "needs": {"curiosity": 0.3}},
}


def clamp01(v, default=0.5):
    try:
        return max(0.0, min(1.0, float(v)))
    except (TypeError, ValueError):
        return default


# `after(...)` is Sismic's elapsed-time guard; `face`/`do_beat`/`apply_evolve`/`revert`
# and the *_secs / beat_i / camera_beats / look_every / traits / registry values are
# injected via initial_context. Beats rotate from `resting`: `musing` (sensors) every
# idle cycle, `looking` (camera) every look_every-th cycle, gated by camera_beats AND the
# registry AND a curiosity threshold. The idle wait is scaled by extraversion (an outgoing
# robot comes alive sooner; default 0.5 -> exactly idle_secs).
PRESENCE_YAML = """
statechart:
  name: Nano presence
  description: Idle "feel alive" OLED-face supervisor (expression only).
  root state:
    name: presence
    initial: greeting
    transitions:
      - event: evolve
        action: apply_evolve(event.traits, event.registry)
      - event: brain_lost
        action: revert()
    states:
      - name: greeting
        on entry: face('happy')
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
          - name: resting
            on entry: face('')
            transitions:
              - target: looking
                guard: after(idle_secs * (1.4 - 0.8 * traits['extraversion']))
                       and camera_beats and registry['looking']['enabled']
                       and traits['curiosity'] >= registry['looking']['needs']['curiosity']
                       and (beat_i % look_every == look_every - 1)
              - target: musing
                guard: after(idle_secs * (1.4 - 0.8 * traits['extraversion']))
                       and registry['musing']['enabled']
                       and not (camera_beats and registry['looking']['enabled']
                                and traits['curiosity'] >= registry['looking']['needs']['curiosity']
                                and (beat_i % look_every == look_every - 1))
              # Nothing eligible this cycle (e.g. musing disabled, not yet a look cycle):
              # self-transition just advances the counter + resets the timer, so the
              # `looking` cadence still progresses. Guards are mutually exclusive with the
              # two above (Sismic would otherwise flag non-determinism).
              - target: resting
                guard: after(idle_secs * (1.4 - 0.8 * traits['extraversion']))
                       and not registry['musing']['enabled']
                       and not (camera_beats and registry['looking']['enabled']
                                and traits['curiosity'] >= registry['looking']['needs']['curiosity']
                                and (beat_i % look_every == look_every - 1))
                action: beat_i = beat_i + 1
          - name: musing
            on entry: do_beat('musing')
            transitions:
              - target: resting
                guard: after(perform_secs)
                action: beat_i = beat_i + 1
          - name: looking
            on entry: do_beat('looking')
            transitions:
              - target: resting
                guard: after(perform_secs)
                action: beat_i = beat_i + 1
      - name: dormant
        transitions:
          - event: resume
            target: idle_life
"""


def build_interpreter(face, do_beat=None, greet_secs=3.0, idle_secs=90.0,
                      perform_secs=4.0, camera_beats=True, look_every=4,
                      traits=None, registry=None, alpha=0.1, clock=None):
    """Parse + validate the chart and return (interpreter, clock), already advanced into
    `greeting`. `traits`/`registry` seed the live personality (merged over the frozen
    defaults); `alpha` is the exponential-smoothing rate for `evolve`. The live dicts are
    `interpreter.context['traits' | 'registry']` — the node reads them to colour prompts,
    persist, and publish. Sismic is imported lazily so importing this module never needs it."""
    from sismic.io import import_from_yaml
    from sismic.interpreter import Interpreter
    from sismic.clock import SimulatedClock

    # Live dicts (deep-copied so they never alias the frozen defaults).
    live_traits = copy.deepcopy(DEFAULT_TRAITS)
    live_traits.update({k: clamp01(v) for k, v in (traits or {}).items() if k in DEFAULT_TRAITS})
    live_registry = copy.deepcopy(DEFAULT_REGISTRY)
    for name, patch in (registry or {}).items():
        live_registry.setdefault(name, {}).update(patch or {})
    a = clamp01(alpha, 0.1)
    # The safe baseline the heartbeat reverts to = the CONFIGURED personality (which is
    # known-good), not the generic defaults — so a brief brain outage drops only the drift
    # accumulated on top, never a deliberately-created character.
    base_traits = copy.deepcopy(live_traits)
    base_registry = copy.deepcopy(live_registry)

    def apply_evolve(traits_patch, registry_patch):
        """Ease live traits toward the target values (exponential smoothing) + merge the
        registry patch. Targets/patches are untrusted → clamped."""
        for k, target in (traits_patch or {}).items():
            if k in live_traits:
                live_traits[k] = round((1 - a) * live_traits[k] + a * clamp01(target), 4)
        for name, patch in (registry_patch or {}).items():
            live_registry.setdefault(name, {}).update(patch or {})

    def revert():
        live_traits.clear(); live_traits.update(copy.deepcopy(base_traits))
        live_registry.clear(); live_registry.update(copy.deepcopy(base_registry))

    statechart = import_from_yaml(PRESENCE_YAML)
    statechart.validate()
    clock = clock if clock is not None else SimulatedClock()
    interpreter = Interpreter(statechart, clock=clock, initial_context={
        "face": face,
        "do_beat": do_beat or (lambda _name: None),
        "apply_evolve": apply_evolve,
        "revert": revert,
        "greet_secs": float(greet_secs),
        "idle_secs": float(idle_secs),
        "perform_secs": float(perform_secs),
        "camera_beats": bool(camera_beats),
        "look_every": max(1, int(look_every)),
        "beat_i": 0,
        "traits": live_traits,
        "registry": live_registry,
    })
    interpreter.execute()        # run the initial step -> enter `greeting`
    return interpreter, clock
