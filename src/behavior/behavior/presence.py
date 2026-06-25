"""The "presence" statechart (Sismic) + a builder — kept free of any ROS import so
it can be unit-tested offline:

    pixi run python -m pytest src/behavior/test

`mood_node` (the ROS node) feeds it real topics; the test feeds it a fake clock and a
recording `face` stub. Same chart, so the test proves the behaviour without hardware.

The chart is the slow "brain": states + timed transitions via Sismic's built-in
``after(...)`` guard. Timing thresholds come in as context variables so they're tuned
from robot.yaml. The only side effect is calling the injected ``face(mood)`` callback.
"""

# `after(...)` is Sismic's built-in elapsed-time guard predicate; `face(...)` and the
# *_secs values are injected via initial_context (see build_interpreter / the node).
PRESENCE_YAML = """
statechart:
  name: Nano presence
  description: Idle "feel alive" OLED-face supervisor (expression only).
  root state:
    name: presence
    initial: greeting
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
              - target: performing
                guard: after(idle_secs)
          - name: performing
            on entry: face('happy')
            transitions:
              - target: resting
                guard: after(perform_secs)
      - name: dormant
        transitions:
          - event: resume
            target: idle_life
"""


def build_interpreter(face, greet_secs=3.0, idle_secs=90.0, perform_secs=4.0,
                      clock=None):
    """Parse + validate the chart and return (interpreter, clock), already advanced
    through its initial step (so it has entered `greeting`). `face` is the callback the
    chart invokes on state entry; *_secs are the timed-transition thresholds.

    Sismic is imported lazily here so importing this module never requires it."""
    from sismic.io import import_from_yaml
    from sismic.interpreter import Interpreter
    from sismic.clock import SimulatedClock

    statechart = import_from_yaml(PRESENCE_YAML)
    statechart.validate()
    clock = clock if clock is not None else SimulatedClock()
    interpreter = Interpreter(statechart, clock=clock, initial_context={
        "face": face,
        "greet_secs": float(greet_secs),
        "idle_secs": float(idle_secs),
        "perform_secs": float(perform_secs),
    })
    interpreter.execute()        # run the initial step -> enter `greeting`
    return interpreter, clock
