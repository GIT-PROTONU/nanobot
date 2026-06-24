---
name: behavior-layer-plan
description: Plan for the robot behaviour/supervisor layer — leaning Sismic statecharts; design + why-not alternatives
metadata:
  node_type: memory
  type: project
---

Planning a **behaviour / supervisor layer** to coordinate modes + reflexes and make the
robot "feel alive". Decided direction (2026-06-24): **probably Sismic** (pure-Python
statechart interpreter, pip/`pypi-dependencies`). Nothing built yet — design only.

**Why Sismic over the alternatives we weighed:**
- **Hand-rolled FSM** — fine for a flat machine, but doing *correct* parallel-region +
  history semantics (LCA exit/entry sets, microstep loop, conflict resolution) by hand is a
  few hundred subtle lines / a bug farm. That's exactly where a tested lib earns its place.
- **YASMIN** — ROS2 FSM, has Python, but blocking `execute()`/outcome model makes
  cross-cutting **preemption** (pickup/lost/thermal interrupting any state) awkward; not in
  RoboStack (would vendor into `src/`); viewer is dev-only.
- **Zenoh-Flow** — wrong category (distributed **dataflow**, not behaviour); reintroduces the
  Rust toolchain we deliberately removed (see pixi.toml note); experimental/intermittent.
- **SCXML** — the right *model* (it's the statechart standard; `<parallel>` regions = clean
  preemption) but the runtime is the problem: Python engines weak (`pyscxml` unmaintained) or
  heavy (Qt `QScxml`); Bosch/CONVINCE SCXML-for-ROS is verification-focused/heavy; XML ceremony.
- **Sismic** wins when we want **true orthogonal reflex region + history** (the right design
  for safety reflexes): correct semantics for free + built-in testing of safety logic, pure
  Python, light.

**Architecture rules (keep it cheap on the 1 GB H5):**
- **Statechart = slow brain** (moods + reflexes, tick 1–5 Hz). **The OLED node keeps the fast
  face animation** ([[oled-display-perf]]) — do NOT drive per-frame eyes through the chart.
- **Reflexes are instant**: `interpreter.queue(Event(...)); interpreter.execute()` *in the
  topic callback*; the slow background tick only handles `after(...)`/idle behaviour.
- **Parallel "reflex/safety" region preempts the nav region; history resumes** afterwards.
- **Fold into an existing executor** (sensor_hub-style) to skip a ~35 MB interpreter; Sismic
  adds only **~5–8 MB**, and CPU is **<0.5% of a core** at a sane tick.
- Consumes existing events: `/left_wheel_suspended`, `/right_wheel_suspended`, relocalize
  status, `/diagnostics` temp, `/cmd_vel` (teleop), `/goal_pose`. Drives `/cmd_vel` intent,
  `/oled_face`, and TTS ([[tts-speech]]).
- It would consolidate behaviour currently half-embedded in `slam_nav`
  (`pickup_pause`/`pickup_face`/`relocalize`/`auto_explore` — see [[slam-autonomy-pickup-relocalize]]).
