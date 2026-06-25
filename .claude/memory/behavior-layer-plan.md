---
name: behavior-layer-plan
description: Plan for the robot behaviour/supervisor layer — leaning Sismic statecharts; design + why-not alternatives
metadata:
  node_type: memory
  type: project
---

Planning a **behaviour / supervisor layer** to coordinate modes + reflexes and make the
robot "feel alive". Decided direction (2026-06-24): **Sismic** (pure-Python statechart
interpreter, pip/`pypi-dependencies`).

**STEP 1 BUILT (2026-06-25): `src/behavior` package + `mood_node`.** First, deliberately
safe slice: an idle "feel alive" **presence supervisor**. Chart = `behavior/presence.py`
(ROS-free, unit-tested offline `pixi run python -m pytest src/behavior/test`); node =
`behavior/mood_node.py`. States: greeting → idle_life{resting↔performing via `after()`}
with a top-level `standdown`/`resume` preemption to `dormant`. **Expression-ONLY**: drives
`/oled_face` during true idle, NEVER `/cmd_vel` (can't affect motion safety). **Yields the
panel** to all existing /oled_face owners — web manual mood, TTS `/oled_word`, slam_nav
pick-up `pickup_face` — via standdown on motion/goal/speech/manual/pick-up, with echo
suppression on its own publishes (so it doesn't fight them; slam_nav left untouched). No-op
if sismic missing or `behavior.enable:=false`. Sismic added to `pixi.toml`
(`sismic>=1.6`); wired into `stack.sh` (launch+down+status) + `robot.yaml` (`behavior:`).
Ran as its OWN process (not folded into sensor_hub yet) so a bug can't sink the sensors —
isolation over the ~35 MB RAM save for v1. NOTE: couldn't run sismic locally (no python on
the win dev box) — validate the chart on the board/dev-host via the pytest before trusting.

**Cost / scaling (estimated, not yet measured on the board):** CPU is a non-issue —
~0.1–0.3% of one core at 4 Hz (each tick = a few cached-`eval()` guard checks on the
*active* states only). RAM is the real cost: **~35–45 MB RSS standalone** (≈30–35 MB
rclpy+rmw process baseline + ~5–8 MB sismic import); folding into `sensor_hub` later drops
the marginal cost to just the ~5–8 MB import. Scaling: CPU ∝ active-config fan-out × tick
rate (NOT total state count), so adding moods/states is ~free; the cost levers are tick
rate, per-sample `execute()` on a fast topic, and # of concurrently-active parallel
regions (all linear, tiny base). RAM ∝ chart size but negligibly (states are small Python
objects). **Real caveat = action design, not Sismic:** never do blocking I/O (e.g. a sync
HTTP POST to TTS) in `on entry`/`on exit` — it stalls the single executor; offload it.

**Next steps (not built):** drive TTS greeting (HTTP POST :8080/tts, OFF the executor);
add a parallel reflex region; eventually consolidate slam_nav's pickup/relocalize faces;
fold into sensor_hub once proven (recovers ~30+ MB); measure real CPU/RAM via the
per-process/per-thread profiling ([[sbc-cpu-profile]]).

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
