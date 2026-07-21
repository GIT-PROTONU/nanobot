# Nano's Brain — the autonomous personality system

How Nano decides to "say something", what it says, and how the personality grows over
time. Written for a new contributor — read this before touching `behavior/` or the LLM
paths in `web_control/`.

## The one principle that explains everything

**The brain is a garnish, never load-bearing.** Nano is a robot first. Every "thinking"
feature degrades to *silence* when there's no internet, no API key, or the model is slow.
The statechart never waits on the LLM, motion never waits on the brain, and nothing the
brain does can make the robot unsafe. Most design choices below fall out of this.

## Three layers, separated on purpose

The brain is split into **when**, **what**, and **how**:

```
  ┌─────────────────────────────────────────────────────────────────┐
  │  WHEN to think      WHAT to say              HOW to show it        │
  │  ──────────────     ─────────────            ──────────────       │
  │  behavior/          web_control/llm.py       OLED face (/oled_face)│
  │  presence.py        + web_server cognition   TTS voice (tts.py)    │
  │  (Sismic            (OpenRouter calls)                             │
  │   statechart)                                                      │
  └─────────────────────────────────────────────────────────────────┘
        the "brain"           the "voice"            the "body"
```

- **`src/behavior/behavior/presence.py`** — a **statechart** (a formal state machine, via
  the [Sismic](https://sismic.readthedocs.io/) library) that decides *when* the robot acts.
  Pure Python, ROS-free, unit-tested offline
  (`pixi run python -m pytest src/behavior/test`). It is the **single brain** — only one
  thing decides to act. `behavior/mood_node.py` is the thin ROS wrapper that maps
  topics→chart events and the chart's faces→`/oled_face`.
- **`src/web_control/web_control/llm.py`** — a dependency-free OpenRouter client that turns
  a prompt into `{"say": "...", "mood": "happy"}`. This is *what* to say. ROS-free, no SDK,
  just stdlib `urllib`.
- **OLED + TTS** — `oled_display` shows the face; `web_control/tts.py` speaks the line.
  This is *how* it's expressed.

Keeping these separate is what makes it safe: the statechart shows a face instantly and
fires off an LLM request that may never come back — and nothing breaks either way.

## The statechart: idle "beats"

When Nano is genuinely idle (nobody driving it, no goal, no manual command), each idle cycle
the chart enters one `performing` state and a **lottery** (`choose_beat`) picks ONE *enrichable
beat* to fire:

| Beat | Sense | Model used | Gated on |
|---|---|---|---|
| `musing` | its own body (CPU, RAM, temp, IMU tilt, pick-up) | cheap text | — (always eligible) |
| `looking` | the camera | vision | curiosity (+ `camera_beats`, + the LLM being usable right now) |
| `wondering` | a curious "deep question" about itself / the room | cheap text | curiosity |
| `listening` | what the microphone hears | cheap text | extraversion |

The lottery is **priority-weighted, novelty-aware, and trait-scaled**: each beat has a base
`priority` in the registry, the weight is scaled up by a live personality trait, and the beat
that just ran is down-weighted so behaviour stays varied. The priorities are **learnable** —
LLM reflection nudges them (and the traits), so the idle mix drifts toward what earns reward.
That makes idle behaviour dynamic and self-learning rather than a fixed cadence (the old
`look_every` knob is retired). `camera_beats` + idle timing still live in `robot.yaml` under
`behavior:`. There are also **reflexes** — `greeting`, `resting`, `dormant`, `reflecting`, and
pick-up reactions. These are deterministic and **not under the brain's control** (see below).

**The lottery also stops offering camera beats when there's no LLM to look for.** `web_control`
publishes a latched `cognition/llm_ready` (enabled + a key configured — a mere network blip is
handled separately by the offline-face standdown, which already pauses ALL beats) that
`mood_node` ANDs with the `camera_beats` admin toggle (`MoodNode._camera_beats_ok`) and feeds
into the chooser as a live callable, so `looking` simply isn't a candidate while the LLM is off
— not just silently dropped after the fact. The dev harness (`scripts/dev_webui.py`) mirrors
this in-process (`state.llm.available()`), and `CognitionCore.run_beat` re-checks
`llm.available()` before ever touching the camera, as a defensive backstop.

When the chart enters a beat, two things happen *independently*:

1. It **immediately** shows a sensible default face (offline-safe — works with no internet).
2. **If enrichment is on**, it fires a **fire-and-forget** request to the LLM layer
   (`/cognition/request`). If the model answers, the robot speaks a line and maybe updates
   its face. If it's slow or absent, you just got the silent default face. The chart already
   moved on.

That fire-and-forget split is the whole trick. The "feel alive" behaviour is guaranteed;
the LLM cleverness is a bonus layered on top. To add a beat: one `BEATS` row (default face +
prompt + camera/audio flags) plus one `DEFAULT_REGISTRY` row (priority + optional needs/trait)
in `presence.py` — the lottery picks it up automatically, no chart surgery. Both are also
hand-editable without touching code: `memory/beats.json` layers over `BEATS`, and
`memory/presence_chart.yaml` can override the Sismic graph itself — either falls back to the
bundled default if missing or broken, so a bad edit degrades gracefully instead of disabling
the behaviour layer.

## A single beat, end to end

```
chart enters `musing`
   │
   ├─► show default "musing" face on OLED         (instant, offline)
   │
   └─► publish /cognition/request {beat, prompt, camera:false}   (fire & forget)
              │
        web_control receives it (_on_cog → _run_beat)
              │
        builds prompt + sensor snapshot ("CPU 22%, 49°C, sitting level…")
              │
        llm.generate()  ──► OpenRouter ──► {"say": "...", "mood": "..."}
              │
        speak the line (TTS) + show the mood face
              │
        record the decision in the log
```

If `camera:true` (a `looking` beat), `web_control` grabs one webcam frame first
(`CameraStream`) and routes to the **vision** model instead.

## Phrase bank — the frequent lines are pre-generated

The single most frequent thing Nano does is react to its own body (the `musing` beats and
the manual **Observe**). Paying a live LLM call for every idle cycle is wasteful — slow,
costs money, and needs internet. So those lines come from a **pre-generated phrase bank**
(`src/web_control/web_control/phrasebank.py`).

How it works:

1. **Pre-generation** — once (on first boot, when the persona drifts, via
   `scripts/pregenerate_phrases.py`, or the web button), the *cheap* model writes a batch of
   in-character lines for each **situation**, using **placeholders** for live values:
   > `"{temp} degrees. Toasting without butter."`  ·  `"Processing at {cpu}%. Time to double-check everything."`

   Situations are classified from the sensors (priority order): `picked_up`, `one_wheel`,
   `tilted`, `jostled`, `leaning`, `hot`, `cold`, `busy`, `idle`. Placeholders available:
   `{name} {cpu} {mem} {temp} {tilt}`.

2. **Runtime** — on a body beat, `classify()` maps the live sensors to a situation, `pick()`
   chooses a line **whose placeholders can actually be filled right now** (so a `{tilt}` line
   isn't picked when tilt is unknown), fills in the numbers, and speaks it. Instant, free,
   offline, still varied. Logged with `status="bank"`, `model="phrasebank"`.

3. **A small `phrasebank_live_ratio`** (default 0.2) sends a fraction of beats to the live
   LLM anyway, so the personality still produces fresh lines over time.

4. **Soul-drift regeneration** — the bank stores the *signature* (persona hash + traits) it
   was made with. When the personality has drifted "too much" (`phrasebank_drift`, summed
   trait change) or the persona text changed, `needs_regen()` is true and the stack
   regenerates the bank **in the background** — old lines keep serving until the new ones
   land. Small day-to-day trait nudges don't trigger it.

The bank is `~/.local/state/nanobot/phrases.json`, shared by the robot and the dev harness.
Inspect/force it: `GET /llm/phrases`, `POST /llm/phrases/regenerate`, or
`python scripts/pregenerate_phrases.py [--show]`.

## Skills — capabilities as self-documenting files

Nano's capabilities live as a **portable library of Markdown files** (`src/web_control/skills/`,
an [OpenClaw](https://github.com/)-style "SKILL.md" idea). Each `.md` is **one capability** —
a machine-readable YAML frontmatter contract plus a human/LLM-readable body that explains the
"how":

```markdown
---
name: read-lidar
description: Report the nearest obstacle from the lidar scan.
trigger: when asked what's around or how close things are
action: {kind: observe, sources: [scan]}
---
# Read LiDAR
Report the nearest object and roughly which way it is, in one short spoken line…
```

Drop a new file in (then **Reload** in the UI or `POST /skills/reload`) and the robot gains a
capability — *no code change*. `web_control/skills.py` just parses + indexes them
(ROS-free, unit-tested like the rest); `web_server` executes.

**Two tiers, matching the "garnish, never unsafe" rule:**

| tier | `action.kind` | what it does |
|---|---|---|
| narrative (always safe) | `say` / `observe` / `look` | speaks a line steered by the body — optionally with the live sensor snapshot, a lidar scan summary, or a camera frame (vision) |
| **gated action** | `topic` | publishes a **whitelisted, clamped** ROS message (`/led`, `/fan_pwm`, `/lds_target_rpm`, `/cmd_vel`) |

An action skill runs **only** when it sets `enabled: true` **and** the node's
`skills_allow_actions` master switch is on (**off by default**). Even then the value is clamped
in `web_server`, and motion is clamped *again* reflexively by `slam_nav` — so a skill can never
push the robot into an unsafe state. That's the same principle as traits-as-guards: the brain
can reach for a capability, but physics/safety always wins.

**Two ways a skill runs:**

1. **Autonomously** — every `skill_every`-th idle body beat becomes a `skill` beat (an upgrade
   of `musing`, just like `pursuing`). `CognitionCore.run_skill_beat` shows the offered catalogue
   (names + descriptions + triggers) to the *cheap* model, which **picks the most fitting one**
   for the moment (or none), and Nano performs it.
2. **On demand** — the web UI's "🛠 Skills" card lists every file with an *Invoke* button
   (`GET /skills`, `POST /skills/invoke`, `POST /skills/reload`).

Every invocation lands in the decision log as `skill:<name>`. `scripts/dev_webui.py` wires the
same panel off-robot (narrative skills speak through your laptop; topic actions no-op, no ROS).

## The workshop — Nano invents its own skills in reflection mode

Reflection mode (formerly "meditation") isn't just a calm pause + consolidation; it's a
**self-improvement loop** that can mint *new* capabilities. When reflection turns on — via the web
toggle, the `forge-skill` capability, or **automatically after a long idle** (the behaviour node
publishes `/reflect_request`; see `reflect_auto_*`) — alongside the usual reflect/consolidate,
`CognitionCore.run_skill_workshop()` runs a bounded **suggest → check → rehearse → trial → adopt**
cycle (the pure pieces are in `web_control/skillsmith.py`, unit-tested offline):

1. **Suggest** — the *smart* model reads the recent decision log (looking for gaps, repeated
   `no-pick`/`stumped`, things people seemed to want) + the existing catalogue, and proposes
   **one** capability: brand `new` or an `adapt` (a fresh variant of an existing one).
2. **Check** — deterministic, local: the candidate must round-trip through the skill parser, use
   a known action kind, not collide with an existing name, and (for action skills) only when
   `skills_allow_actions` is on. Generated action skills are **always born `enabled: false`**.
3. **Rehearse** — the skill is dry-run once (no speaking aloud) and the *actual* output is fed to
   a smart-model **critique** ("useful, safe, in-character, not a duplicate?"). An explicit veto
   discards it.
4. **Trial** — survivors are written into a writable **"learned" dir** (`workshop_dir`, default
   `~/.local/state/nanobot/skills` — deploy-synced like the soul/phrase bank, kept separate from
   the committed catalogue) and tracked in `workshop.json`. A trial is a **normal, immediately
   usable skill** — fully auto-eligible to the skill beat (action tier still gated).
5. **Adopt / retire** — the `gate()` watches each trial's evidence: it **auto-adopts** (permanent)
   after enough good runs **+ net-positive 👍 reward + no errors**, and **auto-retires** (deletes
   the file, rolls back) on errors or net-👎. The 👍/👎 you give right after a skill runs is the
   "happy user" signal. You can always **Keep** or **Kill** a trial yourself from the "🛠 Skills"
   card (`/skills/workshop` + `/keep` + `/kill`).

So the brain doesn't just *use* skills — over time, guided by what actually pleased people, it
**grows new ones and prunes the duds**, and the survivors become a permanent part of who it is.
The same loop runs on the dev harness (skills land in `memory/skills/`), so you can watch it
invent a capability in a browser with no robot.

## On-demand interactions (you, not the chart)

The web UI's "AI" card drives the manual paths, all in `web_control`:

- **Say** (`POST /llm/say`) — one spontaneous line.
- **Chat** (`POST /llm/chat`) — a rolling conversation (uses the smarter model).
- **👁 Observe** (`POST /llm/observe`) — comments on how it physically "feels" from its
  sensors.
- **📷 Look** (`POST /llm/look`) — comments on what the camera sees (vision model).

Same `llm.generate()` underneath; just triggered by a button instead of a beat.

## Personality: traits + registry

Nano has a **parametric personality** — four traits in `0..1`:

> **curiosity · extraversion · caution · playfulness**

These aren't cosmetic; the statechart's guards *read* them:

- **curiosity** gates the camera beat (not curious enough → no `looking`).
- **extraversion** scales how often it acts (idle cadence).
- **caution** is published (latched on `/cognition/traits`) to the navigation layer, which
  maps it to stop-distance / max-speed — but **slam_nav clamps it reflexively**, so the
  brain can *never* push motion into an unsafe range (gated by `trait_motion`).

Alongside traits there's a **registry** — per-beat knobs (priority / enabled / gates) the
brain can tune. Both are seeded from `personality.json` (created by
`scripts/personality_creator.py`, a short questionnaire run through the smart model) and
persisted as they drift. They live as mutable dicts in the Sismic context.

## How the personality *evolves*

Two timescales feed one `evolve` event (the chart smooths every change with exponential
smoothing, so traits drift, never jerk):

- **Fast rules** — reflexes nudge traits immediately (e.g. being picked up → more caution),
  in `mood_node`.
- **Slow reflection** — periodically (`reflect_period`, plus sooner on notable events), the
  *smart* model reads the recent **decision log** + current traits and proposes small,
  justified nudges, published on `/cognition/evolve`. Example from a real run:
  > `[reflect] Growing distrust of unknown handling deepens caution… → {caution: 0.88, playfulness: 0.15}`

There's also a **safety net**: a `brain_lost` heartbeat (`brain_timeout` with no evolve).
If reflection stops arriving (crash, network gone), the chart reverts to the **seeded
baseline personality** — not generic defaults, but *who this robot was configured to be*.

**The brain can never disable its own reflexes.** Greeting, resting, dormant, and pick-up
reactions are *not* in the registry, by design. So no matter how the personality drifts, the
robot keeps its safe, alive base behaviour.

## Drives — the LLM's extra levers (new *states*, not just weights)

Beyond the four traits there are three smoothed 0..1 **drives** + one categorical face, all
steerable by the same `evolve` event (same guardrails: clamped, smoothed, reverted on
`brain_lost`, expression-only):

- **energy** — paces the idle cadence and can trigger an *energetic burst* (the `performing`
  state chains a second beat);
- **focus** — gates a brief alert **`attending`** perk-up (the "I noticed something" pause)
  before a beat;
- **introspection** — how readily it drifts into reflection mode on a long idle;
- **mood** — a baseline face worn briefly (the **`feeling`** state) between beats.

`0.5` is the neutral "off" point: at the defaults none of the new states ever fire, so
behaviour is unchanged until reflection pushes a drive above it.

## Vision reflexes — the camera feeds the personality

While GPU vision is live, `web_control` publishes a compact `/vision/state` JSON at
2 Hz (approach/looming/clutter/novelty/warmth/motion — staleness IS the stand-down
signal). `mood_node` consumes it as **fast rules and expression, never a new motor
path**:

- **Anticipatory greeting** — motion growing + centred = someone walking up →
  greet-face + a `greeting` beat (rate-limited, idle-only).
- **Looming / clutter → caution** — something closing on the lens startles caution up
  (edge-triggered); a visually busy floor *holds* caution at `clutter_caution` and
  releases to the remembered pre-clutter value after. With slam_nav's `trait_motion`
  opt-in this doubles as the **clutter velocity throttle**, through the one existing
  clamped caution→max_lin mapping.
- **Ambient colour mood** — scene warmth (R−B) tints the chart's `feeling` face; the
  LLM's `drives.mood` always wins.
- **Novelty boost** — a transient lottery-weight multiplier on the `looking` beat
  (distinct from the LLM-evolvable registry priority).
- **Visual diary** — scene scalars sampled every 10 min; the trend ("the room has got
  darker and calmer") is folded into the reflection prompts like the trait trajectory.

(Separately — and *not* part of the brain — `slam_nav` can visually servo on the
calibrated colour target: `track_enable` turns the robot in place to keep the blob
centered, pan-only, gated by `enable_motion` like all motion. The controller is a
PD + age-taper + stiction-dither loop refined 2026-07-21 with a smooth deadband,
coast-on-loss, an opt-in integral, a target-velocity feedforward, and
confidence-scaled authority — all `track_*` params are live-tunable from the Camera
tab's "▸ Tracking tuning".)

## Purpose & goals (the "why" layer)

`behavior/brain.py` (ROS-free, shared verbatim with the dev harness) adds a slow goal layer
on top of the beats: a **Purpose Engine** holds an objective + intrinsic-reward weights
(reflected deterministically from the decision log), and a **Pursuit driver** occasionally
upgrades the `musing` slot into a **`pursuing`** beat that narrates the current task —
with an **A/B bandit** trying style variants. The web UI's 👍/👎 credits the variant that
just ran (contextual) or shapes the reward weights (global). Latched readouts:
`/purpose`, `/task_current`, `/experiments` (the web "🧠 Brain" card). Like everything else
it is narrative-only — goals are things to *say and look at*, never motion commands.

## Time awareness (routines)

Nano knows what time it is, in three small ways (all off by default; enabled by the
`quiet_start`/`quiet_end` windows in `robot.yaml` — one under `behavior:`, one under
`web_control:`, kept in sync):

1. **Prompts carry the clock** — `cognition.time_context()` ("It is Tuesday 21:47, in the
   evening.") is folded into the beat / skill-pick / observe prompts, so a morning musing
   reads different from a midnight one.
2. **Quiet hours mute autonomous speech** — inside the window, idle/skill beats, the boot
   greeting, the offline lament, the stats announcer and the reflection bookends stay
   silent (logged as `quiet-hours` in the decision log). **User-initiated speech always
   works** — chat, the Speak box, a manually invoked skill: you asked, it answers. Faces
   still animate: quiet, not dormant. Night reflections become silent "sleep consolidation".
3. **A sleepier idle cadence at night** — the chart's idle guard is multiplied by a live
   `tempo()` callable (`night_tempo`, default 2.0 = beats half as often inside the window),
   deliberately separate from the LLM-owned traits/drives so evolution is untouched.

## Scheduled routines

Beyond reacting to idleness, Nano can be told to do something specific at a specific time —
`behavior.brain.Schedule`, a small local-time cron. Entries are `{"time":"09:00",
"skill":"patrol"}` dicts in `schedule.json` (`schedule_path` in `robot.yaml`, hand-editable
like `beats.json`/`presence_chart.yaml`) — **or live from the web UI's Schedule card**, which
edits the same list. Each tick, `mood_node._check_schedule()` asks `Schedule.due()` whether a
configured HH:MM has passed today and not yet fired; if so it publishes the same
`/cognition/request` a `skill` beat uses, just naming the skill instead of asking the brain to
pick one — `web_control._on_cog` sees the `skill` field and calls `invoke_skill(name)` directly
(the exact path `POST /skills/invoke` uses). A scheduled fire is therefore a **named,
manual-style invocation**: it always talks, even in quiet hours, and doesn't touch the idle-beat
cadence or the skill-beat's autonomous picker. Firing is level-triggered (a tick that lands a
little late, or a node that starts up after the time, still fires it once that day) and is
**not** persisted across a restart — fine for "greet at the door", not a guarantee for anything
that must fire exactly once ever.

**Editing it:** the web UI's Schedule card lists the current entries (add/remove HH:MM + skill
rows) and a Save button POSTs the whole list to the whitelisted `/schedule_edit` topic
(`POST /publish {topic:"/schedule_edit", value:[...]}` — `telemetry.py` does a light shape
check). `mood_node._on_schedule_edit` does the real parsing (drops malformed rows, logs why),
swaps in the new `Schedule` immediately (no restart), persists it to `schedule.json`, and
re-publishes the normalized result on the latched `/schedule` topic — which rides the existing
`/telemetry` SSE frame, so every open browser (and a fresh page load) sees exactly what's
configured, not just what was last typed. The dev harness (`scripts/dev_webui.py`,
`run_behavior`) runs the identical `Schedule` for parity, minting into `memory/schedule.json`.

## Model tiers (and the cost caps)

Three tiers in `llm.py`, each **free-first**: every text tier tries one or more **free**
OpenRouter models and only falls back to a **paid DeepSeek** model when *all* the free ones
are over their (shared, ~daily / upstream) rate limit. So routine chatter costs nothing —
you only pay when the free quota is exhausted.

| Tier | Config keys (free primary → paid fallback) | Used for |
|---|---|---|
| cheap text | `llm_free_model` → `llm_model` | musing, observe, say, beats |
| smart text | `llm_free_smart_model` → `llm_smart_model` | chat + reflection |
| vision | `llm_vision_model` → `llm_vision_fallback_model` (`""` = none, stay silent) | looking + manual Look |

(The concrete model slugs live in `robot.yaml` — as of 2026-07 every tier currently
points at `deepseek/deepseek-v4-flash`; earlier defaults were rotating `:free`
Nemotron/Llama slugs. Check the yaml, not this table, for what actually runs.)

`_candidates(smart, image)` builds the ordered `(model, is_paid)` list (free fields are
comma-separated lists, tried in order); `_chat()` tries each, **falling through to the next
only on a rate/daily-limit error** (`429`/`402` or a limit-ish message) — any other failure
stops. `last_model` records the slug that actually answered (shown in the decision log).

**Hourly caps apply only to the PAID fallback** (a sliding 1-hour window per tier via
`can_call()` / `_rate_consume()`). When the cap is hit, the paid model is skipped and the
call stays silent (`rate-limited`); the free primary is never capped. `llm_smart_max_per_hour`
(15) / `llm_vision_max_per_hour` (10), `0` = unlimited. All knobs live in `robot.yaml`
(`web_control: llm_*`, incl. `llm_free_model` / `llm_free_smart_model`); the API key is read
from `llm_api_key` or, when blank, `$OPENROUTER_API_KEY`. **Free `:free` slugs rotate and the
popular ones get throttled — if a default stops responding, pick a current one from
OpenRouter's `/models` API.**

## The decision log (observability)

Every cognition path — beat, say, chat, observe, look, reflect — records one entry:

```json
{"t": 1750000000.0, "trigger": "beat:musing", "state": "musing", "model": "…flash",
 "status": "spoke", "say": "Fifty-eight degrees. Being held. I don't trust this.",
 "mood": "focused", "ms": 1840}
```

`status` includes skip reasons too (`skipped-busy`, `llm-unavailable`, `no-frame`,
`rate-limited`). It's appended as JSON lines to `cognition_log_path`
(default `~/.local/state/nanobot/cognition.log`), seeded back into a ring buffer on start,
and shown in the web UI's "🧠 Decision log" (`GET /llm/log`). **Both the real robot
(`web_server`) and the dev harness (`dev_webui.py`) write the same file/format**, so history
is shared and survives restarts.

## Testing it without a robot

`scripts/dev_webui.py --behavior` runs the *real* statechart on a real clock on a laptop,
with the *real* LLM, mapping `musing`→synthetic sensors and `looking`→your webcam
(`opencv-python`). So you can watch and hear the entire enriched loop — and read every
decision — with no ROS and no hardware. It serves the real `web/index.html`, runs the **same
`CognitionCore` the robot runs** (only the adapters differ — see *Where things live*), and
wires the `/llm/*` + `/skills/*` + `/tts*` + brain endpoints; telemetry/joystick/map show
offline. Because the brain is one shared base, what you test here is exactly what runs on the
robot.

```bash
set OPENROUTER_API_KEY=...        # or memory/openrouter_key (gitignored)
python scripts/dev_webui.py --behavior        # autonomous enriched beats
python scripts/dev_webui.py --behavior --idle-secs 10   # faster beats
```

On Windows, `scripts/start-dev.ps1` finds a real Python, loads the key, and launches with
`--behavior`.

## Why it's built this way (the short version)

1. **Statechart, not a loop** — *when* to act is formal, inspectable, and testable; it can't
   get into a weird state.
2. **Fire-and-forget LLM** — *what* to say is best-effort; a slow brain = a silent face,
   never a stuck robot.
3. **Traits as guards, clamped downstream** — personality genuinely changes behaviour, but
   physics/safety always wins.
4. **Reflexes outside the registry** — the brain can grow, but can't break its own safe base.
5. **Tiered + capped models** — clever where it matters, cheap everywhere else, bounded cost.

## Where things live

| Concern | File |
|---|---|
| Statechart (when) | `src/behavior/behavior/presence.py` (+ `test/`) |
| Purpose Engine + Pursuit + A/B bandit + Personality orchestration | `src/behavior/behavior/brain.py` |
| ROS wrapper, fast-rule evolution, night tempo | `src/behavior/behavior/mood_node.py` |
| OpenRouter client (what) | `src/web_control/web_control/llm.py` |
| **Cognition core** (execution, reflection, log — shared robot+dev) | `src/web_control/web_control/cognition.py` |
| Pre-generated phrase bank | `src/web_control/web_control/phrasebank.py` (+ `scripts/pregenerate_phrases.py`) |
| Skill library (capabilities) | `src/web_control/web_control/skills.py` + `src/web_control/skills/*.md` |
| ROS node + adapters (face/sensors/actions) | `src/web_control/web_control/web_server.py` |
| TTS | `src/web_control/web_control/tts.py` |
| Face rendering | `src/oled_display/` |
| Config (models, caps, beats, persona) | `src/robot_bringup/config/robot.yaml` |
| Personality seed + creator | `personality.json` ← `scripts/personality_creator.py` |
| Off-robot test harness | `scripts/dev_webui.py`, `scripts/dev_tts_test.py` |

The table above is *code* files. For the *data* side — every persisted
personality/traits/skills/log file, which ones are git-tracked seeds vs.
runtime state, and who reads/writes each — see
[`personality-files.md`](personality-files.md).
