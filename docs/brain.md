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

When Nano is genuinely idle (nobody driving it, no goal, no manual command), the chart
cycles through states. The interesting two are *enrichable beats*:

| Beat | Sense | Model used | When |
|---|---|---|---|
| `musing` | its own body (CPU, RAM, temp, IMU tilt, pick-up) | cheap text | every idle cycle |
| `looking` | the camera | vision | every Nth cycle, **only if curious enough** |

The cadence/gating knobs (`look_every`, `camera_beats`, idle timing) live in `robot.yaml`
under `behavior:`. There are also **reflexes** — `greeting`, `resting`, `dormant`, and
pick-up reactions. These are deterministic and **not under the brain's control** (see
below).

When the chart enters a beat, two things happen *independently*:

1. It **immediately** shows a sensible default face (offline-safe — works with no internet).
2. **If enrichment is on**, it fires a **fire-and-forget** request to the LLM layer
   (`/cognition/request`). If the model answers, the robot speaks a line and maybe updates
   its face. If it's slow or absent, you just got the silent default face. The chart already
   moved on.

That fire-and-forget split is the whole trick. The "feel alive" behaviour is guaranteed;
the LLM cleverness is a bonus layered on top. To add a beat: add a state that calls
`do_beat('name')` plus a `BEATS` entry (default face + prompt + camera flag) in
`presence.py`.

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

## Model tiers (and the cost caps)

Three tiers in `llm.py`, each **free-first**: every text tier tries one or more **free**
OpenRouter models and only falls back to a **paid DeepSeek** model when *all* the free ones
are over their (shared, ~daily / upstream) rate limit. So routine chatter costs nothing —
you only pay when the free quota is exhausted.

| Tier | Free primary (default, tried first) | Paid fallback | Used for |
|---|---|---|---|
| cheap text | `nemotron-3-nano-30b:free`, then `llama-3.3-70b:free` | `deepseek-v4-flash` | musing, observe, say, beats |
| smart text | `nemotron-3-super-120b:free`, then `gpt-oss-120b:free` | `deepseek-v4-pro` | chat + reflection |
| vision | `nemotron-nano-12b-v2-vl:free` | *(none — DeepSeek can't see; set `llm_vision_fallback_model` for one)* | looking + manual Look |

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
decision — with no ROS and no hardware. It serves the real `web/index.html` and wires only
the `/llm/*` + `/tts*` endpoints; telemetry/joystick/map show offline.

```bash
set OPENROUTER_API_KEY=...        # or scripts/.openrouter_key (gitignored)
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
| ROS wrapper, fast-rule evolution | `src/behavior/behavior/mood_node.py` |
| OpenRouter client (what) | `src/web_control/web_control/llm.py` |
| Pre-generated phrase bank | `src/web_control/web_control/phrasebank.py` (+ `scripts/pregenerate_phrases.py`) |
| Cognition execution, reflection, decision log | `src/web_control/web_control/web_server.py` |
| TTS | `src/web_control/web_control/tts.py` |
| Face rendering | `src/oled_display/` |
| Config (models, caps, beats, persona) | `src/robot_bringup/config/robot.yaml` |
| Personality seed + creator | `personality.json` ← `scripts/personality_creator.py` |
| Off-robot test harness | `scripts/dev_webui.py`, `scripts/dev_tts_test.py` |
