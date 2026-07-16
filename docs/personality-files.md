# Personality / brain file map

Every file that stores part of the robot's "soul" — traits, memory, learned
skills, decision history — plus the config that points at them. See
[`brain.md`](brain.md) for how these fit into the behaviour layer itself; this
page is just "where is it and who touches it."

There are three kinds of file here, and the distinction is load-bearing —
**don't merge across kinds**, it's what lets `deploy.sh` push a fresh seed
without clobbering evolved state:

- **Seed** (git-tracked, under `memory/`) — the starting point / hand-editable
  override. Pushed to the board by `deploy.sh` only when you opt in with
  `DEPLOY_SOUL=1` (**off by default** — the robot keeps the personality it has
  evolved on its own).
- **State** (gitignored, `~/.local/state/nanobot/` on the board, `memory/` on
  the dev harness) — runtime-persisted, drifts over time, survives reboots.
  This is what `DEPLOY_SOUL=1` overwrites.
- **Code default** — built into `presence.py`/`brain.py`, used only if the
  state/seed file is absent or fails to parse. A broken hand-edit can never
  take the robot offline.

Every path below is a `robot.yaml` param (`""` = default XDG path shown);
override any of them per-deployment without touching code.

## Traits, drives, purpose

| File | Kind | Written by | Read by | Holds |
|---|---|---|---|---|
| `memory/personality.json` (seed) → `~/.local/state/nanobot/personality.json` (state) | seed+state | `scripts/personality_creator.py` (seed); `Personality.persist` in `brain.py` (state, on drift) | `mood_node` on boot (`personality_path`) | `name`, `persona`, `traits` (curiosity/extraversion/caution/playfulness), `registry` (per-beat priority/enable/needs/trait), `drives` (energy/focus/introspection/mood) |
| `~/.local/state/nanobot/purpose.json` | state | `PurposeBrain.persist` in `brain.py` (`purpose_path`) | `PurposeBrain` on boot | objective + intrinsic-reward weights (the Purpose Engine) |
| `~/.local/state/nanobot/experiments.json` | state | `PurposeBrain.persist` (`experiments_path`) | `PurposeBrain` on boot | the A/B bandit's pursuit trial state |
| `~/.local/state/nanobot/trait_history.json` | state | `cognition.record_trait_snapshot` (`trait_history_path`) | `cognition.trait_trend_text` (folded into reflect/consolidate prompts) | timestamped trait snapshots, so the robot can reason about drift over time |
| `~/.local/state/nanobot/self_model.json` | state | `CognitionCore._save_self_model` (`self_model_path`) | `CognitionCore.get_self_model`, reflection prompts | short first-person self-narrative, rewritten every `consolidate_every`th reflection |

## Chart / beats (how idle behaviour is structured)

| File | Kind | Written by | Read by | Holds |
|---|---|---|---|---|
| `memory/presence_chart.yaml` | seed (optional) | hand-edited | `presence.load_chart_yaml` (`chart_path`) | the Sismic statechart itself — overrides the bundled Python default |
| `memory/beats.json` | seed (optional) | hand-edited, or `personality_creator.py` | `presence.merge_beats` (`beats_path`) | per-beat face/camera/audio/prompt templates, layered onto `BEATS` in `presence.py` |
| `memory/schedule.json` → `~/.local/state/nanobot/schedule.json` | seed+state | hand-edited, or the web Schedule card via `/schedule_edit` (`mood_node` persists) | `behavior.brain.Schedule` (`schedule_path`) | daily routines — `[{"time":"09:00","skill":"patrol"}, …]`, echoed normalized on the latched `/schedule` topic |

## LLM / voice config

| File | Kind | Written by | Read by | Holds |
|---|---|---|---|---|
| `memory/openrouter_key` (gitignored) | seed-ish (local secret, not committed) | pasted by hand from `openrouter_key.example` | `_load_openrouter_key()` (all entry points) | the raw OpenRouter API key, dev-side fallback |
| `~/.local/state/nanobot/llm.json` | state | `LlmClient` on `/llm/config` POST (`llm_settings_path`) | `LlmClient` on boot | enable toggle, model ids, persona override, **and the API key if set via the web UI** (wins over `llm_api_key`/env) |
| `~/.local/state/nanobot/tts.json` | state (voice, not personality proper) | `tts.py` (`tts_settings_path`) | `tts.py` on boot | voice/volume/speed/pitch + stats-announcer settings |

## Skills (capability library)

| File | Kind | Written by | Read by | Holds |
|---|---|---|---|---|
| `src/web_control/skills/*.md` | seed (committed catalogue) | hand-authored | `SkillLibrary` (`skills_dir`) | the built-in capabilities (narrative + action + meta kinds) |
| `memory/skills/*.md` → `~/.local/state/nanobot/skills/*.md` | state (learned) | `skillsmith.py` workshop (`workshop_dir`) | `SkillLibrary(extra_dir=…)`, overrides built-ins by name | skills forged/rehearsed/trialled by the workshop |
| `~/.local/state/nanobot/workshop.json` | state | `WorkshopState` in `skillsmith.py` (`workshop_path`) | the workshop's `gate()` | trial ledger — run counts, 👍/👎, adopt/retire decisions |
| `~/.local/state/nanobot/skill_likes.json` | state | `CognitionCore.like_skill` (`skill_likes_path`) | `CognitionCore._liked_skill_pick` | per-skill 👍/👎 tally, biases autonomous skill-beat picks |

## History / diagnostics (not soul state, but explains *why* it drifted)

| File | Kind | Written by | Read by | Holds |
|---|---|---|---|---|
| `~/.local/state/nanobot/cognition.log` | state (append-only) | `CognitionCore` every generation (`cognition_log_path`) | `GET /llm/log`, seeded into the ring buffer on boot | one line per say/chat/observe/look/beat/skill call — trigger, model, status, latency |
| `~/.local/state/nanobot/phrases.json` | state | `phrasebank.py` (`phrasebank_path`) | `CognitionCore.bank_say` (offline-first idle lines) | pre-generated in-character lines per situation, regenerated on persona drift |
| `~/.local/state/nanobot/vision_diary.json` | state | `cognition.record_vision_snapshot` (`vision_diary_path`) | `vision_trend_text` (folded into reflect/consolidate prompts), `GET /llm/vision_diary` | slow log of scene scalars — sensory continuity for the self-narrative |
| `~/.local/state/nanobot/vision_targets.json` | state (not soul — camera calibration) | `web_server` on calibrate/tune (`vision_targets_path`) | re-applied on boot; `GET /vision/targets` | named colour-target calibrations for blob tracking |

## Dev-only

| File | Kind | Notes |
|---|---|---|
| `memory/dev_sensors.json` | dev harness only | synthetic sensor snapshot for `scripts/dev_webui.py`, not used on the robot |

## Quick answers

- **"Where do I hand-tune traits/persona before first boot?"** → `memory/personality.json` (or run `scripts/personality_creator.py`).
- **"The robot's personality feels off after weeks of running — how do I reset it?"** → delete/edit the `~/.local/state/nanobot/*.json` state files, or redeploy with `DEPLOY_SOUL=1` to overwrite state with the git seed.
- **"I want to add a new capability without touching Python."** → drop a `.md` in `src/web_control/skills/`, `POST /skills/reload`.
- **"Why did the robot say that at 3am?"** → `GET /llm/log` / `~/.local/state/nanobot/cognition.log`.
