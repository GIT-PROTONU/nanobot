---
name: tts-speech
description: Robot text-to-speech (espeak-ng) + OLED karaoke + spoken stats; audio out = H5 internal codec
metadata:
  node_type: memory
  node_type: project
---

The robot can **speak** (English). Added 2026-06-24, switched from pico2wave to espeak-ng 2026-07-03.

- **Engine**: `espeak-ng` (https://github.com/espeak-ng/espeak-ng, installed via
  `deploy/install-espeakng.sh`, which prunes to **en-gb only** to save disk). Lives
  **inside `web_control`** (`src/web_control/web_control/tts.py`) — NOT a new node,
  to avoid another ~35 MB interpreter on the 1 GB board. `espeak-ng`+`aplay` run only
  while speaking → **zero idle cost**. WAV scratch is in `/dev/shm`.
- **Flow**: web UI `POST /tts {text,voice?}` → synth → `aplay` → words streamed one at a
  time on **`/oled_word`**, timed to the clip duration (espeak has no word marks, so timing
  is length-weighted). `oled_display` shows each word big+centred ("karaoke"); `""` →
  dashboard. The Speak box **reuses the old OLED-text field** (no longer publishes
  `/oled_text`). Off rosbridge on purpose (plain HTTP POST).
- **Volume/speed/pitch settings in the UI** — all supported natively by espeak-ng.
  Persisted to **`~/.local/state/nanobot/tts.json`** (param `tts_settings_path`) so they
  survive a reboot. `GET/POST /tts/config`. Binaries are resolved to absolute paths at
  start (setsid/pixi PATH can be trimmed).
- **Spoken system stats**: speaks CPU%/RAM%/temp every `announce_interval` s when enabled —
  **keeps running after the browser closes** and resumes after reboot. `POST /tts/announce`
  = say once. The check **piggy-backs on the existing 1 Hz ESP-ping timer** (no extra
  wakeup); disabled = one dict lookup. It interrupts whatever's playing + takes over the
  OLED each interval (by design).
- **TWO unrelated sources speak temperature/state — don't confuse them:**
  (1) the **periodic stats announcer** above (`_announce_tick`→`announce_now`→`_compose_stats`
  in `web_server.py`) — deterministic raw `/proc` readout ("Temperature 47 degrees"), fixed
  `announce_interval`, NO LLM/personality. **Reflection cannot change its rate** — it's a plain
  timer in the ROS node, independent of the statechart/brain/traits/drives. Only the `announce`
  toggle + interval slider affect it.
  (2) the **in-character body-reaction beats** (`musing`/`observe`) — phrase-bank or LLM lines
  like "its board's running hot ({temp}°C)", chosen by the beat lottery. **Reflection CAN make
  these more/less frequent**: slow LLM reflection's `evolve` nudges the `musing` beat priority
  /gating trait, and during reflection mode beats are paused (so it says them less right then).
  See [[llm-openrouter-personality]], [[meditation-skill-workshop]].
- **Audio out = the SBC's INTERNAL H5 analog codec** (user's choice, not USB). Needs the
  `analog-codec` overlay (now in `deploy/sbc-setup.sh`) AND un-muting — the codec boots
  **muted**. `deploy/enable-h5-audio.sh` un-mutes all playback controls, writes
  `/etc/asound.conf` to make the codec the default device (so `tts_device:""` works), and
  `alsactl store`s it. The USB webcam mic is a separate capture card (untouched).

See [[oled-display-perf]] (the panel this karaokes on), [[sbc-cpu-profile]], [[project-overview]].
