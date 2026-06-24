---
name: tts-speech
description: Robot text-to-speech (pico2wave) + OLED karaoke + spoken stats; audio out = H5 internal codec
metadata:
  node_type: memory
  type: project
---

The robot can **speak** (English + German). Added 2026-06-24.

- **Engine**: `pico2wave` (SVOX Pico, built from https://github.com/ihuguet/picotts via
  `deploy/install-picotts.sh`, which installs **only en-US/en-GB/de-DE** lingware to save
  disk). Lives **inside `web_control`** (`src/web_control/web_control/tts.py`) — NOT a new
  node, to avoid another ~35 MB interpreter on the 1 GB board. `pico2wave`+`aplay` run only
  while speaking → **zero idle cost**. WAV scratch is in `/dev/shm`.
- **Flow**: web UI `POST /tts {text,voice?}` → synth → `aplay` → words streamed one at a
  time on **`/oled_word`**, timed to the clip duration (Pico has no word marks, so timing
  is length-weighted). `oled_display` shows each word big+centred ("karaoke"); `""` →
  dashboard. The Speak box **reuses the old OLED-text field** (no longer publishes
  `/oled_text`). Off rosbridge on purpose (plain HTTP POST).
- **Params** (voice/volume/speed/pitch) = Pico inline `<volume>/<speed>/<pitch>` markup,
  **only emitted when a level != 100** — at defaults it's vanilla `pico2wave` (clearest,
  most-tested path; a Pico build that mishandles a tag can't garble normal speech). UI
  volume is **0–100 (attenuate only)** — Pico `<volume>` >100 clips/distorts; loudness
  comes from the **codec hardware mixer**, not Pico. Text has `< > &` stripped (markup-safe
  + keeps OLED words == spoken words). Persisted to **`~/.local/state/nanobot/tts.json`**
  (param `tts_settings_path`) so they survive a reboot. `GET/POST /tts/config`. Binaries are
  resolved to absolute paths at start (setsid/pixi PATH can be trimmed).
- **Spoken system stats**: speaks CPU%/RAM%/temp every `announce_interval` s when enabled —
  **keeps running after the browser closes** and resumes after reboot. `POST /tts/announce`
  = say once. The check **piggy-backs on the existing 1 Hz ESP-ping timer** (no extra
  wakeup); disabled = one dict lookup. It interrupts whatever's playing + takes over the
  OLED each interval (by design).
- **Audio out = the SBC's INTERNAL H5 analog codec** (user's choice, not USB). Needs the
  `analog-codec` overlay (now in `deploy/sbc-setup.sh`) AND un-muting — the codec boots
  **muted**. `deploy/enable-h5-audio.sh` un-mutes all playback controls, writes
  `/etc/asound.conf` to make the codec the default device (so `tts_device:""` works), and
  `alsactl store`s it. The USB webcam mic is a separate capture card (untouched).

See [[oled-display-perf]] (the panel this karaokes on), [[sbc-cpu-profile]], [[project-overview]].
