---
name: tts-speech
description: "Robot TTS (espeak-ng, apt-only) + OLED karaoke; audio out = H5 internal codec; stack.sh bin/ vs lib/ path gotcha"
metadata: 
  node_type: memory
  type: project
  originSessionId: a23aab8f-6b00-4f75-ab77-206aa9807c22
---

# TTS speech (espeak-ng)

**Stack:** espeak-ng (synthesis) → WAV → aplay (ALSA playback on lineout via the SBC's
internal H5 analog codec — boots muted; `deploy/enable-h5-audio.sh` un-mutes it and makes
it the ALSA default). Lives inside `web_control` (`tts.py`), not a separate node; words
karaoke onto the OLED via `/oled_word`. See [[oled-display-perf]], [[project-overview]].

## Key facts

- **espeak-ng NOT available on conda-forge** — must be apt-installed on the board.
  Run `sudo bash deploy/install-espeakng.sh` (or `sudo apt-get install -y espeak-ng`)
  before the web "Speak" button works. Without it, `tts.py` silently reports no backend
  and says nothing.
- `deploy/install-espeakng.sh` also prunes voice data to en-gb + Lancaster + Scottish only.
- ALSA mixer: Line Out at 84% / -7.5dB, DAC at 100% / 0dB — both ON by default.
  No PulseAudio (pure ALSA on Armbian).
- Playback device: `hw:0,0` (H3 Audio Codec, on-chip).
- Voices pruned to en-gb, en-gb-x-gbclan (Lancaster), en-gb-scotland only.
- Three voice options in the web UI (UK/default, Lancaster, Scottish).

## Paths after install

- Binary: `/usr/bin/espeak-ng`
- Voice data: `/usr/share/espeak-ng-data/voices/` (only `en/` kept)
- aplay: `/usr/bin/aplay` (alsa-utils, part of base image)

## Gotchas

- **First word/part clipped on the first utterance** (a repeat right after is fine): the H5
  codec/amp powers up when aplay opens the PCM and swallows the first ~0.2-0.3 s.
  Fixed 2026-07-05: `tts.py` prepends `LEAD_SILENCE` (0.35 s) to every clip on the
  aplay path so the wake-up ramp burns silence, and the karaoke word timing is
  offset by the pad. **2026-07-08: still clipping on hardware at 0.35 s** — the ramp
  time is apparently hardware/temperature dependent and not fully covered by a fixed
  guess, so the pad is now **live-tunable** instead of a hardcoded constant: web UI
  Speak card → "Lead silence" slider (0-1500 ms, persisted to `tts.json` alongside
  volume/speed/pitch), `TtsEngine.configure(lead_silence=seconds)`,
  `SETTINGS_DEFAULTS["lead_silence"]` in `web_server.py` (ms, default 350). If it
  still clips, raise the slider from the robot's own web UI while listening — no
  redeploy needed. `LEAD_SILENCE`/`LEAD_SILENCE_RANGE` in `tts.py` are just the seed/
  clamp now, not the effective value. **Verified on hardware 2026-07-08**: padding
  genuinely is written into `/dev/shm/nano_tts.wav` (frame count differs by exactly
  the configured pad) and `aplay` genuinely plays the full padded duration (timed,
  no ALSA xrun) — a max (1500 ms) pad does fix the clip, confirming the original
  power-up-ramp theory was right, it just needed more headroom than 0.35 s on this
  unit. Trade-off: a big pad adds an audible pause before a COLD first utterance.
  **Fix: the pad only applies when cold.** `TtsEngine` now tracks
  `_last_speech_end` (monotonic) and skips the pad (`warm` check) if the amp/codec
  spoke within `LEAD_SILENCE_KEEPALIVE` (8 s, hardcoded) of now — matches the
  already-known fact that a back-to-back utterance never clipped (amp still warm).
  So only the first utterance after a real gap (boot greeting, first line after a
  long idle) pays the pause; normal chatter (beats/chat/skills) is back-to-back
  enough to skip it.

- `stack.sh` launches nodes from `install/<pkg>/bin/` not `lib/<pkg>/`.
  On this RoboStack colcon install, Python `console_scripts` entry points
  land in `bin/`. If behavior or map services show `down`, check the path.
  (Fixed Jul 3 2026 — `sim_hardware/bin/map_bridge_node` and
  `behavior/bin/mood_node`.)

- **Shutdown/reboot/restart used to cut the spoken line off mid-sentence** (fixed
  2026-07-15, NOT yet deployed to the board as of that date). `POST /system/{restart,
  reboot,shutdown}` in `web_server.py` speaks the farewell/restart line
  (`system_announce`→`cognition.speak_lifecycle`, fire-and-forget on a background
  thread) then fired the detached systemctl/`stack.sh` command after a **flat 3 s
  sleep** — but line length (and thus playback duration: synth + `LEAD_SILENCE` +
  actual speech) varies, so anything longer than ~3 s of speech got killed early by
  the shutdown itself. Fix: added `TtsEngine.wait(timeout=)` (joins the playback
  thread) and `do_POST` now blocks on it (10 s bound) before firing the command, with
  just a 1 s flush delay after — deterministic instead of a guessed constant. See
  [[cooling-fan-control]] for the same-session fan work this was found alongside.
