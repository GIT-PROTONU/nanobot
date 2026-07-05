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

- **First word clipped on the first utterance** (a repeat right after is fine): the H5
  codec/amp powers up when aplay opens the PCM and swallows the first ~0.2-0.3 s.
  Fixed 2026-07-05: `tts.py` prepends `LEAD_SILENCE` (0.35 s) to every clip on the
  aplay path so the wake-up ramp burns silence, and the karaoke word timing is
  offset by the pad. If a first word EVER clips again, raise `LEAD_SILENCE`.

- `stack.sh` launches nodes from `install/<pkg>/bin/` not `lib/<pkg>/`.
  On this RoboStack colcon install, Python `console_scripts` entry points
  land in `bin/`. If behavior or map services show `down`, check the path.
  (Fixed Jul 3 2026 — `sim_hardware/bin/map_bridge_node` and
  `behavior/bin/mood_node`.)
