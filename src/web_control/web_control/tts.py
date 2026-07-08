"""Text-to-speech for the web UI — espeak-ng on Linux, with SAPI/macOS fallbacks.

The robot speaks a line typed in the web page and, in lock-step, streams the
individual words to the OLED so they appear/disappear as they're said (the line
is far too long for a 128x64 panel, so we karaoke it one word at a time).

Design — deliberately near-zero idle cost on the 1 GB / quad-A53 board:
  * Synthesis is `espeak-ng` (a tiny C binary) writing a WAV to **/dev/shm**
    (tmpfs → no SD-card wear, freed right after).
  * Playback is `aplay` (alsa-utils, already used for the mic). Both processes are
    spawned ONLY while actually speaking, so when nobody triggers TTS it costs
    nothing — no thread, no process, no RAM beyond this object.
  * espeak-ng's CLI emits no word-boundary marks, so we sync the OLED by spreading
    the clip's true duration (read from the WAV header) across the words weighted
    by length. It's an estimate, but it tracks short phrases well and is free.

**Backends.** The robot uses `espeak-ng`+`aplay`. On a dev PC with no robot ROS
stack, the engine falls back to the OS's built-in synthesiser: **Windows SAPI**
(via PowerShell's `System.Speech`) or **macOS `say`**. Every backend produces
a WAV at `WAV_PATH` and reports its duration, so the karaoke/word-timing logic
below is backend-agnostic.

`say()` is fire-and-forget and self-cancelling: a new request stops the current
utterance (audio + OLED) and starts the new one. `on_word(word)` is called as each
word begins, and `on_word("")` once at the end to hand the panel back to the
dashboard.

Volume, speed, and pitch are supported natively by all backends.
The web layer persists settings across reboots.
"""
import contextlib
import os
import platform
import shutil
import subprocess
import tempfile
import threading
import time
import wave

# English voices: UK default, Lancaster, Scottish (the install script prunes to
# these three to keep the rootfs lean).
VOICES = ("en-gb", "en-gb-x-gbclan", "en-gb-scotland")

# tmpfs scratch on the robot (no SD-card wear); the OS temp dir on a dev PC that has
# no /dev/shm (Windows/macOS). Overwritten per utterance either way.
_SCRATCH = "/dev/shm" if os.path.isdir("/dev/shm") else tempfile.gettempdir()
WAV_PATH = os.path.join(_SCRATCH, "nano_tts.wav")
_TXT_PATH = os.path.join(_SCRATCH, "nano_tts.txt")   # for the macOS `say -f` backend
MAX_CHARS = 300                      # hard cap on a single utterance (also bounds synth time)
# Silence prepended to each clip on the aplay path: the H5 codec/amp only powers up
# when aplay opens the PCM and swallows the first ~0.2-0.3 s of samples while it
# ramps — which clipped the first spoken word (an immediate follow-up utterance was
# fine because the codec was still awake). Waking it over silence costs a barely
# noticeable delay instead of a word. The exact ramp time is hardware/temperature
# dependent and can't be measured from here, so it's live-tunable (web "Lead
# silence" slider -> configure(lead_silence=...)) instead of a fixed guess — if
# clipping is still audible, raise it; if the pause before speech feels long, lower
# it. LEAD_SILENCE is just the seed value before any UI/persisted override.
LEAD_SILENCE = 0.35                  # seconds
LEAD_SILENCE_RANGE = (0.0, 2.0)      # seconds; hard clamp regardless of caller
# If we already spoke within this many seconds, the amp/codec is still warm from
# that utterance (a back-to-back utterance was never clipped — see LEAD_SILENCE
# above), so the wake-up pad is skipped: it would only add a perceptible dead-air
# pause with no benefit. Only a genuinely cold start (first speech in a while) pays
# the pad. Chatty behaviour (idle beats, chat replies, stats announcer) is usually
# well within this window after the first utterance of a session.
LEAD_SILENCE_KEEPALIVE = 8.0         # seconds

# Hard clamps for the markup levels (percent; 100 = normal). The UI exposes
# friendlier sub-ranges, but anything out of these is clamped here regardless.
VOLUME_RANGE = (0, 500)
SPEED_RANGE = (20, 500)
PITCH_RANGE = (50, 200)
# Espeak-ng advanced params. Base pitch is espeak's -p (0-99, default 50).
# The UI slider maps 50-200% onto 0-99 via pitch/2.
BASE_PITCH_RANGE = (0, 99)
FLUTTER_RANGE = (0, 100)      # flutter <value> — pitch fluctuation
ROUGHNESS_RANGE = (0, 7)      # roughness <value> — creaky voice
CAP_PITCH_RANGE = (0, 100)    # -k <integer> — pitch boost on capitals


def clamp(v, lo, hi):
    try:
        return max(lo, min(hi, int(round(float(v)))))
    except (TypeError, ValueError):
        return lo


def _clean(text):
    """Make user text safe to speak — drops markdown/non-speech symbols
    (* _ ` ~ # | $ \\ [ ] { }) and collapses whitespace so
    the karaoke word split is clean."""
    t = (text or "")
    for ch in ("<", ">", "&", "*", "_", "`", "~", "#", "|", "$", "\\", "[", "]", "{", "}"):
        t = t.replace(ch, " ")
    return " ".join(t.split())


class TtsEngine:
    """One-at-a-time speaker. Thread-safe: callers just call say()/stop()."""

    def __init__(self, device=None, default_voice="en-gb",
                 enabled=True, on_word=None, logger=None):
        self._aplay = _resolve_bin("aplay")
        self._paplay = _resolve_bin("paplay")
        self._ffplay = _resolve_bin("ffplay")
        self._espeak = _resolve_bin("espeak-ng")
        self._device = device or None                 # aplay -D target; None = ALSA default
        self._enabled = bool(enabled)
        self._on_word = on_word or (lambda _w: None)
        self._log = logger or (lambda *_: None)

        # Pick the synthesis/playback backend once. The robot has espeak+aplay (the
        # preferred path); a dev PC falls back to the OS speech engine so TTS is
        # still testable off-robot. "" = no usable backend (available() -> False).
        self._ps = shutil.which("powershell") or shutil.which("pwsh") or ""  # Windows SAPI
        self._say = shutil.which("say") or ""                                # macOS
        self._afplay = shutil.which("afplay") or ""                          # macOS player
        self._backend = self._pick_backend()
        self._log(f"tts: backend={self._backend or 'none'}")

        # Live voice + markup levels (percent). Set via configure(); the web layer
        # restores the persisted values on startup.
        self._voice = default_voice if default_voice in VOICES else "en-gb"
        self._volume = 100
        self._speed = 100
        self._pitch = 100
        self._base_pitch = 50       # espeak -p; default 50 = normal
        self._flutter = 0
        self._roughness = 0
        self._cap_pitch = 0
        self._lead_silence = LEAD_SILENCE
        self._last_speech_end = 0.0    # monotonic; 0 = never spoken yet (cold)

        self._lock = threading.Lock()
        self._thread = None
        self._stop = threading.Event()
        self._proc = None                             # current playback process
        self._available = None                        # memoized capability check (see available)

    def _pick_backend(self):
        """Choose the backend by what's installed (espeak-ng preferred on Linux, then
        Windows SAPI, then macOS say)."""
        if platform.system() == "Linux" and self._espeak and (self._aplay or self._paplay or self._ffplay):
            return "espeak"
        if platform.system() == "Windows" and self._ps:
            return "sapi"
        if platform.system() == "Darwin" and self._say:
            return "say"
        return ""

    def available(self):
        """True if a synthesis/playback backend was found (and TTS is on). Resolved
        once, so this is just a couple of truthiness checks — cheap to call from the
        always-on stats announcer."""
        if self._available is None:
            self._available = bool(self._enabled and self._backend)
        return self._available

    def configure(self, voice=None, volume=None, speed=None, pitch=None,
                  base_pitch=None, flutter=None, roughness=None, cap_pitch=None,
                  lead_silence=None):
        """Update the current voice and markup levels (each optional). Clamped."""
        if voice is not None and voice in VOICES:
            self._voice = voice
        if volume is not None:
            self._volume = clamp(volume, *VOLUME_RANGE)
        if speed is not None:
            self._speed = clamp(speed, *SPEED_RANGE)
        if pitch is not None:
            self._pitch = clamp(pitch, *PITCH_RANGE)
        if base_pitch is not None:
            self._base_pitch = clamp(base_pitch, *BASE_PITCH_RANGE)
        if flutter is not None:
            self._flutter = clamp(flutter, *FLUTTER_RANGE)
        if roughness is not None:
            self._roughness = clamp(roughness, *ROUGHNESS_RANGE)
        if cap_pitch is not None:
            self._cap_pitch = clamp(cap_pitch, *CAP_PITCH_RANGE)
        if lead_silence is not None:
            try:
                self._lead_silence = max(LEAD_SILENCE_RANGE[0],
                                          min(LEAD_SILENCE_RANGE[1], float(lead_silence)))
            except (TypeError, ValueError):
                pass

    @property
    def voice(self):
        return self._voice

    # ---- public API ----------------------------------------------------------
    def say(self, text, voice=None):
        if not self._enabled:
            self._log("tts: disabled")
            return
        text = _clean(text)[:MAX_CHARS].strip()
        if not text:
            return
        voice = voice if voice in VOICES else self._voice
        # Backends apply volume/speed/pitch from the snapshot levels inside their
        # _synth_* method — the raw text goes through unchanged here.
        with self._lock:
            # Stop any in-flight utterance and WAIT for its worker to fully unwind
            # (incl. its trailing on_word("")) before starting the new one, so the
            # old blank can't land on top of the new utterance's first word. The
            # worker never takes _lock, so joining while holding it can't deadlock.
            self._cancel_locked()
            if self._thread and self._thread.is_alive():
                self._thread.join(timeout=1.5)
            self._stop = threading.Event()
            # Snapshot the current markup levels for backends that apply them in synth.
            levels = (self._volume, self._speed, self._pitch,
                      self._base_pitch, self._flutter, self._roughness, self._cap_pitch)
            self._thread = threading.Thread(
                target=self._run, args=(voice, text, text, levels, self._stop),
                daemon=True)
            self._thread.start()

    def stop(self):
        with self._lock:
            self._cancel_locked()
        self._on_word("")                             # blank the OLED → dashboard

    # ---- internals -----------------------------------------------------------
    def _cancel_locked(self):
        """Signal the worker to stop and kill its playback. Caller holds _lock."""
        self._stop.set()
        p = self._proc
        if p and p.poll() is None:
            try:
                p.terminate()
            except Exception:
                pass

    def _wav_duration(self):
        """Duration (s) of the just-written WAV_PATH, or 0 if it can't be read."""
        try:
            with contextlib.closing(wave.open(WAV_PATH, "rb")) as w:
                rate = w.getframerate() or 1
                return w.getnframes() / float(rate)
        except Exception:
            return 0.0

    def _pad_lead_silence(self, secs=None):
        """Prepend `secs` (default: the live-tunable self._lead_silence) of silence
        to WAV_PATH (same format), so the codec's power-up ramp can't eat the first
        word (see LEAD_SILENCE). Returns the pad actually added — 0.0 on any error
        or a non-positive duration, in which case playback just proceeds with the
        unpadded clip."""
        if secs is None:
            secs = self._lead_silence
        if secs <= 0:
            return 0.0
        try:
            with contextlib.closing(wave.open(WAV_PATH, "rb")) as w:
                params = w.getparams()
                frames = w.readframes(w.getnframes())
            n = int(params.framerate * secs)
            with contextlib.closing(wave.open(WAV_PATH, "wb")) as w:
                w.setparams(params)
                w.writeframes(b"\x00" * (n * params.sampwidth * params.nchannels))
                w.writeframes(frames)
            return n / float(params.framerate)
        except Exception:
            return 0.0

    def _synth(self, voice, text, levels):
        """Run the active backend to write WAV_PATH. Returns the clip duration (s)."""
        try:
            if self._backend == "espeak":
                return self._synth_espeak(voice, text, levels)
            if self._backend == "sapi":
                return self._synth_sapi(voice, text, levels)
            if self._backend == "say":
                return self._synth_say(voice, text, levels)
        except Exception as exc:
            self._log(f"tts: synth failed ({exc})")
        return 0.0

    def _synth_sapi(self, voice, text, levels):
        """Windows SAPI via PowerShell System.Speech → WAV. Text + tunables are passed
        through the environment (never interpolated into the command) so arbitrary
        spoken text can't break or inject into the PowerShell script."""
        volume, speed, _pitch = levels
        env = dict(os.environ)
        env["NANO_TTS_TEXT"] = text
        env["NANO_TTS_WAV"] = WAV_PATH
        env["NANO_TTS_VOICE"] = voice
        env["NANO_TTS_VOL"] = str(clamp(min(volume, 100), 0, 100))   # SAPI Volume 0..100
        env["NANO_TTS_RATE"] = str(clamp(round((speed - 100) / 20.0), -10, 10))  # SAPI Rate
        script = (
            "Add-Type -AssemblyName System.Speech;"
            "$s = New-Object System.Speech.Synthesis.SpeechSynthesizer;"
            "try { $s.SelectVoiceByHints("
            "[System.Speech.Synthesis.VoiceGender]::NotSet,"
            "[System.Speech.Synthesis.VoiceAge]::NotSet, 0,"
            "[System.Globalization.CultureInfo]::GetCultureInfo($env:NANO_TTS_VOICE)) } catch {};"
            "$s.Volume = [int]$env:NANO_TTS_VOL;"
            "$s.Rate = [int]$env:NANO_TTS_RATE;"
            "$s.SetOutputToWaveFile($env:NANO_TTS_WAV);"
            "$s.Speak($env:NANO_TTS_TEXT);"
            "$s.Dispose()"
        )
        subprocess.run([self._ps, "-NoProfile", "-NonInteractive", "-Command", script],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                       timeout=30, check=True, env=env)
        return self._wav_duration()

    def _synth_say(self, voice, text, levels):
        """macOS `say` → 16-bit WAV. Text goes via a temp file (-f) to avoid quoting."""
        _volume, speed, _pitch = levels
        with open(_TXT_PATH, "w") as f:
            f.write(text)
        cmd = [self._say, "-o", WAV_PATH, "--data-format=LEI16@22050", "-f", _TXT_PATH]
        if speed != 100:                                # say -r words/minute (~175 normal)
            cmd += ["-r", str(int(175 * speed / 100.0))]
        if voice.startswith("de"):
            cmd += ["-v", "Anna"]                       # a built-in German voice if present
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                       timeout=30, check=True)
        return self._wav_duration()

    def _espeak_voice(self, voice):
        """Map UI voice codes to espeak-ng -v values."""
        return voice if voice in ("en-gb", "en-gb-x-gbclan", "en-gb-scotland") else "en-gb"

    def _synth_espeak(self, voice, text, levels):
        """Linux espeak-ng → 16-bit WAV. Speed maps to words/min (~175 default, clamped
        80..450 via the -s flag). Volume uses -a (amplitude 0..200). Pitch maps the UI
        percentage (50-200%) onto espeak's -p (0-99, default 50)."""
        _volume, speed, pitch, _base_pitch, _flutter, _roughness, cap_pitch = levels
        cmd = [self._espeak, "-w", WAV_PATH,
               "-v", self._espeak_voice(voice)]
        if speed != 100:
            wpm = clamp(175 * speed / 100.0, 80, 450)
            cmd += ["-s", str(int(wpm))]
        if _volume != 100:
            amp = clamp(_volume // 2, 0, 100)
            cmd += ["-a", str(int(amp))]
        # Pitch: map UI percentage (50-200%) → espeak -p (0-99, default 50).
        # At 100% → -p 50 (normal); 50% → -p 25; 200% → -p 99.
        p_val = clamp(int(pitch / 2), 0, 99)
        if p_val != 50:
            cmd += ["-p", str(p_val)]
        # Capital-letter emphasis: espeak -k <0-100> (0=off, higher=more rise).
        if cap_pitch:
            cmd += ["-k", str(clamp(cap_pitch, 0, 100))]
        cmd.append(text)
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                       timeout=30, check=True)
        return self._wav_duration()

    def _spawn_player(self):
        """Start (and return) a killable process playing WAV_PATH on the active
        backend, or None if there's no player. WAV_PATH is a fixed constant, so
        embedding it in the PowerShell command is safe (no user input)."""
        if self._backend in ("espeak",):
            if self._aplay:
                if self._device:
                    cmd = [self._aplay, "-D", self._device, WAV_PATH]
                else:
                    cmd = [self._aplay, "-q", WAV_PATH]
            elif self._paplay:
                cmd = [self._paplay, WAV_PATH]
            elif self._ffplay:
                cmd = [self._ffplay, "-nodisp", "-autoexit", "-loglevel", "quiet", WAV_PATH]
            else:
                return None
        elif self._backend == "sapi":
            cmd = [self._ps, "-NoProfile", "-NonInteractive", "-Command",
                   f"(New-Object System.Media.SoundPlayer '{WAV_PATH}').PlaySync()"]
        elif self._backend == "say" and self._afplay:
            cmd = [self._afplay, WAV_PATH]
        else:
            return None
        return subprocess.Popen(cmd, stdin=subprocess.DEVNULL,
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def _run(self, voice, display_text, synth_text, levels, stop):
        dur = self._synth(voice, synth_text, levels)
        if stop.is_set():
            return
        if dur <= 0.0:
            self._on_word("")
            return
        # Robot/aplay path only — dev-PC backends (SAPI/afplay) don't clip the start.
        # Skip the pad if we're still warm from a recent utterance (see
        # LEAD_SILENCE_KEEPALIVE) — no benefit, just a needless pause.
        warm = (self._last_speech_end > 0
                and (time.monotonic() - self._last_speech_end) < LEAD_SILENCE_KEEPALIVE)
        pad = self._pad_lead_silence() if (self._backend == "espeak" and not warm) else 0.0

        words = display_text.split()
        # Start fraction of each word = cumulative length / total length. Each word
        # is shown from its start until the next word's start (the last until the end).
        weights = [max(1, len(w)) for w in words]
        total = float(sum(weights)) or 1.0
        starts, acc = [], 0
        for wt in weights:
            starts.append(acc / total)
            acc += wt

        try:
            self._proc = self._spawn_player()
            if self._proc is None:
                raise RuntimeError("no player backend")
        except Exception as exc:
            self._log(f"tts: playback failed ({exc})")
            self._on_word("")
            return
        self._log(f"tts: speaking {len(words)} words in {voice} (~{dur:.1f}s)")

        t0 = time.monotonic() + pad                   # words start after the wake-up pad
        for i, word in enumerate(words):
            if not _wait_until(stop, t0 + dur * starts[i]):
                break                                 # cancelled
            self._on_word(word)
        else:
            # Spoke every word: hold the last one until the audio actually ends.
            while not stop.is_set() and self._proc.poll() is None:
                stop.wait(0.05)

        if self._proc and self._proc.poll() is None:
            try:
                self._proc.terminate()
                self._proc.wait(timeout=1.0)          # reap so it doesn't linger
            except Exception:
                pass
        self._last_speech_end = time.monotonic()      # amp/codec is warm until KEEPALIVE elapses
        self._on_word("")                             # hand the panel back


def _wait_until(stop, deadline):
    """Sleep until `deadline` (monotonic). Returns False if `stop` was set first."""
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return True
        if stop.wait(min(remaining, 0.05)):
            return False


def _resolve_bin(binary):
    """Absolute path to an executable: the given path if it's absolute + runnable,
    else found on PATH, else in the usual install dirs.
    Returns "" if not found anywhere."""
    if os.path.isabs(binary):
        return binary if os.access(binary, os.X_OK) else ""
    # Check /usr/bin first so official packages win over custom builds.
    for d in ("/usr/bin", "/bin", "/usr/local/bin"):
        cand = os.path.join(d, binary)
        if os.access(cand, os.X_OK):
            return cand
    return shutil.which(binary) or ""
