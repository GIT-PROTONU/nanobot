"""Text-to-speech for the web UI, English + German only (SVOX Pico / picotts).

The robot speaks a line typed in the web page and, in lock-step, streams the
individual words to the OLED so they appear/disappear as they're said (the line
is far too long for a 128x64 panel, so we karaoke it one word at a time).

Design — deliberately near-zero idle cost on the 1 GB / quad-A53 board:
  * Synthesis is `pico2wave` (a tiny C binary from https://github.com/ihuguet/picotts)
    writing a WAV to **/dev/shm** (tmpfs → no SD-card wear, freed right after).
  * Playback is `aplay` (alsa-utils, already used for the mic). Both processes are
    spawned ONLY while actually speaking, so when nobody triggers TTS it costs
    nothing — no thread, no process, no RAM beyond this object.
  * SVOX Pico's CLI emits no word-boundary marks, so we sync the OLED by spreading
    the clip's true duration (read from the WAV header) across the words weighted
    by length. It's an estimate, but it tracks short phrases well and is free.

**Backends.** The robot uses `pico2wave`+`aplay` (the only fully-featured path:
voice markup, German lingware, ALSA device targeting). But so the same TTS can be
exercised on a dev PC with no ROS/robot attached, the engine falls back to the OS's
built-in synthesiser when pico isn't installed: **Windows SAPI** (via PowerShell's
`System.Speech`) or **macOS `say`**. Every backend produces a WAV at `WAV_PATH` and
reports its duration, so the karaoke/word-timing logic below is backend-agnostic.
Voice/volume/speed markup is best-effort on the fallback backends (pitch and the
en-GB/de voice nuances are pico-only).

`say()` is fire-and-forget and self-cancelling: a new request stops the current
utterance (audio + OLED) and starts the new one. `on_word(word)` is called as each
word begins, and `on_word("")` once at the end to hand the panel back to the
dashboard.

Volume / speed / pitch are applied as SVOX Pico's inline markup (`<volume>`,
`<speed>`, `<pitch>` level tags — the same ones Android's Pico engine used), so
there's no extra processing or ALSA-mixer dependency. `configure()` sets them and
the current voice; the web layer persists those across reboots (see web_server).
"""
import contextlib
import math
import os
import platform
import re
import shutil
import subprocess
import tempfile
import threading
import time
import wave

# English + German voices only (the UI exposes exactly these; anything else falls
# back to the default). Matching pico lingware is installed by deploy/install-picotts.sh.
VOICES = ("en-US", "en-GB", "de-DE")

# tmpfs scratch on the robot (no SD-card wear); the OS temp dir on a dev PC that has
# no /dev/shm (Windows/macOS). Overwritten per utterance either way.
_SCRATCH = "/dev/shm" if os.path.isdir("/dev/shm") else tempfile.gettempdir()
WAV_PATH = os.path.join(_SCRATCH, "nano_tts.wav")
_TXT_PATH = os.path.join(_SCRATCH, "nano_tts.txt")   # for the macOS `say -f` backend
MAX_CHARS = 300                      # hard cap on a single utterance (also bounds synth time)

# Hard clamps for the Pico markup levels (percent; 100 = normal). The UI exposes
# friendlier sub-ranges, but anything out of these is clamped here regardless.
VOLUME_RANGE = (0, 500)
SPEED_RANGE = (20, 500)
PITCH_RANGE = (50, 200)


def clamp(v, lo, hi):
    try:
        return max(lo, min(hi, int(round(float(v)))))
    except (TypeError, ValueError):
        return lo


def _clean(text):
    """Make user text safe to both speak and (optionally) wrap in Pico markup, and
    keep the spoken words identical to what the OLED shows. Drops the three markup-
    significant chars (< > &) and common markdown/non-speech symbols (* _ ` ~ # |
    $ \\ [ ] { }) — not meaningful to speak anyway — and collapses whitespace so
    the karaoke word split is clean."""
    t = (text or "")
    for ch in ("<", ">", "&", "*", "_", "`", "~", "#", "|", "$", "\\", "[", "]", "{", "}"):
        t = t.replace(ch, " ")
    return " ".join(t.split())


class TtsEngine:
    """One-at-a-time speaker. Thread-safe: callers just call say()/stop()."""

    def __init__(self, pico_bin="pico2wave", device=None, default_voice="en-US",
                 enabled=True, on_word=None, logger=None):
        # Resolve to absolute paths up front: the stack is launched detached via
        # setsid/pixi where PATH can be trimmed, so a bare "pico2wave" might not exec
        # even though it's installed in /usr/local/bin. "" here means "not found".
        self._pico = _resolve_bin(pico_bin or "pico2wave")
        self._aplay = _resolve_bin("aplay")
        self._espeak = _resolve_bin("espeak-ng")
        self._device = device or None                 # aplay -D target; None = ALSA default
        self._enabled = bool(enabled)
        self._on_word = on_word or (lambda _w: None)
        self._log = logger or (lambda *_: None)

        # Pick the synthesis/playback backend once. The robot has pico+aplay (the
        # full-featured path); a dev PC falls back to the OS speech engine so TTS is
        # still testable off-robot. "" = no usable backend (available() -> False).
        self._ps = shutil.which("powershell") or shutil.which("pwsh") or ""  # Windows SAPI
        self._say = shutil.which("say") or ""                                # macOS
        self._afplay = shutil.which("afplay") or ""                          # macOS player
        self._backend = self._pick_backend()
        self._log(f"tts: backend={self._backend or 'none'}")

        # Live voice + markup levels (percent). Set via configure(); the web layer
        # restores the persisted values on startup.
        self._voice = default_voice if default_voice in VOICES else "en-US"
        self._volume = 100
        self._speed = 100
        self._pitch = 100

        self._lock = threading.Lock()
        self._thread = None
        self._stop = threading.Event()
        self._proc = None                             # current playback process
        self._available = None                        # memoized capability check (see available)

    def _pick_backend(self):
        """Choose the backend by what's installed (pico preferred — it's the only one
        with the German lingware + level markup + ALSA device targeting). Falls back to
        espeak-ng on Linux when pico is absent."""
        if self._pico and self._aplay:
            return "pico"
        if platform.system() == "Linux" and self._espeak and self._aplay:
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

    def configure(self, voice=None, volume=None, speed=None, pitch=None):
        """Update the current voice and markup levels (each optional). Clamped."""
        if voice is not None and voice in VOICES:
            self._voice = voice
        if volume is not None:
            self._volume = clamp(volume, *VOLUME_RANGE)
        if speed is not None:
            self._speed = clamp(speed, *SPEED_RANGE)
        if pitch is not None:
            self._pitch = clamp(pitch, *PITCH_RANGE)

    @property
    def voice(self):
        return self._voice

    def _ssml(self, text):
        """Pico2wave does not support SSML (confirmed — it speaks SSML tags as
        literal text), so this is a no-op. The volume/speed/pitch settings are
        stored and respected by non-pico backends that handle them natively."""
        return text

    # ---- public API ----------------------------------------------------------
    def say(self, text, voice=None):
        if not self._enabled:
            self._log("tts: disabled")
            return
        text = _clean(text)[:MAX_CHARS].strip()
        if not text:
            return
        voice = voice if voice in VOICES else self._voice
        # Pico level markup is pico-only; the OS fallback backends speak plain text
        # (they apply volume/speed natively in _synth_* from the snapshot below).
        synth = self._ssml(text) if self._backend == "pico" else text
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
            levels = (self._volume, self._speed, self._pitch)
            self._thread = threading.Thread(
                target=self._run, args=(voice, text, synth, levels, self._stop),
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

    def _synth(self, voice, text, levels):
        """Run the active backend to write WAV_PATH. Returns the clip duration (s)."""
        try:
            if self._backend == "pico":
                return self._synth_pico(voice, text)
            if self._backend == "espeak":
                return self._synth_espeak(voice, text, levels)
            if self._backend == "sapi":
                return self._synth_sapi(voice, text, levels)
            if self._backend == "say":
                return self._synth_say(voice, text, levels)
        except Exception as exc:
            self._log(f"tts: synth failed ({exc})")
        return 0.0

    def _chunk_text(self, text, max_chars=20):
        """Split text at word boundaries so each chunk is <= max_chars characters.
        pico2wave has a bug where text longer than ~24 characters produces garbled
        audio, so we keep each chunk safely under that threshold."""
        words = text.split()
        chunks = []
        cur = []
        cur_len = 0
        for w in words:
            add = len(w) + (1 if cur else 0)
            if cur_len + add > max_chars and cur:
                chunks.append(" ".join(cur))
                cur = [w]
                cur_len = len(w)
            else:
                cur.append(w)
                cur_len += add
        if cur:
            chunks.append(" ".join(cur))
        return chunks

    def _synth_pico(self, voice, text):
        """Synthesize text with pico2wave, splitting into ~20-char chunks to work around
        a bug in the Pico engine where texts longer than ~24 characters produce 6+ seconds
        of garbled audio. Each chunk is synthesized separately and the WAVs are concatenated
        into WAV_PATH."""
        try:
            os.remove(WAV_PATH)
        except FileNotFoundError:
            pass
        chunks = self._chunk_text(text)
        if len(chunks) == 1:
            subprocess.run([self._pico, "-l", voice, "-w", WAV_PATH, chunks[0]],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                           timeout=20, check=True)
            return self._wav_duration()
        temp_dir = tempfile.mkdtemp(prefix="nano_tts_")
        try:
            total_frames = 0
            framerate = None
            chunk_wavs = []
            for i, chunk in enumerate(chunks):
                wav = os.path.join(temp_dir, f"chunk_{i}.wav")
                subprocess.run([self._pico, "-l", voice, "-w", wav, chunk],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                               timeout=20, check=True)
                chunk_wavs.append(wav)
            out_data = bytearray()
            for wav in chunk_wavs:
                with contextlib.closing(wave.open(wav, "rb")) as w:
                    if framerate is None:
                        framerate = w.getframerate()
                    raw = w.readframes(w.getnframes())
                samples = memoryview(raw).cast("h")
                lo = next((i for i, s in enumerate(samples) if abs(s) > 20), 0)
                hi = next((len(samples) - i for i, s in enumerate(reversed(samples)) if abs(s) > 20), 0)
                if hi > lo:
                    out_data.extend(raw[lo * 2:hi * 2])
            with contextlib.closing(wave.open(WAV_PATH, "wb")) as out:
                out.setnchannels(1)
                out.setsampwidth(2)
                out.setframerate(framerate or 16000)
                out.writeframes(bytes(out_data))
                total_frames = out.getnframes()
            return total_frames / float(framerate or 16000)
        finally:
            for wav in chunk_wavs:
                try:
                    os.remove(wav)
                except Exception:
                    pass
            try:
                os.rmdir(temp_dir)
            except Exception:
                pass

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
        """Map pico-style voice codes to espeak-ng -v values."""
        return {"en-US": "en-us", "en-GB": "en-gb", "de-DE": "de"}.get(voice, "en-us")

    def _synth_espeak(self, voice, text, levels):
        """Linux espeak-ng → 16-bit WAV. Speed maps to words/min (~175 default, clamped
        80..600 via the -s flag — espeak understands 80..450 so we cap there but keep the
        clamp range wide for consistency with other backends)."""
        _volume, speed, _pitch = levels
        cmd = [self._espeak, "-w", WAV_PATH,
               "-v", self._espeak_voice(voice)]
        if speed != 100:
            wpm = clamp(175 * speed / 100.0, 80, 450)
            cmd += ["-s", str(int(wpm))]
        if _volume != 100:
            amp = clamp(_volume // 2, 0, 100)
            cmd += ["-a", str(int(amp))]
        cmd.append(text)
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                       timeout=30, check=True)
        return self._wav_duration()

    def _spawn_player(self):
        """Start (and return) a killable process playing WAV_PATH on the active
        backend, or None if there's no player. WAV_PATH is a fixed constant, so
        embedding it in the PowerShell command is safe (no user input)."""
        if self._backend in ("pico", "espeak"):
            cmd = [self._aplay, "-q"]
            if self._device:
                cmd += ["-D", self._device]
            cmd.append(WAV_PATH)
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

        t0 = time.monotonic()
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
    else found on PATH, else in the usual install dirs (preferring /usr/bin over
    /usr/local/bin so the official distribution packages win over custom builds).
    Returns "" if not found anywhere."""
    if os.path.isabs(binary):
        return binary if os.access(binary, os.X_OK) else ""
    # Check /usr/bin first so official packages (e.g. libttspico-utils) win
    # over self-built/custom binaries in /usr/local/bin which may be broken.
    for d in ("/usr/bin", "/bin", "/usr/local/bin"):
        cand = os.path.join(d, binary)
        if os.access(cand, os.X_OK):
            return cand
    return shutil.which(binary) or ""
