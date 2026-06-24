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
import os
import subprocess
import threading
import time
import wave

# English + German voices only (the UI exposes exactly these; anything else falls
# back to the default). Matching pico lingware is installed by deploy/install-picotts.sh.
VOICES = ("en-US", "en-GB", "de-DE")

WAV_PATH = "/dev/shm/nano_tts.wav"   # tmpfs scratch; overwritten per utterance
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
    significant chars (< > &) — not meaningful to speak anyway — and collapses
    whitespace so the karaoke word split is clean."""
    t = (text or "").replace("<", " ").replace(">", " ").replace("&", " ")
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
        self._device = device or None                 # aplay -D target; None = ALSA default
        self._enabled = bool(enabled)
        self._on_word = on_word or (lambda _w: None)
        self._log = logger or (lambda *_: None)

        # Live voice + markup levels (percent). Set via configure(); the web layer
        # restores the persisted values on startup.
        self._voice = default_voice if default_voice in VOICES else "en-US"
        self._volume = 100
        self._speed = 100
        self._pitch = 100

        self._lock = threading.Lock()
        self._thread = None
        self._stop = threading.Event()
        self._proc = None                             # current aplay process
        self._available = None                        # memoized binary check (see available)

    def available(self):
        """True if both the synth and playback binaries were found (and TTS is on).
        Resolved once at construction, so this is just a couple of truthiness checks
        — cheap to call from the always-on stats announcer."""
        if self._available is None:
            self._available = bool(self._enabled and self._pico and self._aplay)
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
        """Add Pico's volume/speed/pitch level markup, but ONLY for a level that's
        actually changed from the 100% default. At all-default this returns the text
        untouched, so the common case is vanilla pico2wave (the clearest, most-tested
        path) with no markup to misparse. The text has already had the markup-special
        chars stripped (see _clean), so nothing here needs escaping."""
        out = text
        if self._speed != 100:
            out = f'<speed level="{self._speed}">{out}</speed>'
        if self._pitch != 100:
            out = f'<pitch level="{self._pitch}">{out}</pitch>'
        if self._volume != 100:
            out = f'<volume level="{self._volume}">{out}</volume>'
        return out

    # ---- public API ----------------------------------------------------------
    def say(self, text, voice=None):
        if not self._enabled:
            self._log("tts: disabled")
            return
        text = _clean(text)[:MAX_CHARS].strip()
        if not text:
            return
        voice = voice if voice in VOICES else self._voice
        synth = self._ssml(text)                       # snapshot current vol/speed/pitch
        with self._lock:
            # Stop any in-flight utterance and WAIT for its worker to fully unwind
            # (incl. its trailing on_word("")) before starting the new one, so the
            # old blank can't land on top of the new utterance's first word. The
            # worker never takes _lock, so joining while holding it can't deadlock.
            self._cancel_locked()
            if self._thread and self._thread.is_alive():
                self._thread.join(timeout=1.5)
            self._stop = threading.Event()
            self._thread = threading.Thread(
                target=self._run, args=(voice, text, synth, self._stop), daemon=True)
            self._thread.start()

    def stop(self):
        with self._lock:
            self._cancel_locked()
        self._on_word("")                             # blank the OLED → dashboard

    # ---- internals -----------------------------------------------------------
    def _cancel_locked(self):
        """Signal the worker to stop and kill its aplay. Caller holds _lock."""
        self._stop.set()
        p = self._proc
        if p and p.poll() is None:
            try:
                p.terminate()
            except Exception:
                pass

    def _synth(self, voice, text):
        """Run pico2wave → WAV_PATH. Returns the clip duration in seconds, or 0."""
        try:
            subprocess.run([self._pico, "-l", voice, "-w", WAV_PATH, text],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                           timeout=20, check=True)
        except Exception as exc:
            self._log(f"tts: pico2wave failed ({exc})")
            return 0.0
        try:
            with contextlib.closing(wave.open(WAV_PATH, "rb")) as w:
                rate = w.getframerate() or 1
                return w.getnframes() / float(rate)
        except Exception:
            return 0.0

    def _run(self, voice, display_text, synth_text, stop):
        dur = self._synth(voice, synth_text)
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
            cmd = [self._aplay, "-q"]
            if self._device:
                cmd += ["-D", self._device]
            cmd.append(WAV_PATH)
            self._proc = subprocess.Popen(cmd, stdin=subprocess.DEVNULL,
                                          stdout=subprocess.DEVNULL,
                                          stderr=subprocess.DEVNULL)
        except Exception as exc:
            self._log(f"tts: aplay failed ({exc})")
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
    else found on PATH, else in the usual install dirs (systemd/pixi PATH can be
    minimal). Returns "" if not found anywhere."""
    if os.path.isabs(binary):
        return binary if os.access(binary, os.X_OK) else ""
    dirs = os.environ.get("PATH", "").split(os.pathsep) + ["/usr/local/bin", "/usr/bin", "/bin"]
    for d in dirs:
        if d:
            cand = os.path.join(d, binary)
            if os.access(cand, os.X_OK):
                return cand
    return ""
