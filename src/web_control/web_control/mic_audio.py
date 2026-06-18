"""Zero-dependency microphone capture for the web UI.

Spawns `arecord` (alsa-utils, a tiny C process) to grab raw PCM from the USB
webcam's mic and fans the bytes out, untranscoded, to every connected HTTP
listener. No encoding (Opus/MP3/ffmpeg) — raw S16LE mono at 16 kHz is only
32 KB/s, costs the board almost nothing, and keeps latency low. arecord runs
only while at least one browser is listening, so it's free when nobody is.

The browser plays the stream with the Web Audio API (see web/index.html).
"""
import os
import queue
import re
import subprocess
import threading


def find_capture_device():
    """Return an ALSA device string for the USB audio card (the webcam mic),
    e.g. 'plughw:U0x46d0x825,0'. plughw lets ALSA convert rate/format/channels
    so we can always ask for 16 kHz mono S16LE regardless of the mic's native
    format. Falls back to None (caller errors) if no USB-Audio card is found."""
    try:
        with open("/proc/asound/cards") as f:
            for line in f:
                # " 1 [U0x46d0x825    ]: USB-Audio - USB Device 0x46d:0x825"
                if "USB-Audio" in line or "USB Audio" in line:
                    m = re.match(r"\s*\d+\s*\[(\S+)", line)
                    if m:
                        return "plughw:%s,0" % m.group(1)
    except Exception:
        pass
    return None


class AudioStream:
    """Shares one arecord capture among HTTP listeners. Starts on the first
    listener, stops when the last leaves. Each listener gets a small bounded
    queue; when a slow client can't keep up we drop the oldest chunk so latency
    and memory stay bounded (real-time audio: skip, don't pile up)."""

    def __init__(self, device=None, rate=16000, channels=1, logger=None):
        self._device = device or None
        self.rate = int(rate)
        self.channels = int(channels)
        self._log = logger or (lambda *_: None)
        self._lock = threading.Lock()
        self._listeners = set()        # set[queue.Queue[bytes]]
        self._proc = None
        self._thread = None
        self._run = False

    def _loop(self):
        dev = self._device or find_capture_device()
        if not dev:
            self._log("mic: no USB-Audio capture device found")
            with self._lock:
                self._run = False
            return
        cmd = ["arecord", "-q", "-D", dev, "-t", "raw",
               "-f", "S16_LE", "-r", str(self.rate), "-c", str(self.channels),
               # small period/buffer -> low latency (~30 ms period, ~150 ms buffer)
               "--period-size=480", "--buffer-size=2400"]
        try:
            self._proc = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)
        except Exception as exc:
            self._log(f"mic: arecord failed to start ({exc})")
            with self._lock:
                self._run = False
            return
        self._log(f"mic: streaming {dev} @ {self.rate} Hz x{self.channels}")
        fd = self._proc.stdout.fileno()
        while self._run:
            try:
                data = os.read(fd, 4096)   # returns as soon as a period is ready
            except Exception:
                break
            if not data:
                break                      # arecord exited
            with self._lock:
                for q in self._listeners:
                    if q.full():
                        try:
                            q.get_nowait()  # drop oldest -> bound latency/memory
                        except queue.Empty:
                            pass
                    try:
                        q.put_nowait(data)
                    except queue.Full:
                        pass
        self._stop_proc()
        with self._lock:
            self._run = False

    def _stop_proc(self):
        p, self._proc = self._proc, None
        if p:
            try:
                p.terminate()
                p.wait(timeout=1.0)
            except Exception:
                try:
                    p.kill()
                except Exception:
                    pass

    def add_listener(self):
        q = queue.Queue(maxsize=32)        # ~32 periods (~1 s) hard cap
        with self._lock:
            self._listeners.add(q)
            if self._thread is None or not self._thread.is_alive():
                self._run = True
                self._thread = threading.Thread(target=self._loop, daemon=True)
                self._thread.start()
        return q

    def remove_listener(self, q):
        with self._lock:
            self._listeners.discard(q)
            if not self._listeners:
                self._run = False          # _loop sees this and stops arecord

    def running(self):
        with self._lock:
            return self._run
