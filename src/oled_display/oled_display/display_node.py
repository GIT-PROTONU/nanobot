"""Status OLED for the SSD1306 (I2C) via luma.oled. Two display modes:

FACE is the **default** idle screen — the panel "feels alive" by showing the eyes
whenever nothing else owns it (the behaviour layer drives the mood; between beats the
panel rests on `idle_face`). The status DASHBOARD is shown only while it is **pinned**
(`/oled_dashboard` True, from the web UI's Dashboard toggle). TTS karaoke words take over
the panel only while `show_words` is on (`/oled_show_words`); with it off, speech keeps
the face up. See `_effective_mood` for the arbitration.

DASHBOARD (pinned) — a compact status panel:

    ┌───────────────────────────────┐
    │ NANOBOT              12:34:56  │  header bar (inverted): brand + clock
    │ 192.168.178.141          47C   │  primary IP  +  SBC CPU temp (right)
    │ ───────────────────────────── │
    │ ESP  ●  39C          CPU 23%   │  left col: subsystems (● online/○ off);
    │ IMU  ●  50Hz         RAM 61%   │  right col: SBC vitals (temp/CPU%/RAM%)
    │ LDS  ○  off         R+2 P-1    │  CPU = busy %, RAM = used %; R/P = IMU
    └───────────────────────────────┘  roll/pitch tilt in degrees (when alive)

FACE — cute animated robot eyes that blink and glance around, used to express a
mood. Enabled from the web UI by publishing a mood name on /oled_face (empty
string returns to the dashboard). Moods (KNOWN_MOODS): "happy" (upward ^_^
crescents), "angry" (slanted scowling brows), "focused" (narrowed eyes with a
staring pupil), "stress" (jittery alarm), "sleepy" (closed eyes + drifting z z z,
the AI-offline mood) and "looking" (wide scanning eyes). A face can also be a
compound "shape:emotion" accent, e.g. "looking:happy" — see _face_tick.

Efficiency (the face must be near-free on a 1 GB / quad-A53 board):
  * The face has its own timer that is **cancelled** while in dashboard mode, so
    it adds zero executor wakeups unless you're actually animating.
  * Even while animating it only pushes a frame to the panel when the rendered
    picture changes (a small "signature" dirty-check) — open, settled eyes draw
    nothing, so I2C traffic happens only during a blink or a glance.
  * Blink/glance timing is randomized, so the animation never looks like it is
    replaying a fixed loop.

Data sources (all lightweight — NO telemetry topics are subscribed anymore):
    /dev/shm/nano_vitals.json         → ESP32 liveness+temp, IMU rate/tilt, LDS hz,
                                        CPU%/RAM%/SBC-temp — the one aggregated body
                                        snapshot sys_monitor writes each tick (read
                                        only while the dashboard is pinned, ≤1 Hz);
                                        falls back to local /proc reads when stale
    /oled_text       (String)         → optional brand override (web UI textbox)
    /oled_face       (String)         → mood name / "" for the resting idle face (web UI)
    /oled_dashboard  (Bool)           → pin the status dashboard always (web UI toggle; default
                                        off = show the face)
    /oled_show_words (Bool)           → show TTS karaoke words on the panel (web UI toggle;
                                        default on; off keeps the face up while speaking)
    /oled_word       (String)         → one TTS word at a time, shown big+centred while
                                        speaking; "" returns to the face/dashboard (web_control)
    /oled_mask       (Bool, latched)  → mirror the GPU colour-tracking mask (web_control);
                                        frames come from /dev/shm/nano_oled_mask.bin
    /sys/class/thermal/thermal_zone0/temp → SBC CPU temp (cached file read)
    /proc/stat                        → SBC CPU busy % (delta-sampled file read)
    /proc/meminfo                     → SBC RAM used % (cached file read)

Subscribe-only and best-effort: if the panel or luma isn't present the node still
runs and just logs once, so the rest of the stack is unaffected.
"""
import json
import math
import os
import random
import signal
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, String

try:
    from luma.core.interface.serial import i2c
    from luma.core.render import canvas
    from luma.oled.device import ssd1306
    from PIL import Image, ImageDraw, ImageFont
    HAVE_LUMA = True
except Exception as exc:  # pragma: no cover - hardware lib
    HAVE_LUMA = False
    _LUMA_ERR = exc

try:
    import numpy as _np
except Exception:  # pragma: no cover
    _np = None


def _patch_fast_display(device):
    """Replace luma's ssd1306.display() frame-pack — a pure-Python per-pixel loop,
    benchmarked at ~10 ms/frame on the H5 — with an np.packbits version (~0.6 ms,
    17x). Wire bytes are identical: SSD1306 pages are 8-row bands, one byte per
    column, LSB = the band's top row (packbits axis=1, bitorder little). No-op if
    numpy is missing or the frame geometry is unexpected (falls back to luma)."""
    if _np is None:
        return

    def fast_display(image):
        image = device.preprocess(image)
        w, h = image.size
        if image.mode != "1" or w % 8 or h % 8:
            return type(device).display(device, image)   # odd geometry: luma path
        device.command(
            device._const.COLUMNADDR, device._colstart, device._colend - 1,
            device._const.PAGEADDR, 0x00, device._pages - 1)
        bits = _np.unpackbits(_np.frombuffer(image.tobytes(), dtype=_np.uint8))
        buf = _np.packbits(bits.reshape(h // 8, 8, w), axis=1, bitorder="little")
        device.data(buf.ravel().tolist())

    device.display = fast_display

IP_REFRESH_S = 30.0    # re-resolve the outbound IP at most this often
TEMP_REFRESH_S = 2.0   # re-read the SBC thermal zone at most this often
SYS_REFRESH_S = 2.0    # re-sample CPU% + RAM at most this often (also the CPU% window)
VITALS_FILE = "/dev/shm/nano_vitals.json"  # sys_monitor's aggregated body snapshot
VITALS_REFRESH_S = 1.0 # re-read the vitals blob at most this often
VITALS_FRESH_S = 5.0   # older than this (writer down) -> local /proc fallback
THERMAL_PATH = "/sys/class/thermal/thermal_zone0/temp"  # cpu-thermal (millidegrees)
MEMINFO_PATH = "/proc/meminfo"                          # RAM totals (kB)
STAT_PATH = "/proc/stat"                                # cpu jiffies for busy %
# The web UI writes "restart" / "shutdown" here just before it stops the stack, so the
# OLED node (stopped via SIGTERM) knows which end-screen to show. /dev/shm is tmpfs the
# stack user can write and is cleared on reboot.
ACTION_FILE = "/dev/shm/nano_oled_action"
# GPU vision's colour-tracking mask, pre-reduced to the panel's exact 128x64 and
# re-binarized (one byte/pixel, 0/255) -- written by web_control's gpu_vision.py while
# the web UI's "mirror mask to OLED" toggle is on; shown while /oled_mask is true.
MASK_FILE = "/dev/shm/nano_oled_mask.bin"
MASK_FRESH_S = 2.0     # blob older than this (writer stopped) -> "waiting" placeholder

# Staleness windows: a source is "alive" only if it arrived within this many
# seconds. Generous enough to ride out one missed message at each topic's rate.
ESP_TIMEOUT_S = 4.0    # /esp32_heartbeat ~1 Hz
IMU_TIMEOUT_S = 2.5    # /imu/web can run as slow as 1 Hz (publish_rate default)
LDS_TIMEOUT_S = 3.0    # /lds_hz a few Hz

BLINK_DUR = 0.16       # seconds for one eyelid close+open
# "sleepy" = the AI-offline mood: closed/content eyes + a slow rising "z z z" (see _sleepy).
KNOWN_MOODS = ("happy", "angry", "focused", "stress", "sleepy", "looking")
# A face can be a plain mood ("looking") OR a compound "shape:emotion" ("looking:happy"): the
# SHAPE (left) sets the eye geometry (the action — what the robot is doing), the EMOTION accent
# (right) is a small overlay drawn ON TOP (how it feels). This composes N shapes x M emotions
# from N+M pieces — see _draw_face / _accent_overlay. A plain mood draws exactly as before, so
# every legacy single mood is byte-identical (the accent path is purely additive).
ACCENT_MOODS = KNOWN_MOODS         # any known mood may also be used as an emotion accent

# Eye geometry, sized to fill a 128x64 panel. The inner gap is 2*(EYE_DX-EYE_W);
# EYE_DX=37 / EYE_W=20 → a 34 px gap between the eyes with ~7 px edge margins, so
# a glance (gaze offset) never pushes an eye off the side of the screen.
EYE_DX = 37            # eye centre offset from screen centre (px)
EYE_W = 20             # eye half-width (px)
EYE_H = 26             # eye half-height when fully open (px)


def _primary_ip() -> str:
    """Best-effort outbound IP without actually sending anything."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except Exception:
        return "0.0.0.0"
    finally:
        s.close()


class DisplayNode(Node):
    def __init__(self):
        super().__init__("oled_display", start_parameter_services=False)
        self.declare_parameters("", [
            ("i2c_bus", 1), ("i2c_address", 0x3C),
            ("width", 128), ("height", 64),
            ("refresh_rate", 2.0), ("show_ip", True),
            ("anim_fps", 20.0),    # face animation tick rate (only ticks while in face mode;
                                   # safely under the ~26 fps i2c-0@400kHz flush ceiling)
            ("rotate", 0),         # screen rotation in 90° steps; 2 = 180° (upside-down mount)
            # The face is the default idle screen; `idle_face` is the resting mood worn between
            # beats (when /oled_face is ""). The dashboard is shown only while pinned.
            ("idle_face", "neutral"),     # resting face when no mood/beat is active
            ("dashboard", False),         # boot default for the dashboard pin (web toggle overrides)
            ("show_words", True),         # boot default for showing TTS karaoke words
        ])
        g = self.get_parameter
        self.show_ip = g("show_ip").value
        self.width = g("width").value
        self.height = g("height").value

        # Latest telemetry — written by callbacks, read by the render timer. The
        # *_at fields are monotonic arrival times used to decide liveness.
        self.text = ""                  # optional brand override from the web UI
        self.face_mood = ""             # raw mood requested on /oled_face ("" = no active mood)
        self.face_accent = ""           # emotion accent from a compound "shape:emotion" request
        self.idle_face = str(g("idle_face").value or "neutral")  # resting face between beats
        self.pin_dashboard = bool(g("dashboard").value)   # True = show the dashboard always
        self.show_words = bool(g("show_words").value)     # True = TTS words take over the panel
        # The EFFECTIVE mood actually rendered: "" means the dashboard, else a face. Derived from
        # face_mood + idle_face + the dashboard pin in _recompute_mood (the single arbiter).
        self._mood = ""
        self._accent = ""               # effective emotion accent overlaid on the shape
        self._reflecting = False        # /oled_reflecting: owns the panel above everything
                                         # else except a spoken word or a shutdown/restart screen
        self._mask_mode = False         # /oled_mask: mirror the GPU tracking mask (below
                                         # reflecting, above the face/dashboard)
        self.speak_word = ""            # current TTS word (karaoke); "" = not speaking
        self.esp_temp = float("nan")
        self.esp_at = -1e9
        self.imu_hz = 0.0
        self.imu_at = -1e9
        self.imu_roll = 0.0             # /imu/euler roll  (deg) — dashboard tilt readout
        self.imu_pitch = 0.0            # /imu/euler pitch (deg)
        self.lds_hz = 0.0
        self.lds_at = -1e9

        # Cache host/IP + SBC temp so the render loop never blocks per frame.
        self._ip = "0.0.0.0"
        self._ip_due = 0.0
        self._sbc = float("nan")
        self._sbc_due = 0.0
        self._cpu_pct = float("nan")     # SBC CPU busy % (delta-sampled)
        self._cpu_prev = None            # last (idle, total) jiffies for the delta
        self._mem_pct = float("nan")     # SBC RAM used %
        self._sys_due = 0.0

        # Face animation state (see _anim_update / _draw_face).
        now = time.monotonic()
        self._anim_t = now
        self._open = 1.0                # eyelid openness 0..1
        self._blink_t = None            # progress through a blink, or None
        self._double = False            # a second quick blink is queued
        self._next_blink = now
        self._gxf = self._gyf = 0.0     # gaze offset (float, eased)
        self._gtx = self._gty = 0.0     # gaze target
        self._gx = self._gy = 0         # gaze offset (int, drawn)
        self._next_gaze = now
        self._face_sig = None           # last drawn face signature (dirty-check)
        self._frame = 0                 # free-running frame counter (stress mood)
        self._next_smile = now          # happy: next recurring crescent "smile" beat
        self._smile_until = 0.0         # happy: crescent shown until this time
        self._off = False               # panel has been powered off (shutdown)
        self._sys = ""                  # "restart"/"shutdown" once a system action starts

        self.device = None
        self.font = None
        if HAVE_LUMA:
            try:
                serial = i2c(port=g("i2c_bus").value, address=g("i2c_address").value)
                self.device = ssd1306(serial, width=self.width, height=self.height,
                                      rotate=int(g("rotate").value) % 4)
                _patch_fast_display(self.device)
                self.font = ImageFont.load_default()
                self.get_logger().info(
                    f"SSD1306 ready on /dev/i2c-{g('i2c_bus').value} "
                    f"@0x{g('i2c_address').value:02x}")
            except Exception as exc:
                self.get_logger().error(f"OLED init failed: {exc}")
        else:
            self.get_logger().error(f"luma.oled unavailable: {_LUMA_ERR}")

        # Telemetry comes from the vitals blob (see the module docstring), not topics —
        # this node subscribes ONLY to its own /oled_* control topics.
        self._vitals_due = 0.0
        self.create_subscription(String, "oled_text", self._on_text, 10)
        self.create_subscription(String, "oled_face", self._on_face, 10)
        self.create_subscription(Bool, "oled_dashboard", self._on_dashboard, 10)
        self.create_subscription(Bool, "oled_show_words", self._on_show_words, 10)
        self.create_subscription(String, "oled_word", self._on_word, 10)
        self.create_subscription(String, "oled_system", self._on_system, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "oled_reflecting", self._on_reflecting, latched)
        self.create_subscription(Bool, "oled_mask", self._on_mask, latched)

        # Dashboard ticks slowly; the face has its own faster timer that runs while a face is
        # shown (the default) and is cancelled (no wakeups) only while the dashboard is pinned.
        self.create_timer(1.0 / g("refresh_rate").value, self._dashboard_tick)
        self._face_timer = self.create_timer(1.0 / max(1.0, g("anim_fps").value),
                                             self._face_tick)
        self._face_timer.cancel()
        self._recompute_mood()          # seed the effective mood (face by default) + face timer

    # ---- subscriptions (record value + arrival time only) ----
    def _on_text(self, msg: String):
        self.text = msg.data
        self.get_logger().info(f"OLED brand set to {msg.data!r}")

    def _on_face(self, msg: String):
        raw = msg.data.strip().lower()
        if raw in ("", "off", "dashboard", "none"):
            self.face_mood = ""           # no active mood -> fall back to the resting idle face
            self.face_accent = ""
        else:
            # Compound "shape:emotion" — the action shape (left) carries the emotion accent
            # (right). A plain mood has no accent and renders exactly as before.
            shape, _, accent = raw.partition(":")
            self.face_mood = shape if shape in KNOWN_MOODS else "neutral"
            self.face_accent = accent if accent in ACCENT_MOODS and accent != self.face_mood else ""
        self._recompute_mood()

    def _on_reflecting(self, msg: Bool):
        """Reflection mode on/off (behavior/mood_node._on_reflect). Not a mood — a
        sustained consolidation state, not a quick reactive expression — so it gets
        its own screen (_reflecting_glyph) rather than borrowing a face."""
        self._reflecting = bool(msg.data)
        self._recompute_mood()

    def _on_mask(self, msg: Bool):
        """GPU tracking-mask mirror on/off (web_control's /oled_mask, latched). A
        sustained tool view like reflecting, not a mood — it borrows the face timer for
        redraw pacing but renders the /dev/shm mask blob instead of eyes. The face
        stands down for it (same yield-the-panel model as every other owner); spoken
        words and shutdown/restart screens still interrupt it."""
        self._mask_mode = bool(msg.data)
        self._recompute_mood()

    def _on_dashboard(self, msg: Bool):
        """Web UI Dashboard toggle: pin the status dashboard always (True) or show the face
        (False, the default)."""
        self.pin_dashboard = bool(msg.data)
        self._recompute_mood()

    def _on_show_words(self, msg: Bool):
        """Web UI 'show spoken text' toggle. Off keeps the face up while speaking; if turned off
        mid-speech, drop the current word now and hand the panel back."""
        self.show_words = bool(msg.data)
        if not self.show_words and self.speak_word:
            self.speak_word = ""
            self._resume_after_word()

    def _recompute_mood(self):
        """Derive the EFFECTIVE mood from reflection mode + the dashboard pin + the
        requested/idle face, and start/stop the face animation timer on a
        face<->dashboard transition. "" = dashboard, "reflecting" = the dedicated
        reflection-mode screen. The single place face vs dashboard vs reflecting is
        decided, so every owner stays consistent. Reflecting outranks the dashboard
        pin AND any requested face — it owns the panel until it ends (a spoken word
        or a shutdown/restart screen can still interrupt it, gated separately in
        _face_tick/_on_word/_on_system)."""
        if self._reflecting:
            new, new_acc = "reflecting", ""
        elif self._mask_mode:
            new, new_acc = "mask", ""
        else:
            new = "" if self.pin_dashboard else (self.face_mood or self.idle_face)
            # The emotion accent rides only an actively-requested mood — the resting idle face is plain.
            new_acc = "" if (self.pin_dashboard or not self.face_mood) else self.face_accent
        if new == self._mood and new_acc == self._accent:
            return
        was_face = bool(self._mood)
        self._mood = new
        self._accent = new_acc
        if new and not was_face:          # entering face mode: re-seed the animation
            now = time.monotonic()
            self._anim_t = now
            self._open = 1.0
            self._blink_t = None
            self._double = False
            self._next_blink = now + random.uniform(1.5, 3.5)
            self._gxf = self._gyf = self._gtx = self._gty = 0.0
            self._gx = self._gy = 0
            self._next_gaze = now + random.uniform(1.0, 2.5)
            self._next_smile = now + random.uniform(2.0, 5.0)
            self._smile_until = 0.0
            self._face_sig = None
            self._face_timer.reset()
        elif not new and was_face:        # dashboard pinned: stop the face timer, redraw now
            self._face_timer.cancel()
            self._dashboard_tick()
        else:                             # face -> different face/accent: redraw with the new mood
            self._face_sig = None
        label = f"{self._mood}:{self._accent}" if self._accent else (self._mood or "dashboard")
        self.get_logger().info(f"OLED mode: {label}")

    def _has(self, mood):
        """True if `mood` is in play as either the base shape OR the emotion accent. Lets the
        animation cadence + draw paths treat "looking:happy" as both looking AND happy."""
        return self._mood == mood or self._accent == mood

    def _resume_after_word(self):
        """Hand the panel back after a karaoke word ends (or is suppressed): reseed the face if
        one is up, else redraw the dashboard immediately."""
        if self._mood:                    # resume the face cleanly on the next tick
            self._anim_t = time.monotonic()
            self._face_sig = None
        else:
            self._dashboard_tick()        # redraw the dashboard immediately

    def _on_word(self, msg: String):
        """TTS karaoke: web_control streams one word at a time as it's spoken. A word
        takes over the panel (big + centred); the empty string ends speech and hands
        the panel back to the face (if a mood is up) or the dashboard. Drawn inline
        here — words arrive at speech rate, so this stays in sync with the audio."""
        if self._sys:                       # a shutdown/restart screen owns the panel
            return
        word = msg.data.strip()
        if word and self.show_words:        # karaoke disabled -> ignore, keep the face up
            self.speak_word = word
            self._draw_word(word)
        elif self.speak_word:
            self.speak_word = ""
            self._resume_after_word()

    def _on_system(self, msg: String):
        """Web UI sends 'restart' (ROS stack) / 'reboot' (whole SBC) / 'shutdown' the
        instant the button is clicked, so the end-screen appears immediately (well before
        the stack teardown's SIGTERM)."""
        s = msg.data.strip().lower()
        if s not in ("restart", "reboot", "shutdown") or self._sys:
            return
        self._sys = s
        try:
            self._face_timer.cancel()       # freeze normal rendering on the end-screen
        except Exception:
            pass
        if self.device:
            if s == "shutdown":
                self._shutdown_screen()
            else:                           # 'restart' = stack only, 'reboot' = whole board
                self._restart_screen("Restarting stack" if s == "restart" else "Restarting")

    def _refresh_vitals(self, now):
        """Fold the vitals blob into the telemetry fields (throttled file read; only
        called from the dashboard tick, so face mode costs nothing). Per-source ages in
        the blob are wall-clock-adjusted by the file's own age, so a dead sensor_hub
        naturally ages every row into 'off'. Returns True when the blob is fresh enough
        to also trust its CPU/RAM/temp (else the caller falls back to local /proc)."""
        if now < self._vitals_due:
            return getattr(self, "_vitals_fresh", False)
        self._vitals_due = now + VITALS_REFRESH_S
        self._vitals_fresh = False
        try:
            with open(VITALS_FILE) as f:
                v = json.load(f)
        except (OSError, ValueError):
            return False
        file_age = max(0.0, time.time() - float(v.get("t", 0)))
        if file_age > VITALS_FRESH_S:
            return False
        self._vitals_fresh = True
        esp = v.get("esp") or {}
        if esp.get("hb_age") is not None:
            self.esp_at = now - (esp["hb_age"] + file_age)
        if esp.get("temp") is not None:
            self.esp_temp = float(esp["temp"])
        imu = v.get("imu") or {}
        if imu.get("hz") is not None:
            self.imu_hz = float(imu["hz"])
            self.imu_at = now - (float(imu.get("age", 0)) + file_age)
        eul = v.get("eul") or {}
        if eul.get("r") is not None:
            self.imu_roll = float(eul["r"])
            self.imu_pitch = float(eul.get("p") or 0.0)
        lds = v.get("lds") or {}
        if lds.get("hz") is not None:
            self.lds_hz = float(lds["hz"])
            self.lds_at = now - (float(lds.get("age", 0)) + file_age)
        if v.get("temp") is not None:
            self._sbc = float(v["temp"])
        if v.get("cpu") is not None:
            self._cpu_pct = float(v["cpu"])
        if v.get("mem") is not None:
            self._mem_pct = float(v["mem"])
        return True

    # ---- cached, non-blocking reads for the render loop (vitals fallback) ----
    def _cached_ip(self) -> str:
        now = time.monotonic()
        if now >= self._ip_due:
            self._ip = _primary_ip()
            self._ip_due = now + IP_REFRESH_S
        return self._ip

    def _cached_sbc_temp(self) -> float:
        now = time.monotonic()
        if now >= self._sbc_due:
            self._sbc_due = now + TEMP_REFRESH_S
            try:
                with open(THERMAL_PATH) as f:
                    self._sbc = int(f.read().strip()) / 1000.0
            except Exception:
                self._sbc = float("nan")
        return self._sbc

    def _cached_sys(self):
        """(CPU busy %, RAM used %) — cheap throttled /proc reads so the render loop
        never recomputes per frame. CPU% is delta-sampled over SYS_REFRESH_S from the
        aggregate /proc/stat line, matching the web UI's cpu_percent."""
        now = time.monotonic()
        if now >= self._sys_due:
            self._sys_due = now + SYS_REFRESH_S
            try:
                with open(STAT_PATH) as f:
                    parts = [int(x) for x in f.readline().split()[1:]]
                idle = parts[3] + (parts[4] if len(parts) > 4 else 0)  # idle + iowait
                total = sum(parts)
                if self._cpu_prev is not None:
                    di, dt = idle - self._cpu_prev[0], total - self._cpu_prev[1]
                    self._cpu_pct = 100.0 * (1.0 - di / dt) if dt > 0 else float("nan")
                self._cpu_prev = (idle, total)
            except Exception:
                self._cpu_pct = float("nan")
            try:
                tot = avail = 0
                with open(MEMINFO_PATH) as f:
                    for line in f:
                        if line.startswith("MemTotal:"):
                            tot = int(line.split()[1])
                        elif line.startswith("MemAvailable:"):
                            avail = int(line.split()[1])
                        if tot and avail:
                            break
                self._mem_pct = 100.0 * (tot - avail) / tot if tot else float("nan")
            except Exception:
                self._mem_pct = float("nan")
        return self._cpu_pct, self._mem_pct

    def _text_w(self, s: str) -> int:
        try:
            return int(self.font.getlength(s))
        except Exception:
            return len(s) * 6

    # ---- dashboard ----
    def _row(self, draw, y, name, alive, value):
        """One subsystem row: name, status dot (filled=alive), metric value."""
        draw.text((2, y), name, font=self.font, fill=255)
        cx, cy, r = 30, y + 1, 3
        draw.ellipse((cx, cy, cx + 2 * r, cy + 2 * r), outline=255,
                     fill=255 if alive else 0)
        draw.text((48, y), value, font=self.font, fill=255)

    def _dashboard_tick(self):
        if not self.device or self._mood or self.speak_word or self._sys:
            return                                           # face/speech/system owns it
        now = time.monotonic()
        vitals = self._refresh_vitals(now)   # feeds the esp/imu/lds fields + SBC stats
        esp_up = (now - self.esp_at) < ESP_TIMEOUT_S
        imu_up = (now - self.imu_at) < IMU_TIMEOUT_S
        lds_up = (now - self.lds_at) < LDS_TIMEOUT_S and self.lds_hz > 0.1
        W = self.width

        with canvas(self.device) as draw:
            # Header bar: inverted brand (left) + clock (right).
            draw.rectangle((0, 0, W - 1, 11), fill=255)
            brand = (self.text or "NANOBOT")[:12]
            draw.text((2, 2), brand, font=self.font, fill=0)
            clock = time.strftime("%H:%M:%S")
            draw.text((W - 2 - self._text_w(clock), 2), clock, font=self.font, fill=0)

            # IP (left) + SBC CPU temp (right).
            if self.show_ip:
                draw.text((2, 14), self._cached_ip(), font=self.font, fill=255)
            sbc = self._sbc if vitals else self._cached_sbc_temp()
            if sbc == sbc:                       # not NaN
                s = f"{sbc:.0f}C"
                draw.text((W - 2 - self._text_w(s), 14), s, font=self.font, fill=255)

            draw.line((0, 25, W - 1, 25), fill=255)

            # Subsystem status rows (left column).
            self._row(draw, 28, "ESP", esp_up,
                      f"{self.esp_temp:.0f}C" if esp_up and self.esp_temp == self.esp_temp else "off")
            self._row(draw, 40, "IMU", imu_up, f"{self.imu_hz:.0f}Hz" if imu_up else "off")
            self._row(draw, 52, "LDS", lds_up, f"{self.lds_hz:.1f}Hz" if lds_up else "off")

            # SBC vitals (right column, below the temp): CPU busy % + RAM used %.
            # Right-aligned with a 3px margin and an x>=0 clamp so a wide value
            # (e.g. "CPU 100%") can never run off the right edge.
            cpu_pct, mem_pct = ((self._cpu_pct, self._mem_pct) if vitals
                                else self._cached_sys())
            for y, val in ((28, cpu_pct), (40, mem_pct)):
                if val != val:               # NaN -> not ready yet
                    continue
                s = f"{'CPU' if y == 28 else 'RAM'} {val:.0f}%"
                draw.text((max(0, W - 3 - self._text_w(s)), y), s, font=self.font, fill=255)

            # IMU tilt (right column, row 52 — beside the LDS row): roll/pitch in degrees.
            # Right-aligned like the vitals above; only shown while the IMU is alive. No
            # degree glyph (the default font lacks it — matches the "47C" temp style).
            if imu_up:
                tilt = f"R{self.imu_roll:+.0f} P{self.imu_pitch:+.0f}"
                draw.text((max(0, W - 3 - self._text_w(tilt)), 52), tilt, font=self.font, fill=255)

    # ---- TTS karaoke (one word, big + centred) ----
    def _draw_word(self, word):
        """Render a single word as large as it'll fit, centred on the panel. The
        default luma font is tiny, so we draw the word once at 1x then nearest-scale
        it up (integer, mode '1') — no TTF file needed, so it costs no extra disk.

        The crop box comes from the RENDERED image's own `.getbbox()` (actual ink),
        not `font.getbbox(word)` (predicted metrics) — the two can disagree for a
        small bitmap font (hinting/bearing quirks), which previously understated a
        word's true width and clipped one side off the panel. Drawing onto a
        generously oversized scratch canvas first means the true ink is never
        clipped before we even get to measuring it."""
        if not self.device:
            return
        W, H = self.width, self.height
        pad, scratch_w, scratch_h = 8, max(W * 4, 256), max(H * 2, 32)
        scratch = Image.new("1", (scratch_w, scratch_h), 0)
        ImageDraw.Draw(scratch).text((pad, pad), word, font=self.font, fill=1)
        bbox = scratch.getbbox()
        if bbox is None:                       # blank/whitespace word: nothing to show
            return
        glyph = scratch.crop(bbox)
        tw, th = glyph.size
        # Fit to the panel both ways: upscale (integer, capped so short words don't
        # pixelate into illegibility) when there's room, downscale when the word is
        # already wider/taller than the panel at 1x (rather than clip it).
        fit = min((W - 4) / tw, (H - 8) / th)
        if fit >= 1:
            scale = min(int(fit), 6)
            if scale > 1:
                glyph = glyph.resize((tw * scale, th * scale), Image.NEAREST)
        else:
            glyph = glyph.resize((max(1, int(tw * fit)), max(1, int(th * fit))), Image.NEAREST)
        gw, gh = glyph.size
        with canvas(self.device) as draw:
            draw.bitmap((max(0, (W - gw) // 2), max(0, (H - gh) // 2)), glyph, fill=255)

    # ---- face / mood animation ----
    def _anim_update(self, now):
        """Advance blink + gaze by real elapsed time (frame-rate independent)."""
        dt = now - self._anim_t
        self._anim_t = now

        # Blink: openness follows a 1->0->1 triangle over BLINK_DUR.
        if self._blink_t is not None:
            self._blink_t += dt
            p = self._blink_t / BLINK_DUR
            if p >= 1.0:
                self._blink_t = None
                self._open = 1.0
                if self._double:             # occasional cute double-blink
                    self._double = False
                    self._next_blink = now + 0.18
                elif self._has("focused"):   # an intent (focused) stare blinks rarely
                    self._next_blink = now + random.uniform(5.0, 9.0)
                    self._double = False
                else:
                    self._next_blink = now + random.uniform(2.0, 5.0)
                    self._double = random.random() < 0.3
            else:
                self._open = abs(1.0 - 2.0 * p)
        elif now >= self._next_blink:
            self._blink_t = 0.0
            self._open = 1.0
        else:
            self._open = 1.0

        # Gaze: pick a new target now and then (biased to centre), ease toward it.
        # All moods use this to dart the *pupils* within fixed eye-whites (see
        # _face_tick); it settles between glances so the dirty-check keeps idle
        # frames free. "focused" retargets fastest (active scanning).
        if now >= self._next_gaze:
            if random.random() < 0.35:
                self._gtx, self._gty = 0.0, 0.0
            else:
                self._gtx = random.uniform(-3.0, 3.0)
                self._gty = random.uniform(-3.0, 3.0)
            # "focused" + "looking" both scan actively, so they retarget the gaze fastest —
            # whether it's the base shape or the emotion accent.
            lo, hi = (0.6, 1.8) if (self._has("focused") or self._has("looking")) else (1.0, 3.0)
            self._next_gaze = now + random.uniform(lo, hi)

        # Happy (base OR accent): a recurring crescent "smile" beat keeps it lively between glances.
        if self._has("happy") and now >= self._next_smile:
            self._smile_until = now + 0.45
            self._next_smile = now + random.uniform(3.0, 7.0)
        k = min(1.0, dt * 6.0)
        self._gxf += (self._gtx - self._gxf) * k
        self._gyf += (self._gty - self._gyf) * k
        if abs(self._gtx - self._gxf) < 0.4:
            self._gxf = self._gtx
        if abs(self._gty - self._gyf) < 0.4:
            self._gyf = self._gty
        self._gx = int(round(self._gxf))
        self._gy = int(round(self._gyf))

    def _pupil(self, draw, cx, cy, eh, px, py, pw, ph, sparkle=False):
        """Dark pupil that darts to offset (px, py), clamped to stay inside the
        eye-white (half-extents EYE_W x eh). Optional white glint for cuteness."""
        mx, my = max(0, EYE_W - pw - 2), max(0, eh - ph - 2)
        pcx = cx + max(-mx, min(mx, px))
        pcy = cy + max(-my, min(my, py))
        draw.ellipse((pcx - pw, pcy - ph, pcx + pw, pcy + ph), fill=0)
        if sparkle:
            draw.ellipse((pcx - pw + 1, pcy - ph + 1, pcx - pw + 4, pcy - ph + 4), fill=255)

    def _smile_eye(self, draw, cx, cy):
        """The happy upward crescent (^_^) — a white blob with a black bite from below. Used
        for the native happy face AND as the happy *accent* (folds any shape into a smile on a
        blink / the recurring smile beat), so the two stay pixel-identical."""
        top, bot = cy - 16, cy + 10
        draw.ellipse((cx - EYE_W, top, cx + EYE_W, bot), fill=255)
        draw.ellipse((cx - EYE_W, top + 8, cx + EYE_W, bot + 8), fill=0)  # upward crescent

    def _happy_eye(self, draw, cx, cy, px, py, smiling):
        """Happy eye: big round white + a sparkly pupil that looks around; folds to
        an upward crescent (^_^) on the recurring smile beat or a blink."""
        if smiling or self._open < 0.3:
            self._smile_eye(draw, cx, cy)
            return
        eh = max(2, int(EYE_H * self._open))
        draw.ellipse((cx - EYE_W, cy - eh, cx + EYE_W, cy + eh), fill=255)
        self._pupil(draw, cx, cy, eh, px, py, 7, 9, sparkle=True)

    def _brow(self, draw, cx, cy, inner, eh):
        """A slanted scowl brow cut deepest on the *inner* side (toward the nose) for the
        classic \\ / look. Drawn last so it sits over the pupil. Used by the native angry face
        AND as the angry *accent* over any shape. `inner` is +1 for the left eye, -1 for right."""
        x_in = cx + inner * (EYE_W + 2)        # inner edge (deep cut)
        x_out = cx - inner * (EYE_W + 2)        # outer edge (shallow cut)
        brow = int(eh * 1.1)
        draw.polygon([(x_out, cy - eh - 3), (x_in, cy - eh - 3),
                      (x_in, cy - eh + brow), (x_out, cy - eh + 3)], fill=0)

    def _droop(self, draw, cx, cy):
        """A heavy upper eyelid (black wedge from the top) — the sleepy *accent* over any shape:
        leaves only a lower sliver of the eye, so the robot looks drowsy without going fully
        closed like the standalone `sleepy` mood."""
        eh = max(2, int(EYE_H * self._open))
        draw.rectangle((cx - EYE_W - 1, cy - eh - 1, cx + EYE_W + 1, cy + int(eh * 0.35)), fill=0)

    def _angry_eye(self, draw, cx, cy, inner, px, py):
        """Angry eye: a tall white with a darting pupil and a slanted brow (see _brow)."""
        eh = max(2, int(EYE_H * self._open))
        if self._open < 0.25:
            draw.line((cx - EYE_W, cy, cx + EYE_W, cy), fill=255)
            return
        draw.ellipse((cx - EYE_W, cy - eh, cx + EYE_W, cy + eh), fill=255)
        self._pupil(draw, cx, cy, eh, px, py, 6, 7)       # harsh, no glint
        self._brow(draw, cx, cy, inner, eh)               # drawn last -> brow over the pupil

    def _accent_overlay(self, draw, lcx, rcx, cy):
        """Draw the emotion accent ON TOP of an already-drawn base shape (the "meta" layer):
        angry -> scowl brows, sleepy -> heavy lids. The happy accent is handled earlier (it folds
        the whole eye to a smile crescent on a blink / smile beat, see _face_tick). focused /
        looking / neutral accents contribute cadence only (faster gaze / rare blink, in
        _anim_update) and need no overlay. Skipped when the accent equals the base shape."""
        a = self._accent
        if not a or a == self._mood:
            return
        if a == "angry":
            eh = max(2, int(EYE_H * self._open))
            self._brow(draw, lcx, cy, +1, eh)
            self._brow(draw, rcx, cy, -1, eh)
        elif a == "sleepy":
            self._droop(draw, lcx, cy)
            self._droop(draw, rcx, cy)

    def _focused_eye(self, draw, cx, cy, px, py):
        """Focused eye: a narrowed (squinted) white with a dark pupil that darts
        around inside it for an intent, scanning look."""
        if self._open < 0.25:
            draw.line((cx - EYE_W, cy, cx + EYE_W, cy), fill=255)
            return
        eh = max(3, int(EYE_H * 0.6 * self._open))
        draw.ellipse((cx - EYE_W, cy - eh, cx + EYE_W, cy + eh), fill=255)
        self._pupil(draw, cx, cy, eh, px, py, 5, max(2, eh - 3))

    def _looking_eye(self, draw, cx, cy, px, py):
        """Looking / 'peeking' eye: a wide, alert round white with a dark pupil that darts
        actively to the sides — the searching, taking-a-look expression shown while the camera
        is in use. Distinct from `focused` (which squints) — here the eyes are wide open."""
        if self._open < 0.25:
            draw.line((cx - EYE_W, cy, cx + EYE_W, cy), fill=255)
            return
        eh = max(2, int(EYE_H * self._open))
        draw.ellipse((cx - EYE_W, cy - eh, cx + EYE_W, cy + eh), fill=255)
        self._pupil(draw, cx, cy, eh, px, py, 6, 8)

    def _sleepy(self, draw, zc):
        """Sleepy / 'AI offline' face: two closed, content eyes (downward arcs) and a slow
        rising 'z z z' in the corner. Drawn at near-zero cost (dirty-checked, ~1 fps)."""
        W, H = self.width, self.height
        cy = H // 2 + 4
        for cx in (W // 2 - EYE_DX, W // 2 + EYE_DX):
            for dy in (0, 1, 2):                     # thicken the closed-eye arc
                draw.arc((cx - EYE_W, cy - 8 + dy, cx + EYE_W, cy + 8 + dy), 20, 160, fill=255)
        zx, zy = W - 30, 18                          # rising z z z, top-right
        for i in range(zc):
            draw.text((zx + i * 8, zy - i * 6), "z", font=self.font, fill=255)

    def _reflecting_glyph(self, draw, ang):
        """Reflection-mode screen: a slowly rotating open ring (same arc-glyph
        language as _shutdown_screen/_restart_screen, but animated to signal ongoing
        background work rather than a one-shot terminal state) + a static label."""
        W, H = self.width, self.height
        cx, gy, r = W // 2, H // 2 - 12, 8
        draw.arc((cx - r, gy - r, cx + r, gy + r), ang, ang + 260, fill=255)
        msg = "Reflecting"
        draw.text(((W - self._text_w(msg)) // 2, gy + r + 4), msg, font=self.font, fill=255)

    def _stress(self, draw):
        """Worst-case animation: a full-screen pattern that changes every single
        frame, so the dirty-check never skips and we flush at the bus ceiling.
        Used only to measure the maximum cost of the OLED path."""
        W, H = self.width, self.height
        ph = self._frame
        o = ph % 8
        for x in range(-H, W, 8):                  # scrolling diagonal stripes
            draw.line((x + o, 0, x + o + H, H), fill=255)
        bx = int(W / 2 + (W / 2 - 9) * math.sin(ph * 0.5))   # bouncing ball
        by = int(H / 2 + (H / 2 - 9) * math.cos(ph * 0.33))
        draw.ellipse((bx - 9, by - 9, bx + 9, by + 9), fill=0)
        draw.ellipse((bx - 9, by - 9, bx + 9, by + 9), outline=255)

    def _face_tick(self):
        if not self.device or not self._mood or self.speak_word or self._sys:
            return
        now = time.monotonic()
        self._anim_update(now)
        self._frame += 1

        W, H = self.width, self.height
        if self._mood == "stress":
            # No dirty-check: force a flush every frame (max load).
            self._face_sig = ("stress", self._frame)
            with canvas(self.device) as draw:
                self._stress(draw)
            return

        if self._mood == "sleepy":
            # Mostly static (cheap) closed eyes + a slowly cycling "z z z". Shown while the
            # LLM brain is unreachable, so it can sit up for a long time at ~no cost.
            zc = int(now * 1.2) % 3 + 1              # 1..3 z's, ~1.2 Hz cycle
            sig = ("sleepy", zc)
            if sig == self._face_sig:
                return
            self._face_sig = sig
            with canvas(self.device) as draw:
                self._sleepy(draw, zc)
            return

        if self._mood == "mask":
            self._mask_tick()
            return

        if self._mood == "reflecting":
            # A slow spinner (not an eye shape at all -- reflection is a sustained
            # background-processing state, not a reactive expression) + a static
            # label, same visual language as _shutdown_screen/_restart_screen.
            ang = int(now * 90) % 360                # one full turn every 4s
            sig = ("reflecting", ang // 6)            # coarse bucket: redraw ~60x/turn, not 60fps
            if sig == self._face_sig:
                return
            self._face_sig = sig
            with canvas(self.device) as draw:
                self._reflecting_glyph(draw, ang)
            return

        # All moods: eye-whites stay fixed at the base position and the gaze drives
        # the *pupils* (amplified, 1px steps off the eased float so they slide
        # smoothly). Whites never move, so nothing can clip at the sides.
        cy = H // 2
        lcx, rcx = W // 2 - EYE_DX, W // 2 + EYE_DX
        px = int(round(self._gxf * 3.0))
        py = int(round(self._gyf * 2.5))
        openq = int(round(self._open * 8))
        smiling = self._has("happy") and now < self._smile_until

        # Dirty-check: only redraw/flush when the picture actually changes (accent included).
        sig = (self._mood, self._accent, openq, px, py, smiling)
        if sig == self._face_sig:
            return
        self._face_sig = sig

        with canvas(self.device) as draw:
            # Happy (base OR accent) folds the whole eye into a smile crescent on the smile beat
            # or a blink — so "looking:happy" periodically beams, not just the native happy face.
            if self._has("happy") and (smiling or self._open < 0.3):
                self._smile_eye(draw, lcx, cy)
                self._smile_eye(draw, rcx, cy)
                return
            if self._mood == "angry":
                self._angry_eye(draw, lcx, cy, +1, px, py)
                self._angry_eye(draw, rcx, cy, -1, px, py)
            elif self._mood == "focused":
                self._focused_eye(draw, lcx, cy, px, py)
                self._focused_eye(draw, rcx, cy, px, py)
            elif self._mood == "looking":
                self._looking_eye(draw, lcx, cy, px, py)
                self._looking_eye(draw, rcx, cy, px, py)
            else:                                  # happy / neutral (calm open eyes)
                self._happy_eye(draw, lcx, cy, px, py, smiling)
                self._happy_eye(draw, rcx, cy, px, py, smiling)
            self._accent_overlay(draw, lcx, rcx, cy)   # emotion overlay on top of the shape

    # ---- GPU tracking-mask mirror ----
    def _read_mask_blob(self):
        """One JSON header line + w*h raw bytes (0/255) — written atomically by
        gpu_vision.py, so a plain read never sees a torn frame. (None, None) on any
        problem (absent, malformed, short read)."""
        try:
            with open(MASK_FILE, "rb") as f:
                header = json.loads(f.readline().decode())
                w, h = int(header["w"]), int(header["h"])
                raw = f.read(w * h)
            if len(raw) != w * h:
                return None, None
            return header, raw
        except Exception:
            return None, None

    def _mask_tick(self):
        """Render the tracking-mask blob (white = pixels matching the calibrated
        colour). The blob's seq is the dirty-check — the panel flushes only when the
        GPU actually produced a new mask frame, so a paused tracker costs nothing. A
        stale/absent blob (no target colour set, capture stopped) shows a static
        placeholder instead of a frozen mask masquerading as live."""
        header, raw = self._read_mask_blob()
        if header is None or (time.time() - float(header.get("t", 0))) > MASK_FRESH_S:
            sig = ("mask", "stale")
            if sig == self._face_sig:
                return
            self._face_sig = sig
            with canvas(self.device) as draw:
                msg = "mask: waiting..."
                draw.text(((self.width - self._text_w(msg)) // 2, self.height // 2 - 4),
                          msg, font=self.font, fill=255)
            return
        sig = ("mask", header.get("seq"))
        if sig == self._face_sig:
            return
        self._face_sig = sig
        w, h = int(header["w"]), int(header["h"])
        img = Image.frombytes("L", (w, h), raw)
        if (w, h) != (self.width, self.height):
            img = img.resize((self.width, self.height))
        self.device.display(img.convert("1"))

    # ---- shutdown ----
    def _shutdown_screen(self):
        """A centred 'Shutting down' screen with a power glyph (shown on stop)."""
        if not self.device:
            return
        W, H = self.width, self.height
        with canvas(self.device) as draw:
            cx, gy, r = W // 2, H // 2 - 12, 8
            draw.arc((cx - r, gy - r, cx + r, gy + r), 300, 240, fill=255)  # ring, gap at top
            draw.line((cx, gy - r - 1, cx, gy + 2), fill=255)               # power bar
            msg = "Shutting down"
            draw.text(((W - self._text_w(msg)) // 2, gy + r + 4), msg, font=self.font, fill=255)

    def _restart_screen(self, msg="Restarting"):
        """A centred circular-arrow glyph screen. Used for both a stack restart
        ('Restarting stack') and a full SBC reboot ('Restarting'). The panel is left
        on — for a stack restart the relaunched node redraws over it; for a reboot it
        simply holds until the board power-cycles."""
        if not self.device:
            return
        W, H = self.width, self.height
        cx, gy, r = W // 2, H // 2 - 12, 8
        with canvas(self.device) as draw:
            draw.arc((cx - r, gy - r, cx + r, gy + r), 70, 360, fill=255)   # ~open ring
            ax = cx + int(r * math.cos(math.radians(70)))                   # arrowhead at the gap
            ay = gy + int(r * math.sin(math.radians(70)))
            draw.polygon([(ax - 3, ay - 1), (ax + 3, ay - 1), (ax, ay + 4)], fill=255)
            draw.text(((W - self._text_w(msg)) // 2, gy + r + 4), msg, font=self.font, fill=255)

    def _power_off_panel(self):
        """Blank the framebuffer and turn the SSD1306 off (display-off, 0xAE) so the
        panel goes dark instead of holding its last frame after the board halts."""
        if not self.device or self._off:
            return
        self._off = True
        try:
            self.device.clear()
            if hasattr(self.device, "hide"):
                self.device.hide()
        except Exception:
            pass

    def shutdown_sequence(self):
        """On stop (SIGTERM from stack.sh down / systemd poweroff, or Ctrl-C): show an
        end-screen. For a restart, leave "Restarting" up (no power-off, so no race with
        the incoming node); otherwise show "Shutting down" and turn the panel off."""
        if not self.device or self._off:
            return
        try:
            self._face_timer.cancel()       # stop the animation owning the panel
        except Exception:
            pass
        # Prefer the live signal from the web UI (/oled_system); fall back to the hint
        # file (e.g. CLI stop with no topic). Default to a safe shutdown.
        action = self._sys
        if not action:
            try:
                with open(ACTION_FILE) as f:
                    action = f.read().strip()
            except Exception:
                pass
        try:
            os.remove(ACTION_FILE)
        except Exception:
            pass
        if action in ("restart", "reboot"):
            # leave the glyph up (no power-off): a stack restart's relaunched node
            # redraws over it; a reboot holds it until the board power-cycles.
            self._restart_screen("Restarting stack" if action == "restart" else "Restarting")
        else:
            self._shutdown_screen()         # idempotent; already up if the topic arrived
            time.sleep(1.2)                 # leave it readable during the rest of shutdown
            self._power_off_panel()

    def destroy_node(self):
        # If we didn't already power the panel off (e.g. an unclean stop), blank it.
        if self.device and not self._off:
            try:
                self.device.clear()
            except Exception:
                pass
        super().destroy_node()


def main():
    rclpy.init()
    node = DisplayNode()
    # SIGTERM is how stack.sh down / systemd poweroff stop us; trip a flag and let the
    # main thread run the shutdown sequence (drawing from a signal handler could re-enter
    # a mid-flight I2C flush).
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    try:
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_sequence()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
