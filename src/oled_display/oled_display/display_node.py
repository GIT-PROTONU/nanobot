"""Status OLED for the SSD1306 (I2C) via luma.oled. Two display modes:

DASHBOARD (default) — a compact status panel:

    ┌───────────────────────────────┐
    │ NANOBOT              12:34:56  │  header bar (inverted): brand + clock
    │ 192.168.178.141          47C   │  primary IP  +  SBC CPU temp (right)
    │ ───────────────────────────── │
    │ ESP  ●  39C                    │  ● filled = online / ○ hollow = offline
    │ IMU  ●  50Hz                   │  data rate when running, else "off"
    │ LDS  ○  off                    │
    └───────────────────────────────┘

FACE — cute animated robot eyes that blink and glance around, used to express a
mood. Enabled from the web UI by publishing a mood name on /oled_face (empty
string returns to the dashboard). Moods: "happy" (upward ^_^ crescents), "angry"
(slanted scowling brows) and "focused" (narrowed eyes with a staring pupil);
more can be added later by extending _face_tick.

Efficiency (the face must be near-free on a 1 GB / quad-A53 board):
  * The face has its own timer that is **cancelled** while in dashboard mode, so
    it adds zero executor wakeups unless you're actually animating.
  * Even while animating it only pushes a frame to the panel when the rendered
    picture changes (a small "signature" dirty-check) — open, settled eyes draw
    nothing, so I2C traffic happens only during a blink or a glance.
  * Blink/glance timing is randomized, so the animation never looks like it is
    replaying a fixed loop.

Data sources (all lightweight — no heavy message is deserialized per frame):
    /esp32_heartbeat (Int32)          → ESP32 liveness
    /esp32_temp      (Float32)        → ESP32 temperature
    /imu/web   (Vector3Stamped) z=measured /imu/data Hz → IMU liveness + rate
    /lds_hz          (Float32)        → LDS valid-frame rate (0/stale = offline)
    /oled_text       (String)         → optional brand override (web UI textbox)
    /oled_face       (String)         → mood name / "" for dashboard (web UI)
    /sys/class/thermal/thermal_zone0/temp → SBC CPU temp (cached file read)

Subscribe-only and best-effort: if the panel or luma isn't present the node still
runs and just logs once, so the rest of the stack is unaffected.
"""
import math
import random
import socket
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, String
from geometry_msgs.msg import Vector3Stamped

try:
    from luma.core.interface.serial import i2c
    from luma.core.render import canvas
    from luma.oled.device import ssd1306
    from PIL import ImageFont
    HAVE_LUMA = True
except Exception as exc:  # pragma: no cover - hardware lib
    HAVE_LUMA = False
    _LUMA_ERR = exc

IP_REFRESH_S = 30.0    # re-resolve the outbound IP at most this often
TEMP_REFRESH_S = 2.0   # re-read the SBC thermal zone at most this often
THERMAL_PATH = "/sys/class/thermal/thermal_zone0/temp"  # cpu-thermal (millidegrees)

# Staleness windows: a source is "alive" only if it arrived within this many
# seconds. Generous enough to ride out one missed message at each topic's rate.
ESP_TIMEOUT_S = 4.0    # /esp32_heartbeat ~1 Hz
IMU_TIMEOUT_S = 1.5    # /imu/web ~15 Hz
LDS_TIMEOUT_S = 3.0    # /lds_hz a few Hz

BLINK_DUR = 0.16       # seconds for one eyelid close+open
KNOWN_MOODS = ("happy", "angry", "focused", "stress")

# Eye geometry, sized to fill a 128x64 panel. The inner gap is 2*(EYE_DX-EYE_W);
# EYE_DX=37 / EYE_W=20 → a 34 px gap between the eyes with ~7 px edge margins, so
# a glance (gaze offset) never pushes an eye off the side of the screen.
EYE_DX = 37            # eye centre offset from screen centre (px)
EYE_W = 20             # eye half-width (px)
EYE_H = 26             # eye half-height when fully open (px)
EYE_PAD = 2            # widest decoration overhang past EYE_W (the angry brow)


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
        super().__init__("oled_display")
        self.declare_parameters("", [
            ("i2c_bus", 1), ("i2c_address", 0x3C),
            ("width", 128), ("height", 64),
            ("refresh_rate", 2.0), ("show_ip", True),
            ("anim_fps", 20.0),    # face animation tick rate (only ticks while in face mode;
                                   # safely under the ~26 fps i2c-0@400kHz flush ceiling)
            ("rotate", 0),         # screen rotation in 90° steps; 2 = 180° (upside-down mount)
        ])
        g = self.get_parameter
        self.show_ip = g("show_ip").value
        self.width = g("width").value
        self.height = g("height").value

        # Latest telemetry — written by callbacks, read by the render timer. The
        # *_at fields are monotonic arrival times used to decide liveness.
        self.text = ""                  # optional brand override from the web UI
        self.face_mood = ""             # "" = dashboard; else a mood name
        self.esp_temp = float("nan")
        self.esp_at = -1e9
        self.imu_hz = 0.0
        self.imu_at = -1e9
        self.lds_hz = 0.0
        self.lds_at = -1e9

        # Cache host/IP + SBC temp so the render loop never blocks per frame.
        self._ip = "0.0.0.0"
        self._ip_due = 0.0
        self._sbc = float("nan")
        self._sbc_due = 0.0

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

        self.device = None
        self.font = None
        if HAVE_LUMA:
            try:
                serial = i2c(port=g("i2c_bus").value, address=g("i2c_address").value)
                self.device = ssd1306(serial, width=self.width, height=self.height,
                                      rotate=int(g("rotate").value) % 4)
                self.font = ImageFont.load_default()
                self.get_logger().info(
                    f"SSD1306 ready on /dev/i2c-{g('i2c_bus').value} "
                    f"@0x{g('i2c_address').value:02x}")
            except Exception as exc:
                self.get_logger().error(f"OLED init failed: {exc}")
        else:
            self.get_logger().error(f"luma.oled unavailable: {_LUMA_ERR}")

        self.create_subscription(Int32, "esp32_heartbeat", self._on_esp_beat, 10)
        self.create_subscription(Float32, "esp32_temp", self._on_esp_temp, 10)
        self.create_subscription(Vector3Stamped, "imu/web", self._on_imu_web, 10)
        self.create_subscription(Float32, "lds_hz", self._on_lds_hz, 10)
        self.create_subscription(String, "oled_text", self._on_text, 10)
        self.create_subscription(String, "oled_face", self._on_face, 10)

        # Dashboard ticks slowly; the face has its own faster timer that stays
        # cancelled (no wakeups) until a mood is selected.
        self.create_timer(1.0 / g("refresh_rate").value, self._dashboard_tick)
        self._face_timer = self.create_timer(1.0 / max(1.0, g("anim_fps").value),
                                             self._face_tick)
        self._face_timer.cancel()

    # ---- subscriptions (record value + arrival time only) ----
    def _on_text(self, msg: String):
        self.text = msg.data
        self.get_logger().info(f"OLED brand set to {msg.data!r}")

    def _on_face(self, msg: String):
        mood = msg.data.strip().lower()
        if mood in ("", "off", "dashboard", "none"):
            new = ""
        else:
            new = mood if mood in KNOWN_MOODS else "neutral"
        if new == self.face_mood:
            return
        was_face = bool(self.face_mood)
        self.face_mood = new
        if new and not was_face:          # entering face mode: re-seed animation
            now = time.monotonic()
            self._anim_t = now
            self._open = 1.0
            self._blink_t = None
            self._double = False
            self._next_blink = now + random.uniform(1.5, 3.5)
            self._gxf = self._gyf = self._gtx = self._gty = 0.0
            self._gx = self._gy = 0
            self._next_gaze = now + random.uniform(1.8, 4.5)
            self._face_sig = None
            self._face_timer.reset()
        elif not new and was_face:        # back to dashboard: stop the face timer
            self._face_timer.cancel()
        self.get_logger().info(f"OLED mode: {self.face_mood or 'dashboard'}")

    def _on_esp_beat(self, _msg: Int32):
        self.esp_at = time.monotonic()

    def _on_esp_temp(self, msg: Float32):
        self.esp_temp = msg.data
        self.esp_at = time.monotonic()      # temp arriving also proves the ESP is alive

    def _on_imu_web(self, msg: Vector3Stamped):
        self.imu_hz = msg.vector.z          # z = measured /imu/data publish rate
        self.imu_at = time.monotonic()

    def _on_lds_hz(self, msg: Float32):
        self.lds_hz = msg.data
        self.lds_at = time.monotonic()

    # ---- cached, non-blocking reads for the render loop ----
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
        if not self.device or self.face_mood:    # face mode owns the screen
            return
        now = time.monotonic()
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
            sbc = self._cached_sbc_temp()
            if sbc == sbc:                       # not NaN
                s = f"{sbc:.0f}C"
                draw.text((W - 2 - self._text_w(s), 14), s, font=self.font, fill=255)

            draw.line((0, 25, W - 1, 25), fill=255)

            # Subsystem status rows.
            self._row(draw, 28, "ESP", esp_up,
                      f"{self.esp_temp:.0f}C" if esp_up and self.esp_temp == self.esp_temp else "off")
            self._row(draw, 40, "IMU", imu_up, f"{self.imu_hz:.0f}Hz" if imu_up else "off")
            self._row(draw, 52, "LDS", lds_up, f"{self.lds_hz:.1f}Hz" if lds_up else "off")

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
                elif self.face_mood == "focused":   # an intent stare blinks rarely
                    self._next_blink = now + random.uniform(5.0, 9.0)
                    self._double = False
                else:
                    self._next_blink = now + random.uniform(2.5, 6.0)
                    self._double = random.random() < 0.25
            else:
                self._open = abs(1.0 - 2.0 * p)
        elif now >= self._next_blink:
            self._blink_t = 0.0
            self._open = 1.0
        else:
            self._open = 1.0

        # Gaze: pick a new target now and then (biased to centre), ease toward it.
        # "focused" stares — its glances are smaller and rarer than other moods.
        if now >= self._next_gaze:
            amp = 0.3 if self.face_mood == "focused" else 1.0
            if random.random() < (0.7 if self.face_mood == "focused" else 0.45):
                self._gtx, self._gty = 0.0, 0.0
            else:
                self._gtx = random.uniform(-3.0, 3.0) * amp
                self._gty = random.uniform(-3.0, 3.0) * amp
            self._next_gaze = now + random.uniform(1.8, 4.5)
        k = min(1.0, dt * 6.0)
        self._gxf += (self._gtx - self._gxf) * k
        self._gyf += (self._gty - self._gyf) * k
        if abs(self._gtx - self._gxf) < 0.4:
            self._gxf = self._gtx
        if abs(self._gty - self._gyf) < 0.4:
            self._gyf = self._gty
        self._gx = int(round(self._gxf))
        self._gy = int(round(self._gyf))

    def _happy_eye(self, draw, cx, cy):
        """Happy eye: an upward crescent (^_^); a blink flattens it to a line."""
        if self._open < 0.3:
            draw.line((cx - EYE_W, cy, cx + EYE_W, cy), fill=255)
            return
        top, bot = cy - 16, cy + 10
        draw.ellipse((cx - EYE_W, top, cx + EYE_W, bot), fill=255)
        draw.ellipse((cx - EYE_W, top + 8, cx + EYE_W, bot + 8), fill=0)  # cut -> upward crescent

    def _angry_eye(self, draw, cx, cy, inner):
        """Angry eye: a tall oval with a slanted brow cut deepest on the *inner*
        side (toward the nose), giving the classic \\ / scowl. `inner` is +1 for
        the left eye (inner edge to the right), -1 for the right eye."""
        eh = max(2, int(EYE_H * self._open))
        if self._open < 0.25:
            draw.line((cx - EYE_W, cy, cx + EYE_W, cy), fill=255)
            return
        draw.ellipse((cx - EYE_W, cy - eh, cx + EYE_W, cy + eh), fill=255)
        x_in = cx + inner * (EYE_W + 2)       # inner edge (deep cut)
        x_out = cx - inner * (EYE_W + 2)       # outer edge (shallow cut)
        brow = int(eh * 1.1)
        draw.polygon([(x_out, cy - eh - 3), (x_in, cy - eh - 3),
                      (x_in, cy - eh + brow), (x_out, cy - eh + 3)], fill=0)

    def _focused_eye(self, draw, cx, cy):
        """Focused eye: a narrowed (squinted) oval with a dark central pupil for
        an intent stare."""
        if self._open < 0.25:
            draw.line((cx - EYE_W, cy, cx + EYE_W, cy), fill=255)
            return
        eh = max(3, int(EYE_H * 0.6 * self._open))
        draw.ellipse((cx - EYE_W, cy - eh, cx + EYE_W, cy + eh), fill=255)
        draw.ellipse((cx - 5, cy - eh + 3, cx + 5, cy + eh - 3), fill=0)   # pupil

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
        if not self.device or not self.face_mood:
            return
        now = time.monotonic()
        self._anim_update(now)
        self._frame += 1

        W, H = self.width, self.height
        if self.face_mood == "stress":
            # No dirty-check: force a flush every frame (max load).
            self._face_sig = ("stress", self._frame)
            with canvas(self.device) as draw:
                self._stress(draw)
            return

        # Dirty-check: only redraw/flush when the picture actually changes, so
        # open, settled eyes cost nothing.
        sig = (self.face_mood, int(round(self._open * 8)), self._gx, self._gy)
        if sig == self._face_sig:
            return
        self._face_sig = sig

        cy = H // 2 + self._gy
        # Clamp the horizontal glance so the outermost eye pixel (incl. the brow
        # overhang) can never fall outside [0, W-1] — no clipping at the sides.
        gx_lim = (W - 1 - W // 2 - EYE_DX) - EYE_W - EYE_PAD
        gx = max(-gx_lim, min(gx_lim, self._gx))
        lcx, rcx = W // 2 - EYE_DX + gx, W // 2 + EYE_DX + gx
        with canvas(self.device) as draw:
            if self.face_mood == "angry":
                self._angry_eye(draw, lcx, cy, +1)
                self._angry_eye(draw, rcx, cy, -1)
            elif self.face_mood == "focused":
                self._focused_eye(draw, lcx, cy)
                self._focused_eye(draw, rcx, cy)
            else:                                  # happy
                self._happy_eye(draw, lcx, cy)
                self._happy_eye(draw, rcx, cy)

    def destroy_node(self):
        if self.device:
            try:
                self.device.clear()
            except Exception:
                pass
        super().destroy_node()


def main():
    rclpy.init()
    node = DisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
