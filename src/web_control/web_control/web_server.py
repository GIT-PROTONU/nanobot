"""Tiny static-file HTTP server for the control page, plus an MJPEG webcam stream.

Kept as a ROS node so it starts/stops with the rest of the launch and shows up
in `ros2 node list`. It serves the package's installed `web/` directory (which
contains index.html). The page talks to ROS over the rosbridge websocket, not to
this server — this only delivers the HTML/JS and the camera stream.

`/stream.mjpg` serves the USB webcam as multipart/x-mixed-replace, fed by a
zero-dependency V4L2 MJPEG passthrough (see mjpeg_camera). `/audio.pcm` streams
the webcam mic as raw PCM via arecord (see mic_audio). Both the camera and the
mic are started only while a client is connected, so they cost nothing idle.

`POST /tts` ({"voice","text"}) speaks a line via SVOX Pico (see tts) and streams
its words to the OLED on /oled_word so they appear/disappear as they're said.
"""
import functools
import http.server
import json
import os
import subprocess
import threading
import time
from collections import deque

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, Int32, String
from geometry_msgs.msg import Vector3Stamped

from .mjpeg_camera import CameraStream
from .mic_audio import AudioStream
from .tts import TtsEngine, VOICES, clamp
from .llm import LlmClient, MOODS

# Persisted, web-tunable TTS settings (merged over the file on disk). `voice` is
# seeded from the tts_default_voice param at load time.
SETTINGS_DEFAULTS = {
    "voice": "en-US",
    "volume": 100,            # Pico level %, 100 = normal
    "speed": 100,
    "pitch": 100,
    "announce": False,        # speak CPU/RAM/temp every `announce_interval` s
    "announce_interval": 30,  # seconds (clamped to >= ANNOUNCE_MIN)
}
ANNOUNCE_MIN = 5              # don't let the announcer spam faster than this (s)
ANNOUNCE_MAX = 3600

# Web-tunable LLM (OpenRouter) settings, merged over the file on disk and seeded from
# the robot.yaml params. The API key is NEVER stored here — it stays in robot.yaml /
# the OPENROUTER_API_KEY env var — so this file (and any backup of it) holds no secret.
LLM_DEFAULTS = {
    "enabled": False,         # opt-in: it costs money + needs the network
    "model": "",              # "" -> llm.DEFAULT_MODEL
}
# NOTE: the persona is NOT here — it's single-sourced from personality.json (written by
# scripts/personality_creator.py, the same file the behaviour node loads), falling back to
# the llm_persona param. So one run of the creator + a restart updates the whole character.
LLM_HISTORY_MAX = 8          # chat turns kept for context (user+assistant messages)
LLM_LOG_MAX = 50             # decision-log ring buffer length (also what the file tail loads)
REFLECT_TRAITS = ("curiosity", "extraversion", "caution", "playfulness")  # personality axes

# SBC vitals for the spoken stats (same cheap /proc + thermal reads the OLED uses).
THERMAL_PATH = "/sys/class/thermal/thermal_zone0/temp"
STAT_PATH = "/proc/stat"
MEMINFO_PATH = "/proc/meminfo"


class WebServerNode(Node):
    def __init__(self):
        super().__init__("web_control")
        self.declare_parameter("web_port", 8080)
        self.declare_parameter("rosbridge_port", 9090)
        self.declare_parameter("cam_device", "")      # "" = auto-detect the UVC cam
        self.declare_parameter("cam_width", 640)
        self.declare_parameter("cam_height", 480)
        self.declare_parameter("cam_fps", 15)
        self.declare_parameter("mic_device", "")       # "" = auto-detect USB mic
        self.declare_parameter("mic_rate", 16000)      # Hz; 16k mono = 32 KB/s
        self.declare_parameter("tts_enabled", True)
        self.declare_parameter("tts_pico_bin", "pico2wave")  # from PATH (deploy/install-picotts.sh)
        self.declare_parameter("tts_device", "")       # aplay -D target; "" = ALSA default
        self.declare_parameter("tts_default_voice", "en-US")
        # Where the live TTS settings (voice/volume/speed/pitch + stats-announcer) are
        # persisted so they survive an SBC reboot. "" = XDG state dir under $HOME.
        self.declare_parameter("tts_settings_path", "")
        # ---- LLM (OpenRouter) personality generation ----------------------------
        # api_key lives in robot.yaml (or, if left "", the OPENROUTER_API_KEY env var).
        # The rest are seeded here and then web-tunable + persisted (see llm settings).
        self.declare_parameter("llm_enabled", False)
        self.declare_parameter("llm_api_key", "")          # "" -> $OPENROUTER_API_KEY
        self.declare_parameter("llm_model", "")            # "" -> llm.DEFAULT_MODEL (cheap)
        self.declare_parameter("llm_smart_model", "")      # "" -> llm.DEFAULT_SMART_MODEL (chat)
        self.declare_parameter("llm_vision_model", "")     # "" -> llm.DEFAULT_VISION_MODEL
        self.declare_parameter("llm_base_url", "")         # "" -> OpenRouter default
        self.declare_parameter("llm_persona", "")          # FALLBACK persona (when no personality.json)
        self.declare_parameter("personality_path", "")     # "" -> ~/.local/state/nanobot/personality.json
        self.declare_parameter("llm_face_hold", 10.0)      # s to hold an LLM mood (0=keep)
        self.declare_parameter("llm_timeout", 20.0)        # s HTTP timeout per call
        self.declare_parameter("llm_max_tokens", 160)      # cap reply length
        self.declare_parameter("llm_settings_path", "")    # "" -> XDG state dir
        self.declare_parameter("cognition_log_path", "")   # decision log; "" -> XDG state dir
        self.declare_parameter("reflect_enable", True)     # slow personality reflection
        self.declare_parameter("reflect_period", 600.0)    # s floor between reflections
        g = self.get_parameter
        port = g("web_port").value

        self._cam = CameraStream(
            dev=g("cam_device").value or None,
            width=g("cam_width").value, height=g("cam_height").value,
            fps=g("cam_fps").value, logger=self.get_logger().info)

        self._mic = AudioStream(
            device=g("mic_device").value or None,
            rate=g("mic_rate").value, channels=1, logger=self.get_logger().info)

        # TTS publishes one word at a time to the OLED (it shows them karaoke-style),
        # blanking with "" at the end so the panel returns to the dashboard.
        self._word_pub = self.create_publisher(String, "oled_word", 10)
        self._tts = TtsEngine(
            pico_bin=g("tts_pico_bin").value, device=g("tts_device").value or None,
            default_voice=g("tts_default_voice").value, enabled=g("tts_enabled").value,
            on_word=lambda w: self._word_pub.publish(String(data=w)),
            logger=self.get_logger().info)

        # Persisted live settings (voice/volume/speed/pitch + periodic stats announcer).
        # Loaded from disk so they survive a reboot, then pushed into the engine. The
        # announce schedule is driven off the existing 1 Hz ping timer (see
        # _publish_ping) — no extra timer/wakeup — and a disabled announcer is a single
        # dict lookup, so it keeps working with the node even after every browser closes.
        self._cpu_prev = None                          # (idle, total) jiffies for CPU%
        self._settings = self._load_settings()
        self._apply_engine()
        self._cpu_prev = self._cpu_sample()
        self._announce_next = time.monotonic() + float(self._settings["announce_interval"])

        web_dir = os.path.join(get_package_share_directory("web_control"), "web")
        handler = functools.partial(_Handler, directory=web_dir, stream=self._cam,
                                    audio=self._mic, tts=self._tts, node=self)
        self._httpd = http.server.ThreadingHTTPServer(("0.0.0.0", port), handler)
        self._thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        self._thread.start()
        self.get_logger().info(
            f"control page at http://0.0.0.0:{port}  (serving {web_dir})")

        # Liveness ping for the ESP32 coprocessor. The ESP joins the zenoh graph over a raw
        # UART that can't detect the peer vanishing, so if the router/SBC restarts it would
        # keep publishing into the void until a manual reset. This always-on 1 Hz heartbeat
        # lets the firmware reboot itself when the pings stop (see LINK_RX_TIMEOUT_MS in
        # firmware/nanobot_coprocessor/src/main.cpp). Independent of any browser being open.
        self._ping_pub = self.create_publisher(Int32, "esp32_ping", 10)
        self._ping_seq = 0
        self.create_timer(1.0, self._publish_ping)

        # ---- LLM personality (OpenRouter) ---------------------------------------
        # Build the client (key from param or $OPENROUTER_API_KEY) + the /oled_face
        # publisher it drives. Off the ROS critical path: a missing key / no network just
        # makes it a no-op. Autonomous "chatter" is NOT driven here anymore — the Sismic
        # behaviour chart is the single brain; it sends /cognition/request, which we
        # execute below. We still serve the on-demand say/chat/observe/look endpoints.
        self._llm_settings = self._load_llm_settings()
        self._persona = self._load_persona()           # single-sourced from personality.json
        self._llm = LlmClient(
            enabled=self._llm_settings["enabled"],
            api_key=g("llm_api_key").value,
            model=self._llm_settings["model"], base_url=g("llm_base_url").value,
            persona=self._persona, vision_model=g("llm_vision_model").value,
            smart_model=g("llm_smart_model").value,
            timeout=float(g("llm_timeout").value), max_tokens=int(g("llm_max_tokens").value),
            logger=self.get_logger().info)
        self._llm_face_hold = float(g("llm_face_hold").value)
        self._llm_history = deque(maxlen=LLM_HISTORY_MAX)
        self._llm_lock = threading.Lock()              # serialise generate calls
        self._llm_busy = False
        self._face_pub = self.create_publisher(String, "oled_face", 10)
        # Decision log (viewable in the web UI): a ring buffer backed by a file so it
        # survives reboots. Seeded from the file tail on start.
        self._log_lock = threading.Lock()
        self._cog_log = deque(self._load_cog_log(), maxlen=LLM_LOG_MAX)
        # Statechart-driven enrichment requests: the behaviour node decides when/what,
        # we execute (capture frame if asked, add sensors, generate, speak + emote).
        self.create_subscription(String, "cognition/request", self._on_cog, 10)
        # Personality reflection (the "deep/slow" tier): read the current traits (latched
        # from the behaviour node) + recent events, and propose smoothed trait drift back.
        self._traits = {"curiosity": 0.5, "extraversion": 0.5, "caution": 0.6, "playfulness": 0.5}
        self._reflect_busy = False
        self._reflect_next = time.monotonic() + float(g("reflect_period").value)
        self._was_picked = False
        self._evolve_pub = self.create_publisher(String, "cognition/evolve", 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "cognition/traits", self._on_traits, latched)
        # Sensor snapshot for the "Observe" feature (light: we just store the latest
        # value from each, with its arrival time for staleness). CPU/RAM/temp come from
        # the same cheap /proc reads the announcer uses; these add IMU + pick-up.
        self._imu_accel = self._imu_gyro = 0.0
        self._roll = self._pitch = self._yaw = 0.0
        self._imu_at = self._eul_at = -1e9
        self._susp_l = self._susp_r = False
        self.create_subscription(Vector3Stamped, "imu/web", self._on_imu_web, 10)
        self.create_subscription(Vector3Stamped, "imu/euler", self._on_imu_euler, 10)
        self.create_subscription(Bool, "left_wheel_suspended", self._on_susp_l, 10)
        self.create_subscription(Bool, "right_wheel_suspended", self._on_susp_r, 10)
        self.get_logger().info(
            f"llm: {'enabled' if self._llm.available() else 'idle (no key / disabled)'}"
            f" model={self._llm.model}")

    def _publish_ping(self):
        self._ping_seq = (self._ping_seq + 1) & 0x7FFFFFFF
        self._ping_pub.publish(Int32(data=self._ping_seq))
        self._announce_tick()                          # piggy-backs on this 1 Hz tick
        self._reflect_tick()                           # ditto (cheap when not due)

    # ---- persisted TTS settings ---------------------------------------------
    def _settings_file(self):
        p = self.get_parameter("tts_settings_path").value
        return p or os.path.expanduser("~/.local/state/nanobot/tts.json")

    def _load_settings(self):
        s = dict(SETTINGS_DEFAULTS)
        s["voice"] = self.get_parameter("tts_default_voice").value or "en-US"
        try:
            with open(self._settings_file()) as f:
                saved = json.load(f)
            s.update({k: v for k, v in saved.items() if k in SETTINGS_DEFAULTS})
        except Exception:
            pass                                       # no/invalid file -> defaults
        return _sanitize_settings(s)

    def _save_settings(self):
        try:
            path = self._settings_file()
            os.makedirs(os.path.dirname(path), exist_ok=True)
            tmp = path + ".tmp"
            with open(tmp, "w") as f:
                json.dump(self._settings, f)
            os.replace(tmp, path)                      # atomic; never a half-written file
        except Exception as exc:
            self.get_logger().warning(f"tts: could not persist settings ({exc})")

    def _apply_engine(self):
        """Push the current voice + markup levels into the TTS engine."""
        s = self._settings
        self._tts.configure(voice=s["voice"], volume=s["volume"],
                            speed=s["speed"], pitch=s["pitch"])

    def get_settings(self):
        return dict(self._settings)

    def update_settings(self, data):
        """Merge a partial settings dict from the web UI, persist, and apply."""
        old = self._settings
        s = dict(old)
        for k in SETTINGS_DEFAULTS:
            if k in data:
                s[k] = data[k]
        self._settings = _sanitize_settings(s)
        self._save_settings()
        self._apply_engine()
        # (Re)arm the announcer ONLY when it was just enabled or its interval changed,
        # so adjusting an unrelated slider can't keep postponing the next announcement.
        if self._settings["announce"] and (
                not old["announce"]
                or self._settings["announce_interval"] != old["announce_interval"]):
            self._cpu_prev = self._cpu_sample()
            self._announce_next = time.monotonic() + float(self._settings["announce_interval"])
        return self._settings

    # ---- periodic spoken system stats ---------------------------------------
    def _announce_tick(self):
        # Cheapest checks first (this runs every second): a disabled announcer is one
        # dict lookup. available() (and the /proc reads) only happen at announce time.
        if not self._settings["announce"]:
            return
        now = time.monotonic()
        if now < self._announce_next:
            return
        self._announce_next = now + float(self._settings["announce_interval"])
        self.announce_now()

    def announce_now(self):
        if not self._tts.available():
            return
        text = self._compose_stats(self._cpu_percent(), self._mem_percent(),
                                   self._cpu_temp())
        if text:
            self._tts.say(text)

    def _compose_stats(self, cpu, mem, temp):
        de = self._settings["voice"].startswith("de")
        parts = []
        if cpu == cpu:                                 # not NaN
            parts.append(f"Prozessor {cpu:.0f} Prozent" if de else f"C P U {cpu:.0f} percent")
        if mem == mem:
            parts.append(f"Arbeitsspeicher {mem:.0f} Prozent" if de else f"RAM {mem:.0f} percent")
        if temp == temp:
            parts.append(f"Temperatur {temp:.0f} Grad" if de else f"Temperature {temp:.0f} degrees")
        if not parts:
            return "Keine Daten" if de else "No data"
        return ". ".join(parts)

    # ---- LLM (OpenRouter) personality ---------------------------------------
    def _llm_settings_file(self):
        p = self.get_parameter("llm_settings_path").value
        return p or os.path.expanduser("~/.local/state/nanobot/llm.json")

    def _load_llm_settings(self):
        s = dict(LLM_DEFAULTS)
        for k, param in (("enabled", "llm_enabled"), ("model", "llm_model")):
            v = self.get_parameter(param).value
            if v is not None:
                s[k] = v
        try:                                           # persisted UI changes win over params
            with open(self._llm_settings_file()) as f:
                saved = json.load(f)
            s.update({k: v for k, v in saved.items() if k in LLM_DEFAULTS})
        except Exception:
            pass
        return _sanitize_llm_settings(s)

    def _load_persona(self):
        """Single source of the persona: personality.json (creator output, the same file
        the behaviour node seeds from), else the llm_persona param fallback."""
        path = (self.get_parameter("personality_path").value
                or os.path.expanduser("~/.local/state/nanobot/personality.json"))
        try:
            with open(path, encoding="utf-8") as f:
                persona = (json.load(f) or {}).get("persona")
            if isinstance(persona, str) and persona.strip():
                return persona.strip()
        except Exception:
            pass
        return self.get_parameter("llm_persona").value or ""

    def _save_llm_settings(self):
        try:
            path = self._llm_settings_file()
            os.makedirs(os.path.dirname(path), exist_ok=True)
            tmp = path + ".tmp"
            with open(tmp, "w") as f:
                json.dump(self._llm_settings, f)       # note: never contains the API key
            os.replace(tmp, path)
        except Exception as exc:
            self.get_logger().warning(f"llm: could not persist settings ({exc})")

    def llm_available(self):
        return self._llm.available()

    def get_llm_settings(self):
        s = dict(self._llm_settings)
        s["available"] = self._llm.available()         # enabled AND a key is configured
        s["configured"] = self._llm.available()
        s["model_effective"] = self._llm.model
        s["smart_model"] = self._llm.smart_model
        s["vision_model"] = self._llm.vision_model
        s["persona"] = self._persona              # read-only: single-sourced from personality.json
        s["moods"] = list(MOODS)
        return s

    def update_llm_settings(self, data):
        old = dict(self._llm_settings)
        s = dict(old)
        for k in LLM_DEFAULTS:
            if k in data:
                s[k] = data[k]
        self._llm_settings = _sanitize_llm_settings(s)
        self._save_llm_settings()
        self._llm.configure(enabled=self._llm_settings["enabled"],
                            model=self._llm_settings["model"])
        return self.get_llm_settings()

    # --- decision log (viewable in the web UI) -------------------------------
    def _cog_log_file(self):
        p = self.get_parameter("cognition_log_path").value
        return p or os.path.expanduser("~/.local/state/nanobot/cognition.log")

    def _load_cog_log(self):
        """Seed the in-memory ring from the file's last LLM_LOG_MAX JSON lines, so the
        web view shows history across reboots. Best-effort (returns [] on any problem)."""
        try:
            with open(self._cog_log_file()) as f:
                lines = f.readlines()[-LLM_LOG_MAX:]
        except Exception:
            return []
        out = []
        for ln in lines:
            ln = ln.strip()
            if ln:
                try:
                    out.append(json.loads(ln))
                except Exception:
                    pass
        return out

    def _log_decision(self, trigger, state="", camera=False, status="", model="",
                      prompt="", say="", mood="", ms=0, detail=""):
        """Record one cognition decision (+ outcome) to the ring buffer and append it as
        a JSON line to the log file. Log failures never block a decision."""
        entry = {"t": time.time(), "trigger": trigger, "state": state,
                 "camera": bool(camera), "model": model, "prompt": (prompt or "")[:160],
                 "say": say, "mood": mood, "status": status, "detail": detail, "ms": ms}
        with self._log_lock:
            self._cog_log.append(entry)
        try:
            path = self._cog_log_file()
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "a") as f:
                f.write(json.dumps(entry) + "\n")
        except Exception:
            pass
        return entry

    def get_cog_log(self):
        with self._log_lock:
            return {"entries": list(self._cog_log)[::-1]}   # newest first

    # --- statechart enrichment executor (/cognition/request) -----------------
    def _on_cog(self, msg: String):
        """A beat request from the behaviour chart: {beat, state, prompt, camera}. We
        execute it (capture a frame if asked, add live sensors, generate, speak + emote)
        off the executor thread. Best-effort: skips/logs when busy / unavailable."""
        try:
            req = json.loads(msg.data)
        except Exception:
            return
        beat = str(req.get("beat") or "beat")
        state = str(req.get("state") or "")
        prompt = str(req.get("prompt") or "")
        camera = bool(req.get("camera"))
        if isinstance(req.get("traits"), dict):        # freshest personality snapshot
            self._traits.update({k: req["traits"][k] for k in REFLECT_TRAITS
                                 if k in req["traits"]})
        trigger = "beat:" + beat
        if not self._llm.available():
            self._log_decision(trigger, state, camera, status="llm-unavailable")
            return
        threading.Thread(target=self._run_beat,
                         args=(trigger, state, prompt, camera), daemon=True).start()

    def _run_beat(self, trigger, state, prompt, camera):
        frame = None
        if camera:
            frame = self._capture_frame()
            if frame is None:
                self._log_decision(trigger, state, camera, status="no-frame")
                return
        full = (prompt + " Your current personality (0..1) is " + self._traits_phrase()
                + ", and your body senses: " + self._sensor_snapshot())
        self._generate(full, image_jpeg=frame, trigger=trigger, state=state, camera=camera)

    def _traits_phrase(self):
        return ", ".join(f"{k} {self._traits.get(k, 0.5):.2f}" for k in REFLECT_TRAITS)

    def _on_traits(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        if isinstance(data.get("traits"), dict):
            self._traits.update({k: data["traits"][k] for k in REFLECT_TRAITS
                                 if k in data["traits"]})

    # --- personality reflection (deep/slow tier) -----------------------------
    def _reflect_tick(self):
        if not (self.get_parameter("reflect_enable").value and self._llm.available()):
            return
        now = time.monotonic()
        picked = self._susp_l and self._susp_r
        if picked and not self._was_picked:            # a notable event -> reflect soon
            self._reflect_next = min(self._reflect_next, now + 5.0)
        self._was_picked = picked
        if now < self._reflect_next or self._reflect_busy:
            return
        self._reflect_next = now + float(self.get_parameter("reflect_period").value)
        threading.Thread(target=self._reflect, daemon=True).start()

    def _recent_events_text(self, n=25):
        with self._log_lock:
            entries = list(self._cog_log)[-n:]
        lines = [f"- {e.get('trigger','')} [{e.get('status','')}] "
                 f"{e.get('say') or e.get('detail') or ''}".rstrip() for e in entries]
        return "\n".join(lines) or "(no recent events)"

    def _reflect(self):
        self._reflect_busy = True
        t0 = time.monotonic()
        try:
            system = (
                "You are the slow, reflective mind of a small robot named Nano. You review "
                "what just happened and gently adjust its personality so it grows over time. "
                "Traits are 0..1: curiosity, extraversion, caution, playfulness. Output ONLY "
                'compact JSON: {"traits": {<trait>: <new target 0..1>}, "registry": '
                '{optional: {"musing"/"looking": {"priority":0..1,"enabled":bool}}}, '
                '"note": "<one short reason>"}. Propose only SMALL, justified nudges to a few '
                "traits (omit ones you would not change); the value is a TARGET that gets "
                "smoothed over time. No prose outside the JSON.")
            user = (f"Current traits: {self._traits_phrase()}.\nRecent events:\n"
                    f"{self._recent_events_text()}\n\nReflect and propose adjustments.")
            content = self._llm.complete(system, user, smart=True, json_object=True)
        finally:
            self._reflect_busy = False
        ms = int((time.monotonic() - t0) * 1000)
        obj = _extract_json(content or "")
        traits = {k: clamp(obj["traits"][k] * 100, 0, 100) / 100.0
                  for k in REFLECT_TRAITS
                  if isinstance(obj.get("traits"), dict) and k in obj["traits"]}
        registry = obj.get("registry") if isinstance(obj.get("registry"), dict) else {}
        if not traits and not registry:
            self._log_decision("reflect", status="no-reply",
                               model=self._llm.smart_model, ms=ms)
            return
        self._evolve_pub.publish(String(data=json.dumps({"traits": traits, "registry": registry})))
        self._log_decision("reflect", status="spoke", model=self._llm.smart_model,
                           say=f"{obj.get('note','')} -> {traits}", ms=ms)

    # --- sensor snapshot (for "Observe") -------------------------------------
    def _on_imu_web(self, msg: Vector3Stamped):     # x=|accel| m/s^2, y=|gyro| rad/s
        self._imu_accel, self._imu_gyro = msg.vector.x, msg.vector.y
        self._imu_at = time.monotonic()

    def _on_imu_euler(self, msg: Vector3Stamped):   # x=roll, y=pitch, z=yaw (degrees)
        self._roll, self._pitch, self._yaw = msg.vector.x, msg.vector.y, msg.vector.z
        self._eul_at = time.monotonic()

    def _on_susp_l(self, msg: Bool):
        self._susp_l = bool(msg.data)

    def _on_susp_r(self, msg: Bool):
        self._susp_r = bool(msg.data)

    def _cpu_percent_quick(self):
        """A short standalone CPU% sample for the snapshot — does NOT touch _cpu_prev
        (which belongs to the periodic stats announcer), so the two never interfere."""
        a = self._cpu_sample()
        time.sleep(0.12)
        b = self._cpu_sample()
        if a and b:
            di, dt = b[0] - a[0], b[1] - a[1]
            if dt > 0:
                return 100.0 * (1.0 - di / dt)
        return float("nan")

    def _sensor_snapshot(self):
        """A short plain-English description of how the robot's body feels right now,
        for the LLM to react to. Only includes sources that are present + fresh."""
        parts = []
        cpu, mem, temp = self._cpu_percent_quick(), self._mem_percent(), self._cpu_temp()
        if cpu == cpu:                                  # not NaN
            parts.append(f"CPU load {cpu:.0f}%")
        if mem == mem:
            parts.append(f"memory {mem:.0f}% used")
        if temp == temp:
            parts.append(f"main board {temp:.0f} degrees C")
        now = time.monotonic()
        if (now - self._imu_at) < 3.0:                  # moving / being jostled?
            moving = self._imu_gyro > 0.3 or abs(self._imu_accel - 9.81) > 1.5
            parts.append("being moved or jostled" if moving else "physically still")
        if (now - self._eul_at) < 3.0:                  # tilt from roll/pitch
            tilt = max(abs(self._roll), abs(self._pitch))
            if tilt > 25:
                parts.append(f"tilted over at about {tilt:.0f} degrees")
            elif tilt > 10:
                parts.append(f"leaning slightly ({tilt:.0f} degrees)")
            else:
                parts.append("sitting level")
        if self._susp_l and self._susp_r:               # pick-up
            parts.append("lifted off the ground (being held)")
        elif self._susp_l or self._susp_r:
            parts.append("with one wheel off the ground")
        else:
            parts.append("resting on the ground")
        return ", ".join(parts) if parts else "no sensor data available"

    def llm_observe(self):
        """Build a sensor snapshot and have the robot comment on how it feels."""
        snap = self._sensor_snapshot()
        self.get_logger().info(f"llm observe: {snap}")
        prompt = (f"Your own body's sensors report right now: {snap}. In character, say "
                  "one short spoken line reacting to how you physically feel or what your "
                  "sensors notice, and pick a fitting mood.")
        return self._generate(prompt, trigger="observe")

    def _capture_frame(self, timeout=4.0):
        """Grab one JPEG from the webcam via the shared CameraStream (starts the camera
        if nobody's watching, stops it after). Returns bytes, or None if unavailable."""
        if self._cam is None:
            return None
        self._cam.add_viewer()
        try:
            seq, deadline = 0, time.monotonic() + timeout
            while time.monotonic() < deadline:
                seq, jpeg = self._cam.get_frame(seq, timeout=deadline - time.monotonic())
                if jpeg:
                    return bytes(jpeg)
                if not self._cam.running():
                    break                                  # camera failed / no device
            return None
        finally:
            self._cam.remove_viewer()

    def llm_look(self):
        """Capture a camera frame (+ the sensor snapshot) and have the robot comment on
        what it SEES, via the vision model. Falls back to None if no frame is available."""
        frame = self._capture_frame()
        if frame is None:
            self.get_logger().warning("llm look: no camera frame")
            self._log_decision("look", "", True, status="no-frame")
            return {"error": "no camera frame"}
        snap = self._sensor_snapshot()
        self.get_logger().info(f"llm look: {len(frame)} byte frame; {snap}")
        prompt = ("This is the live view from your own camera. Your body also senses: "
                  f"{snap}. In character, say one short spoken line about what you can "
                  "see in front of you right now, and pick a fitting mood.")
        return self._generate(prompt, image_jpeg=frame, trigger="look", camera=True)

    def _express(self, mood, say):
        """Show a mood on /oled_face and speak the line. Optionally clear the face back
        to the dashboard after llm_face_hold seconds (0 = leave it up like a manual mood)."""
        if mood and mood != "neutral":
            self._face_pub.publish(String(data=mood))
            if self._llm_face_hold > 0:
                t = threading.Timer(self._llm_face_hold,
                                    lambda: self._face_pub.publish(String(data="")))
                t.daemon = True
                t.start()
        if say and self._tts is not None and self._tts.available():
            self._tts.say(say)

    def _generate(self, prompt, history=None, image_jpeg=None, trigger="manual",
                  state="", camera=False, smart=False):
        """Blocking generate + express, guarded so only one call runs at a time (the
        board has little RAM/CPU and the API costs money). Records the decision + outcome
        to the log. Returns the reply dict or None. Safe to call from any worker thread."""
        if not self._llm.available():
            self._log_decision(trigger, state, camera, status="llm-unavailable")
            return None
        with self._llm_lock:
            if self._llm_busy:
                self._log_decision(trigger, state, camera, status="skipped-busy")
                return None
            self._llm_busy = True
        t0 = time.monotonic()
        try:
            reply = self._llm.generate(prompt, history=history, image_jpeg=image_jpeg,
                                       smart=smart)
        finally:
            self._llm_busy = False
        model = self._llm.model_for(smart=smart, image=bool(image_jpeg))
        ms = int((time.monotonic() - t0) * 1000)
        if reply:
            self._express(reply["mood"], reply["say"])
            self._log_decision(trigger, state, camera, status="spoke", model=model,
                               prompt=prompt, say=reply["say"], mood=reply["mood"], ms=ms)
        else:
            self._log_decision(trigger, state, camera, status="no-reply", model=model,
                               prompt=prompt, ms=ms)
        return reply

    def llm_say(self, prompt=""):
        """On-demand 'say something': a one-shot reaction (no chat history)."""
        prompt = (prompt or "").strip() or (
            "Say one short, friendly, spontaneous line out loud to whoever is near you "
            "right now, and pick a fitting mood.")
        return self._generate(prompt, trigger="say")

    def llm_chat(self, message):
        """Conversational turn: keeps a short rolling history so replies have context."""
        message = (message or "").strip()
        if not message:
            return None
        history = list(self._llm_history)
        reply = self._generate(message, history=history, trigger="chat", smart=True)
        if reply:                                      # remember the exchange for context
            self._llm_history.append({"role": "user", "content": message})
            self._llm_history.append({"role": "assistant", "content": reply["say"]})
        return reply

    def _cpu_sample(self):
        try:
            with open(STAT_PATH) as f:
                parts = [int(x) for x in f.readline().split()[1:]]
            idle = parts[3] + (parts[4] if len(parts) > 4 else 0)  # idle + iowait
            return idle, sum(parts)
        except Exception:
            return None

    def _cpu_percent(self):
        """Busy % since the previous sample (the gap between announcements)."""
        cur = self._cpu_sample()
        pct = float("nan")
        if cur and self._cpu_prev:
            di, dt = cur[0] - self._cpu_prev[0], cur[1] - self._cpu_prev[1]
            if dt > 0:
                pct = 100.0 * (1.0 - di / dt)
        self._cpu_prev = cur
        return pct

    def _mem_percent(self):
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
            return 100.0 * (tot - avail) / tot if tot else float("nan")
        except Exception:
            return float("nan")

    def _cpu_temp(self):
        try:
            with open(THERMAL_PATH) as f:
                return int(f.read().strip()) / 1000.0
        except Exception:
            return float("nan")

    def destroy_node(self):
        try:
            self._httpd.shutdown()
        except Exception:
            pass
        super().destroy_node()


def _sanitize_settings(s):
    """Coerce/clamp a settings dict to safe types + ranges (UI is untrusted)."""
    out = dict(SETTINGS_DEFAULTS)
    out.update(s)
    out["voice"] = out["voice"] if out["voice"] in VOICES else SETTINGS_DEFAULTS["voice"]
    out["volume"] = clamp(out["volume"], 0, 500)
    out["speed"] = clamp(out["speed"], 20, 500)
    out["pitch"] = clamp(out["pitch"], 50, 200)
    out["announce"] = bool(out["announce"])
    out["announce_interval"] = clamp(out["announce_interval"], ANNOUNCE_MIN, ANNOUNCE_MAX)
    return out


def _sanitize_llm_settings(s):
    """Coerce/clamp the LLM settings dict to safe types (UI + on-disk file untrusted)."""
    out = dict(LLM_DEFAULTS)
    out.update(s)
    out["enabled"] = bool(out["enabled"])
    out["model"] = str(out["model"] or "")[:120]
    return out


class _Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, stream=None, audio=None, tts=None, node=None, **kwargs):
        self._stream = stream
        self._audio = audio
        self._tts = tts
        self._node = node
        super().__init__(*args, **kwargs)

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path == "/tts/config":
            # Current persisted settings, so the page can restore its controls on load.
            return self._respond_json(self._node.get_settings() if self._node else {})
        if path == "/llm/config":
            return self._respond_json(self._node.get_llm_settings() if self._node else {})
        if path == "/llm/log":
            return self._respond_json(self._node.get_cog_log() if self._node else {"entries": []})
        if path == "/stream.mjpg":
            return self._stream_mjpeg()
        if path == "/audio.pcm":
            return self._stream_audio()
        if path == "/map":
            return self._serve_map()
        if path == "/scan.bin":
            return self._serve_scan()
        return super().do_GET()

    def do_POST(self):
        path = self.path.split("?", 1)[0]
        if path == "/tts":
            # Speak a line and karaoke its words to the OLED. Body: {"text","voice"?}.
            data = self._read_json()
            text = (data.get("text") or "").strip()
            voice = (data.get("voice") or "").strip() or None
            if self._tts is None or not self._tts.available():
                self._respond(503, "tts unavailable")
            elif not text:
                self._respond(400, "empty text")
            else:
                self._tts.say(text, voice=voice)
                self._respond(200, "speaking")
        elif path == "/tts/config":
            # Update + persist live settings (voice/volume/speed/pitch/announce/interval).
            if self._node is None:
                self._respond(503, "no node")
            else:
                self._respond_json(self._node.update_settings(self._read_json()))
        elif path == "/tts/announce":
            # Speak the system stats once, right now (independent of the periodic toggle).
            if self._node is None or self._tts is None or not self._tts.available():
                self._respond(503, "tts unavailable")
            else:
                self._node.announce_now()
                self._respond(200, "announcing")
        elif path == "/tts/stop":
            if self._tts is not None:
                self._tts.stop()
            self._respond(200, "stopped")
        elif path == "/llm/say":
            # Generate one in-character spoken line + mood and perform it. Optional
            # {"prompt"} steers it. Blocks on the OpenRouter call (handler thread).
            if self._node is None or not self._node.llm_available():
                self._respond(503, "llm unavailable")
            else:
                reply = self._node.llm_say((self._read_json().get("prompt") or ""))
                self._respond_json(reply or {"error": "no reply"})
        elif path == "/llm/chat":
            # Conversational turn: {"message"} -> the robot replies (speaks + emotes)
            # with short rolling context. Returns the reply so the UI can show it.
            if self._node is None or not self._node.llm_available():
                self._respond(503, "llm unavailable")
            else:
                msg = (self._read_json().get("message") or "").strip()
                if not msg:
                    self._respond(400, "empty message")
                else:
                    reply = self._node.llm_chat(msg)
                    self._respond_json(reply or {"error": "no reply"})
        elif path == "/llm/observe":
            # Snapshot the robot's own sensors (CPU/RAM/temp, IMU motion/tilt, pick-up)
            # and have it comment in character on how it feels. Speaks + emotes.
            if self._node is None or not self._node.llm_available():
                self._respond(503, "llm unavailable")
            else:
                reply = self._node.llm_observe()
                self._respond_json(reply or {"error": "no reply"})
        elif path == "/llm/look":
            # Capture a camera frame (+ sensors) and have the robot comment on what it
            # SEES, via the vision model. Speaks + emotes.
            if self._node is None or not self._node.llm_available():
                self._respond(503, "llm unavailable")
            else:
                reply = self._node.llm_look()
                self._respond_json(reply or {"error": "no reply"})
        elif path == "/llm/config":
            if self._node is None:
                self._respond(503, "no node")
            else:
                self._respond_json(self._node.update_llm_settings(self._read_json()))
        elif path == "/system/restart":
            # Restart the whole ROS stack. Detached + new session so it survives
            # do_down killing this very web server, then do_up brings it back.
            self._set_oled_action("restart")   # tells the OLED to show "Restarting stack"
            self._run_detached(
                'cd "$HOME/Nano" && "$HOME/.pixi/bin/pixi" run bash scripts/stack.sh restart')
            self._respond(200, "restarting stack")
        elif path == "/system/reboot":
            # Reboot the whole SBC (needs the scoped NOPASSWD sudo rule for systemctl).
            self._set_oled_action("reboot")    # tells the OLED to show "Restarting"
            self._run_detached("sudo -n /usr/bin/systemctl reboot")
            self._respond(200, "rebooting")
        elif path == "/system/shutdown":
            # Power off the SBC (needs the scoped NOPASSWD sudo rule for systemctl).
            self._set_oled_action("shutdown")  # tells the OLED to show "Shutting down" + go dark
            self._run_detached("sudo -n /usr/bin/systemctl poweroff")
            self._respond(200, "shutting down")
        else:
            self.send_error(404)

    @staticmethod
    def _set_oled_action(action):
        # Hint the OLED node (read in its SIGTERM shutdown sequence) which end-screen to
        # show. Written synchronously here so it exists before the (delayed) stop runs.
        try:
            with open("/dev/shm/nano_oled_action", "w") as f:
                f.write(action)
        except Exception:
            pass

    @staticmethod
    def _run_detached(cmd):
        # 1 s delay lets the HTTP response flush before the action runs.
        subprocess.Popen(["bash", "-lc", "sleep 1; " + cmd],
                         stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL, start_new_session=True)

    def _read_json(self):
        """Parse the request body as JSON; {} on any problem (length-bounded)."""
        try:
            n = int(self.headers.get("Content-Length", 0) or 0)
            raw = self.rfile.read(min(n, 8192)) if n > 0 else b""
            return json.loads(raw or b"{}")
        except Exception:
            return {}

    def _respond(self, code, msg):
        body = msg.encode()
        self.send_response(code)
        self.send_header("Content-Type", "text/plain")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _respond_json(self, obj):
        body = json.dumps(obj).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _stream_mjpeg(self):
        if self._stream is None:
            self.send_error(503, "no camera")
            return
        self._stream.add_viewer()
        try:
            self.send_response(200)
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=FRAME")
            self.end_headers()
            seq = 0
            while True:
                seq, jpeg = self._stream.get_frame(seq, timeout=5.0)
                if jpeg is None:
                    if not self._stream.running():
                        break          # camera failed / no device
                    continue
                self.wfile.write(b"--FRAME\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(b"Content-Length: %d\r\n\r\n" % len(jpeg))
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            pass                       # client closed the stream
        finally:
            self._stream.remove_viewer()

    def _stream_audio(self):
        if self._audio is None:
            self.send_error(503, "no microphone")
            return
        q = self._audio.add_listener()
        try:
            # Stream as HTTP/1.1 chunked. Browsers buffer an HTTP/1.0 (close-
            # delimited) streaming body and never hand it to fetch()'s reader until
            # the connection closes — which for a live mic is never — so without
            # chunked the page would receive nothing. Chunked is surfaced live.
            self.protocol_version = "HTTP/1.1"
            self.send_response(200)
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            # raw signed 16-bit little-endian PCM; rate/channels in headers so the
            # browser can configure the Web Audio decoder without hardcoding.
            self.send_header("Content-Type", "audio/L16;rate=%d;channels=%d"
                             % (self._audio.rate, self._audio.channels))
            self.send_header("X-Sample-Rate", str(self._audio.rate))
            self.send_header("X-Channels", str(self._audio.channels))
            self.send_header("Transfer-Encoding", "chunked")
            self.end_headers()
            import queue as _q
            while True:
                try:
                    data = q.get(timeout=5.0)
                except _q.Empty:
                    if not self._audio.running():
                        break          # mic failed / no device
                    continue
                # one HTTP chunk: <hex len>CRLF <data> CRLF, flushed immediately
                self.wfile.write(b"%X\r\n" % len(data))
                self.wfile.write(data)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
            self.wfile.write(b"0\r\n\r\n")
            self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass                       # client stopped listening
        finally:
            self._audio.remove_listener(q)

    def _serve_map(self):
        # The slam_nav node writes the live occupancy map to a RAM file (/dev/shm);
        # we just hand the bytes over same-origin so the page's map canvas can render
        # them. No ROS subscription / OccupancyGrid serialization in this process.
        self._serve_shm("/dev/shm/nano_map.bin", "no map yet")

    def _serve_scan(self):
        # The lidar driver writes each scan as a compact blob to /dev/shm (JSON header +
        # raw float32 ranges); the page polls it here instead of bridging the heavy
        # /scan LaserScan over rosbridge. Same idea as the map — keeps rosbridge light.
        self._serve_shm("/dev/shm/nano_scan.bin", "no scan yet")

    def _serve_shm(self, path, missing_msg):
        try:
            with open(path, "rb") as f:
                data = f.read()
        except OSError:
            self.send_error(503, missing_msg)
            return
        self.send_response(200)
        self.send_header("Content-Type", "application/octet-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        try:
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, *args):      # silence per-request stderr spam
        pass


def main():
    rclpy.init()
    node = WebServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
