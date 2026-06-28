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
import array
import functools
import http.server
import json
import math
import os
import subprocess
import threading
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, Int32, Float32, String
from geometry_msgs.msg import Twist, Vector3Stamped

from .mjpeg_camera import CameraStream
from .mic_audio import AudioStream
from .tts import TtsEngine, VOICES, clamp
from .llm import LlmClient
from .skills import resolve_skills_dir
from .cognition import CognitionCore

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
# (Chat-history / log-ring sizes + the trait list now live in cognition.CognitionCore.)

# SBC vitals for the spoken stats (same cheap /proc + thermal reads the OLED uses).
THERMAL_PATH = "/sys/class/thermal/thermal_zone0/temp"
STAT_PATH = "/proc/stat"
MEMINFO_PATH = "/proc/meminfo"
SCAN_FILE = "/dev/shm/nano_scan.bin"          # compact lidar blob (for the read-lidar skill)

# The GATED "action tier" for topic-skills: the ONLY ROS topics a skill may publish, each
# with a hard clamp. Anything else is refused. Motion is ALSO clamped reflexively by
# slam_nav downstream, so a skill can never push the robot into an unsafe state. Builders
# turn a skill's `value` into a ROS message; web_server only wires these when the
# skills_allow_actions master switch is on. (topic -> relative ROS topic name.)
SKILL_MOTION_LIN_MAX = 0.15                    # m/s   cap on a skill's commanded linear speed
SKILL_MOTION_ANG_MAX = 0.8                     # rad/s cap on a skill's commanded yaw rate
SKILL_MOTION_DUR_MAX = 3.0                     # s     cap on /cmd_vel drive time before auto-stop


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
        self.declare_parameter("llm_model", "")            # paid cheap fallback ("" -> DEFAULT_MODEL)
        self.declare_parameter("llm_smart_model", "")      # paid smart fallback ("" -> DEFAULT_SMART_MODEL)
        self.declare_parameter("llm_vision_model", "")     # "" -> llm.DEFAULT_VISION_MODEL (free)
        self.declare_parameter("llm_free_model", "")       # FREE cheap primary ("" -> DEFAULT_FREE_MODEL)
        self.declare_parameter("llm_free_smart_model", "") # FREE smart primary ("" -> DEFAULT_FREE_SMART_MODEL)
        self.declare_parameter("llm_vision_fallback_model", "")  # optional PAID vision fallback ("" -> none)
        self.declare_parameter("llm_base_url", "")         # "" -> OpenRouter default
        self.declare_parameter("llm_persona", "")          # FALLBACK persona (when no personality.json)
        self.declare_parameter("personality_path", "")     # "" -> ~/.local/state/nanobot/personality.json
        self.declare_parameter("llm_face_hold", 10.0)      # s to hold an LLM mood (0=keep)
        self.declare_parameter("llm_timeout", 20.0)        # s HTTP timeout per call
        self.declare_parameter("llm_max_tokens", 160)      # cap reply length
        self.declare_parameter("llm_smart_max_per_hour", 15)   # hourly cap on pro/smart text (0=off)
        self.declare_parameter("llm_vision_max_per_hour", 10)  # hourly cap on camera/vision (0=off)
        # Phrase bank: pre-generated lines for the frequent body-reaction beats (instant,
        # free, offline). See phrasebank.py.
        self.declare_parameter("phrasebank_enable", True)
        self.declare_parameter("phrasebank_path", "")          # "" -> XDG state dir
        self.declare_parameter("phrasebank_live_ratio", 0.2)   # P(use live LLM anyway for variety)
        self.declare_parameter("phrasebank_drift", 0.6)        # trait drift that triggers regen
        self.declare_parameter("phrasebank_per_category", 6)   # lines generated per situation
        self.declare_parameter("llm_settings_path", "")    # "" -> XDG state dir
        self.declare_parameter("cognition_log_path", "")   # decision log; "" -> XDG state dir
        self.declare_parameter("reflect_enable", True)     # slow personality reflection
        self.declare_parameter("reflect_period", 600.0)    # s floor between reflections
        self.declare_parameter("self_model_enable", True)  # durable smart-LLM self-narrative
        self.declare_parameter("self_model_path", "")      # "" -> XDG state dir
        self.declare_parameter("consolidate_every", 6)     # rewrite the self-narrative every Nth reflect
        self.declare_parameter("prelude_enable", True)     # instant "thinking" filler + "stumped" on fail
        # Skill library: a portable, self-documenting capability catalogue (skills/*.md).
        # The brain can pick one on a `skill` beat + the web UI can invoke any of them. The
        # gated action tier (topic-publishing skills) is OFF unless skills_allow_actions is on.
        self.declare_parameter("skills_enable", True)      # load + offer the skill library
        self.declare_parameter("skills_allow_actions", False)  # permit topic-publishing skills
        self.declare_parameter("skills_dir", "")           # "" -> installed share / source skills/
        # Skill workshop: meditation's experience-driven skill-synthesis loop (suggest -> check
        # -> rehearse -> trial -> adopt/retire). Minted skills live in workshop_dir (a writable,
        # deploy-synced "learned" area, separate from the committed catalogue).
        self.declare_parameter("workshop_enable", True)        # mint/adapt skills while meditating
        self.declare_parameter("workshop_dir", "")             # "" -> ~/.local/state/nanobot/skills
        self.declare_parameter("workshop_path", "")            # ledger; "" -> XDG state dir
        self.declare_parameter("workshop_rounds", 1)           # candidates proposed per meditation
        self.declare_parameter("workshop_min_runs", 3)         # runs before a trial may adopt
        self.declare_parameter("workshop_retire_errors", 2)    # errors that retire a trial
        self.declare_parameter("workshop_retire_net_neg", 2)   # net 👎 that retire a trial
        # Lifecycle speech + the persistent "AI offline" indicator (all offline-safe via the
        # phrase bank's FALLBACK_LINES, so they work with no key / no network).
        self.declare_parameter("startup_greeting", True)   # speak a greeting line on boot
        self.declare_parameter("llm_offline_indicator", True)  # persistent face+line when LLM is down
        self.declare_parameter("offline_face", "sleepy")   # OLED mood shown while the LLM is unreachable
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

        # ---- Cognition core (shared, ROS-free) ----------------------------------
        # ALL the LLM-personality logic lives in web_control.cognition.CognitionCore, shared
        # verbatim with the dev harness (scripts/dev_webui.py) — one base to maintain. We build
        # it here with ROS-backed adapters: face -> /oled_face, capture_frame -> webcam, sensors
        # -> /proc+IMU, the gated action tier -> whitelisted publishers, persist -> llm.json. Off
        # the ROS critical path: a missing key / no network just makes it a no-op. Autonomous
        # chatter is NOT driven here — the Sismic chart sends /cognition/request, which _on_cog
        # hands to the core. We still serve the on-demand say/chat/observe/look endpoints.
        self._llm_settings = self._load_llm_settings()
        self._persona = self._load_persona()           # single-sourced from personality.json
        self._persona_name = self._load_persona_name() # the robot's name (for the phrase bank)
        self._face_pub = self.create_publisher(String, "oled_face", 10)
        llm = LlmClient(
            enabled=self._llm_settings["enabled"], api_key=g("llm_api_key").value,
            model=self._llm_settings["model"], base_url=g("llm_base_url").value,
            persona=self._persona, vision_model=g("llm_vision_model").value,
            smart_model=g("llm_smart_model").value, free_model=g("llm_free_model").value,
            free_smart_model=g("llm_free_smart_model").value,
            vision_fallback_model=g("llm_vision_fallback_model").value,
            smart_max_per_hour=int(g("llm_smart_max_per_hour").value),
            vision_max_per_hour=int(g("llm_vision_max_per_hour").value),
            timeout=float(g("llm_timeout").value), max_tokens=int(g("llm_max_tokens").value),
            logger=self.get_logger().info)
        # Gated action-tier publishers — only created when actions are permitted, so
        # web_control doesn't appear as a /cmd_vel talker etc. while the tier is off.
        self._skills_allow_actions = bool(g("skills_allow_actions").value)
        self._skill_pubs = {}
        if self._skills_allow_actions:
            self._skill_pubs = {
                "/led": self.create_publisher(Bool, "led", 10),
                "/fan_pwm": self.create_publisher(Float32, "fan_pwm", 10),
                "/lds_target_rpm": self.create_publisher(Float32, "lds_target_rpm", 10),
                "/cmd_vel": self.create_publisher(Twist, "cmd_vel", 10),
            }
        self._cog = CognitionCore(
            llm=llm, tts=self._tts, persona=self._persona, persona_name=self._persona_name,
            settings=self._llm_settings,
            face=lambda m: self._face_pub.publish(String(data=m)),
            capture_frame=self._capture_frame, sensor_snapshot=self._sensor_snapshot,
            sensor_signals=self._sensor_signals, scan_summary=self._scan_summary,
            audio_summary=self._audio_summary,
            publish_action=self._publish_skill_action, logger=self.get_logger().info,
            persist_settings=self._save_llm_settings,
            cog_log_path=(g("cognition_log_path").value or ""),
            face_hold=float(g("llm_face_hold").value),
            bank_path=(g("phrasebank_path").value or None),
            bank_enable=bool(g("phrasebank_enable").value),
            bank_live_ratio=float(g("phrasebank_live_ratio").value),
            bank_drift=float(g("phrasebank_drift").value),
            bank_per_category=int(g("phrasebank_per_category").value),
            skills_dir=resolve_skills_dir(g("skills_dir").value,
                                          get_package_share_directory("web_control")),
            skills_enable=bool(g("skills_enable").value),
            skills_allow_actions=self._skills_allow_actions,
            self_model_enable=bool(g("self_model_enable").value),
            self_model_path=(g("self_model_path").value or None),
            consolidate_every=int(g("consolidate_every").value),
            prelude_enable=bool(g("prelude_enable").value),
            workshop_enable=bool(g("workshop_enable").value),
            workshop_dir=(g("workshop_dir").value or ""),
            workshop_path=(g("workshop_path").value or None),
            workshop_rounds=int(g("workshop_rounds").value),
            workshop_min_runs=int(g("workshop_min_runs").value),
            workshop_retire_errors=int(g("workshop_retire_errors").value),
            workshop_retire_net_neg=int(g("workshop_retire_net_neg").value))
        # Statechart-driven enrichment requests: the behaviour node decides when/what, the
        # core executes (capture frame if asked, add sensors, generate, speak + emote).
        self.create_subscription(String, "cognition/request", self._on_cog, 10)
        # Reflection (deep/slow tier) scheduling + the persistent offline indicator. The WORK
        # lives in the core; the node only schedules it and delivers the result (publishes the
        # proposed evolve on /cognition/evolve, shows the offline face).
        self._reflect_busy = False
        self._reflect_next = time.monotonic() + float(g("reflect_period").value)
        self._was_picked = False
        self._llm_offline = None         # None=unknown, True=showing offline, False=online
        self._llm_offline_reassert = 0.0 # monotonic of last face re-assert while offline
        self._evolve_pub = self.create_publisher(String, "cognition/evolve", 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "cognition/traits", self._on_traits, latched)
        # Brain controls from the web UI (the behaviour node consumes these): human reward
        # -> A/B bandit + reward shaping; meditation -> consolidation/sleep mode.
        self._reward_pub = self.create_publisher(String, "cognition/reward", 10)
        self._meditate_pub = self.create_publisher(Bool, "meditate", latched)
        self._meditating = False
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
            f"llm: {'enabled' if self._cog.available() else 'idle (no key / disabled)'}"
            f" model={self._cog.llm.model}")
        self._cog.bank_regen_check()                    # build/refresh the bank if needed
        if bool(g("startup_greeting").value):
            # Say hello a few seconds after boot (once the OLED/TTS are up). Offline-safe via
            # the phrase bank's greeting fallback; the boot face is the behaviour node's job.
            t = threading.Timer(3.0, lambda: self._cog.speak_lifecycle("greeting"))
            t.daemon = True
            t.start()

    def _publish_ping(self):
        self._ping_seq = (self._ping_seq + 1) & 0x7FFFFFFF
        self._ping_pub.publish(Int32(data=self._ping_seq))
        self._announce_tick()                          # piggy-backs on this 1 Hz tick
        self._reflect_tick()                           # ditto (cheap when not due)
        self._llm_health_tick()                        # persistent "AI offline" indicator

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

    def _load_persona_name(self):
        """The robot's name from personality.json (used in the phrase bank's {name}); 'Nano'
        as a fallback."""
        path = (self.get_parameter("personality_path").value
                or os.path.expanduser("~/.local/state/nanobot/personality.json"))
        try:
            with open(path, encoding="utf-8") as f:
                name = (json.load(f) or {}).get("name")
            if isinstance(name, str) and name.strip():
                return name.strip()
        except Exception:
            pass
        return "Nano"

    def _save_llm_settings(self, settings):
        """The core's `persist_settings` adapter: write the UI {enabled,model} to llm.json
        (never the API key). Atomic via .tmp + rename."""
        self._llm_settings = dict(settings)
        try:
            path = self._llm_settings_file()
            os.makedirs(os.path.dirname(path), exist_ok=True)
            tmp = path + ".tmp"
            with open(tmp, "w") as f:
                json.dump(self._llm_settings, f)       # note: never contains the API key
            os.replace(tmp, path)
        except Exception as exc:
            self.get_logger().warning(f"llm: could not persist settings ({exc})")

    # --- cognition-core delegators (the shared LLM brain lives in cognition.py) ----
    # These thin forwards are the node's public API for the HTTP handler; all the logic is in
    # CognitionCore, shared verbatim with scripts/dev_webui.py (one base to maintain).
    def llm_available(self):
        return self._cog.available()

    def get_llm_settings(self):
        return self._cog.get_llm_settings()

    def update_llm_settings(self, data):
        return self._cog.update_llm_settings(data)

    def get_cog_log(self):
        return self._cog.get_cog_log()

    def get_phrasebank(self):
        return self._cog.get_phrasebank()

    def regenerate_phrasebank(self):
        return self._cog.regenerate_phrasebank()

    def get_skills(self):
        return self._cog.get_skills()

    def reload_skills(self):
        return self._cog.reload_skills()

    def invoke_skill(self, name):
        return self._cog.invoke_skill(name)

    def llm_say(self, prompt=""):
        return self._cog.llm_say(prompt)

    def llm_chat(self, message):
        return self._cog.llm_chat(message)

    def llm_observe(self):
        return self._cog.llm_observe()

    def llm_look(self):
        return self._cog.llm_look()

    # --- brain controls (reward + meditation; ROS-side, logged via the core) ------
    def brain_reward(self, data):
        """Record a human reward and publish it for the behaviour node. Contextual reward
        (with a `target` echoed from /task_current) credits a specific A/B arm; global reward
        shapes the intrinsic-reward weights via the decision log on the next reflection."""
        try:
            value = clamp(float(data.get("value", 0)), -1, 1)
        except (TypeError, ValueError):
            value = 0.0
        scope = "global" if data.get("scope") == "global" else "contextual"
        target = data.get("target") if isinstance(data.get("target"), dict) else None
        status = "up" if value > 0 else ("down" if value < 0 else "neutral")
        detail = scope
        if target:
            detail += " " + json.dumps({k: target.get(k) for k in ("exp", "variant", "task")})
        self._cog.log_decision("reward", status=status, detail=detail,
                               say=("👍" if value > 0 else "👎" if value < 0 else "·"))
        # Also credit a trial skill that just ran (the workshop's "happy user" signal): a
        # contextual 👍/👎 right after a skill helps it earn (or lose) permanence.
        if scope == "contextual":
            self._cog.reward_trial_skill(value)
        self._reward_pub.publish(String(data=json.dumps(
            {"value": value, "scope": scope, "target": target})))
        return {"status": "ok", "value": value, "scope": scope}

    def brain_meditate(self, data):
        """Toggle meditation/consolidation. Publishes /meditate for the behaviour node (calm
        face + paused beats + local purpose/A/B consolidation) and, on entry, kicks the LLM
        reflection + phrase-bank regeneration here so the whole brain consolidates at once."""
        on = bool(data.get("on"))
        self._meditating = on
        self._meditate_pub.publish(Bool(data=on))
        if on:
            self._reflect_next = time.monotonic()  # reflect now, then every <=60 s while on
            self._cog.bank_regen_check()           # refresh the phrase bank (background)
            threading.Thread(target=self._cog.consolidate, daemon=True).start()  # long-term self
            # The skill workshop: mine experience -> mint/adapt a skill on trial (off-thread,
            # it makes several LLM calls). Sweeps the adopt/retire gate when it finishes.
            threading.Thread(target=self._cog.run_skill_workshop, daemon=True).start()
        self._cog.log_decision("meditate", status=("on" if on else "off"))
        return {"status": "ok", "meditating": on}

    def brain_workshop(self):
        return self._cog.get_workshop()

    def workshop_keep(self, data):
        return self._cog.keep_skill(str(data.get("name", "")))

    def workshop_kill(self, data):
        return self._cog.kill_skill(str(data.get("name", "")))

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
            self._cog.update_traits(req["traits"])
        trigger = "beat:" + beat
        if not self._cog.available():
            self._cog.log_decision(trigger, state, camera, status="llm-unavailable")
            return
        if beat == "skill":                            # let the brain pick a skill to perform
            threading.Thread(target=self._cog.run_skill_beat,
                             args=(state or "acting",), daemon=True).start()
            return
        threading.Thread(target=self._cog.run_beat,
                         args=(trigger, state, prompt, camera), daemon=True).start()

    # --- action-tier publish + lidar summary (the core's ROS-backed adapters) ----
    def _scan_summary(self):
        """One plain-English line about the latest lidar scan: nearest range + rough bearing
        (for the read-lidar skill). Reads the same /dev/shm blob the web map polls."""
        try:
            with open(SCAN_FILE, "rb") as f:
                blob = f.read()
            nl = blob.index(b"\n")
            head = json.loads(blob[:nl])
            n = int(head.get("n", 0))
            ranges = array.array("f")
            ranges.frombytes(blob[nl + 1:nl + 1 + 4 * n])
        except Exception:
            return "no scan available"
        amin = float(head.get("amin", 0.0))
        ainc = float(head.get("ainc", 0.0))
        best = None
        for i, r in enumerate(ranges):
            if r == r and 0.05 < r < 1e6 and (best is None or r < best[0]):  # r==r: not NaN/inf
                best = (r, amin + i * ainc)
        if best is None:
            return "nothing within range — open space all around"
        dist, ang = best
        d = (math.degrees(ang) + 360) % 360            # robot frame: 0 ahead, +90 left
        where = ("ahead" if d < 45 or d >= 315 else "to your left" if d < 135
                 else "behind you" if d < 225 else "to your right")
        return "the nearest object is about %.2f m %s" % (dist, where)

    def _audio_summary(self, listen=0.4):
        """One plain-English line about what the mic currently hears (for the `listen` skill).
        Briefly taps the ref-counted AudioStream — piggybacks on arecord if a browser is
        already listening, else spins it up for ~`listen` s — and turns the raw S16LE level
        into words. The RMS/peak thresholds are rough and meant to be TUNED ON HARDWARE.
        Returns 'no audio available' if the mic isn't there or nothing arrives."""
        import queue as _q
        if getattr(self, "_mic", None) is None:
            return "no audio available"
        sink = self._mic.add_listener()
        chunks, deadline = [], time.monotonic() + max(0.1, float(listen))
        try:
            while time.monotonic() < deadline:
                try:
                    chunks.append(sink.get(timeout=0.2))
                except _q.Empty:
                    pass
        finally:
            self._mic.remove_listener(sink)
        raw = b"".join(chunks)
        if len(raw) < 2:
            return "no audio available"
        samples = array.array("h")
        samples.frombytes(raw[: len(raw) // 2 * 2])     # whole 16-bit frames only
        if not samples:
            return "no audio available"
        peak = max(abs(s) for s in samples)
        rms = (sum(s * s for s in samples) / len(samples)) ** 0.5
        level = ("near silence" if rms < 200 else "quiet" if rms < 800
                 else "a steady ambient murmur" if rms < 3000
                 else "noticeably loud" if rms < 8000 else "very loud")
        sharp = peak > 12000 and peak > rms * 6          # a transient over the background
        return "the room sounds like %s%s" % (level, "; a sharp sound just spiked" if sharp else "")

    def _publish_skill_action(self, action):
        """The core's `publish_action` adapter: turn a skill's `action` into a clamped ROS
        message on a whitelisted topic and publish it. Returns (ok, human-readable detail).
        Never raises on bad input. (Gating + logging live in the core's _do_topic_skill.)"""
        topic = str(action.get("topic") or "").strip()
        pub = self._skill_pubs.get(topic)
        if pub is None:
            return False, "topic not whitelisted: " + (topic or "(none)")
        val = action.get("value")
        try:
            if topic == "/led":
                on = bool(val)
                pub.publish(Bool(data=on))
                off_after = action.get("off_after")
                if on and off_after:                   # auto-revert the LED after a moment
                    self._later(min(float(off_after), 10.0),
                                lambda: pub.publish(Bool(data=False)))
                return True, "/led=%s" % on
            if topic == "/fan_pwm":
                duty = clamp(float(val), 0.0, 1.0)
                pub.publish(Float32(data=duty))
                return True, "/fan_pwm=%.2f" % duty
            if topic == "/lds_target_rpm":
                rpm = clamp(float(val), 0.0, 400.0)
                pub.publish(Float32(data=rpm))
                return True, "/lds_target_rpm=%.0f" % rpm
            if topic == "/cmd_vel":
                v = val if isinstance(val, dict) else {}
                lin = clamp(float(v.get("lin", 0.0)), -SKILL_MOTION_LIN_MAX, SKILL_MOTION_LIN_MAX)
                ang = clamp(float(v.get("ang", 0.0)), -SKILL_MOTION_ANG_MAX, SKILL_MOTION_ANG_MAX)
                dur = clamp(float(action.get("duration", 1.0)), 0.0, SKILL_MOTION_DUR_MAX)
                tw = Twist()
                tw.linear.x = lin
                tw.angular.z = ang
                pub.publish(tw)
                self._later(dur, lambda: pub.publish(Twist()))   # always auto-stop
                return True, "/cmd_vel lin=%.2f ang=%.2f for %.1fs" % (lin, ang, dur)
        except (TypeError, ValueError) as exc:
            return False, "bad value: %s" % exc
        return False, "unhandled topic: " + topic

    @staticmethod
    def _later(delay, fn):
        t = threading.Timer(max(0.0, float(delay)), fn)
        t.daemon = True
        t.start()

    def _on_traits(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        if isinstance(data.get("traits"), dict):
            self._cog.update_traits(data["traits"])
            self._cog.bank_regen_check()                # soul moved -> refresh bank if too far

    # --- personality reflection (deep/slow tier): scheduled here, done by the core ----
    def _reflect_tick(self):
        if not (self.get_parameter("reflect_enable").value and self._cog.available()):
            return
        now = time.monotonic()
        picked = self._susp_l and self._susp_r
        if picked and not self._was_picked:            # a notable event -> reflect soon
            self._reflect_next = min(self._reflect_next, now + 5.0)
        self._was_picked = picked
        if now < self._reflect_next or self._reflect_busy:
            return
        period = float(self.get_parameter("reflect_period").value)
        if self._meditating:
            period = min(period, 60.0)             # consolidate faster while meditating/charging
        self._reflect_next = now + period
        threading.Thread(target=self._reflect, daemon=True).start()

    def _reflect(self):
        """Run the core's reflection and DELIVER the proposal on /cognition/evolve (the
        behaviour node smooths it into the live traits). The prompt/parse/log are in the core."""
        self._reflect_busy = True
        try:
            res = self._cog.reflect()
        finally:
            self._reflect_busy = False
        if res:
            self._evolve_pub.publish(String(data=json.dumps(
                {"traits": res["traits"], "registry": res["registry"]})))

    # --- sensor snapshot (the core's ROS-backed adapters) --------------------
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

    def _sensor_signals(self):
        """The same body state as _sensor_snapshot(), but structured for the phrase bank's
        classifier (NaN/None where a source is missing or stale)."""
        now = time.monotonic()
        cpu, mem, temp = self._cpu_percent_quick(), self._mem_percent(), self._cpu_temp()
        moving = None
        if (now - self._imu_at) < 3.0:
            moving = self._imu_gyro > 0.3 or abs(self._imu_accel - 9.81) > 1.5
        tilt = None
        if (now - self._eul_at) < 3.0:
            tilt = max(abs(self._roll), abs(self._pitch))
        pickup = 2 if (self._susp_l and self._susp_r) else (1 if (self._susp_l or self._susp_r) else 0)
        return {"cpu": cpu, "mem": mem, "temp": temp, "moving": moving,
                "tilt": tilt, "pickup": pickup}

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

    # --- lifecycle speech (offline-safe via the phrase bank, done by the core) ----
    def system_announce(self, action):
        """Speak the matching farewell/restart line just before a stack/board action."""
        cat = "farewell" if action in ("shutdown", "poweroff") else "restarting"
        return self._cog.speak_lifecycle(cat)

    def _llm_health_tick(self):
        """Persistent 'AI offline' indicator: when the LLM is enabled but unreachable, show
        the offline face + speak the offline line once, and clear when it recovers. Edge-
        triggered, with a slow re-assert so a transient TTS word / manual mood doesn't lose
        the face. Counts repeated real-call failures too, so a network drop (key present) also
        trips it — not just a missing key. The behaviour node stands down on the foreign face,
        so its idle beats pause while we're offline."""
        if not self.get_parameter("llm_offline_indicator").value:
            return
        if not bool(self._cog.settings.get("enabled")):
            offline = False                            # LLM opted out -> presence runs normally
        elif not self._cog.available():
            offline = True                             # enabled but no key
        else:
            offline = self._cog.llm_fail_streak >= 2   # key present but calls keep failing
        face = str(self.get_parameter("offline_face").value or "sleepy")
        now = time.monotonic()
        if offline != self._llm_offline:
            prev = self._llm_offline
            self._llm_offline = offline
            if offline:
                self.get_logger().warning("LLM unreachable - showing offline mood")
                self._cog.speak_lifecycle("offline", face=face)
                self._llm_offline_reassert = now
            elif prev:                                     # only if we WERE showing offline
                self.get_logger().info("LLM reachable again - clearing offline mood")
                self._face_pub.publish(String(data=""))    # hand the panel back
        elif offline and (now - self._llm_offline_reassert) > 20.0:
            self._llm_offline_reassert = now               # best-effort re-assert
            self._face_pub.publish(String(data=face))

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
        if path == "/llm/phrases":
            return self._respond_json(self._node.get_phrasebank() if self._node else {})
        if path == "/skills":
            return self._respond_json(self._node.get_skills() if self._node else {"skills": []})
        if path == "/skills/workshop":
            return self._respond_json(
                self._node.brain_workshop() if self._node else {"trials": []})
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
        elif path == "/llm/phrases/regenerate":
            if self._node is None:
                self._respond(503, "no node")
            else:
                self._respond_json(self._node.regenerate_phrasebank())
        elif path == "/skills/invoke":
            # Run one skill from the library now: {"name": "<skill>"}. Blocks on any LLM /
            # express call (like the /llm/* endpoints) and returns the outcome.
            if self._node is None:
                self._respond(503, "no node")
            else:
                name = (self._read_json().get("name") or "").strip()
                if not name:
                    self._respond(400, "empty name")
                else:
                    self._respond_json(self._node.invoke_skill(name))
        elif path == "/skills/reload":
            # Re-scan the skills directory so a freshly-added .md shows up without a restart.
            if self._node is None:
                self._respond(503, "no node")
            else:
                self._respond_json(self._node.reload_skills())
        elif path == "/skills/workshop/keep":
            # Manually adopt a trial skill now: {"name":"<skill>"} (Keep button).
            if self._node is None:
                self._respond(503, "no node")
            else:
                self._respond_json(self._node.workshop_keep(self._read_json()))
        elif path == "/skills/workshop/kill":
            # Manually discard a trial skill now: {"name":"<skill>"} (Kill button).
            if self._node is None:
                self._respond(503, "no node")
            else:
                self._respond_json(self._node.workshop_kill(self._read_json()))
        elif path == "/brain/reward":
            # Reward the current behaviour: {"value":±1,"scope":"contextual"|"global","target"?}.
            if self._node is None:
                self._respond(503, "no node")
            else:
                self._respond_json(self._node.brain_reward(self._read_json()))
        elif path == "/brain/meditate":
            # Toggle meditation/consolidation mode: {"on":bool}.
            if self._node is None:
                self._respond(503, "no node")
            else:
                self._respond_json(self._node.brain_meditate(self._read_json()))
        elif path == "/system/restart":
            # Restart the whole ROS stack. Detached + new session so it survives
            # do_down killing this very web server, then do_up brings it back.
            self._set_oled_action("restart")   # tells the OLED to show "Restarting stack"
            if self._node:
                self._node.system_announce("restart")   # speak the restart line first
            self._run_detached(
                'cd "$HOME/Nano" && "$HOME/.pixi/bin/pixi" run bash scripts/stack.sh restart',
                delay=3)                       # let the spoken line play before teardown
            self._respond(200, "restarting stack")
        elif path == "/system/reboot":
            # Reboot the whole SBC (needs the scoped NOPASSWD sudo rule for systemctl).
            self._set_oled_action("reboot")    # tells the OLED to show "Restarting"
            if self._node:
                self._node.system_announce("reboot")
            self._run_detached("sudo -n /usr/bin/systemctl reboot", delay=3)
            self._respond(200, "rebooting")
        elif path == "/system/shutdown":
            # Power off the SBC (needs the scoped NOPASSWD sudo rule for systemctl).
            self._set_oled_action("shutdown")  # tells the OLED to show "Shutting down" + go dark
            if self._node:
                self._node.system_announce("shutdown")  # speak the farewell line first
            self._run_detached("sudo -n /usr/bin/systemctl poweroff", delay=3)
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
    def _run_detached(cmd, delay=1):
        # The delay lets the HTTP response flush (and any spoken line play) before the
        # action runs. start_new_session so it survives this web server being killed.
        subprocess.Popen(["bash", "-lc", f"sleep {int(delay)}; " + cmd],
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
