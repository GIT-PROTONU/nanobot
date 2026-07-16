"""The robot's ONLY browser gateway: static control page + telemetry + control + media.

Kept as a ROS node so it starts/stops with the rest of the launch and shows up
in `ros2 node list`. It serves the package's installed `web/` directory
(index.html + style.css + the per-panel *.js). There is NO rosbridge anymore —
the page talks exclusively to this server:

- `GET /telemetry` — ONE Server-Sent-Events stream carrying every light readout
  (odom/IMU/diagnostics/ESP32/LDS/OLED mirror/brain) as a compact JSON frame at
  `telemetry_rate` Hz. `POST /publish` + `POST /param` are the whitelisted write
  paths (goal, setpoints, OLED owners, tuning sliders). See telemetry.py.
- `POST /drive` ({"v","w"}) — HTTP teleop. Publishes /cmd_vel directly with a
  node-side 10 Hz keepalive + dead-man (see the drive_* params).
- `/map`, `/scan.bin` — the /dev/shm blobs slam_nav / lds_driver_py write,
  served same-origin so the big messages never cross rosbridge.
- `/stream.mjpg` — USB webcam as multipart/x-mixed-replace via a zero-dependency
  V4L2 MJPEG passthrough (mjpeg_camera); `/snapshot.jpg` is one still frame.
- `/audio.pcm` — the webcam mic as raw PCM via arecord (mic_audio). Camera and
  mic run only while a client is connected, so they cost nothing idle.
- `POST /tts` ({"text","voice"?}) — speaks via espeak-ng (see tts) and streams
  the words to the OLED on /oled_word as they're said.
- `GET /health/log` — tail of sys_monitor's durable ESP32/LDS outage log.
"""
import array
import functools
import http.server
import copy
import json
import math
import os
import queue
import subprocess
import threading
import time
from datetime import datetime

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, Int8, Int32, Float32, String
from geometry_msgs.msg import Twist

from . import procstats
from .jsonio import read_json, write_json
from .mjpeg_camera import CameraStream
from .gpu_vision import GpuVision
from .mic_audio import AudioStream
from .telemetry import TelemetryHub
from .tts import TtsEngine, VOICES, clamp
from .llm import LlmClient
from .skills import resolve_skills_dir
from .cognition import CognitionCore, sanitize_personality_patch
from .stress import StressTest
from .imu_interference import IMUInterferenceTest

# Persisted, web-tunable TTS settings (merged over the file on disk). `voice` is
# seeded from the tts_default_voice param at load time.
SETTINGS_DEFAULTS = {
    "voice": "en-gb",            # seeded from tts_default_voice param at load time
    "volume": 100,            # Pico level %, 100 = normal
    "speed": 100,
    "pitch": 100,
    "base_pitch": 50,         # espeak -p 0-99, default 50 (normal)
    "lead_silence": 350,      # ms of silence prepended so the H5 codec's power-up
                               # ramp can't clip the first word (tts.LEAD_SILENCE);
                               # live-tunable here since the right value is hardware/
                               # temperature dependent and can only be judged by ear.
    "announce": False,        # speak CPU/RAM/temp every `announce_interval` s
    "announce_interval": 30,  # seconds (clamped to >= ANNOUNCE_MIN)
}
ANNOUNCE_MIN = 5              # don't let the announcer spam faster than this (s)
ANNOUNCE_MAX = 3600
MAX_BODY = 65536             # cap on a POST body we'll buffer (bytes); larger = drained + dropped

# Web-tunable LLM (OpenRouter) settings, merged over the file on disk and seeded from
# the robot.yaml params. The API key is NEVER stored here — it stays in robot.yaml /
# the OPENROUTER_API_KEY env var — so this file (and any backup of it) holds no secret.
LLM_DEFAULTS = {
    "enabled": False,         # opt-in: it costs money + needs the network
    "model": "",              # "" -> llm.DEFAULT_MODEL
    "smart_model": "",        # "" -> llm.DEFAULT_SMART_MODEL
    "vision_model": "",       # "" -> llm.DEFAULT_VISION_MODEL
    "vision_fallback_model": "",  # optional PAID vision fallback ("" -> none)
    "free_model": "",         # "" -> llm.DEFAULT_FREE_MODEL
    "free_smart_model": "",   # "" -> llm.DEFAULT_FREE_SMART_MODEL
    "api_key": "",            # "" -> $OPENROUTER_API_KEY (see cognition.get_llm_settings:
                               # never echoed back to the browser, only an api_key_set flag)
}
# Each LLM "variant" (normal / deep-think / vision) has a PRIMARY (free-first) and a
# SECONDARY (paid fallback). The robot.yaml param feeding each settings key as its default.
LLM_PARAM_FOR = {
    "enabled": "llm_enabled",
    "model": "llm_model",                       # normal secondary (paid fallback)
    "smart_model": "llm_smart_model",           # deep-think secondary (paid fallback)
    "vision_model": "llm_vision_model",         # vision primary
    "vision_fallback_model": "llm_vision_fallback_model",  # vision secondary (paid fallback)
    "free_model": "llm_free_model",             # normal primary (free-first)
    "free_smart_model": "llm_free_smart_model", # deep-think primary (free-first)
    "api_key": "llm_api_key",                   # seeds from robot.yaml; a UI-saved key wins later
}
# NOTE: the persona is NOT here — it's single-sourced from personality.json (written by
# scripts/personality_creator.py, the same file the behaviour node loads), falling back to
# the llm_persona param. So one run of the creator + a restart updates the whole character.
# (Chat-history / log-ring sizes + the trait list now live in cognition.CognitionCore.)

SCAN_FILE = "/dev/shm/nano_scan.bin"          # compact lidar blob (for the read-lidar skill)
VITALS_FILE = "/dev/shm/nano_vitals.json"     # sys_monitor's aggregated body snapshot

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
        # SSE telemetry frame rate for GET /telemetry (the rosbridge replacement —
        # see telemetry.py). Costs nothing while no browser is connected.
        self.declare_parameter("telemetry_rate", 5.0)
        self.declare_parameter("cam_device", "")      # "" = auto-detect the UVC cam
        self.declare_parameter("cam_width", 640)
        self.declare_parameter("cam_height", 480)
        self.declare_parameter("cam_fps", 15)
        # GPU vision (Mali-450/GLES2 via lima, gpu_vision.py): OFF by default. When on,
        # GpuVision becomes the sole continuous camera owner (YUYV, zero-copy) and the
        # browser's /stream.mjpg + /snapshot.jpg are served from its JPEG tee instead of
        # a second MjpegCamera session -- see memory gpu-vision-camera-architecture for
        # why (the C270 only exposes one true capture-capable V4L2 node). Requires
        # `sudo modprobe lima` + the Mesa/EGL/GBM apt packages (deploy/sbc-setup.sh);
        # missing GPU userspace degrades to a log line + no-op, same as `sismic` absent.
        self.declare_parameter("gpu_vision_enable", False)
        # Flashlight/dark reflex (GPU vision Tier-B extension, opt-in, independent of
        # skills_allow_actions): auto-toggles /led when the GPU's average frame
        # luminance drops below/rises above these thresholds (hysteresis avoids
        # flicker at the boundary). OFF by default -- gpu_vision_enable alone never
        # makes the robot's LED move on its own.
        self.declare_parameter("vision_dark_reflex_enable", False)
        self.declare_parameter("vision_dark_threshold", 0.15)   # luma below this -> LED on
        self.declare_parameter("vision_dark_recover", 0.25)     # luma above this -> LED off
        # Optical virtual bumper (GPU vision Tier-B extension, telemetry.py's
        # _optical_bumper): commanded to move but the GPU's motion score stays under
        # motion_floor for confirm_secs -> likely a wheel stall/slip. Purely
        # informational (nothing acts on it), but its thresholds are real ROS params
        # (not fixed constants) so the web UI's sliders actually take effect, and
        # telemetry now also surfaces `commanded`/`cmd_vel` -- "always clear" almost
        # always just means "not currently being driven," not that it's broken; this
        # makes that visible instead of a single opaque bool.
        self.declare_parameter("vision_bumper_cmd_eps", 0.03)      # m/s or rad/s floor to count as "commanded"
        self.declare_parameter("vision_bumper_motion_floor", 0.01)  # gpu motion score below this = "nothing moved"
        self.declare_parameter("vision_bumper_confirm_secs", 0.6)   # s the stall condition must hold before alerting
        # IMU drift check (telemetry.py's _imu_drift_tick): while the robot is
        # provably stationary (same "not commanded" test as the bumper/vibration
        # checks above, reusing vision_bumper_cmd_eps, + wheels grounded), any change
        # in the reported roll/pitch/yaw isn't real motion -- it's gyro bias or
        # magnetometer interference (see the selftest-spin-imu-mismatch investigation).
        # Purely observational; a still period shorter than this is too brief to be a
        # meaningful reading and isn't latched into the last-result summary.
        self.declare_parameter("imu_drift_min_secs", 10.0)
        # Cheap GPU-vision alert thresholds (2026-07-12 batch, see gpu_vision.py's raw
        # signals + telemetry.py's _vision_alerts): all informational only, all live
        # web UI sliders, all initial guesses pending real hardware tuning.
        # 400.0: informed by a real reading (an ordinary indoor scene measured
        # luma_variance ~2700-2770 live, 2026-07-12) -- see robot.yaml's comment.
        self.declare_parameter("vision_obstruction_var_max", 400.0)  # luma variance below this = "flat"
        self.declare_parameter("vision_obstruction_dark_max", 0.15)  # AND luma below this = obstructed
        self.declare_parameter("vision_clutter_alert", 0.12)         # edge_density above this = "busy"
        self.declare_parameter("vision_overhead_alert", 0.12)        # overhead_edge_density above this
        self.declare_parameter("vision_focus_blur_max", 0.03)        # edge_density below this (+ lit) = blurred
        self.declare_parameter("vision_backlit_delta_min", 0.35)     # luma_max - luma above this = backlit
        self.declare_parameter("vision_highlight_alert", 0.05)       # highlight_fraction above this = shiny
        self.declare_parameter("vision_looming_alert", 0.3)          # motion_intercept_rate above this
        self.declare_parameter("vision_colorcast_alert", 0.12)       # max-min colour_cast spread above this
        self.declare_parameter("vision_motiontarget_match_max", 0.15)  # distance BELOW this = "matches"
        # 2026-07-13 batch: novelty score, camera-freeze + vibration diagnostics, glare
        # rejection, the anticipatory-approach greeting signal -- all live-tunable via
        # /param like the rest (see telemetry.PARAM_WHITELIST).
        self.declare_parameter("vision_novelty_alert", 0.25)         # novelty score above this = "something changed"
        self.declare_parameter("vision_camera_stall_secs", 2.0)      # frame age / exact-zero diff held this long = frozen
        self.declare_parameter("vision_vibration_ratio", 0.5)        # driving edge_density below still-baseline*this = blur
        self.declare_parameter("vision_vibration_confirm_secs", 1.5) # s the blur must hold before the vibration flag
        self.declare_parameter("vision_glare_derate", 0.0)           # blob confidence *= 1-k*highlight_fraction (0 = off)
        self.declare_parameter("vision_approach_rate", 0.5)          # motion growth (/s) above this = approaching
        self.declare_parameter("vision_approach_band", 0.25)         # AND motion centred within +-this of frame centre
        # Named colour-target palette: calibrations persist here and survive a restart
        # (previously a picked colour was lost on every stack restart).
        self.declare_parameter("vision_targets_path", "")   # "" -> ~/.local/state/nanobot/vision_targets.json
        # Visual diary: a slow durable log of the scene scalars (luma/motion/edge/
        # novelty/warmth) folded into the reflection prompts -- sensory continuity for
        # the self-narrative, same mechanism as the trait trajectory.
        self.declare_parameter("vision_diary_enable", True)
        self.declare_parameter("vision_diary_period", 600.0)   # s between snapshots
        self.declare_parameter("vision_diary_max", 288)        # ring size (288 @10min = 2 days)
        self.declare_parameter("vision_diary_window", 86400.0) # trend window for the prompts (1 day)
        self.declare_parameter("vision_diary_path", "")        # "" -> ~/.local/state/nanobot/vision_diary.json
        self.declare_parameter("mic_device", "")       # "" = auto-detect USB mic
        self.declare_parameter("mic_rate", 16000)      # Hz; 16k mono = 32 KB/s
        self.declare_parameter("tts_enabled", True)
        self.declare_parameter("tts_device", "")       # aplay -D target; "" = ALSA default
        self.declare_parameter("tts_default_voice", "en-gb")
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
        self.declare_parameter("llm_timeout", 20.0)        # s HTTP timeout per call (per-socket-op)
        self.declare_parameter("llm_hard_deadline", 45.0)  # s HARD wall-clock cap per call (anti-hang)
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
        self.declare_parameter("phrasebank_grow_enable", True)    # add fresh offline lines over time
        self.declare_parameter("phrasebank_grow_period", 1800.0)  # min seconds between growth attempts
        self.declare_parameter("phrasebank_grow_max", 24)         # per-category cap growth fills to
        self.declare_parameter("phrasebank_grow_batch", 3)        # new lines requested per growth call
        self.declare_parameter("llm_settings_path", "")    # "" -> XDG state dir
        self.declare_parameter("cognition_log_path", "")   # decision log; "" -> XDG state dir
        self.declare_parameter("reflect_enable", True)     # slow personality reflection
        self.declare_parameter("reflect_period", 600.0)    # s floor between reflections
        self.declare_parameter("self_model_enable", True)  # durable smart-LLM self-narrative
        self.declare_parameter("self_model_path", "")      # "" -> XDG state dir
        self.declare_parameter("consolidate_every", 6)     # rewrite the self-narrative every Nth reflect
        # Trait trajectory: durable (timestamp, traits) snapshots so reflection/consolidation can
        # reason about HOW the personality has drifted, not just the latest events.
        self.declare_parameter("trait_history_enable", True)
        self.declare_parameter("trait_history_path", "")      # "" -> XDG state dir
        self.declare_parameter("trait_history_period", 3600.0)  # min s between snapshots
        self.declare_parameter("trait_history_max", 336)        # snapshots kept (>=8)
        self.declare_parameter("trait_history_window", 604800.0)  # s trend looks back over (7d)
        self.declare_parameter("prelude_enable", True)     # instant "thinking" filler + "stumped" on fail
        self.declare_parameter("camera_announce", True)    # ALWAYS speak a "peeking" line before any camera use
        self.declare_parameter("camera_face", "looking")   # OLED face shown during the peek ("" = none)
        # Skill library: a portable, self-documenting capability catalogue (skills/*.md).
        # The brain can pick one on a `skill` beat + the web UI can invoke any of them. The
        # gated action tier (topic-publishing skills) is OFF unless skills_allow_actions is on.
        self.declare_parameter("skills_enable", True)      # load + offer the skill library
        self.declare_parameter("skills_allow_actions", False)  # permit topic-publishing skills
        self.declare_parameter("skills_dir", "")           # "" -> installed share / source skills/
        # Skill workshop: reflection mode's experience-driven skill-synthesis loop (suggest ->
        # check -> rehearse -> trial -> adopt/retire). Minted skills live in workshop_dir (a
        # writable, deploy-synced "learned" area, separate from the committed catalogue).
        self.declare_parameter("workshop_enable", True)        # mint/adapt skills while reflecting
        self.declare_parameter("workshop_dir", "")             # "" -> ~/.local/state/nanobot/skills
        self.declare_parameter("workshop_path", "")            # ledger; "" -> XDG state dir
        self.declare_parameter("workshop_rounds", 1)           # candidates proposed per reflection
        self.declare_parameter("workshop_min_runs", 3)         # runs before a trial may adopt
        self.declare_parameter("workshop_retire_errors", 2)    # errors that retire a trial
        self.declare_parameter("workshop_retire_net_neg", 2)   # net 👎 that retire a trial
        self.declare_parameter("workshop_adopt_quiet_runs", 5) # clean runs (no 👎) that auto-adopt w/o praise
        self.declare_parameter("workshop_trial_ttl", 172800.0) # s a trial may linger before rollback (0=off)
        self.declare_parameter("workshop_trial_bias", 0.5)     # P(a skill beat exercises a due trial)
        self.declare_parameter("skill_likes_path", "")         # 👍 likes ledger; "" -> XDG state dir
        self.declare_parameter("skill_like_bias", 0.6)         # P(a skill beat picks a liked skill by weight)
        self.declare_parameter("reflect_announce", True)       # speak reflection conclusions out loud
        # Quiet hours (local time, wrap-aware; negative = off): autonomous speech —
        # idle/skill beats, boot greeting, offline line, stats announcer, reflection
        # bookends — is muted inside the window; user-initiated speech still talks.
        # Keep in sync with behavior.quiet_start/quiet_end (the night idle slowdown).
        self.declare_parameter("quiet_start", -1.0)
        self.declare_parameter("quiet_end", -1.0)
        # Lifecycle speech + the persistent "AI offline" indicator (all offline-safe via the
        # phrase bank's FALLBACK_LINES, so they work with no key / no network).
        self.declare_parameter("startup_greeting", True)   # speak a greeting line on boot
        self.declare_parameter("llm_offline_indicator", True)  # persistent face+line when LLM is down
        self.declare_parameter("offline_face", "sleepy")   # OLED mood shown while the LLM is unreachable
        g = self.get_parameter
        port = g("web_port").value

        # The direct hardware-MJPEG passthrough is ALWAYS constructed (cheap -- lazily
        # opens the camera only once a viewer actually connects, same as before
        # GPU vision existed) so it's available as the fallback backend when GPU
        # vision is off/unavailable. Idle cost is zero either way.
        self._cam_direct = CameraStream(
            dev=g("cam_device").value or None,
            width=g("cam_width").value, height=g("cam_height").value,
            fps=g("cam_fps").value, logger=self.get_logger().info)
        self._gpu_vision = None
        self._camera_disabled = False      # master off switch, see set_camera_enable() below
        self._dark_led_pub = None
        self._dark_led_on = False
        # Named colour-target palette (persisted; empty when GPU vision is off).
        self._vision_targets_path = os.path.expanduser(
            g("vision_targets_path").value or "~/.local/state/nanobot/vision_targets.json")
        self._vision_targets = {}
        self._vision_target_active = None
        self._vision_approach = False      # anticipatory-approach signal (see _vision_state_tick)
        self._oled_mask_on = False         # OLED tracking-mask mirror state
        self._oled_mask_pub = None
        self._vision_state_pub = None
        if g("gpu_vision_enable").value:
            self._gpu_vision = GpuVision(
                dev=g("cam_device").value or None,
                width=g("cam_width").value, height=g("cam_height").value,
                fps=g("cam_fps").value, logger=self.get_logger().info)
            self._gpu_vision.start()
            self._load_vision_targets()    # re-apply the persisted calibration, if any
            # Publisher + timer always created (cheap, idle-safe) whenever GPU vision is
            # on, regardless of vision_dark_reflex_enable's startup value -- the tick
            # itself reads that param live, so toggling it via the web UI's /param POST
            # (see PARAM_WHITELIST) actually takes effect instead of silently no-op'ing
            # because the timer was never scheduled.
            self._dark_led_pub = self.create_publisher(Bool, "led", 5)
            self.create_timer(1.0, self._dark_reflex_tick)
            # Compact vision-state feed for the behaviour layer (mood_node): the alert
            # booleans + scalars its reflexes consume (approach greeting, looming/
            # clutter caution, ambient colour mood, novelty-boosted curiosity). Only
            # published while the pipeline is actually live, so a stale message can't
            # drive a reflex (mood_node also applies its own freshness window). The
            # same tick pushes the live vision_glare_derate param into the GL thread.
            self._vision_state_pub = self.create_publisher(String, "vision/state", 5)
            self._oled_mask_pub = self.create_publisher(
                Bool, "oled_mask",
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_timer(0.5, self._vision_state_tick)

        self._mic = AudioStream(
            device=g("mic_device").value or None,
            rate=g("mic_rate").value, channels=1, logger=self.get_logger().info)

        # TTS publishes one word at a time to the OLED (it shows them karaoke-style),
        # blanking with "" at the end so the panel returns to the dashboard.
        self._word_pub = self.create_publisher(String, "oled_word", 10)
        self._tts = TtsEngine(
            device=g("tts_device").value or None,
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
        self._cpu_prev = procstats.cpu_sample()
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

        # ---- HTTP teleop (POST /drive) -------------------------------------------
        # rosbridge is the busiest process on the board, so driving through it adds
        # latency spikes that can outlast the ESP32's 500 ms /cmd_vel watchdog — the
        # motors cut out and the drive stutters. The page POSTs {v,w} here over its
        # existing kept-alive HTTP socket instead; we publish /cmd_vel immediately and
        # then keep a steady 10 Hz keepalive from THIS node while the command is
        # non-zero, so the firmware sees fresh commands regardless of browser jank.
        # A dead-man zeroes the motors if the page stops refreshing (tab killed,
        # network drop). The page falls back to rosbridge if /drive is absent.
        self.declare_parameter("drive_max_lin", 0.4)    # m/s clamp (ESP32 maps 0.4 to full PWM)
        self.declare_parameter("drive_max_ang", 3.0)    # rad/s clamp
        self.declare_parameter("drive_timeout", 0.6)    # s without a POST -> stop
        # Same default as sys_monitor's health_log_path (it writes, we serve).
        self.declare_parameter("health_log_path", "~/.local/state/nanobot/health.log")
        self._drive_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self._drive_lock = threading.Lock()
        self._drive_v = self._drive_w = 0.0
        self._drive_at = 0.0                            # monotonic of last POST; 0 = idle
        self.create_timer(0.1, self._drive_tick)

        # ---- Stress test mode (POST /stress/start|stop, GET /stress/status) -----------
        # Deliberately loads every CPU core to validate the hardening tier (systemd
        # watchdogs, MemoryMax, the fan curve) under real load — see stress.py for why
        # niced worker subprocesses can't starve this web server. Auto-stops at
        # stress_max_duration (a forgotten test can't run forever) and can abort early on
        # temperature via stress_temp_abort_c (0 = no thermal abort).
        self.declare_parameter("stress_max_duration", 300.0)  # s hard cap on any run
        self.declare_parameter("stress_temp_abort_c", 82.0)   # 0 = disable the thermal abort
        self._stress = StressTest(
            logger=self.get_logger().info,
            max_duration=float(g("stress_max_duration").value),
            abort_temp_c=float(g("stress_temp_abort_c").value),
            read_temp=procstats.cpu_temp)

        # ---- IMU mounting-interference self-test (see imu_interference.py) ------------
        # Automates the "walk the loose IMU around by hand" mag-noise hunt from the IMU
        # card's hint: cycles the LDS/fan/LED/motors one at a time while parked and
        # scores each one's magnetometer disturbance. Reuses the gated skill-action
        # publishers below (created next), so this is instantiated after them but reads
        # them lazily at run time -- order here doesn't matter.
        self.declare_parameter("imu_test_lds_rpm", 300.0)   # matches slam_nav's lds_active_rpm
        self.declare_parameter("imu_test_motor_ang", 0.35)  # rad/s, optional motor-wiggle phase
        self._imu_test = IMUInterferenceTest(
            self, logger=self.get_logger().info,
            lds_rpm=float(g("imu_test_lds_rpm").value),
            motor_ang=float(g("imu_test_motor_ang").value))

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
        ls = self._llm_settings   # merged robot.yaml defaults + persisted UI changes
        llm = LlmClient(
            enabled=ls["enabled"], api_key=ls["api_key"],
            model=ls["model"], base_url=g("llm_base_url").value,
            persona=self._persona, vision_model=ls["vision_model"],
            smart_model=ls["smart_model"], free_model=ls["free_model"],
            free_smart_model=ls["free_smart_model"],
            vision_fallback_model=ls["vision_fallback_model"],
            smart_max_per_hour=int(g("llm_smart_max_per_hour").value),
            vision_max_per_hour=int(g("llm_vision_max_per_hour").value),
            timeout=float(g("llm_timeout").value), max_tokens=int(g("llm_max_tokens").value),
            hard_deadline=float(g("llm_hard_deadline").value),
            logger=self.get_logger().info)
        # Auto-enable the LLM when a key is present: the user who provides a key
        # (via robot.yaml, env var, or memory/openrouter_key) clearly wants it on,
        # without also having to toggle the web UI switch. The web UI can still
        # turn it off explicitly (which persists to llm.json and survives a restart).
        if not ls["enabled"]:
            raw_key = (ls["api_key"] or "").strip() or os.environ.get("OPENROUTER_API_KEY", "").strip()
            if raw_key:
                self.get_logger().info("llm: key detected, auto-enabling")
                ls["enabled"] = True
                llm.configure(enabled=True)
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
            bank_grow_enable=bool(g("phrasebank_grow_enable").value),
            bank_grow_period=float(g("phrasebank_grow_period").value),
            bank_grow_max=int(g("phrasebank_grow_max").value),
            bank_grow_batch=int(g("phrasebank_grow_batch").value),
            skills_dir=resolve_skills_dir(g("skills_dir").value,
                                          get_package_share_directory("web_control")),
            skills_enable=bool(g("skills_enable").value),
            skills_allow_actions=self._skills_allow_actions,
            self_model_enable=bool(g("self_model_enable").value),
            self_model_path=(g("self_model_path").value or None),
            consolidate_every=int(g("consolidate_every").value),
            trait_history_enable=bool(g("trait_history_enable").value),
            trait_history_path=(g("trait_history_path").value or None),
            trait_history_period=float(g("trait_history_period").value),
            trait_history_max=int(g("trait_history_max").value),
            trait_history_window=float(g("trait_history_window").value),
            vision_diary_enable=bool(g("vision_diary_enable").value),
            vision_diary_path=(g("vision_diary_path").value or None),
            vision_diary_period=float(g("vision_diary_period").value),
            vision_diary_max=int(g("vision_diary_max").value),
            vision_diary_window=float(g("vision_diary_window").value),
            prelude_enable=bool(g("prelude_enable").value),
            camera_announce=bool(g("camera_announce").value),
            camera_face=str(g("camera_face").value or ""),
            workshop_enable=bool(g("workshop_enable").value),
            workshop_dir=(g("workshop_dir").value or ""),
            workshop_path=(g("workshop_path").value or None),
            workshop_rounds=int(g("workshop_rounds").value),
            workshop_min_runs=int(g("workshop_min_runs").value),
            workshop_retire_errors=int(g("workshop_retire_errors").value),
            workshop_retire_net_neg=int(g("workshop_retire_net_neg").value),
            workshop_adopt_quiet_runs=int(g("workshop_adopt_quiet_runs").value),
            workshop_trial_ttl=float(g("workshop_trial_ttl").value),
            workshop_trial_bias=float(g("workshop_trial_bias").value),
            skill_likes_path=(g("skill_likes_path").value or None),
            skill_like_bias=float(g("skill_like_bias").value),
            reflect_announce=bool(g("reflect_announce").value),
            quiet_start=float(g("quiet_start").value),
            quiet_end=float(g("quiet_end").value))
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
        # Tells the behaviour node whether the LLM is actually usable right now (enabled + a key
        # configured), so its idle-beat lottery can stop picking camera beats ("looking"/
        # "pursuing") when there's nothing to process the frame — a mere network blip is already
        # handled separately by the offline-face standdown below, which pauses ALL beats.
        self._llm_ready_pub = self.create_publisher(Bool, "cognition/llm_ready", latched)
        self._llm_ready = None            # None=not yet published
        self._update_llm_ready()
        self.create_subscription(String, "cognition/traits", self._on_traits, latched)
        # Brain controls from the web UI (the behaviour node consumes these): human reward
        # -> A/B bandit + reward shaping; reflection mode -> consolidation + skill forging.
        self._reward_pub = self.create_publisher(String, "cognition/reward", 10)
        self._reflect_pub = self.create_publisher(Bool, "reflect", latched)
        self._reflecting = False
        # The behaviour node can ask us to enter reflection mode on its own (long idle); we turn
        # that request into the same brain_reflect() the web toggle calls, so there's one path.
        self.create_subscription(Bool, "reflect_request", self._on_reflect_request, 10)
        # Sensor state for the "Observe" feature + the telemetry frame. IMU motion/tilt
        # comes from sys_monitor's vitals blob (read, not subscribed — see vitals());
        # only the event-driven pick-up switches remain topics here. CPU/RAM/temp come
        # from the same cheap /proc reads the announcer uses.
        self._vitals_cache = ({}, -1e9)
        self._susp_l = self._susp_r = False
        self._susp_override = -1        # /pickup_override: -1 auto, 0 grounded, 1 lifted
        self.create_subscription(Bool, "left_wheel_suspended", self._on_susp_l, 10)
        self.create_subscription(Bool, "right_wheel_suspended", self._on_susp_r, 10)
        # test override for the off-ground switches (latched so a restart mid-test still sees it)
        self.create_subscription(Int8, "pickup_override", self._on_pickup_override, latched)
        # Browser gateway (the rosbridge replacement): GET /telemetry SSE + POST
        # /publish + POST /param. Its browser-only subscriptions are lazy — created on
        # the first connected client, dropped after the last — so it costs ~nothing idle.
        self._system_pub = self.create_publisher(String, "oled_system", 5)
        self.telemetry = TelemetryHub(self, rate=float(g("telemetry_rate").value))
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
        self._vision_diary_tick()                      # visual diary (core rate-limits)

    # ---- HTTP teleop ---------------------------------------------------------
    def drive(self, data):
        """POST /drive {"v","w"}: clamp, publish /cmd_vel now, and arm the 10 Hz
        keepalive until the page stops refreshing (dead-man) or sends zero."""
        g = self.get_parameter
        max_lin = float(g("drive_max_lin").value)
        max_ang = float(g("drive_max_ang").value)
        try:
            # NOT tts.clamp — that one rounds to int, which would turn 0.2 m/s into 0.
            v = min(max_lin, max(-max_lin, float(data.get("v", 0.0))))
            w = min(max_ang, max(-max_ang, float(data.get("w", 0.0))))
        except (TypeError, ValueError):
            v = w = 0.0
        with self._drive_lock:
            self._drive_v, self._drive_w = v, w
            self._drive_at = time.monotonic() if (v or w) else 0.0
        self._publish_drive(v, w)
        return {"status": "ok", "v": v, "w": w}

    def _publish_drive(self, v, w):
        tw = Twist()
        tw.linear.x = float(v)
        tw.angular.z = float(w)
        self._drive_pub.publish(tw)

    def _drive_tick(self):
        """10 Hz: re-assert the active HTTP-teleop command (the ESP32 stops the motors
        if /cmd_vel goes stale) and dead-man-stop when the page vanishes mid-drive."""
        with self._drive_lock:
            if not self._drive_at:
                return
            stale = (time.monotonic() - self._drive_at
                     > float(self.get_parameter("drive_timeout").value))
            if stale:
                self._drive_v = self._drive_w = 0.0
                self._drive_at = 0.0
            v, w = self._drive_v, self._drive_w
        self._publish_drive(v, w)                      # a stale drive publishes one stop

    # ---- health-event log (written by sys_monitor, served for the web card) ----
    def get_health_log(self, limit=200):
        """Tail of the durable ESP32/LDS outage log — the first stop for diagnosing
        intermittent failures, now visible without an ssh session."""
        path = os.path.expanduser(self.get_parameter("health_log_path").value
                                  or "~/.local/state/nanobot/health.log")
        try:
            with open(path, "rb") as f:
                f.seek(0, os.SEEK_END)
                start = max(0, f.tell() - 64 * 1024)   # last 64 KB is plenty for a tail
                f.seek(start)
                lines = f.read().decode(errors="replace").splitlines()
        except OSError:
            return {"lines": [], "path": path}
        if start and lines:
            lines = lines[1:]                          # drop the line the seek cut in half
        return {"lines": lines[-int(limit):], "path": path}

    # ---- merged log stream: decision log + health log, interleaved by time ----
    def get_merged_log(self, limit=200):
        """Read-only merge of the two append-only event logs into one chronological
        stream: the decision log (cognition.py — LLM/beat/skill activity, kept in an
        in-memory ring buffer) and the health log (sys_monitor — ESP32/lidar outages,
        read fresh from disk). Doesn't touch either file — each stays single-writer,
        this just interleaves reads for the web UI's always-on Logs panel."""
        limit = int(limit)
        entries = []
        for e in self._cog.get_cog_log()["entries"]:
            d = dict(e)
            d["source"] = "cognition"
            entries.append(d)
        for line in self.get_health_log(limit=limit)["lines"]:
            t, text = _parse_health_line(line)
            entries.append({"t": t, "source": "health", "text": text})
        entries.sort(key=lambda e: e.get("t") or 0, reverse=True)
        return {"entries": entries[:limit]}

    # ---- stress test mode (see stress.py) ------------------------------------
    def stress_start(self, data):
        data = data or {}
        try:
            duration = float(data.get("duration", 30.0))
        except (TypeError, ValueError):
            duration = 30.0
        try:
            workers = int(data.get("workers", 0))
        except (TypeError, ValueError):
            workers = 0
        return self._stress.start(duration=duration, workers=workers)

    def stress_stop(self):
        return self._stress.stop()

    def stress_status(self):
        return self._stress.status()

    # ---- IMU mounting-interference self-test (see imu_interference.py) --------
    def imu_interference_start(self, data):
        data = data or {}
        return self._imu_test.start(include_motor=bool(data.get("include_motor", False)))

    def imu_interference_stop(self):
        return self._imu_test.stop()

    def imu_interference_status(self):
        return self._imu_test.status()

    # ---- persisted TTS settings ---------------------------------------------
    def _settings_file(self):
        p = self.get_parameter("tts_settings_path").value
        return p or os.path.expanduser("~/.local/state/nanobot/tts.json")

    def _load_settings(self):
        s = dict(SETTINGS_DEFAULTS)
        s["voice"] = self.get_parameter("tts_default_voice").value or "en-gb"
        saved = read_json(self._settings_file())       # no/invalid file -> defaults
        if isinstance(saved, dict):
            s.update({k: v for k, v in saved.items() if k in SETTINGS_DEFAULTS})
        return _sanitize_settings(s)

    def _save_settings(self):
        if not write_json(self._settings_file(), self._settings):
            self.get_logger().warning("tts: could not persist settings")

    def _apply_engine(self):
        """Push the current voice + markup levels into the TTS engine."""
        s = self._settings
        self._tts.configure(voice=s["voice"], volume=s["volume"],
                            speed=s["speed"], pitch=s["pitch"],
                            base_pitch=s["base_pitch"],
                            lead_silence=s["lead_silence"] / 1000.0)

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
            self._cpu_prev = procstats.cpu_sample()
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
        if self._cog.quiet_now():           # the periodic announcer respects quiet hours
            return                          # (POST /tts/announce — user-initiated — doesn't)
        self.announce_now()

    def announce_now(self):
        if not self._tts.available():
            return
        # Busy % since the previous sample (the gap between announcements).
        cpu, self._cpu_prev = procstats.cpu_percent(self._cpu_prev)
        text = procstats.compose_stats(cpu, procstats.mem_percent(), procstats.cpu_temp())
        if text:
            self._tts.say(text)

    # ---- LLM (OpenRouter) personality ---------------------------------------
    def _llm_settings_file(self):
        p = self.get_parameter("llm_settings_path").value
        if p:
            return os.path.expanduser(p)
        project_path = os.path.expanduser("~/Nano/memory/llm.json")
        if os.path.exists(project_path):
            return project_path
        return os.path.expanduser("~/.local/state/nanobot/llm.json")

    def _load_llm_settings(self):
        s = dict(LLM_DEFAULTS)
        for k, param in LLM_PARAM_FOR.items():           # robot.yaml params seed the defaults
            v = self.get_parameter(param).value
            if v is not None:
                s[k] = v
        saved = read_json(self._llm_settings_file())   # persisted UI changes win over params
        if isinstance(saved, dict):
            s.update({k: v for k, v in saved.items() if k in LLM_DEFAULTS})
        return _sanitize_llm_settings(s)

    def _load_persona(self):
        """Single source of the persona: personality.json (creator output, the same file
        the behaviour node seeds from), else the llm_persona param fallback."""
        path = (self.get_parameter("personality_path").value
                or os.path.expanduser("~/.local/state/nanobot/personality.json"))
        persona = (read_json(path) or {}).get("persona")
        if isinstance(persona, str) and persona.strip():
            return persona.strip()
        return self.get_parameter("llm_persona").value or ""

    def _load_persona_name(self):
        """The robot's name from personality.json (used in the phrase bank's {name}); 'Nano'
        as a fallback."""
        path = (self.get_parameter("personality_path").value
                or os.path.expanduser("~/.local/state/nanobot/personality.json"))
        name = (read_json(path) or {}).get("name")
        if isinstance(name, str) and name.strip():
            return name.strip()
        return "Nano"

    def _save_llm_settings(self, settings):
        """The core's `persist_settings` adapter: write the UI settings (enabled/model ids
        + the OpenRouter API key, if the user set one here) to llm.json so they survive a
        reboot. The key is never sent back to the browser (see cognition.get_llm_settings);
        llm.json lives under ~/.local/state, outside the git-tracked repo. Atomic write."""
        self._llm_settings = dict(settings)
        if not write_json(self._llm_settings_file(), self._llm_settings):
            self.get_logger().warning("llm: could not persist settings")

    # --- cognition-core delegators (the shared LLM brain lives in cognition.py) ----
    # These thin forwards are the node's public API for the HTTP handler; all the logic is in
    # CognitionCore, shared verbatim with scripts/dev_webui.py (one base to maintain).
    def llm_available(self):
        return self._cog.available()

    def get_llm_settings(self):
        return self._cog.get_llm_settings()

    def update_llm_settings(self, data):
        return self._cog.update_llm_settings(data)

    def get_personality(self):
        return self._cog.get_personality()

    def set_personality(self, data):
        """Web-UI edit of traits/registry/drives -> a HARD /cognition/evolve (set exactly +
        re-baseline in the behaviour node, see presence.apply_evolve) so it sticks instead of
        being smoothed/reverted like an LLM-reflection nudge. Applied server-side too (not just
        published) so a fast GET right after a POST already reflects it, even before the
        behaviour node's next tick round-trips back over /cognition/traits."""
        patch = sanitize_personality_patch(data if isinstance(data, dict) else {})
        if not patch:
            return {"error": "empty or invalid personality patch"}
        reg = None
        if patch.get("registry"):        # registry is a per-beat PATCH, not a full replacement
            reg = copy.deepcopy(self._cog.registry)
            for name, p in patch["registry"].items():
                reg.setdefault(name, {}).update(p)
        self._cog.update_personality(traits=patch.get("traits"), registry=reg,
                                      drives=patch.get("drives"))
        patch["hard"] = True
        self._evolve_pub.publish(String(data=json.dumps(patch)))
        return self.get_personality()

    def get_cog_log(self):
        return self._cog.get_cog_log()

    def get_vision_diary(self):
        return self._cog.get_vision_diary()

    def get_phrasebank(self):
        return self._cog.get_phrasebank()

    def regenerate_phrasebank(self):
        return self._cog.regenerate_phrasebank()

    def get_skills(self):
        return self._cog.get_skills()              # like counts merged in by the core

    def reload_skills(self):
        return self._cog.reload_skills()

    def invoke_skill(self, name):
        return self._cog.invoke_skill(name)

    def like_skill(self, data):
        """Adjust a skill's like count (a 👍 makes the brain favour it; repeatable). Body:
        {"name": "<skill>", "delta": ±1}. Persisted + biases the autonomous skill beat."""
        name = str((data or {}).get("name") or "").strip()
        if not name:
            return {"error": "empty name"}
        try:
            delta = int((data or {}).get("delta", 1))
        except (TypeError, ValueError):
            delta = 1
        return self._cog.like_skill(name, delta)

    def llm_say(self, prompt=""):
        return self._cog.llm_say(prompt)

    def llm_chat(self, message):
        return self._cog.llm_chat(message)

    def llm_observe(self):
        return self._cog.llm_observe()

    def llm_look(self):
        return self._cog.llm_look()

    # --- brain controls (reward + reflection mode; ROS-side, logged via the core) -
    def brain_reward(self, data):
        """Record a human reward and publish it for the behaviour node. Contextual reward
        (with a `target` echoed from /task_current) credits a specific A/B arm; global reward
        shapes the intrinsic-reward weights via the decision log on the next reflection.
        Skill reward (scope="skill", target=skill_name) is a 👍/👎 like on a specific skill —
        it adjusts the skill's like weight in the core (which biases the autonomous skill beat)."""
        try:
            value = clamp(float(data.get("value", 0)), -1, 1)
        except (TypeError, ValueError):
            value = 0.0
        scope = data.get("scope", "contextual")
        if scope not in ("global", "contextual", "skill"):
            scope = "contextual"
        target = data.get("target")
        # Skill like: scope="skill", target=skill_name (string). +1 favours it, -1 takes one back.
        if scope == "skill" and isinstance(target, str) and target.strip():
            res = self._cog.like_skill(target, 1 if value >= 0 else -1)
            return {"status": "ok", "value": value, "scope": scope, "target": target, **res}
        # Contextual/global reward (target is a dict for A/B experiments)
        target_dict = target if isinstance(target, dict) else None
        status = "up" if value > 0 else ("down" if value < 0 else "neutral")
        detail = scope
        if target_dict:
            detail += " " + json.dumps({k: target_dict.get(k) for k in ("exp", "variant", "task")})
        self._cog.log_decision("reward", status=status, detail=detail,
                               say=("👍" if value > 0 else "👎" if value < 0 else "·"))
        # Also credit a trial skill that just ran (the workshop's "happy user" signal): a
        # contextual 👍/👎 right after a skill helps it earn (or lose) permanence.
        if scope == "contextual":
            self._cog.reward_trial_skill(value)
        self._reward_pub.publish(String(data=json.dumps(
            {"value": value, "scope": scope, "target": target_dict})))
        return {"status": "ok", "value": value, "scope": scope}

    def brain_reflect(self, data):
        """Toggle reflection mode. Publishes /reflect for the behaviour node (calm face +
        paused beats + local purpose/A/B consolidation) and, on entry, kicks the LLM reflection,
        phrase-bank regeneration, and the skill workshop here so the whole brain consolidates +
        forges a skill at once."""
        on = bool(data.get("on"))
        self._reflecting = on
        self._cog.set_reflecting(on)               # so the core speaks self/forge conclusions
        self._cog.announce_reflect(on)             # say a short bookend line (turning inward / done)
        self._reflect_pub.publish(Bool(data=on))
        if on:
            self._reflect_next = time.monotonic()  # reflect now, then every <=60 s while on
            self._cog.bank_regen_check()           # refresh the phrase bank if the soul drifted
            self._cog.bank_grow_check()            # else grow it: add fresh offline lines (background)
            threading.Thread(target=self._cog.consolidate, daemon=True).start()  # long-term self
            # The skill workshop: mine experience -> mint/adapt a skill on trial (off-thread,
            # it makes several LLM calls). Sweeps the adopt/retire gate when it finishes.
            threading.Thread(target=self._cog.run_skill_workshop, daemon=True).start()
        self._cog.log_decision("reflect_mode", status=("on" if on else "off"))
        return {"status": "ok", "reflecting": on}

    def _on_reflect_request(self, msg: Bool):
        """The behaviour node asking us to enter/leave reflection mode autonomously (long idle).
        Edge-triggered so a repeated request is a no-op; routes through the same brain_reflect()
        the web toggle uses."""
        on = bool(msg.data)
        if on != self._reflecting:
            self.brain_reflect({"on": on})

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
        audio = bool(req.get("audio"))
        face = str(req.get("face") or "")              # the beat's action eye-shape (for the accent)
        if isinstance(req.get("traits"), dict):        # freshest personality snapshot
            self._cog.update_traits(req["traits"])
        trigger = "beat:" + beat
        if not self._cog.available():
            self._cog.log_decision(trigger, state, camera, status="llm-unavailable")
            return
        if beat == "skill":
            name = str(req.get("skill") or "")
            if name:                                    # a named request (e.g. a scheduled routine)
                threading.Thread(target=self._cog.invoke_skill, args=(name,), daemon=True).start()
            else:                                        # let the brain pick a skill to perform
                threading.Thread(target=self._cog.run_skill_beat,
                                 args=(state or "acting",), daemon=True).start()
            return
        threading.Thread(target=self._cog.run_beat,
                         args=(trigger, state, prompt, camera, audio, face), daemon=True).start()

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
        if getattr(self, "_mic", None) is None:
            return "no audio available"
        sink = self._mic.add_listener()
        chunks, deadline = [], time.monotonic() + max(0.1, float(listen))
        try:
            while time.monotonic() < deadline:
                try:
                    chunks.append(sink.get(timeout=0.2))
                except queue.Empty:
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
            self._cog.update_personality(traits=data.get("traits"), registry=data.get("registry"),
                                          drives=data.get("drives"))
            self._cog.bank_regen_check()                # soul moved -> refresh bank if too far

    # --- personality reflection (deep/slow tier): scheduled here, done by the core ----
    def _reflect_tick(self):
        if not (self.get_parameter("reflect_enable").value and self._cog.available()):
            return
        now = time.monotonic()
        picked = all(self._susp_eff())
        if picked and not self._was_picked:            # a notable event -> reflect soon
            self._reflect_next = min(self._reflect_next, now + 5.0)
        self._was_picked = picked
        if now < self._reflect_next or self._reflect_busy:
            return
        period = float(self.get_parameter("reflect_period").value)
        if self._reflecting:
            period = min(period, 60.0)             # consolidate faster while reflecting/charging
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
                {"traits": res["traits"], "registry": res["registry"],
                 "drives": res.get("drives", {})})))

    # --- sensor snapshot (the core's ROS-backed adapters) --------------------
    def vitals(self):
        """sys_monitor's aggregated body snapshot (/dev/shm/nano_vitals.json), with the
        per-source ages adjusted by the file's own staleness so a dead writer ages
        everything out naturally. Cached ~0.5 s; {} when absent/unreadable."""
        now = time.monotonic()
        cached, at = self._vitals_cache
        if now - at < 0.5:
            return cached
        v = {}
        try:
            with open(VITALS_FILE) as f:
                v = json.load(f)
            file_age = max(0.0, time.time() - float(v.get("t", 0)))
            for k in ("imu", "eul", "lds"):
                sec = v.get(k)
                if isinstance(sec, dict) and sec.get("age") is not None:
                    sec["age"] = round(float(sec["age"]) + file_age, 2)
            esp = v.get("esp")
            if isinstance(esp, dict):
                for k in ("hb_age", "temp_age"):
                    if esp.get(k) is not None:
                        esp[k] = round(float(esp[k]) + file_age, 2)
        except (OSError, ValueError, TypeError):
            v = {}
        self._vitals_cache = (v, now)
        return v

    def _on_susp_l(self, msg: Bool):
        self._susp_l = bool(msg.data)

    def _on_susp_r(self, msg: Bool):
        self._susp_r = bool(msg.data)

    def _on_pickup_override(self, msg: Int8):
        v = int(msg.data)
        self._susp_override = v if v in (0, 1) else -1

    def _susp_eff(self):
        """Effective off-ground switch pair, honoring the /pickup_override test hook."""
        if self._susp_override == 0:
            return False, False
        if self._susp_override == 1:
            return True, True
        return self._susp_l, self._susp_r

    def _cpu_percent_quick(self):
        """A short standalone CPU% sample for the snapshot — does NOT touch _cpu_prev
        (which belongs to the periodic stats announcer), so the two never interfere."""
        a = procstats.cpu_sample()
        time.sleep(0.12)
        pct, _ = procstats.cpu_percent(a)
        return pct

    def _imu_state(self):
        """(moving, tilt) from the vitals blob — either may be None when the source is
        missing/stale (>4 s: one vitals write period + margin)."""
        v = self.vitals()
        moving = tilt = None
        imu = v.get("imu") or {}
        if imu.get("age", 1e9) < 4.0 and imu.get("g") is not None:
            moving = imu["g"] > 0.3 or abs((imu.get("a") or 9.81) - 9.81) > 1.5
        eul = v.get("eul") or {}
        if eul.get("age", 1e9) < 4.0 and eul.get("r") is not None:
            tilt = max(abs(eul["r"]), abs(eul.get("p") or 0.0))
        return moving, tilt

    def _sensor_snapshot(self):
        """A short plain-English description of how the robot's body feels right now,
        for the LLM to react to. Only includes sources that are present + fresh."""
        parts = []
        cpu, mem, temp = self._cpu_percent_quick(), procstats.mem_percent(), procstats.cpu_temp()
        if cpu == cpu:                                  # not NaN
            parts.append(f"CPU load {cpu:.0f}%")
        if mem == mem:
            parts.append(f"memory {mem:.0f}% used")
        if temp == temp:
            parts.append(f"main board {temp:.0f} degrees C")
        moving, tilt = self._imu_state()
        if moving is not None:                          # moving / being jostled?
            parts.append("being moved or jostled" if moving else "physically still")
        if tilt is not None:                            # tilt from roll/pitch
            if tilt > 25:
                parts.append(f"tilted over at about {tilt:.0f} degrees")
            elif tilt > 10:
                parts.append(f"leaning slightly ({tilt:.0f} degrees)")
            else:
                parts.append("sitting level")
        susp_l, susp_r = self._susp_eff()
        if susp_l and susp_r:                           # pick-up
            parts.append("lifted off the ground (being held)")
        elif susp_l or susp_r:
            parts.append("with one wheel off the ground")
        else:
            parts.append("resting on the ground")
        return ", ".join(parts) if parts else "no sensor data available"

    def _sensor_signals(self):
        """The same body state as _sensor_snapshot(), but structured for the phrase bank's
        classifier (NaN/None where a source is missing or stale)."""
        cpu, mem, temp = self._cpu_percent_quick(), procstats.mem_percent(), procstats.cpu_temp()
        moving, tilt = self._imu_state()
        susp_l, susp_r = self._susp_eff()
        pickup = 2 if (susp_l and susp_r) else (1 if (susp_l or susp_r) else 0)
        return {"cpu": cpu, "mem": mem, "temp": temp, "moving": moving,
                "tilt": tilt, "pickup": pickup}

    @property
    def _cam(self):
        """The active browser-facing camera backend: None if the master camera switch
        is off (see set_camera_enable below -- every existing caller, _capture_frame/
        _stream_mjpeg/the mask streams, already treats a None/no-camera backend as
        "503, no camera", so this needed no new handling downstream); otherwise the
        direct hardware-MJPEG passthrough (CameraStream, zero CPU/GPU cost) when GPU
        vision is off/unavailable, else GpuVision's JPEG tee. Both `_cam_direct` and
        `_gpu_vision` share the same add_viewer/get_frame/remove_viewer/running shape,
        so every existing caller needed zero other changes."""
        if self._camera_disabled:
            return None
        if self._gpu_vision is None:
            return self._cam_direct
        return self._gpu_vision

    def set_camera_enable(self, d):
        """POST /vision/camera_enable {enabled: bool}: master on/off for ALL camera
        processing (GPU vision AND the direct passthrough), live, no restart needed --
        for when the extra cheap-tier vision passes' CPU/GPU cost isn't wanted right
        now (see gpu-vision-implemented memory's gpu_duty finding: the fuller pass set
        can run 60-190% of the frame budget per tick). Disabling stops GpuVision's
        capture thread entirely if it was running (releasing the V4L2 device);
        re-enabling resumes it."""
        want_enabled = bool(d.get("enabled", True))
        if want_enabled == (not self._camera_disabled):
            return {"ok": True, "camera_enabled": not self._camera_disabled}
        if not want_enabled:
            if self._oled_mask_on:          # no live mask to mirror while capture is off
                self._set_oled_mask_state(False)
            if self._gpu_vision is not None:
                self._gpu_vision.stop()     # release the V4L2 device
            self._camera_disabled = True
        else:
            self._camera_disabled = False
            if self._gpu_vision is not None:
                self._gpu_vision.start()    # reacquire + resume PIR/blob/luma/dark-reflex
        return {"ok": True, "camera_enabled": not self._camera_disabled}

    def _dark_reflex_tick(self):
        """Flashlight/dark reflex: auto-toggle /led from the GPU's average frame
        luminance, with hysteresis (dark_threshold to turn on, dark_recover > threshold
        to turn back off) so it doesn't flicker right at the boundary. Reads
        vision_dark_reflex_enable LIVE (not just at startup) so the web UI's toggle
        actually takes effect -- the timer always runs while GPU vision is on, cheap
        either way (one param read + one property read per second)."""
        if self._gpu_vision is None or self._dark_led_pub is None:
            return
        # The master camera-disable switch stops GpuVision entirely (see
        # set_camera_enable) -- .luma would be frozen at whatever it last was, not a
        # live reading. Treat it the same as "disabled": release the LED rather than
        # act on stale data.
        if (not self.get_parameter("vision_dark_reflex_enable").value
                or self._camera_disabled):
            if self._dark_led_on:            # was on when disabled -- don't leave it stuck
                self._dark_led_on = False
                self._dark_led_pub.publish(Bool(data=False))
            return
        luma = self._gpu_vision.luma
        low = self.get_parameter("vision_dark_threshold").value
        high = self.get_parameter("vision_dark_recover").value
        if high <= low:
            # The web UI's slider JS keeps recover > threshold, but /param can be hit
            # directly (bypassing that) -- an inverted or equal band has no stable
            # luma range and oscillates every tick. Widen it server-side rather than
            # trust the caller (confirmed by testing: this really does oscillate).
            high = low + 0.05
        if not self._dark_led_on and luma < low:
            self._dark_led_on = True
            self._dark_led_pub.publish(Bool(data=True))
        elif self._dark_led_on and luma > high:
            self._dark_led_on = False
            self._dark_led_pub.publish(Bool(data=False))

    def _vision_state_tick(self):
        """2 Hz vision->behaviour feed (only scheduled when GPU vision is on). Pushes
        the live vision_glare_derate param into the GL thread, computes the
        anticipatory-approach signal (motion growing fast AND centred = someone/
        something coming toward the lens -- the greeting reflex's trigger, cheaper and
        earlier than waiting for pickup), and publishes the compact /vision/state JSON
        the behaviour node's reflexes consume. Nothing is published while the pipeline
        is frozen (camera off), so mood_node's freshness window naturally stands the
        vision reflexes down."""
        gv = self._gpu_vision
        if gv is None:
            return
        g = self.get_parameter
        gv.set_glare_derate(g("vision_glare_derate").value)
        if self._camera_disabled or not gv.running():
            self._vision_approach = False
            return
        rate = gv.motion_intercept_rate
        center = gv.motion_center
        self._vision_approach = (
            rate > g("vision_approach_rate").value
            and center is not None
            and abs(center[0] - 0.5) < g("vision_approach_band").value
            and gv.motion_score > 0.02)
        cast = gv.color_cast
        payload = {
            "approach": self._vision_approach,
            "looming": gv.motion_intercept_rate > g("vision_looming_alert").value,
            "clutter": gv.edge_density > g("vision_clutter_alert").value,
            "novelty": round(gv.novelty, 3),
            # warmth: R-B of the scene's average colour -- positive = warm (evening
            # lamps), negative = cool (daylight/fluorescent). The ambient-mood input.
            "warmth": round(cast[0] - cast[2], 3) if cast else 0.0,
            "motion": round(gv.motion_score, 3),
            # calibrated colour-blob target, for slam_nav's optional pan-tracking:
            # [x, y, confidence] normalized 0..1 image coords, or None if no lock.
            "target": list(gv.target) if gv.target is not None else None,
        }
        self._vision_state_pub.publish(String(data=json.dumps(payload)))

    def _vision_diary_tick(self):
        """1 Hz (piggybacked on _publish_ping): offer the current scene scalars to the
        cognition core's visual diary; the core itself rate-limits to
        vision_diary_period, so this is a couple of property reads when not due."""
        gv = self._gpu_vision
        if gv is None or self._camera_disabled or not gv.running():
            return
        cast = gv.color_cast
        self._cog.record_vision_snapshot({
            "luma": gv.luma, "motion": gv.motion_score, "edge": gv.edge_density,
            "novelty": gv.novelty, "warmth": (cast[0] - cast[2]) if cast else 0.0})

    def _set_oled_mask_state(self, enabled):
        self._oled_mask_on = bool(enabled)
        if self._gpu_vision is not None:
            self._gpu_vision.set_oled_mask(self._oled_mask_on)
        if self._oled_mask_pub is not None:
            self._oled_mask_pub.publish(Bool(data=self._oled_mask_on))

    def set_oled_mask(self, d):
        """POST /vision/oled_mask {enabled: bool}: mirror the colour-tracking mask to
        the physical OLED. web_control owns the arbitration signal (the latched
        /oled_mask Bool) and gpu_vision writes the 128x64 blob; oled_display renders it
        above the face but below reflection mode / spoken words / shutdown screens --
        the same yield-the-panel model as every other owner."""
        if self._gpu_vision is None:
            return {"error": "gpu vision not enabled"}
        want = bool((d or {}).get("enabled"))
        if want and not self._gpu_vision.has_target_color:
            return {"error": "no target colour calibrated -- nothing to mirror"}
        self._set_oled_mask_state(want)
        return {"ok": True, "oled_mask": self._oled_mask_on}

    # ---- named colour-target palette ------------------------------------------
    # Calibrations are stored under a NAME ("ball", "dock marker", ...) in a small
    # persisted JSON file, one active at a time -- so a target survives a stack
    # restart, and skills/the schedule/the UI can re-select one without re-calibrating.
    # The GPU still tracks exactly ONE colour at a time (selection, not simultaneous
    # multi-target -- the dock-approach/play use cases want "pick which", and one pass
    # keeps the per-frame cost unchanged).
    def _load_vision_targets(self):
        """Read the palette + re-apply the persisted active target into GpuVision."""
        data = read_json(self._vision_targets_path)
        if isinstance(data, dict) and isinstance(data.get("targets"), dict):
            self._vision_targets = {str(k)[:32]: v for k, v in data["targets"].items()
                                    if isinstance(v, dict)}
            active = data.get("active")
            if active in self._vision_targets:
                self._apply_vision_target(active)

    def _save_vision_targets(self):
        if not write_json(self._vision_targets_path,
                          {"active": self._vision_target_active,
                           "targets": self._vision_targets}):
            self.get_logger().warning("vision targets: save failed")

    def _apply_vision_target(self, name):
        """Push a stored target's colour + blob tuning into the GL thread and mark it
        active. set_target_color resets the tuning, so the stored tuning goes second."""
        t = self._vision_targets[name]
        try:
            rgb = tuple(max(0.0, min(1.0, float(t[k]))) for k in ("r", "g", "b"))
        except (KeyError, TypeError, ValueError):
            return False
        self._gpu_vision.set_target_color(rgb, float(t.get("threshold", 0.22)))
        self._gpu_vision.set_blob_tuning(
            min_confidence=t.get("min_confidence"), max_confidence=t.get("max_confidence"))
        self._vision_target_active = name
        return True

    def get_vision_targets(self):
        """GET /vision/targets: the palette + which one is live."""
        return {"targets": self._vision_targets, "active": self._vision_target_active}

    def vision_target_select(self, d):
        """POST /vision/target_select {name}: make a stored calibration the live one."""
        if self._gpu_vision is None:
            return {"error": "gpu vision not enabled"}
        name = str((d or {}).get("name") or "").strip()[:32]
        if name not in self._vision_targets:
            return {"error": f"no stored target named '{name}'"}
        if not self._apply_vision_target(name):
            return {"error": f"stored target '{name}' is malformed"}
        self._save_vision_targets()
        t = self._vision_targets[name]
        return {"ok": True, "active": name, "target": t}

    def vision_target_delete(self, d):
        """POST /vision/target_delete {name}: forget a stored calibration. Deleting the
        active one also stops tracking (there's nothing meaningful to fall back to)."""
        name = str((d or {}).get("name") or "").strip()[:32]
        if name not in self._vision_targets:
            return {"error": f"no stored target named '{name}'"}
        del self._vision_targets[name]
        if self._vision_target_active == name:
            self._vision_target_active = None
            if self._gpu_vision is not None:
                self._gpu_vision.set_target_color(None)
        self._save_vision_targets()
        return {"ok": True, "active": self._vision_target_active}

    def vision_calibrate(self, d):
        """POST /vision/calibrate: set or clear the GPU blob-tracker's target colour.
        Body: {r,g,b (0..1), threshold?, name?} to set, or {clear:true} to disable
        tracking (the stored palette is kept). The browser samples the pixel colour
        itself (canvas getImageData on the live <img>) and posts the RGB value directly
        -- no server-side coordinate mapping needed. `name` (default "default") stores
        the calibration in the named palette, persisted across restarts."""
        if self._gpu_vision is None:
            return {"error": "gpu vision not enabled"}
        if d.get("clear"):
            self._gpu_vision.set_target_color(None)
            self._vision_target_active = None
            self._save_vision_targets()
            return {"ok": True, "target_color": None}
        try:
            rgb = tuple(max(0.0, min(1.0, float(d[k]))) for k in ("r", "g", "b"))
        except (KeyError, TypeError, ValueError):
            return {"error": "bad color"}
        try:
            threshold = max(0.02, min(1.0, float(d.get("threshold", 0.22))))
        except (TypeError, ValueError):
            threshold = 0.22
        name = str(d.get("name") or "default").strip()[:32] or "default"
        self._gpu_vision.set_target_color(rgb, threshold)
        self._vision_targets[name] = {"r": rgb[0], "g": rgb[1], "b": rgb[2],
                                      "threshold": threshold,
                                      "min_confidence": 0.0, "max_confidence": 1.0}
        self._vision_target_active = name
        self._save_vision_targets()
        return {"ok": True, "target_color": list(rgb), "threshold": threshold,
                "name": name}

    def vision_blob_tune(self, d):
        """POST /vision/blob_tune: adjust colour-match sensitivity and blob-size gating
        WITHOUT re-picking the target colour (unlike /vision/calibrate, which sets the
        colour itself). Body: any of {threshold, min_confidence, max_confidence} (each
        0..1) -- omitted fields keep their current value. min/max_confidence gate the
        matched-fraction-of-frame range that counts as a valid lock: min rejects noise
        (a couple of stray matching pixels), max rejects "matched almost the whole
        frame" false locks (e.g. a colour that also matches a wall/background)."""
        if self._gpu_vision is None:
            return {"error": "gpu vision not enabled"}

        def opt_float(key):
            if key not in d:
                return None
            try:
                return float(d[key])
            except (TypeError, ValueError):
                return None

        self._gpu_vision.set_blob_tuning(
            threshold=opt_float("threshold"),
            min_confidence=opt_float("min_confidence"),
            max_confidence=opt_float("max_confidence"))
        threshold, min_conf, max_conf = self._gpu_vision.blob_tuning
        # Keep the active named target's stored tuning in sync, so re-selecting it
        # later (or a restart) restores the tuned values, not the pick-time defaults.
        active = self._vision_target_active
        if active in self._vision_targets:
            self._vision_targets[active].update(
                threshold=threshold, min_confidence=min_conf, max_confidence=max_conf)
            self._save_vision_targets()
        return {"ok": True, "threshold": threshold, "min_confidence": min_conf,
                "max_confidence": max_conf}

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

    def snapshot(self):
        """One JPEG from the webcam for GET /snapshot.jpg (ref-counted capture)."""
        return self._capture_frame()

    # --- lifecycle speech (offline-safe via the phrase bank, done by the core) ----
    def system_announce(self, action):
        """Speak the matching farewell/restart line just before a stack/board action, and
        flip the OLED to its end-screen immediately (the page used to publish /oled_system
        over rosbridge for this; now the server owns it)."""
        if action in ("restart", "reboot", "shutdown"):
            self._system_pub.publish(String(data=action))
        cat = "farewell" if action in ("shutdown", "poweroff") else "restarting"
        return self._cog.speak_lifecycle(cat)

    def _update_llm_ready(self):
        """Publish (on change) whether the LLM is enabled + keyed right now — see the
        cognition/llm_ready publisher above. Latched, so a behaviour node that (re)starts later
        still picks up the current value immediately."""
        ready = self._cog.available()
        if ready != self._llm_ready:
            self._llm_ready = ready
            self._llm_ready_pub.publish(Bool(data=ready))

    def _llm_health_tick(self):
        """Persistent 'AI offline' indicator: when the LLM is enabled but unreachable, show
        the offline face + speak the offline line once, and clear when it recovers. Edge-
        triggered, with a slow re-assert so a transient TTS word / manual mood doesn't lose
        the face. Counts repeated real-call failures too, so a network drop (key present) also
        trips it — not just a missing key. The behaviour node stands down on the foreign face,
        so its idle beats pause while we're offline."""
        self._update_llm_ready()
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

    def destroy_node(self):
        try:
            self._stress.stop()
        except Exception:
            pass
        try:
            self._imu_test.stop()
        except Exception:
            pass
        if self._gpu_vision is not None:
            try:
                self._gpu_vision.stop()
            except Exception:
                pass
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
    out["base_pitch"] = clamp(out["base_pitch"], 0, 99)
    out["lead_silence"] = clamp(out["lead_silence"], 0, 2000)
    out["announce"] = bool(out["announce"])
    out["announce_interval"] = clamp(out["announce_interval"], ANNOUNCE_MIN, ANNOUNCE_MAX)
    return out


def _sanitize_llm_settings(s):
    """Coerce/clamp the LLM settings dict to safe types (UI + on-disk file untrusted)."""
    out = dict(LLM_DEFAULTS)
    out.update(s)
    out["enabled"] = bool(out["enabled"])
    for k in ("model", "smart_model", "vision_model", "vision_fallback_model",
              "free_model", "free_smart_model"):
        out[k] = str(out.get(k) or "")[:120]
    out["api_key"] = str(out.get("api_key") or "").strip()[:200]
    return out


def _parse_health_line(line):
    """Parse a health-log line ("YYYY-MM-DDTHH:MM:SS message", written by
    sys_monitor.health_log) into (epoch_seconds, message). Best-effort: an
    unparseable line still gets served (t=0.0 sorts it oldest) rather than dropped."""
    try:
        ts, msg = line.split(" ", 1)
        return datetime.fromisoformat(ts).timestamp(), msg
    except (ValueError, IndexError):
        return 0.0, line


class _Handler(http.server.SimpleHTTPRequestHandler):
    # HTTP/1.1 so the browser reuses ONE keep-alive connection for the frequent
    # /map + /scan.bin + config polls instead of a fresh TCP connect (and a fresh
    # ThreadingHTTPServer thread) per request — the cheapest steady-state win on the
    # 1 GB SBC. Every non-streaming response below sends Content-Length, so keep-alive
    # framing is well-defined; the streaming handlers opt back out with close_connection.
    # `timeout` reaps an idle kept-alive socket so abandoned connections can't pin a thread.
    protocol_version = "HTTP/1.1"
    timeout = 30

    def __init__(self, *args, stream=None, audio=None, tts=None, node=None, **kwargs):
        self._stream = stream
        self._audio = audio
        self._tts = tts
        self._node = node
        super().__init__(*args, **kwargs)

    # ---- route tables ---------------------------------------------------------
    # Plain JSON endpoints dispatch through these (path -> node call); endpoints that
    # validate input, speak, stream, or act on the OS keep explicit branches below.
    # The node is never None in production (main() always passes it) — the 503 guard
    # is belt-and-braces for tests.
    GET_JSON = {
        "/tts/config": lambda n: n.get_settings(),      # page restores controls on load
        "/llm/config": lambda n: n.get_llm_settings(),
        "/personality": lambda n: n.get_personality(),
        "/llm/log": lambda n: n.get_cog_log(),
        "/llm/phrases": lambda n: n.get_phrasebank(),
        "/skills": lambda n: n.get_skills(),
        "/skills/workshop": lambda n: n.brain_workshop(),
        "/health/log": lambda n: n.get_health_log(),
        "/logs": lambda n: n.get_merged_log(),
        "/stress/status": lambda n: n.stress_status(),
        "/imu/interference/status": lambda n: n.imu_interference_status(),
        "/vision/targets": lambda n: n.get_vision_targets(),
        "/llm/vision_diary": lambda n: n.get_vision_diary(),
    }
    POST_JSON = {
        "/drive": lambda n, d: n.drive(d),              # hot path: ~10 Hz while driving
        "/publish": lambda n, d: n.telemetry.publish_json(d),   # whitelisted topic pokes
        "/param": lambda n, d: n.telemetry.set_param_json(d),   # whitelisted live-tune params
        "/stress/start": lambda n, d: n.stress_start(d),
        "/stress/stop": lambda n, d: n.stress_stop(),
        "/imu/interference/start": lambda n, d: n.imu_interference_start(d),
        "/imu/interference/stop": lambda n, d: n.imu_interference_stop(),
        "/tts/config": lambda n, d: n.update_settings(d),
        "/llm/config": lambda n, d: n.update_llm_settings(d),
        "/personality": lambda n, d: n.set_personality(d),
        "/llm/phrases/regenerate": lambda n, d: n.regenerate_phrasebank(),
        "/skills/reload": lambda n, d: n.reload_skills(),
        "/skills/like": lambda n, d: n.like_skill(d),   # {"name","delta":±1}
        "/skills/workshop/keep": lambda n, d: n.workshop_keep(d),
        "/skills/workshop/kill": lambda n, d: n.workshop_kill(d),
        "/brain/reward": lambda n, d: n.brain_reward(d),
        "/brain/reflect": lambda n, d: n.brain_reflect(d),
        "/vision/calibrate": lambda n, d: n.vision_calibrate(d),
        "/vision/blob_tune": lambda n, d: n.vision_blob_tune(d),
        "/vision/camera_enable": lambda n, d: n.set_camera_enable(d),
        "/vision/target_select": lambda n, d: n.vision_target_select(d),
        "/vision/target_delete": lambda n, d: n.vision_target_delete(d),
        "/vision/oled_mask": lambda n, d: n.set_oled_mask(d),
    }
    # LLM generation endpoints: all gated on llm_available(), all blocking on the
    # OpenRouter call (handler thread), all replying {say,mood} or an error.
    POST_LLM = {
        "/llm/say": lambda n, d: n.llm_say(d.get("prompt") or ""),
        "/llm/chat": lambda n, d: n.llm_chat((d.get("message") or "").strip()),
        "/llm/observe": lambda n, d: n.llm_observe(),
        "/llm/look": lambda n, d: n.llm_look(),
    }
    # /system/*: (OLED end-screen hint, spoken line, detached command, HTTP reply).
    # Detached + new session so the command survives do_down killing this very web server;
    # do_POST below blocks on tts.wait() until the spoken line actually finishes (lines vary
    # in length, so a fixed delay could cut one off) before firing it, with a short flush
    # delay after. Reboot/poweroff need the scoped NOPASSWD sudo rule for systemctl (sbc-setup.sh).
    POST_SYSTEM = {
        "/system/restart": ("restart", "restart",
                            'bash "$HOME/Nano/scripts/stack.sh" restart',
                            "restarting stack"),
        "/system/reboot": ("reboot", "reboot",
                           "sudo -n /usr/bin/systemctl reboot", "rebooting"),
        "/system/shutdown": ("shutdown", "shutdown",
                             "sudo -n /usr/bin/systemctl poweroff", "shutting down"),
    }

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        route = self.GET_JSON.get(path)
        if route:
            if self._node is None:
                return self._respond(503, "no node")
            return self._respond_json(route(self._node))
        if path == "/telemetry":
            return self._stream_telemetry()
        if path == "/snapshot.jpg":
            return self._serve_snapshot()
        if path == "/stream.mjpg":
            return self._stream_mjpeg()
        if path == "/stream_mask.mjpg":
            return self._stream_mask_mjpeg()
        if path == "/stream_motion_mask.mjpg":
            return self._stream_motion_mask_mjpeg()
        if path == "/audio.pcm":
            return self._stream_audio()
        if path == "/map":
            return self._serve_map()
        if path == "/scan.bin":
            return self._serve_scan()
        return super().do_GET()

    def do_POST(self):
        path = self.path.split("?", 1)[0]
        # Consume the body up front so it's always drained — a kept-alive connection
        # would otherwise desync on endpoints that ignore the body (or on a payload
        # larger than _read_json would have read).
        self._body = self._read_body()
        if self._node is None:
            return self._respond(503, "no node")
        route = self.POST_JSON.get(path)
        if route:
            return self._respond_json(route(self._node, self._read_json()))
        route = self.POST_LLM.get(path)
        if route:
            if not self._node.llm_available():
                return self._respond(503, "llm unavailable")
            data = self._read_json()
            if path == "/llm/chat" and not (data.get("message") or "").strip():
                return self._respond(400, "empty message")
            return self._respond_json(route(self._node, data) or {"error": "no reply"})
        system = self.POST_SYSTEM.get(path)
        if system:
            oled, line, cmd, reply = system
            self._set_oled_action(oled)        # which end-screen the OLED shows
            self._node.system_announce(line)   # speak the farewell/restart line first
            if self._tts is not None:
                # Wait for the actual utterance to finish (length varies, so a fixed delay
                # can cut it off mid-sentence) before firing the action; bounded so a stuck
                # audio pipeline can't block shutdown/reboot/restart forever.
                self._tts.wait(timeout=10)
            self._run_detached(cmd, delay=1)   # just enough for the HTTP response to flush
            return self._respond(200, reply)
        if path == "/tts":
            # Speak a line and karaoke its words to the OLED. Body: {"text","voice"?}.
            data = self._read_json()
            text = (data.get("text") or "").strip()
            if self._tts is None or not self._tts.available():
                return self._respond(503, "tts unavailable")
            if not text:
                return self._respond(400, "empty text")
            self._tts.say(text, voice=(data.get("voice") or "").strip() or None)
            return self._respond(200, "speaking")
        if path == "/tts/announce":
            # Speak the system stats once, right now (independent of the periodic toggle).
            if self._tts is None or not self._tts.available():
                return self._respond(503, "tts unavailable")
            self._node.announce_now()
            return self._respond(200, "announcing")
        if path == "/tts/stop":
            if self._tts is not None:
                self._tts.stop()
            return self._respond(200, "stopped")
        if path == "/skills/invoke":
            # Run one skill from the library now: {"name"}. Blocks on any LLM call.
            name = (self._read_json().get("name") or "").strip()
            if not name:
                return self._respond(400, "empty name")
            return self._respond_json(self._node.invoke_skill(name))
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

    def _read_body(self):
        """Read and FULLY consume the request body (keeping the kept-alive connection in
        sync). A body over MAX_BODY is drained but dropped, so a bad/huge payload can't
        buffer megabytes or leave the socket mid-message."""
        try:
            n = int(self.headers.get("Content-Length", 0) or 0)
        except ValueError:
            return b""
        if n <= 0:
            return b""
        if n <= MAX_BODY:
            return self.rfile.read(n)
        remaining = n                                  # too big: drain to resync, keep nothing
        while remaining > 0:
            chunk = self.rfile.read(min(remaining, 65536))
            if not chunk:
                break
            remaining -= len(chunk)
        return b""

    def _read_json(self):
        """Parse the already-read request body as JSON; {} on any problem."""
        try:
            return json.loads(getattr(self, "_body", b"") or b"{}")
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
        # Resolve the active camera backend ONCE, at the start of this streaming
        # session, not on every call -- `node._cam` is a property that can swap
        # between GpuVision and the direct hardware-MJPEG CameraStream (GPU vision
        # off/unavailable, see WebServerNode._cam). Pinning it here means a mid-stream
        # toggle takes effect on the NEXT connection, not this already-open one --
        # avoids split-brain viewer-count accounting between two different backend
        # objects.
        cam = self._node._cam if self._node is not None else self._stream
        if cam is None:
            self.send_error(503, "no camera")
            return
        cam.add_viewer()
        self.close_connection = True   # never-ending multipart body: no keep-alive
        try:
            self.send_response(200)
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=FRAME")
            self.end_headers()
            seq = 0
            while True:
                seq, jpeg = cam.get_frame(seq, timeout=5.0)
                if jpeg is None:
                    if not cam.running():
                        break          # camera failed / no device
                    continue
                self.wfile.write(b"--FRAME\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(b"Content-Length: %d\r\n\r\n" % len(jpeg))
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
        except OSError:
            pass                       # client closed the stream / write timed out
        finally:
            cam.remove_viewer()

    def _stream_mask_mjpeg(self):
        """GET /stream_mask.mjpg: the live colour-threshold mask (white = matches the
        calibrated target colour, black = doesn't) instead of the normal camera feed --
        "where is the tracked colour actually showing up." Only meaningful while GPU
        vision owns the camera AND a target colour is calibrated; both are checked up
        front so this fails fast with a clear reason instead of hanging on a mask
        frame that will never be computed."""
        gv = getattr(self._node, "_gpu_vision", None) if self._node is not None else None
        if gv is None:
            self.send_error(503, "gpu vision not active")
            return
        if not gv.has_target_color:
            self.send_error(503, "no target colour set -- pick one first")
            return
        gv.add_mask_viewer()
        self.close_connection = True
        try:
            self.send_response(200)
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=FRAME")
            self.end_headers()
            seq = 0
            while True:
                seq, jpeg = gv.get_mask_frame(seq, timeout=5.0)
                if jpeg is None:
                    if not gv.running() or not gv.has_target_color:
                        break     # camera failed, or the target was cleared mid-stream
                    continue
                self.wfile.write(b"--FRAME\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(b"Content-Length: %d\r\n\r\n" % len(jpeg))
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
        except OSError:
            pass
        finally:
            gv.remove_mask_viewer()

    def _stream_motion_mask_mjpeg(self):
        """GET /stream_motion_mask.mjpg: the live PIR/motion-diff mask (brighter = more
        change since the last frame) instead of the normal camera feed -- "where is
        something actually moving," as opposed to the colour mask's "where is the
        tracked colour." Only meaningful while GPU vision owns the camera; unlike the
        colour mask, no target colour needs to be set -- motion diff runs
        unconditionally once a second frame has arrived."""
        gv = getattr(self._node, "_gpu_vision", None) if self._node is not None else None
        if gv is None:
            self.send_error(503, "gpu vision not active")
            return
        gv.add_motion_mask_viewer()
        self.close_connection = True
        try:
            self.send_response(200)
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=FRAME")
            self.end_headers()
            seq = 0
            while True:
                seq, jpeg = gv.get_motion_mask_frame(seq, timeout=5.0)
                if jpeg is None:
                    if not gv.running():
                        break     # camera failed
                    continue
                self.wfile.write(b"--FRAME\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(b"Content-Length: %d\r\n\r\n" % len(jpeg))
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
        except OSError:
            pass
        finally:
            gv.remove_motion_mask_viewer()

    def _stream_telemetry(self):
        """GET /telemetry: Server-Sent Events. One JSON frame per telemetry tick, shared
        across all clients (built once in TelemetryHub). Chunked like /audio.pcm so the
        browser surfaces events live; a comment line keeps idle connections alive."""
        if self._node is None:
            self.send_error(503, "no node")
            return
        hub = self._node.telemetry
        hub.add_client()
        self.close_connection = True   # never-ending stream: no keep-alive reuse
        try:
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Transfer-Encoding", "chunked")
            self.end_headers()
            seq = 0
            while True:
                new_seq, frame = hub.wait_frame(seq, timeout=5.0)
                if new_seq == seq:
                    chunk = b": ping\n\n"              # SSE comment = keepalive
                else:
                    seq = new_seq
                    chunk = b"data: " + frame + b"\n\n"
                self.wfile.write(b"%X\r\n" % len(chunk))
                self.wfile.write(chunk)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
        except OSError:
            pass                       # client left / write timed out
        finally:
            hub.remove_client()

    def _stream_audio(self):
        if self._audio is None:
            self.send_error(503, "no microphone")
            return
        q = self._audio.add_listener()
        self.close_connection = True   # live mic stream runs until the client leaves
        try:
            # Stream as HTTP/1.1 chunked. Browsers buffer a close-delimited streaming
            # body and never hand it to fetch()'s reader until the connection closes —
            # which for a live mic is never — so without chunked the page would receive
            # nothing. Chunked is surfaced live. (protocol_version is HTTP/1.1 class-wide.)
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
            while True:
                try:
                    data = q.get(timeout=5.0)
                except queue.Empty:
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
        except OSError:
            pass                       # client stopped listening / write timed out
        finally:
            self._audio.remove_listener(q)

    def _serve_snapshot(self):
        # One still JPEG from the webcam (starts the camera if nobody's streaming,
        # stops it after) — for the 📸 button / quick checks without the MJPEG stream.
        jpeg = self._node.snapshot() if self._node else None
        if not jpeg:
            self.send_error(503, "no camera")
            return
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(jpeg)))
        self.end_headers()
        try:
            self.wfile.write(jpeg)
        except OSError:
            pass

    def _serve_map(self):
        # The slam_nav node writes the live occupancy map to a RAM file (/dev/shm);
        # we just hand the bytes over same-origin so the page's map canvas can render
        # them. No ROS subscription / OccupancyGrid serialization in this process.
        self._serve_shm("/dev/shm/nano_map.bin", "no map yet")

    def _serve_scan(self):
        # The lidar driver writes each scan as a compact blob to /dev/shm (JSON header +
        # raw float32 ranges); the page polls it here instead of receiving the heavy
        # /scan LaserScan over the telemetry stream. Same idea as the map — heavy data
        # stays off the SSE frame.
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
        except OSError:
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
