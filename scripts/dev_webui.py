#!/usr/bin/env python3
"""Run the real web UI on a dev PC — no ROS, no robot — to test the AI card, TTS, the
decision log, and (with --behavior) the full LLM-enriched statechart loop.

`web_control.web_server` is a ROS node (needs rclpy), so it only runs on the robot. This
is a stripped-down, ROS-free stand-in that serves the *same* `web/index.html` and the
endpoints the AI card + Speak box use (`/llm/*`, `/tts*`), backed by the same
`web_control.llm` + `web_control.tts` modules. Open the page on your laptop, use the
controls, and hear the result through your speakers (Windows SAPI / macOS `say`).

`--behavior` additionally runs the **ROS-free** `behavior.presence` statechart on a real
clock in a background thread (needs `sismic` + `pyyaml`), mapping its beats to the local
LLM: `musing`→sensors (synthetic) and `looking`→your webcam (needs `opencv-python`). So
you can watch/hear the whole enriched-behaviour loop — and every decision shows in the
web UI "🧠 Decision log".

The **🧠 Brain card** (AI tab) is fully wired here too: this harness runs the real
ROS-free brain (`behavior.brain`: Purpose Engine + Pursuit driver + A/B bandit),
serves the readouts the robot streams in its /telemetry frame (`/purpose`,
`/task_current`, `/experiments`) over plain HTTP (the page polls them when the telemetry
stream is absent), and handles `/brain/reward` + `/brain/reflect`. So you can reward the current
line (👍/👎 → the A/B bandit learns), watch the reward weights drift, and toggle
reflection mode — all without the board. With `--behavior` the idle `musing` beat upgrades to a
`pursuing` beat (planner task → webcam observation), so 👍/👎 becomes *contextual* and
credits the chosen variant. (All dev state — the personality "soul" + decision log + goal/
reward state — persists to the project-local `memory/` folder; delete it for a clean slate.)

Telemetry/joystick/map show offline (no /telemetry stream), but the hero's **OLED** view works: it's
a client-side mirror of the physical panel and reads the current face/word from GET /oled/state
(served here), so you can watch the same screen the robot's SSD1306 would show. The API key
comes from $OPENROUTER_API_KEY, or (if unset) a one-line memory/openrouter_key file
(gitignored — see _load_openrouter_key); persona/model are read from robot.yaml.

The robot's body sensors (CPU/RAM/disk/temp/IMU/...) are **faked** here. By default they jitter
randomly (lifelike); open **http://localhost:PORT/dev** (or start with `--manual-sensors`) to
switch to **manual mode** and freeze every value to one you set — handy for reproducing a state
(e.g. hot + picked up) while testing faces/beats. The manual reading feeds BOTH the OLED-mirror
dashboard AND the LLM body snapshot, persists to `memory/dev_sensors.json`, and is controllable
live via GET/POST `/dev/sensors`.

    set OPENROUTER_API_KEY first (or drop it in memory/openrouter_key), then:
    python scripts/dev_webui.py                       # AI card + TTS only
    python scripts/dev_webui.py --behavior            # + autonomous enriched beats
    python scripts/dev_webui.py --behavior --idle-secs 10
"""
import argparse
import functools
import http.server
import json
import os
import random
import sys
import threading
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.normpath(os.path.join(_HERE, ".."))
sys.path.insert(0, os.path.join(_ROOT, "src", "web_control"))
sys.path.insert(0, os.path.join(_ROOT, "src", "behavior"))   # ROS-free presence chart

from web_control.tts import TtsEngine, VOICES, clamp                 # noqa: E402
from web_control.llm import LlmClient                        # noqa: E402
from web_control.skills import resolve_skills_dir            # noqa: E402
from web_control.jsonio import read_json, write_json         # noqa: E402  (atomic dev-state I/O)
from web_control.stress import StressTest                    # noqa: E402  (ROS-free CPU stress)
# The SAME cognition core the robot runs — one base to maintain (see cognition.py).
from web_control.cognition import CognitionCore              # noqa: E402
# The SAME brain orchestration the robot's behaviour node runs (Purpose Engine + Pursuit
# driver), ROS-free — so the dev harness exercises the real goal/reward/A-B layer, not a copy.
from behavior.brain import Personality, PurposeBrain         # noqa: E402
from behavior.presence import merge_beats                    # noqa: E402  (beat table)

WEB_DIR = os.path.join(_ROOT, "src", "web_control", "web")
ROBOT_YAML = os.path.join(_ROOT, "src", "robot_bringup", "config", "robot.yaml")
# All dev-harness state — the "soul" (personality.json), the decision log (cognition.log),
# the goal/reward state (purpose.json, experiments.json, phrases.json), and hand-editable
# data like the sismic chart/beat templates — lives in ONE project-local folder so you can
# see/edit it in the repo while developing, instead of buried in ~/.local/state/nanobot
# (where the robot keeps its own). It's gitignored (volatile state).
DEV_STATE_DIR = os.path.join(_ROOT, "memory")
os.makedirs(DEV_STATE_DIR, exist_ok=True)


def _dev_state(name):
    """A path inside the project-local dev-state folder (a robot.yaml *_path override wins)."""
    return os.path.join(DEV_STATE_DIR, name)


def _load_openrouter_key():
    """If $OPENROUTER_API_KEY isn't already set, load it from memory/openrouter_key (one
    line, gitignored) so you don't have to export it every shell session. Falls back to the
    old scripts/.openrouter_key location. No-op (never overwrites) if the env var is set."""
    if os.environ.get("OPENROUTER_API_KEY", "").strip():
        return
    for path in (_dev_state("openrouter_key"),
                 os.path.join(_HERE, ".openrouter_key")):
        if os.path.isfile(path):
            with open(path, "r", encoding="utf-8") as f:
                key = f.read().strip()
            if key:
                os.environ["OPENROUTER_API_KEY"] = key
            return


_load_openrouter_key()


# Decision-log file (same JSON-lines format the robot's web_server uses). A robot.yaml
# cognition_log_path overrides; "" falls back to the project-local folder.
DEFAULT_COG_LOG = _dev_state("cognition.log")

# Hand-editable beat templates (memory/beats.json, see behavior.presence.merge_beats) layered
# over the built-in BEATS defaults — same file the robot reads via mood_node's `beats_path`.
BEATS = merge_beats(read_json(_dev_state("beats.json")))
# Hand-editable sismic chart (memory/presence_chart.yaml) — "" (missing) falls back to the
# bundled default (behavior.presence.PRESENCE_YAML), same as the robot's `chart_path`.
CHART_PATH = _dev_state("presence_chart.yaml")

# --- synthetic sensor control (dev harness only) -----------------------------
# The dev host has no real sensors, so the harness fakes the robot's body telemetry (CPU/RAM/
# disk/temp/IMU/...). By default it's RANDOM (jittering, lifelike). Flip to MANUAL to freeze it
# and set every value yourself — useful for reproducing a specific state (e.g. "hot + picked up")
# while testing faces/beats. Controlled live via GET/POST /dev/sensors and the /dev page, started
# manual with --manual-sensors, and persisted here so it survives a restart.
DEV_SENSORS_FILE = _dev_state("dev_sensors.json")

# Web-tunable LLM settings (enable + the model ids: normal/deepthink/vision/free). Persisted
# here so UI changes survive a dev restart, exactly like the robot's ~/.local/state/nanobot/llm.json.
DEV_LLM_FILE = _dev_state("llm.json")
LLM_SETTINGS_KEYS = ("enabled", "model", "smart_model", "vision_model",
                     "vision_fallback_model", "free_model", "free_smart_model", "api_key")
# field -> editable spec (drives coercion AND the auto-generated /dev form, so they never drift).
SENSOR_FIELDS = {
    "cpu":      {"label": "CPU %",            "type": "int",   "min": 0,    "max": 100},
    "mem":      {"label": "RAM %",            "type": "int",   "min": 0,    "max": 100},
    "disk":     {"label": "Disk %",           "type": "int",   "min": 0,    "max": 100},
    "temp":     {"label": "CPU temp °C",      "type": "int",   "min": 0,    "max": 120},
    "esp_temp": {"label": "ESP temp °C",      "type": "float", "min": 0,    "max": 120},
    "imu_hz":   {"label": "IMU rate Hz",      "type": "float", "min": 0,    "max": 200},
    "roll":     {"label": "IMU roll °",       "type": "float", "min": -180, "max": 180},
    "pitch":    {"label": "IMU pitch °",      "type": "float", "min": -180, "max": 180},
    "lds_hz":   {"label": "LDS rate Hz",      "type": "float", "min": 0,    "max": 20},
    "tilt":     {"label": "Tilt ° (snapshot)", "type": "int",  "min": 0,    "max": 90},
    "pickup":   {"label": "Pickup 0/1/2",     "type": "int",   "min": 0,    "max": 2},
    "moving":   {"label": "Moving",           "type": "bool"},
    "esp_alive": {"label": "ESP online",      "type": "bool"},
}
# A plausible static reading used as the manual-mode starting point (matches the random ranges).
DEFAULT_SENSORS = {"cpu": 25, "mem": 45, "disk": 50, "temp": 48, "esp_temp": 44.0,
                   "imu_hz": 15.0, "roll": 6.0, "pitch": -2.0, "lds_hz": 0.0,
                   "moving": False, "tilt": 6, "pickup": 0, "esp_alive": True}

# Persisted TTS settings (same defaults as the robot so the dev harness behaves identically).
SETTINGS_DEFAULTS = {
    "voice": "en-gb",
    "volume": 100,
    "speed": 100,
    "pitch": 100,
    "announce": False,
    "announce_interval": 30,
}
ANNOUNCE_MIN = 5
ANNOUNCE_MAX = 3600
TTS_SETTINGS_FILE = _dev_state("tts.json")


def _coerce_sensor(val, spec):
    """Clamp + type a single posted sensor value per its SENSOR_FIELDS spec. Returns None for an
    unparseable number so the caller keeps the previous value (untrusted input is never trusted)."""
    if spec["type"] == "bool":
        return bool(val)
    try:
        x = float(val)
    except (TypeError, ValueError):
        return None
    x = max(spec.get("min", x), min(spec.get("max", x), x))
    return int(round(x)) if spec["type"] == "int" else round(x, 2)


def _capture_webcam_jpeg(log=lambda m: print(m, file=sys.stderr)):
    """Grab one JPEG from the laptop webcam via OpenCV (dev-PC analogue of the robot's
    V4L2 CameraStream). Returns bytes, or None. cv2 is an optional dev-only dependency:
    `pip install opencv-python`."""
    try:
        import cv2
    except ImportError:
        log("[cam] OpenCV not installed — run:  pip install opencv-python")
        return None
    cap = None
    try:
        cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)         # DirectShow opens fast on Windows
        if not cap or not cap.isOpened():
            log("[cam] could not open webcam (device index 0)")
            return None
        ok, frame = False, None
        for _ in range(8):                               # discard first frames (auto-exposure)
            ok, frame = cap.read()
        if not ok or frame is None:
            log("[cam] webcam returned no frame")
            return None
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return buf.tobytes() if ok else None
    except Exception as exc:
        log(f"[cam] capture failed: {exc}")
        return None
    finally:
        if cap is not None:
            cap.release()


def _load_cfg(section):
    """Best-effort read of a robot.yaml ros__parameters section (needs PyYAML). {} on any
    problem so the harness still runs with defaults."""
    try:
        import yaml
    except Exception:
        print("(PyYAML not installed — using defaults; `pip install pyyaml` to load the "
              "real persona/model/behaviour from robot.yaml)", file=sys.stderr)
        return {}
    try:
        with open(ROBOT_YAML, encoding="utf-8") as f:   # not the Windows cp1252 default
            return yaml.safe_load(f)[section]["ros__parameters"]
    except Exception as exc:
        print(f"(couldn't read robot.yaml [{section}]: {exc} — using defaults)", file=sys.stderr)
        return {}


class DevState:
    """The bits of the robot node the page's endpoints need — minus ROS."""

    def __init__(self, voice):
        cfg = _load_cfg("web_control")
        bcfg = _load_cfg("behavior")
        # Current OLED panel state, so the web UI's client-side OLED mirror (see web/index.html)
        # can reflect the exact same screen off-robot, polled via GET /oled/state.
        self.oled_face = ""
        self.oled_word = ""
        # Synthetic-sensor source: random (jittering, default) or manual (frozen, user-set). Loaded
        # from the persisted dev-state so a chosen manual reading survives restarts. See _sensor_values.
        self._sensors_lock = threading.Lock()
        saved = read_json(DEV_SENSORS_FILE, {}) or {}
        vals = dict(DEFAULT_SENSORS)
        vals.update({k: v for k, v in (saved.get("values") or {}).items() if k in SENSOR_FIELDS})
        self._sensors = {"manual": bool(saved.get("manual", False)), "values": vals}
        self._rand_cache = None        # (t, full-dict) cache so random telemetry doesn't jitter every poll
        # The SAME Personality the robot's mood_node uses (behavior.brain) — it owns the
        # chart-context traits/registry/drives AND their throttled persistence back to
        # personality.json. Wiring it here (instead of the old read-only dict) is what makes
        # evolved temperament COMPOUND across dev sessions, exactly like on the robot: the
        # reflection loop drifts the soul, publish_and_persist saves it, next run reseeds from it.
        self._personality = Personality(
            path=_dev_state("personality.json"),
            logger=lambda m: print(f"[personality] {m}", file=sys.stderr),
            heartbeat_enable=True, brain_timeout=float(bcfg.get("brain_timeout", 1800.0)),
            nudge_pickup_caution=float(bcfg.get("nudge_pickup_caution", 0.92)),
            nudge_pickup_playful=float(bcfg.get("nudge_pickup_playful", 0.3)),
            publish=None)                                    # no ROS topic off-robot
        persona = self._personality.persona or cfg.get("llm_persona", "")
        name = self._personality.name
        self.tts = TtsEngine(default_voice=voice, on_word=self._on_word,
                             logger=lambda m: print(f"[tts] {m}", file=sys.stderr))
        # Persisted live settings (same as the robot: voice/volume/speed/pitch +
        # periodic stats announcer), loaded from memory/tts.json so they survive a
        # dev-harness restart. The announce schedule piggy-backs on _health_loop.
        self._settings = self._load_tts_settings()
        self._apply_tts_settings()
        self._cpu_prev = self._cpu_sample()
        self._announce_next = time.monotonic() + float(self._settings["announce_interval"])
        # Seed the web-tunable LLM settings from robot.yaml, then let any persisted UI changes
        # (memory/llm.json) win — so a model id picked in the browser sticks across restarts.
        llm_settings = {
            "enabled": True,
            "model": cfg.get("llm_model", ""),
            "smart_model": cfg.get("llm_smart_model", ""),
            "vision_model": cfg.get("llm_vision_model", ""),
            "vision_fallback_model": cfg.get("llm_vision_fallback_model", ""),
            "free_model": cfg.get("llm_free_model", ""),
            "free_smart_model": cfg.get("llm_free_smart_model", ""),
            "api_key": "",           # "" -> $OPENROUTER_API_KEY; a UI-saved key (below) wins
        }
        saved_llm = read_json(DEV_LLM_FILE, {}) or {}
        if isinstance(saved_llm, dict):
            llm_settings.update({k: v for k, v in saved_llm.items() if k in LLM_SETTINGS_KEYS})
        self.llm = LlmClient(
            enabled=bool(llm_settings["enabled"]), api_key=llm_settings["api_key"],
            model=llm_settings["model"], persona=persona,
            vision_model=llm_settings["vision_model"],
            smart_model=llm_settings["smart_model"],
            free_model=llm_settings["free_model"],
            free_smart_model=llm_settings["free_smart_model"],
            vision_fallback_model=llm_settings["vision_fallback_model"],
            smart_max_per_hour=int(cfg.get("llm_smart_max_per_hour", 15)),
            vision_max_per_hour=int(cfg.get("llm_vision_max_per_hour", 10)),
            timeout=float(cfg.get("llm_timeout", 20.0)),
            hard_deadline=float(cfg.get("llm_hard_deadline", 45.0)),
            logger=lambda m: print(f"[llm] {m}", file=sys.stderr))
        # Auto-enable when a key is present (the persisted llm.json may have disabled it).
        if not llm_settings["enabled"] and (
                (llm_settings["api_key"] or "").strip()
                or os.environ.get("OPENROUTER_API_KEY", "").strip()):
            llm_settings["enabled"] = True
            self.llm.configure(enabled=True)
            print("[llm] key detected, auto-enabling", file=sys.stderr)
        # The SAME cognition core the robot runs (web_control.cognition) — one base to
        # maintain. We give it dev adapters: face -> print, camera -> webcam, sensors ->
        # synthetic, scan/actions -> unavailable (no ROS). The decision log + phrase bank +
        # skills + reflection are all the real, shared code.
        self.cog = CognitionCore(
            llm=self.llm, tts=self.tts, persona=persona, persona_name=name,
            traits=self._personality.traits,
            settings=dict(llm_settings),
            face=self._set_face,
            capture_frame=_capture_webcam_jpeg, sensor_snapshot=self._synth_snapshot,
            sensor_signals=self._synth_signals, scan_summary=lambda: "no scan (dev harness)",
            audio_summary=lambda: "a quiet room with a faint hum (dev harness — no mic)",
            publish_action=lambda _a: (False, "no ROS on the dev harness — actions are robot-only"),
            logger=lambda m: print(f"[cog] {m}", file=sys.stderr),
            persist_settings=lambda s: write_json(
                DEV_LLM_FILE, {k: s[k] for k in LLM_SETTINGS_KEYS if k in s}),
            cog_log_path=(cfg.get("cognition_log_path") or "").strip() or DEFAULT_COG_LOG,
            face_hold=0.0,
            bank_path=(cfg.get("phrasebank_path") or "").strip() or _dev_state("phrases.json"),
            bank_enable=bool(cfg.get("phrasebank_enable", True)),
            bank_live_ratio=float(cfg.get("phrasebank_live_ratio", 0.2)),
            bank_drift=float(cfg.get("phrasebank_drift", 0.6)),
            bank_per_category=int(cfg.get("phrasebank_per_category", 6)),
            bank_grow_enable=bool(cfg.get("phrasebank_grow_enable", True)),
            bank_grow_period=float(cfg.get("phrasebank_grow_period", 1800.0)),
            bank_grow_max=int(cfg.get("phrasebank_grow_max", 24)),
            bank_grow_batch=int(cfg.get("phrasebank_grow_batch", 3)),
            skills_dir=resolve_skills_dir(cfg.get("skills_dir", "")),
            skills_enable=bool(cfg.get("skills_enable", True)),
            skills_allow_actions=bool(cfg.get("skills_allow_actions", False)),
            self_model_enable=bool(cfg.get("self_model_enable", True)),
            self_model_path=(cfg.get("self_model_path") or "").strip() or _dev_state("self_model.json"),
            consolidate_every=int(cfg.get("consolidate_every", 6)),
            trait_history_enable=bool(cfg.get("trait_history_enable", True)),
            trait_history_path=(cfg.get("trait_history_path") or "").strip() or _dev_state("trait_history.json"),
            trait_history_period=float(cfg.get("trait_history_period", 3600.0)),
            trait_history_max=int(cfg.get("trait_history_max", 336)),
            trait_history_window=float(cfg.get("trait_history_window", 604800.0)),
            prelude_enable=bool(cfg.get("prelude_enable", True)),
            camera_announce=bool(cfg.get("camera_announce", True)),
            camera_face=str(cfg.get("camera_face", "looking")),
            # The skill workshop mints/adapts skills into a project-local "learned" dir so you
            # can see them in the repo (deploy carries memory/ to the board, like the soul).
            workshop_enable=bool(cfg.get("workshop_enable", True)),
            workshop_dir=(cfg.get("workshop_dir") or "").strip() or _dev_state("skills"),
            workshop_path=(cfg.get("workshop_path") or "").strip() or _dev_state("workshop.json"),
            workshop_rounds=int(cfg.get("workshop_rounds", 1)),
            workshop_min_runs=int(cfg.get("workshop_min_runs", 3)),
            workshop_retire_errors=int(cfg.get("workshop_retire_errors", 2)),
            workshop_retire_net_neg=int(cfg.get("workshop_retire_net_neg", 2)),
            workshop_adopt_quiet_runs=int(cfg.get("workshop_adopt_quiet_runs", 5)),
            workshop_trial_ttl=float(cfg.get("workshop_trial_ttl", 172800.0)),
            workshop_trial_bias=float(cfg.get("workshop_trial_bias", 0.5)),
            skill_likes_path=(cfg.get("skill_likes_path") or "").strip() or _dev_state("skill_likes.json"),
            skill_like_bias=float(cfg.get("skill_like_bias", 0.6)),
            reflect_announce=bool(cfg.get("reflect_announce", True)),
            # Quiet hours (same robot.yaml values as the robot, so the dev harness goes
            # quiet at night too — the decision log shows "quiet-hours" when it does).
            quiet_start=float(cfg.get("quiet_start", -1.0)),
            quiet_end=float(cfg.get("quiet_end", -1.0)))
        self._pending_evolve = None                         # reflection -> chart (drained by loop)
        self._lock_pe = threading.Lock()
        # --- Purpose Engine + Pursuit driver (the real ROS-free brain orchestration) -----
        # On the dev host there's no behaviour node, so we run the SAME PurposeBrain the robot
        # runs (behavior.brain) so the web "🧠 Brain" card is fully exercisable: objective +
        # reward weights, the pursuing/skill beats, A/B variants + 👍/👎 reward, and reflection.
        # Dev adapters: experience = the shared decision log; never picked up; no publishing
        # (the page polls the getters below over HTTP instead of latched topics).
        self._brain = PurposeBrain(
            name=name, enable=bool(bcfg.get("purpose_enable", True)), rng=random.Random(),
            epsilon=float(bcfg.get("ab_epsilon", 0.2)), pursue_min_interval=60.0,
            skills_enable=bool(bcfg.get("skills_enable", True)),
            skill_every=max(1, int(bcfg.get("skill_every", 6))),
            purpose_path=_dev_state("purpose.json"),
            experiments_path=_dev_state("experiments.json"), cog_log_path=DEFAULT_COG_LOG,
            read_cog_log=lambda: self.cog.get_cog_log()["entries"],
            traits_snapshot=lambda: dict(self.cog.traits),
            logger=lambda m: print(f"[brain] {m}", file=sys.stderr))
        # Same ROS-free stress-test manager the robot's web_server uses, so the System
        # tab's Stress test card works on the dev harness too (niced CPU busy loops, no
        # ROS/cgroup involved off-robot). No thermal abort here — no real sensor to trip it.
        self.stress = StressTest(logger=lambda m: print(f"[stress] {m}", file=sys.stderr))
        print(f"[dev] TTS backend ready={self.tts.available()}  "
              f"LLM ready={self.llm.available()} (model {self.llm.model})", file=sys.stderr)
        if not self.llm.available():
            print("[dev] ! No OPENROUTER_API_KEY set — the AI card will say 'unavailable'.",
                  file=sys.stderr)
        self.cog.bank_regen_check()                         # build/refresh the bank if needed
        # Lifecycle speech: greet on launch + a 1 Hz "AI offline" indicator (mirrors the
        # robot). Remove the key to hear/see the offline line on the dev host.
        self._dev_offline = None
        t = threading.Timer(2.0, lambda: self.cog.speak_lifecycle("greeting")); t.daemon = True
        t.start()
        threading.Thread(target=self._health_loop, daemon=True).start()

    def _health_loop(self):
        while True:
            time.sleep(1.0)
            self._announce_tick()
            offline = bool(self.cog.settings.get("enabled")) and not self.llm.available()
            if offline != self._dev_offline:
                prev = self._dev_offline
                self._dev_offline = offline
                if offline:
                    self.cog.speak_lifecycle("offline", face="sleepy")
                elif prev:                                  # only if we WERE offline
                    self._set_face("")                      # back to the dashboard
                    print("(LLM online)", file=sys.stderr)

    @staticmethod
    def _gen_random():
        """One fresh random full sensor reading (the lifelike default). IMU/ESP telemetry is
        derived from the body so the dashboard stays internally consistent."""
        cpu, mem, temp = random.randint(8, 90), random.randint(28, 70), random.randint(34, 64)
        tilt = random.choice([2, 6, 14, 22, 30])
        return {"cpu": cpu, "mem": mem, "disk": random.randint(30, 80), "temp": temp,
                "esp_temp": float(temp) - 4.0, "imu_hz": 15.0, "roll": float(tilt),
                "pitch": -float(tilt) / 3.0, "lds_hz": 0.0,
                "moving": random.random() < 0.25, "tilt": tilt,
                "pickup": random.choice([0, 0, 0, 1, 2]), "esp_alive": True}

    def _sensor_values(self):
        """The current FULL sensor reading feeding every adapter + the OLED mirror. Manual mode
        returns the frozen user-set values; random mode returns a fresh reading cached ~2 s so the
        dashboard doesn't jitter every poll AND one beat's classifier + snapshot agree."""
        with self._sensors_lock:
            if self._sensors["manual"]:
                return dict(self._sensors["values"])
        now = time.monotonic()
        if self._rand_cache is None or now - self._rand_cache[0] > 2.0:
            self._rand_cache = (now, self._gen_random())
        return dict(self._rand_cache[1])

    def _synth_signals(self):
        """The core's sensor_signals adapter — the structured body subset (random or manual)."""
        v = self._sensor_values()
        return {"cpu": v["cpu"], "mem": v["mem"], "disk": v["disk"], "temp": v["temp"],
                "moving": bool(v["moving"]), "tilt": v["tilt"], "pickup": v["pickup"]}

    def _synth_snapshot(self):
        """A plain-English body snapshot (the core's sensor_snapshot adapter)."""
        sig = self._synth_signals()
        body = ["being moved or jostled" if sig["moving"] else "physically still",
                f"tilted ({sig['tilt']} degrees)" if sig["tilt"] > 10 else "sitting level",
                {0: "resting on the ground", 1: "with one wheel off the ground",
                 2: "lifted off the ground (being held)"}[sig["pickup"]]]
        return (f"CPU load {sig['cpu']}%, memory {sig['mem']}% used, disk {sig['disk']}% full, "
                f"main board {sig['temp']} degrees C, " + ", ".join(body))

    # ---- synthetic-sensor control (random vs manual) ------------------------
    def get_sensors(self):
        """Current sensor source for the /dev page: mode + values + the field specs (so the form
        is generated from one table)."""
        with self._sensors_lock:
            return {"manual": self._sensors["manual"], "values": dict(self._sensors["values"]),
                    "fields": SENSOR_FIELDS}

    def set_sensors(self, patch):
        """Update the sensor source from the /dev page or POST /dev/sensors: toggle manual and/or
        set individual values (clamped/typed). Persists so it survives a restart; returns the new
        state. Switching mode clears the random cache so the change takes effect immediately."""
        patch = patch if isinstance(patch, dict) else {}
        with self._sensors_lock:
            if "manual" in patch:
                self._sensors["manual"] = bool(patch["manual"])
            for k, v in (patch.get("values") or {}).items():
                if k in SENSOR_FIELDS:
                    c = _coerce_sensor(v, SENSOR_FIELDS[k])
                    if c is not None:
                        self._sensors["values"][k] = c
            snap = {"manual": self._sensors["manual"], "values": dict(self._sensors["values"])}
        self._rand_cache = None
        write_json(DEV_SENSORS_FILE, snap)
        return self.get_sensors()

    def _set_face(self, m):
        """Cognition's face adapter: print it AND record it so GET /oled/state can mirror
        the panel in the browser (the robot does this over /oled_face)."""
        self.oled_face = (m or "").strip()
        print(f"\n[face] {self.oled_face or 'dashboard'}", file=sys.stderr)

    def _on_word(self, w):
        # TTS karaoke: track the current word so the OLED mirror shows it big+centred (the
        # robot streams these on /oled_word); "" hands the panel back to the face/dashboard.
        self.oled_word = w or ""
        print(w, end=" ", flush=True) if w else print()

    # ---- TTS settings (persisted, same as the robot) ------------------------

    def _tts_settings_file(self):
        return TTS_SETTINGS_FILE

    def _load_tts_settings(self):
        s = dict(SETTINGS_DEFAULTS)
        s["voice"] = self.tts.voice or "en-gb"
        saved = read_json(self._tts_settings_file())
        if isinstance(saved, dict):
            s.update({k: v for k, v in saved.items() if k in SETTINGS_DEFAULTS})
        return self._sanitize_settings(s)

    def _save_tts_settings(self):
        write_json(self._tts_settings_file(), self._settings)

    def _apply_tts_settings(self):
        s = self._settings
        self.tts.configure(voice=s["voice"], volume=s["volume"],
                           speed=s["speed"], pitch=s["pitch"])

    def get_tts_settings(self):
        return dict(self._settings)

    def update_tts_settings(self, data):
        old = self._settings
        s = dict(old)
        for k in SETTINGS_DEFAULTS:
            if k in data:
                s[k] = data[k]
        self._settings = self._sanitize_settings(s)
        self._save_tts_settings()
        self._apply_tts_settings()
        if self._settings["announce"] and (
                not old["announce"]
                or self._settings["announce_interval"] != old["announce_interval"]):
            self._cpu_prev = self._cpu_sample()
            self._announce_next = time.monotonic() + float(self._settings["announce_interval"])
        return self._settings

    @staticmethod
    def _sanitize_settings(s):
        out = dict(SETTINGS_DEFAULTS)
        out.update(s)
        out["voice"] = out["voice"] if out["voice"] in VOICES else SETTINGS_DEFAULTS["voice"]
        out["volume"] = clamp(out["volume"], 0, 500)
        out["speed"] = clamp(out["speed"], 20, 500)
        out["pitch"] = clamp(out["pitch"], 50, 200)
        out["announce"] = bool(out["announce"])
        out["announce_interval"] = clamp(out["announce_interval"], ANNOUNCE_MIN, ANNOUNCE_MAX)
        return out

    # ---- periodic spoken system stats (matches the robot's announce) ---------

    def _announce_tick(self):
        if not self._settings["announce"]:
            return
        now = time.monotonic()
        if now < self._announce_next:
            return
        self._announce_next = now + float(self._settings["announce_interval"])
        if self.cog.quiet_now():            # the periodic announcer respects quiet hours
            return
        self.announce_now()

    def announce_now(self):
        if not self.tts.available():
            return
        text = self._compose_stats(self._cpu_percent(), self._mem_percent(),
                                   self._cpu_temp())
        if text:
            self.tts.say(text)

    def _compose_stats(self, cpu, mem, temp):
        parts = []
        if cpu == cpu:
            parts.append(f"C P U {cpu:.0f} percent")
        if mem == mem:
            parts.append(f"RAM {mem:.0f} percent")
        if temp == temp:
            parts.append(f"Temperature {temp:.0f} degrees")
        if not parts:
            return "No data"
        return ". ".join(parts)

    def _cpu_sample(self):
        try:
            with open("/proc/stat") as f:
                parts = [int(x) for x in f.readline().split()[1:]]
            idle = parts[3] + (parts[4] if len(parts) > 4 else 0)
            return idle, sum(parts)
        except Exception:
            return None

    def _cpu_percent(self):
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
            with open("/proc/meminfo") as f:
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
            with open("/sys/class/thermal/thermal_zone0/temp") as f:
                return int(f.read().strip()) / 1000.0
        except Exception:
            return float("nan")

    def oled_state(self):
        """The exact inputs the physical OLED renders from, for the web UI's client-side mirror.
        Telemetry is synthetic on the dev host (random by default, or the frozen manual reading) —
        see _sensor_values."""
        v = self._sensor_values()
        return {"face": self.oled_face, "word": self.oled_word, "brand": "", "system": "",
                "esp_alive": bool(v["esp_alive"]), "esp_temp": float(v["esp_temp"]),
                "imu_hz": float(v["imu_hz"]), "roll": float(v["roll"]), "pitch": float(v["pitch"]),
                "lds_hz": float(v["lds_hz"]),
                "cpu": v["cpu"], "mem": v["mem"], "temp": v["temp"], "ip": "dev-host"}

    # ---- cognition-core delegators (the shared LLM brain lives in cognition.py) ----
    # The page's endpoints call these; all the logic is the same CognitionCore the robot runs.
    def get_cog_log(self):
        return self.cog.get_cog_log()

    def get_phrasebank(self):
        return self.cog.get_phrasebank()

    def regenerate_phrasebank(self):
        return self.cog.regenerate_phrasebank()

    def llm_config(self):
        return self.cog.get_llm_settings()

    def update_llm_config(self, data):
        return self.cog.update_llm_settings(data)

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
        return self.stress.start(duration=duration, workers=workers)

    def stress_stop(self):
        return self.stress.stop()

    def stress_status(self):
        return self.stress.status()

    def llm_say(self, prompt):
        return self.cog.llm_say(prompt)

    def llm_chat(self, message):
        return self.cog.llm_chat(message)

    def llm_observe(self, trigger="observe", state=""):
        return self.cog.llm_observe(trigger=trigger, state=state)

    def llm_look(self, trigger="look", state=""):
        return self.cog.llm_look(trigger=trigger, state=state)

    def get_skills(self):
        return self.cog.get_skills()

    def reload_skills(self):
        return self.cog.reload_skills()

    def invoke_skill(self, name):
        return self.cog.invoke_skill(name)

    def like_skill(self, data):
        """👍 a skill so the brain favours it (repeatable). Body {"name","delta":±1}."""
        name = str((data or {}).get("name") or "").strip()
        if not name:
            return {"error": "empty name"}
        try:
            delta = int((data or {}).get("delta", 1))
        except (TypeError, ValueError):
            delta = 1
        return self.cog.like_skill(name, delta)

    # ---- statechart beat (used by --behavior) -------------------------------
    def fire_beat(self, name):
        """The chart's do_beat: run the matching enrichment in a worker thread (async,
        like the robot's fire-and-forget request) so the chart never waits on the LLM.

        Goal-pursuit + skill upgrades are decided by the shared PurposeBrain (exactly as on the
        robot): the `musing` body beat becomes a `pursuing` beat when the planner has a verified
        task, else a `skill` beat every Nth body beat; otherwise the chosen beat
        (musing/looking/wondering/listening) runs through the shared `run_beat` — the same path
        web_server._on_cog uses, so the new beats' prompts + audio are exercised here too.
        Reflection mode pauses all beats."""
        if self._brain.reflecting:
            print(f"\n[beat] {name} (paused — reflecting)", file=sys.stderr)
            return
        print(f"\n[beat] {name}", file=sys.stderr)
        spec = self._brain.next_pursuing(time.monotonic()) if name == "musing" else None
        skill_beat = name == "musing" and spec is None and self._brain.take_skill_beat()
        beat = BEATS.get(name)

        def work():
            if spec is not None:
                self._deliver_pursuing(spec)
            elif skill_beat:
                self.cog.run_skill_beat("acting")       # let the brain pick a skill (shared core)
            elif beat is not None:
                self.cog.run_beat("beat:" + name, name, beat.prompt, beat.camera, beat.audio,
                                  beat.face)
        threading.Thread(target=work, daemon=True).start()

    def reflect(self, traits_dict):
        """Deep/slow tier: sync the chart's current traits into the core, run the (shared)
        reflection, and stash the proposal for the chart loop to apply (queueing on the
        interpreter from this worker thread would race the loop's execute())."""
        self.cog.update_traits(traits_dict)
        res = self.cog.reflect()                         # prompt/parse/log all in the shared core
        if res:
            with self._lock_pe:
                self._pending_evolve = (res["traits"], res["registry"], res.get("drives", {}))
            print(f"\n[reflect] {res.get('note','')} -> {res['traits']} "
                  f"{res.get('drives') or ''}", file=sys.stderr)

    def take_evolve(self):
        with self._lock_pe:
            ev, self._pending_evolve = self._pending_evolve, None
        return ev

    # ---- purpose engine + planner (served from the shared PurposeBrain) ------
    def get_purpose(self):
        return self._brain.purpose

    def get_task_current(self):
        return self._brain.task or {}

    def get_experiments(self):
        return self._brain.summary()

    def _deliver_pursuing(self, spec):
        """Narrate the planner's task as a pursuing beat (dev: webcam = the robot's camera).
        PurposeBrain.next_pursuing already set /task_current (so 👍/👎 credits the right A/B
        arm) even when the LLM is offline."""
        print(f"[pursue] {spec['task']} (variant={spec['variant']})", file=sys.stderr)
        prompt = self._brain.pursuing_prompt(
            spec, "Say one short spoken line as you {task} right now.")
        frame = None
        if spec["camera"] and self.llm.available():
            if not self.llm.can_call(image=True):
                self.cog.log_decision("beat:pursuing", "pursuing", True, status="rate-limited")
                return
            frame = _capture_webcam_jpeg()
            if frame is None:
                self.cog.log_decision("beat:pursuing", "pursuing", True, status="no-frame")
                return
        self.cog.generate(prompt, image_jpeg=frame, trigger="beat:pursuing", state="pursuing",
                          camera=bool(spec["camera"]), prelude=True, base_face=BEATS["pursuing"].face)

    def brain_reward(self, data):
        """Mirror web_server.brain_reward: log + (contextual) credit the A/B arm + reflect so
        the reward weights visibly move on the card."""
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
        self.cog.log_decision("reward", status=status, detail=detail,
                              say=("👍" if value > 0 else "👎" if value < 0 else "·"))
        if scope == "contextual":
            self.cog.reward_trial_skill(value)          # credit a trial skill that just ran
        self._brain.apply_reward(value, target, scope=scope)
        self._brain.run_reflection()                    # reflect now so the weights update
        print(f"[reward] {status} ({scope})", file=sys.stderr)
        return {"status": "ok", "value": value, "scope": scope}

    def brain_reflect(self, data):
        """Mirror web_server.brain_reflect: pause beats, consolidate (reflect + A/B + bank) and
        forge a skill (the workshop)."""
        on = bool(data.get("on"))
        self._brain.set_reflecting(on)                  # flag + (on entry) reflect + A/B finalize
        self.cog.set_reflecting(on)                     # so the core speaks self/forge conclusions
        self.cog.announce_reflect(on)                   # say a short bookend line (turning inward / done)
        if on:
            self.cog.bank_regen_check()
            self.cog.bank_grow_check()                  # grow the bank: add fresh offline lines
            threading.Thread(target=self.cog.consolidate, daemon=True).start()  # long-term self
            threading.Thread(target=self.cog.run_skill_workshop, daemon=True).start()  # mint a skill
        self.cog.log_decision("reflect_mode", status=("on" if on else "off"))
        print(f"[reflect] {'on' if on else 'off'}", file=sys.stderr)
        return {"status": "ok", "reflecting": self._brain.reflecting}

    def brain_workshop(self):
        return self.cog.get_workshop()

    def workshop_keep(self, data):
        return self.cog.keep_skill(str(data.get("name", "")))

    def workshop_kill(self, data):
        return self.cog.kill_skill(str(data.get("name", "")))


def run_behavior(state, idle_secs, reflect_secs):
    """Run the ROS-free presence statechart on a real clock; its beats drive the local LLM
    (see DevState.fire_beat) and a periodic reflection drifts the traits (see
    DevState.reflect). Honours camera_beats + the seed personality from robot.yaml
    / personality.json."""
    try:
        from sismic.clock import SimulatedClock
        from sismic.model import Event
        from behavior.presence import build_interpreter
    except Exception as exc:
        print(f"[behavior] unavailable ({exc}) — run `pip install sismic pyyaml`",
              file=sys.stderr)
        return
    b = _load_cfg("behavior")
    camera_beats = bool(b.get("camera_beats", True))
    # Night idle slowdown (parity with mood_node._tempo): stretch the idle cadence inside
    # the quiet window. Speech muting rides the cognition core's quiet_start/quiet_end.
    q_s, q_e = float(b.get("quiet_start", -1.0)), float(b.get("quiet_end", -1.0))
    n_tempo = max(1.0, float(b.get("night_tempo", 2.0)))

    def tempo():
        if n_tempo <= 1.0 or q_s < 0 or q_e < 0 or q_s == q_e:
            return 1.0
        lt = time.localtime()
        h = lt.tm_hour + lt.tm_min / 60.0
        in_q = (h >= q_s or h < q_e) if q_s > q_e else (q_s <= h < q_e)
        return n_tempo if in_q else 1.0

    auto_on = bool(b.get("reflect_auto_enable", True))   # autonomously drift into reflection mode
    auto_idle = max(0.0, float(b.get("reflect_auto_idle", 1200.0)))
    auto_secs = max(1.0, float(b.get("reflect_auto_secs", 120.0)))
    clock = SimulatedClock()
    clock.time = 0.0
    interp, _ = build_interpreter(
        face=state._set_face,                  # mirror the chart's reflex faces to /oled/state
        do_beat=state.fire_beat,
        greet_secs=1.0, idle_secs=idle_secs, perform_secs=4.0,
        # A live callable (re-checked on every draw), not a snapshot: stop offering camera
        # beats ("looking") the moment the LLM isn't available, same as the robot wires
        # cognition/llm_ready into MoodNode._camera_beats_ok.
        camera_beats=lambda: camera_beats and state.llm.available(),
        traits=state._personality.traits, registry=state._personality.registry,
        drives=state._personality.drives,
        attend_secs=float(b.get("attend_secs", 2.0)), feel_secs=float(b.get("feel_secs", 2.5)),
        attend_face=str(b.get("attend_face", "looking")),
        alpha=float(b.get("smoothing_alpha", 0.1)), clock=clock,
        chart_path=CHART_PATH, beats=BEATS, tempo=tempo)
    state._personality.attach(interp)          # bind so it can read/persist the live soul
    print(f"[behavior] statechart running — idle_secs={idle_secs}, camera_beats="
          f"{camera_beats}, reflect_secs={reflect_secs}. "
          f"Stop clicking and listen; watch the Decision log.", file=sys.stderr)
    t0 = last_reflect = last_purpose = time.monotonic()
    next_auto = t0 + auto_idle                  # when to auto-enter reflection (dev: time-based)
    auto_until = None                           # deadline of a self-started reflection (or None)
    while True:
        time.sleep(0.5)
        now = time.monotonic()
        clock.time = now - t0
        # Autonomous reflection mode (parity with mood_node): the dev harness has no real
        # activity signal, so it's purely time-based — enter every auto_idle s, run auto_secs.
        if auto_on:
            if auto_until is not None:
                if now >= auto_until:
                    state.brain_reflect({"on": False})
                    auto_until = None
                    next_auto = now + auto_idle
                    interp.queue(Event("wake"))
            elif not state._brain.reflecting and now >= next_auto:
                state.brain_reflect({"on": True})
                auto_until = now + auto_secs
                interp.queue(Event("reflect"))
        if (now - last_purpose) > 30.0:        # local Purpose Engine: drift reward weights
            last_purpose = now
            state._brain.run_reflection()
        ev = state.take_evolve()               # apply reflection drift (queued single-threaded)
        if ev:
            # Route through Personality.on_evolve (NOT a raw queue) so the heartbeat sees the
            # brain is alive AND the drift gets persisted by publish_and_persist below — exactly
            # the path mood_node uses on the robot.
            state._personality.on_evolve({"traits": ev[0], "registry": ev[1],
                                          "drives": (ev[2] if len(ev) > 2 else {})})
        try:
            interp.execute()
        except Exception as exc:
            print(f"[behavior] step error: {exc}", file=sys.stderr)
        # Heartbeat (reverts to the seed if the cognitive layer goes quiet for brain_timeout)
        # + throttled persist of the drifted soul to memory/personality.json — the parity
        # bits that make temperament compound across dev sessions.
        if state._personality.tick_events(now, picked=False) == "lost":
            print("[personality] brain lost — reverting toward the seed", file=sys.stderr)
        if ev:
            print(f"[behavior] traits now {dict(interp.context['traits'])}; "
                  f"drives {dict(interp.context['drives'])}", file=sys.stderr)
            state.cog.update_traits(dict(interp.context["traits"]))  # track soul for bank-regen
            state.cog.bank_regen_check()                     # refresh bank if it drifted too far
        state._personality.publish_and_persist(now)          # save drift (throttled, on change)
        if reflect_secs > 0 and state.llm.available() and (now - last_reflect) > reflect_secs:
            last_reflect = now
            threading.Thread(target=state.reflect,
                             args=(dict(interp.context["traits"]),), daemon=True).start()


# A tiny self-contained control page for the synthetic sensors (dev-only). Kept separate from the
# shared web/index.html (which also runs on the robot) so the robot UI stays clean — open it at
# /dev. The form is generated from /dev/sensors' `fields`, so it never drifts from SENSOR_FIELDS.
DEV_SENSORS_PAGE = """<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Nano dev sensors</title>
<style>
 body{font:14px system-ui,sans-serif;background:#0d1117;color:#e6edf3;margin:0;padding:18px;max-width:560px}
 h1{font-size:18px;margin:0 0 4px} p.sub{color:#8b949e;margin:0 0 16px}
 .mode{display:flex;align-items:center;gap:10px;padding:12px;border:1px solid #30363d;border-radius:10px;margin-bottom:16px}
 .mode b{font-size:15px}
 fieldset{border:1px solid #30363d;border-radius:10px;margin:0 0 16px;padding:12px}
 fieldset[disabled]{opacity:.45}
 .row{display:flex;align-items:center;gap:10px;margin:7px 0}
 .row label{flex:0 0 180px} .row input[type=range]{flex:1} .row .val{flex:0 0 64px;text-align:right;font-variant-numeric:tabular-nums}
 .row input[type=number]{width:84px;background:#161b22;color:#e6edf3;border:1px solid #30363d;border-radius:6px;padding:4px}
 button{background:#238636;color:#fff;border:0;border-radius:8px;padding:9px 16px;font-size:14px;cursor:pointer}
 button.sec{background:#30363d} .bar{display:flex;gap:10px;align-items:center}
 #status{color:#8b949e}
 input[type=checkbox]{width:18px;height:18px}
</style></head><body>
<h1>Synthetic sensors</h1>
<p class="sub">Dev harness only. Random by default; turn on <b>Manual</b> to freeze and set every
value yourself. Changes apply live (the OLED mirror + the LLM body snapshot read these).</p>
<div class="mode">
  <input type="checkbox" id="manual"><b><label for="manual">Manual mode</label></b>
  <span id="status" style="margin-left:auto"></span>
</div>
<fieldset id="fs"></fieldset>
<div class="bar">
  <button id="apply">Apply &amp; save</button>
  <button class="sec" id="reload">Reload</button>
  <a href="/" style="margin-left:auto;color:#58a6ff">&larr; main UI</a>
</div>
<script>
let fields={}, vals={};
const $=id=>document.getElementById(id);
function build(){
  const fs=$("fs"); fs.innerHTML="";
  for(const [k,spec] of Object.entries(fields)){
    const row=document.createElement("div"); row.className="row";
    const lab=document.createElement("label"); lab.textContent=spec.label; row.appendChild(lab);
    if(spec.type==="bool"){
      const cb=document.createElement("input"); cb.type="checkbox"; cb.id="f_"+k; cb.checked=!!vals[k];
      cb.onchange=()=>vals[k]=cb.checked; row.appendChild(cb);
    } else {
      const rng=document.createElement("input"); rng.type="range"; rng.id="f_"+k;
      rng.min=spec.min; rng.max=spec.max; rng.step=(spec.type==="int")?1:0.5; rng.value=vals[k];
      const num=document.createElement("input"); num.type="number"; num.className="val";
      num.min=spec.min; num.max=spec.max; num.step=rng.step; num.value=vals[k];
      rng.oninput=()=>{num.value=rng.value; vals[k]=parseFloat(rng.value);};
      num.oninput=()=>{rng.value=num.value; vals[k]=parseFloat(num.value);};
      row.appendChild(rng); row.appendChild(num);
    }
    fs.appendChild(row);
  }
  $("fs").disabled = !$("manual").checked;
}
function load(){
  fetch("/dev/sensors").then(r=>r.json()).then(s=>{
    fields=s.fields; vals=Object.assign({},s.values); $("manual").checked=s.manual;
    build(); status(s.manual?"manual":"random");
  });
}
function status(m){ $("status").textContent = "source: "+m; }
$("manual").onchange=()=>{ $("fs").disabled=!$("manual").checked; };
$("apply").onclick=()=>{
  fetch("/dev/sensors",{method:"POST",headers:{"Content-Type":"application/json"},
    body:JSON.stringify({manual:$("manual").checked, values:vals})})
    .then(r=>r.json()).then(s=>{ vals=Object.assign({},s.values); $("manual").checked=s.manual;
      build(); status((s.manual?"manual":"random")+" — saved"); });
};
$("reload").onclick=load;
load();
</script></body></html>"""


class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *a, state=None, **k):
        self.state = state
        super().__init__(*a, **k)

    def _json(self, obj, code=200):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _text(self, code, msg):
        body = msg.encode()
        self.send_response(code)
        self.send_header("Content-Type", "text/plain")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _body(self):
        try:
            n = int(self.headers.get("Content-Length", 0) or 0)
            return json.loads(self.rfile.read(n) or b"{}") if n else {}
        except Exception:
            return {}

    def do_GET(self):
        p = self.path.split("?", 1)[0]
        if p == "/llm/config":
            return self._json(self.state.llm_config())
        if p == "/llm/log":
            return self._json(self.state.get_cog_log())
        if p == "/llm/phrases":
            return self._json(self.state.get_phrasebank())
        if p == "/skills":
            return self._json(self.state.get_skills())
        if p == "/skills/workshop":
            return self._json(self.state.brain_workshop())
        if p == "/stress/status":
            return self._json(self.state.stress_status())
        if p == "/tts/config":
            return self._json(self.state.get_tts_settings())
        if p == "/oled/state":
            return self._json(self.state.oled_state())
        if p == "/dev/sensors":
            return self._json(self.state.get_sensors())
        if p == "/dev" or p == "/dev/":
            body = DEV_SENSORS_PAGE.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        # Brain readouts — the robot streams these in its /telemetry frame; the dev
        # harness serves them over HTTP and the page polls them while disconnected.
        if p == "/purpose":
            return self._json(self.state.get_purpose())
        if p == "/task_current":
            return self._json(self.state.get_task_current())
        if p == "/experiments":
            return self._json(self.state.get_experiments())
        return super().do_GET()

    def do_POST(self):
        p = self.path.split("?", 1)[0]
        s = self.state
        if p == "/drive":
            # HTTP teleop no-op: accept the page's {v,w} POSTs (there are no motors
            # here to move) so the joystick UI behaves normally off-robot.
            return self._json({"status": "ok", "dev": True})
        if p == "/llm/say":
            if not s.llm.available():
                return self._text(503, "llm unavailable")
            return self._json(s.llm_say(self._body().get("prompt") or "") or {"error": "no reply"})
        if p == "/llm/chat":
            if not s.llm.available():
                return self._text(503, "llm unavailable")
            msg = (self._body().get("message") or "").strip()
            if not msg:
                return self._text(400, "empty message")
            return self._json(s.llm_chat(msg) or {"error": "no reply"})
        if p == "/llm/observe":
            if not s.llm.available():
                return self._text(503, "llm unavailable")
            return self._json(s.llm_observe() or {"error": "no reply"})
        if p == "/llm/look":
            if not s.llm.available():
                return self._text(503, "llm unavailable")
            return self._json(s.llm_look() or {"error": "no reply"})
        if p == "/llm/config":
            return self._json(s.update_llm_config(self._body()))
        if p == "/llm/phrases/regenerate":
            return self._json(s.regenerate_phrasebank())
        if p == "/skills/invoke":
            name = (self._body().get("name") or "").strip()
            if not name:
                return self._text(400, "empty name")
            return self._json(s.invoke_skill(name))
        if p == "/skills/reload":
            return self._json(s.reload_skills())
        if p == "/skills/like":
            return self._json(s.like_skill(self._body()))
        if p == "/skills/workshop/keep":
            return self._json(s.workshop_keep(self._body()))
        if p == "/skills/workshop/kill":
            return self._json(s.workshop_kill(self._body()))
        if p == "/stress/start":
            return self._json(s.stress_start(self._body()))
        if p == "/stress/stop":
            return self._json(s.stress_stop())
        if p == "/dev/sensors":
            return self._json(s.set_sensors(self._body()))
        if p == "/brain/reward":
            return self._json(s.brain_reward(self._body()))
        if p == "/brain/reflect":
            return self._json(s.brain_reflect(self._body()))
        if p == "/tts":
            d = self._body()
            text = (d.get("text") or "").strip()
            if not s.tts.available():
                return self._text(503, "tts unavailable")
            if not text:
                return self._text(400, "empty text")
            s.tts.say(text, voice=(d.get("voice") or None))
            return self._text(200, "speaking")
        if p == "/tts/config":
            return self._json(self.state.update_tts_settings(self._body()))
        if p == "/tts/stop":
            s.tts.stop()
            return self._text(200, "stopped")
        if p == "/tts/announce":
            if not s.tts.available():
                return self._text(503, "tts unavailable")
            s.announce_now()
            return self._text(200, "announcing")
        return self._text(404, "not found")

    def log_message(self, *a):
        pass


def main():
    ap = argparse.ArgumentParser(description="Dev-only web UI harness (AI card + TTS, no ROS).")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--voice", default="en-gb", help="voice: en-gb | en-gb-x-gbclan | en-gb-scotland")
    ap.add_argument("--behavior", action="store_true",
                    help="also run the presence statechart so beats drive the LLM")
    ap.add_argument("--idle-secs", type=float, default=15.0,
                    help="(--behavior) seconds idle before each beat (default 15)")
    ap.add_argument("--reflect-secs", type=float, default=90.0,
                    help="(--behavior) seconds between personality reflections (0=off, default 90)")
    ap.add_argument("--manual-sensors", action="store_true",
                    help="start with synthetic sensors FROZEN at manual values (set them at /dev) "
                         "instead of the default random jitter")
    args = ap.parse_args()

    state = DevState(args.voice)
    if args.manual_sensors:
        state.set_sensors({"manual": True})
    if args.behavior:
        threading.Thread(target=run_behavior,
                         args=(state, args.idle_secs, args.reflect_secs), daemon=True).start()
    handler = functools.partial(Handler, directory=WEB_DIR, state=state)
    httpd = http.server.ThreadingHTTPServer(("0.0.0.0", args.port), handler)
    sensors = "manual (frozen)" if state.get_sensors()["manual"] else "random"
    print(f"\n  Dev web UI:  http://localhost:{args.port}\n"
          f"  Sensors:     http://localhost:{args.port}/dev   (synthetic source: {sensors})\n"
          f"  AI tab -> 'AI · OpenRouter' card; open '🧠 Decision log' to watch decisions.\n"
          f"  {'Behaviour beats ON. ' if args.behavior else ''}"
          f"(Telemetry/joystick/map show offline — no rosbridge. Ctrl+C to stop.)\n",
          file=sys.stderr)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nbye.", file=sys.stderr)
    finally:
        httpd.shutdown()


if __name__ == "__main__":
    main()
