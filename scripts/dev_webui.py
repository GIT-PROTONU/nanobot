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

The **🧠 Brain card** (Speak tab) is fully wired here too: this harness runs the real
ROS-free `behavior.purpose` (Purpose Engine) + `behavior.planner` (Horizon Planner + A/B
bandit), serves the readouts the robot publishes over rosbridge (`/purpose`,
`/task_current`, `/experiments`) over plain HTTP (the page polls them when rosbridge is
absent), and handles `/brain/reward` + `/brain/meditate`. So you can reward the current
line (👍/👎 → the A/B bandit learns), watch the reward weights drift, and toggle
meditation — all without the board. With `--behavior` the idle `musing` beat upgrades to a
`pursuing` beat (planner task → webcam observation), so 👍/👎 becomes *contextual* and
credits the chosen variant. (All dev state — the personality "soul" + decision log + goal/
reward state — persists to the project-local `devstate/` folder; delete it for a clean slate.)

Telemetry/joystick/map show offline (no rosbridge). The API key comes from
$OPENROUTER_API_KEY; persona/model are read from robot.yaml.

    set OPENROUTER_API_KEY first, then:
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

from web_control.tts import TtsEngine, clamp                 # noqa: E402
from web_control.llm import LlmClient                        # noqa: E402
from web_control.skills import resolve_skills_dir            # noqa: E402
# The SAME cognition core the robot runs — one base to maintain (see cognition.py).
from web_control.cognition import CognitionCore              # noqa: E402
# The SAME brain orchestration the robot's behaviour node runs (Purpose Engine + Horizon
# Planner), ROS-free — so the dev harness exercises the real goal/reward/A-B layer, not a copy.
from behavior.brain import PurposeBrain                      # noqa: E402

WEB_DIR = os.path.join(_ROOT, "src", "web_control", "web")
ROBOT_YAML = os.path.join(_ROOT, "src", "robot_bringup", "config", "robot.yaml")
# All dev-harness state — the "soul" (personality.json), the decision log (cognition.log),
# and the goal/reward state (purpose.json, experiments.json, phrases.json) — lives in ONE
# project-local folder so you can see/edit it in the repo while developing, instead of buried
# in ~/.local/state/nanobot (where the robot keeps its own). It's gitignored (volatile state).
DEV_STATE_DIR = os.path.join(_ROOT, "devstate")
os.makedirs(DEV_STATE_DIR, exist_ok=True)


def _dev_state(name):
    """A path inside the project-local dev-state folder (a robot.yaml *_path override wins)."""
    return os.path.join(DEV_STATE_DIR, name)


# Decision-log file (same JSON-lines format the robot's web_server uses). A robot.yaml
# cognition_log_path overrides; "" falls back to the project-local folder.
DEFAULT_COG_LOG = _dev_state("cognition.log")


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


def _load_personality():
    """Seed personality from personality.json (made by personality_creator.py); {} fields
    on any problem so the harness still runs with the chart's built-in defaults."""
    base = {"name": "Nano", "persona": "", "traits": {}, "registry": {}}
    try:
        with open(_dev_state("personality.json"), encoding="utf-8") as f:
            saved = json.load(f)
        if isinstance(saved, dict):
            for k in base:
                if k in saved:
                    base[k] = saved[k]
    except Exception:
        pass
    return base


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
        self.personality = _load_personality()              # seed traits/registry/persona
        persona = self.personality.get("persona") or cfg.get("llm_persona", "")
        name = self.personality.get("name", "Nano")
        self.tts = TtsEngine(default_voice=voice, on_word=self._on_word,
                             logger=lambda m: print(f"[tts] {m}", file=sys.stderr))
        self.llm = LlmClient(
            enabled=True, api_key="",                       # key from $OPENROUTER_API_KEY
            model=cfg.get("llm_model", ""), persona=persona,
            vision_model=cfg.get("llm_vision_model", ""),
            smart_model=cfg.get("llm_smart_model", ""),
            free_model=cfg.get("llm_free_model", ""),
            free_smart_model=cfg.get("llm_free_smart_model", ""),
            vision_fallback_model=cfg.get("llm_vision_fallback_model", ""),
            smart_max_per_hour=int(cfg.get("llm_smart_max_per_hour", 15)),
            vision_max_per_hour=int(cfg.get("llm_vision_max_per_hour", 10)),
            logger=lambda m: print(f"[llm] {m}", file=sys.stderr))
        # The SAME cognition core the robot runs (web_control.cognition) — one base to
        # maintain. We give it dev adapters: face -> print, camera -> webcam, sensors ->
        # synthetic, scan/actions -> unavailable (no ROS). The decision log + phrase bank +
        # skills + reflection are all the real, shared code.
        self.cog = CognitionCore(
            llm=self.llm, tts=self.tts, persona=persona, persona_name=name,
            traits=self.personality.get("traits"),
            settings={"enabled": True, "model": cfg.get("llm_model", "")},
            face=lambda m: print(f"\n[face] {m or 'dashboard'}", file=sys.stderr),
            capture_frame=_capture_webcam_jpeg, sensor_snapshot=self._synth_snapshot,
            sensor_signals=self._synth_signals, scan_summary=lambda: "no scan (dev harness)",
            audio_summary=lambda: "a quiet room with a faint hum (dev harness — no mic)",
            publish_action=lambda _a: (False, "no ROS on the dev harness — actions are robot-only"),
            logger=lambda m: print(f"[cog] {m}", file=sys.stderr), persist_settings=None,
            cog_log_path=(cfg.get("cognition_log_path") or "").strip() or DEFAULT_COG_LOG,
            face_hold=0.0,
            bank_path=(cfg.get("phrasebank_path") or "").strip() or _dev_state("phrases.json"),
            bank_enable=bool(cfg.get("phrasebank_enable", True)),
            bank_live_ratio=float(cfg.get("phrasebank_live_ratio", 0.2)),
            bank_drift=float(cfg.get("phrasebank_drift", 0.6)),
            bank_per_category=int(cfg.get("phrasebank_per_category", 6)),
            skills_dir=resolve_skills_dir(cfg.get("skills_dir", "")),
            skills_enable=bool(cfg.get("skills_enable", True)),
            skills_allow_actions=bool(cfg.get("skills_allow_actions", False)),
            self_model_enable=bool(cfg.get("self_model_enable", True)),
            self_model_path=(cfg.get("self_model_path") or "").strip() or _dev_state("self_model.json"),
            consolidate_every=int(cfg.get("consolidate_every", 6)),
            prelude_enable=bool(cfg.get("prelude_enable", True)),
            # The skill workshop mints/adapts skills into a project-local "learned" dir so you
            # can see them in the repo (deploy carries devstate/ to the board, like the soul).
            workshop_enable=bool(cfg.get("workshop_enable", True)),
            workshop_dir=(cfg.get("workshop_dir") or "").strip() or _dev_state("skills"),
            workshop_path=(cfg.get("workshop_path") or "").strip() or _dev_state("workshop.json"),
            workshop_rounds=int(cfg.get("workshop_rounds", 1)),
            workshop_min_runs=int(cfg.get("workshop_min_runs", 3)),
            workshop_retire_errors=int(cfg.get("workshop_retire_errors", 2)),
            workshop_retire_net_neg=int(cfg.get("workshop_retire_net_neg", 2)))
        self._pending_evolve = None                         # reflection -> chart (drained by loop)
        self._lock_pe = threading.Lock()
        # --- Purpose Engine + Horizon Planner (the real ROS-free brain orchestration) -----
        # On the dev host there's no behaviour node, so we run the SAME PurposeBrain the robot
        # runs (behavior.brain) so the web "🧠 Brain" card is fully exercisable: objective +
        # reward weights, the pursuing/skill beats, A/B variants + 👍/👎 reward, and meditation.
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
            offline = bool(self.cog.settings.get("enabled")) and not self.llm.available()
            if offline != self._dev_offline:
                prev = self._dev_offline
                self._dev_offline = offline
                if offline:
                    self.cog.speak_lifecycle("offline", face="sleepy")
                elif prev:                                  # only if we WERE offline
                    print("\n[face] dashboard (LLM online)", file=sys.stderr)

    @staticmethod
    def _synth_signals():
        """A plausible, slightly-random structured body snapshot (dev stand-in for the
        robot's real sensors) — the core's sensor_signals adapter."""
        return {"cpu": random.randint(8, 90), "mem": random.randint(28, 70),
                "temp": random.randint(34, 64),
                "moving": random.random() < 0.25,
                "tilt": random.choice([2, 6, 14, 22, 30]),
                "pickup": random.choice([0, 0, 0, 1, 2])}

    def _synth_snapshot(self):
        """A synthetic plain-English body snapshot (the core's sensor_snapshot adapter)."""
        sig = self._synth_signals()
        body = ["being moved or jostled" if sig["moving"] else "physically still",
                f"tilted ({sig['tilt']} degrees)" if sig["tilt"] > 10 else "sitting level",
                {0: "resting on the ground", 1: "with one wheel off the ground",
                 2: "lifted off the ground (being held)"}[sig["pickup"]]]
        return (f"CPU load {sig['cpu']}%, memory {sig['mem']}% used, main board "
                f"{sig['temp']} degrees C, " + ", ".join(body))

    def _on_word(self, w):
        print(w, end=" ", flush=True) if w else print()

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

    # ---- statechart beat (used by --behavior) -------------------------------
    def fire_beat(self, name):
        """The chart's do_beat: run the matching enrichment in a worker thread (async,
        like the robot's fire-and-forget request) so the chart never waits on the LLM.

        Goal-pursuit + skill upgrades are decided by the shared PurposeBrain (exactly as on the
        robot): the idle musing beat becomes a `pursuing` beat when the planner has a verified
        task, else a `skill` beat every Nth body beat. Meditation pauses all beats."""
        if self._brain.meditating:
            print(f"\n[beat] {name} (paused — meditating)", file=sys.stderr)
            return
        print(f"\n[beat] {name}", file=sys.stderr)
        spec = self._brain.next_pursuing(time.monotonic()) if name == "musing" else None
        skill_beat = name == "musing" and spec is None and self._brain.take_skill_beat()

        def work():
            if spec is not None:
                self._deliver_pursuing(spec)
            elif skill_beat:
                self.cog.run_skill_beat("acting")       # let the brain pick a skill (shared core)
            elif name == "looking":
                self.cog.llm_look(trigger="beat:looking", state="looking")
            else:
                self.cog.llm_observe(trigger="beat:musing", state="musing")
        threading.Thread(target=work, daemon=True).start()

    def reflect(self, traits_dict):
        """Deep/slow tier: sync the chart's current traits into the core, run the (shared)
        reflection, and stash the proposal for the chart loop to apply (queueing on the
        interpreter from this worker thread would race the loop's execute())."""
        self.cog.update_traits(traits_dict)
        res = self.cog.reflect()                         # prompt/parse/log all in the shared core
        if res:
            with self._lock_pe:
                self._pending_evolve = (res["traits"], res["registry"])
            print(f"\n[reflect] {res.get('note','')} -> {res['traits']}", file=sys.stderr)

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
                          camera=bool(spec["camera"]), prelude=True)

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

    def brain_meditate(self, data):
        """Mirror web_server.brain_meditate: pause beats, consolidate (reflect + A/B + bank)."""
        on = bool(data.get("on"))
        self._brain.set_meditating(on)                  # flag + (on entry) reflect + A/B finalize
        if on:
            self.cog.bank_regen_check()
            threading.Thread(target=self.cog.consolidate, daemon=True).start()  # long-term self
            threading.Thread(target=self.cog.run_skill_workshop, daemon=True).start()  # mint a skill
        self.cog.log_decision("meditate", status=("on" if on else "off"))
        print(f"[meditate] {'on' if on else 'off'}", file=sys.stderr)
        return {"status": "ok", "meditating": self._brain.meditating}

    def brain_workshop(self):
        return self.cog.get_workshop()

    def workshop_keep(self, data):
        return self.cog.keep_skill(str(data.get("name", "")))

    def workshop_kill(self, data):
        return self.cog.kill_skill(str(data.get("name", "")))

    def tts_config(self):
        return {"voice": self.tts.voice, "volume": 100, "speed": 100, "pitch": 100,
                "announce": False, "announce_interval": 30}


def run_behavior(state, idle_secs, reflect_secs):
    """Run the ROS-free presence statechart on a real clock; its beats drive the local LLM
    (see DevState.fire_beat) and a periodic reflection drifts the traits (see
    DevState.reflect). Honours camera_beats/look_every + the seed personality from robot.yaml
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
    look_every = int(b.get("look_every", 4))
    clock = SimulatedClock()
    clock.time = 0.0
    interp, _ = build_interpreter(
        face=lambda _m: None,                  # dev: no OLED, faces are not shown
        do_beat=state.fire_beat,
        greet_secs=1.0, idle_secs=idle_secs, perform_secs=4.0,
        camera_beats=camera_beats, look_every=look_every,
        traits=state.personality.get("traits"), registry=state.personality.get("registry"),
        alpha=float(b.get("smoothing_alpha", 0.1)), clock=clock)
    print(f"[behavior] statechart running — idle_secs={idle_secs}, camera_beats="
          f"{camera_beats}, look_every={look_every}, reflect_secs={reflect_secs}. "
          f"Stop clicking and listen; watch the Decision log.", file=sys.stderr)
    t0 = last_reflect = last_purpose = time.monotonic()
    while True:
        time.sleep(0.5)
        now = time.monotonic()
        clock.time = now - t0
        if (now - last_purpose) > 30.0:        # local Purpose Engine: drift reward weights
            last_purpose = now
            state._brain.run_reflection()
        ev = state.take_evolve()               # apply reflection drift (queued single-threaded)
        if ev:
            interp.queue(Event("evolve", traits=ev[0], registry=ev[1]))
            print(f"[behavior] traits now {dict(interp.context['traits'])}", file=sys.stderr)
            state.cog.update_traits(dict(interp.context["traits"]))  # track soul for bank-regen
            state.cog.bank_regen_check()                     # refresh bank if it drifted too far
        try:
            interp.execute()
        except Exception as exc:
            print(f"[behavior] step error: {exc}", file=sys.stderr)
        if reflect_secs > 0 and state.llm.available() and (now - last_reflect) > reflect_secs:
            last_reflect = now
            threading.Thread(target=state.reflect,
                             args=(dict(interp.context["traits"]),), daemon=True).start()


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
        if p == "/tts/config":
            return self._json(self.state.tts_config())
        # Brain readouts — the robot serves these over rosbridge (latched topics); the dev
        # harness serves them over HTTP and the page polls them when rosbridge is absent.
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
        if p == "/skills/workshop/keep":
            return self._json(s.workshop_keep(self._body()))
        if p == "/skills/workshop/kill":
            return self._json(s.workshop_kill(self._body()))
        if p == "/brain/reward":
            return self._json(s.brain_reward(self._body()))
        if p == "/brain/meditate":
            return self._json(s.brain_meditate(self._body()))
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
            d = self._body()
            s.tts.configure(voice=d.get("voice"),
                            volume=d.get("volume"), speed=d.get("speed"), pitch=d.get("pitch"))
            return self._json(s.tts_config())
        if p == "/tts/stop":
            s.tts.stop()
            return self._text(200, "stopped")
        if p == "/tts/announce":
            return self._text(503, "no stats on the dev harness")
        return self._text(404, "not found")

    def log_message(self, *a):
        pass


def main():
    ap = argparse.ArgumentParser(description="Dev-only web UI harness (AI card + TTS, no ROS).")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--voice", default="en-US", help="en-US | en-GB | de-DE")
    ap.add_argument("--behavior", action="store_true",
                    help="also run the presence statechart so beats drive the LLM")
    ap.add_argument("--idle-secs", type=float, default=15.0,
                    help="(--behavior) seconds idle before each beat (default 15)")
    ap.add_argument("--reflect-secs", type=float, default=90.0,
                    help="(--behavior) seconds between personality reflections (0=off, default 90)")
    args = ap.parse_args()

    state = DevState(args.voice)
    if args.behavior:
        threading.Thread(target=run_behavior,
                         args=(state, args.idle_secs, args.reflect_secs), daemon=True).start()
    handler = functools.partial(Handler, directory=WEB_DIR, state=state)
    httpd = http.server.ThreadingHTTPServer(("0.0.0.0", args.port), handler)
    print(f"\n  Dev web UI:  http://localhost:{args.port}\n"
          f"  Speak tab -> 'AI · OpenRouter' card; open '🧠 Decision log' to watch decisions.\n"
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
