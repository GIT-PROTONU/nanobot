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
from collections import deque

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.normpath(os.path.join(_HERE, ".."))
sys.path.insert(0, os.path.join(_ROOT, "src", "web_control"))
sys.path.insert(0, os.path.join(_ROOT, "src", "behavior"))   # ROS-free presence chart

from web_control.tts import TtsEngine, VOICES, clamp        # noqa: E402
from web_control.llm import LlmClient, MOODS, _extract_json  # noqa: E402

WEB_DIR = os.path.join(_ROOT, "src", "web_control", "web")
ROBOT_YAML = os.path.join(_ROOT, "src", "robot_bringup", "config", "robot.yaml")
LOG_MAX = 50
TRAITS = ("curiosity", "extraversion", "caution", "playfulness")


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
        with open(os.path.expanduser("~/.local/state/nanobot/personality.json"),
                  encoding="utf-8") as f:
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
        self.personality = _load_personality()              # seed traits/registry/persona
        persona = self.personality.get("persona") or cfg.get("llm_persona", "")
        self.tts = TtsEngine(default_voice=voice, on_word=self._on_word,
                             logger=lambda m: print(f"[tts] {m}", file=sys.stderr))
        self.llm = LlmClient(
            enabled=True, api_key="",                       # key from $OPENROUTER_API_KEY
            model=cfg.get("llm_model", ""), persona=persona,
            vision_model=cfg.get("llm_vision_model", ""),
            smart_model=cfg.get("llm_smart_model", ""),
            logger=lambda m: print(f"[llm] {m}", file=sys.stderr))
        self.settings = {"enabled": True, "model": cfg.get("llm_model", ""), "persona": persona}
        self.history = []
        self._busy = False
        self._lock = threading.Lock()
        self._log = deque(maxlen=LOG_MAX)                   # decision log (in-memory on dev)
        self._loglock = threading.Lock()
        self._pending_evolve = None                         # reflection -> chart (drained by loop)
        self._lock_pe = threading.Lock()
        self._busy_reflect = False
        print(f"[dev] TTS backend ready={self.tts.available()}  "
              f"LLM ready={self.llm.available()} (model {self.llm.model})", file=sys.stderr)
        if not self.llm.available():
            print("[dev] ! No OPENROUTER_API_KEY set — the AI card will say 'unavailable'.",
                  file=sys.stderr)

    def _on_word(self, w):
        print(w, end=" ", flush=True) if w else print()

    # ---- decision log -------------------------------------------------------
    def _log_decision(self, trigger, state="", camera=False, status="", model="",
                      prompt="", say="", mood="", ms=0, detail=""):
        entry = {"t": time.time(), "trigger": trigger, "state": state,
                 "camera": bool(camera), "model": model, "prompt": (prompt or "")[:160],
                 "say": say, "mood": mood, "status": status, "detail": detail, "ms": ms}
        with self._loglock:
            self._log.append(entry)
        return entry

    def get_cog_log(self):
        with self._loglock:
            return {"entries": list(self._log)[::-1]}

    # ---- the same shapes web_server returns ----
    def llm_config(self):
        s = dict(self.settings)
        s.update(available=self.llm.available(), configured=self.llm.available(),
                 model_effective=self.llm.model, smart_model=self.llm.smart_model,
                 vision_model=self.llm.vision_model, moods=list(MOODS))
        return s

    def update_llm_config(self, data):
        for k in ("enabled", "model", "persona"):
            if k in data:
                self.settings[k] = data[k]
        self.llm.configure(enabled=bool(self.settings["enabled"]),
                          model=str(self.settings["model"] or ""),
                          persona=str(self.settings["persona"] or ""))
        return self.llm_config()

    def _express(self, reply):
        if reply and reply.get("mood") and reply["mood"] != "neutral":
            print(f"\n[face] {reply['mood']}", file=sys.stderr)   # no OLED on a dev PC
        if reply and reply.get("say") and self.tts.available():
            print("speaking: ", end="", flush=True)
            self.tts.say(reply["say"])

    def _generate(self, prompt, history=None, image_jpeg=None, trigger="manual",
                  state="", camera=False, smart=False):
        """Blocking generate + express + log (matches web_server's guard + logging)."""
        if not self.llm.available():
            self._log_decision(trigger, state, camera, status="llm-unavailable")
            return None
        with self._lock:
            if self._busy:
                self._log_decision(trigger, state, camera, status="skipped-busy")
                return None
            self._busy = True
        t0 = time.monotonic()
        try:
            reply = self.llm.generate(prompt, history=history, image_jpeg=image_jpeg,
                                      smart=smart)
        finally:
            self._busy = False
        model = self.llm.model_for(smart=smart, image=bool(image_jpeg))
        ms = int((time.monotonic() - t0) * 1000)
        if reply:
            self._express(reply)
            self._log_decision(trigger, state, camera, status="spoke", model=model,
                               prompt=prompt, say=reply["say"], mood=reply["mood"], ms=ms)
        else:
            self._log_decision(trigger, state, camera, status="no-reply", model=model,
                               prompt=prompt, ms=ms)
        return reply

    def llm_look(self, trigger="look", state=""):
        frame = _capture_webcam_jpeg()
        if frame is None:
            self._log_decision(trigger, state, True, status="no-frame")
            return {"error": "no webcam frame (install opencv-python? camera in use?)"}
        print(f"[look] captured {len(frame)} byte webcam frame -> {self.llm.vision_model}",
              file=sys.stderr)
        prompt = ("This is the live view from your own camera. In character, say one short "
                  "spoken line about what you can see in front of you right now, and pick "
                  "a fitting mood.")
        return self._generate(prompt, image_jpeg=frame, trigger=trigger, state=state, camera=True)

    def llm_say(self, prompt):
        prompt = (prompt or "").strip() or (
            "Say one short, friendly, spontaneous line and pick a fitting mood.")
        return self._generate(prompt, trigger="say")

    def llm_observe(self, trigger="observe", state=""):
        # No ROS sensors on a dev PC, so synthesise a plausible snapshot (slightly random
        # so it varies) — enough to test that the robot comments on its sensor state.
        cpu, mem, temp = random.randint(8, 45), random.randint(28, 62), random.randint(43, 59)
        body = random.choice([
            "physically still, sitting level, resting on the ground",
            "sitting level and resting on the ground",
            "being moved or jostled, sitting level, on the ground",
            "leaning slightly (14 degrees), resting on the ground",
            "physically still, lifted off the ground (being held)",
        ])
        snap = f"CPU load {cpu}%, memory {mem}% used, main board {temp} degrees C, {body}"
        print(f"[observe] (synthetic) {snap}", file=sys.stderr)
        prompt = (f"Your own body's sensors report right now: {snap}. In character, say "
                  "one short spoken line reacting to how you physically feel or what your "
                  "sensors notice, and pick a fitting mood.")
        return self._generate(prompt, trigger=trigger, state=state)

    def llm_chat(self, message):
        message = (message or "").strip()
        if not message:
            return None
        reply = self._generate(message, history=list(self.history), trigger="chat", smart=True)
        if reply:
            self.history += [{"role": "user", "content": message},
                             {"role": "assistant", "content": reply["say"]}]
            self.history = self.history[-8:]
        return reply

    # ---- statechart beat (used by --behavior) -------------------------------
    def fire_beat(self, name):
        """The chart's do_beat: run the matching enrichment in a worker thread (async,
        like the robot's fire-and-forget request) so the chart never waits on the LLM."""
        print(f"\n[beat] {name}", file=sys.stderr)

        def work():
            if name == "looking":
                self.llm_look(trigger="beat:looking", state="looking")
            else:
                self.llm_observe(trigger="beat:musing", state="musing")
        threading.Thread(target=work, daemon=True).start()

    def reflect(self, traits_dict):
        """Deep/slow tier: read recent events + current traits, propose smoothed trait
        drift, and stash it for the chart loop to apply (queueing on the interpreter from
        this worker thread would race the loop's execute())."""
        if self._busy_reflect:
            return
        self._busy_reflect = True
        try:
            tp = ", ".join(f"{k} {traits_dict.get(k, 0.5):.2f}" for k in TRAITS)
            with self._loglock:
                ev = "\n".join(f"- {e['trigger']} [{e['status']}] {e.get('say','')}".rstrip()
                               for e in list(self._log)[-20:]) or "(no recent events)"
            system = (
                "You are the slow, reflective mind of a small robot named Nano. You review "
                "what just happened and gently adjust its personality so it grows over time. "
                "Traits are 0..1: curiosity, extraversion, caution, playfulness. Output ONLY "
                'compact JSON: {"traits": {<trait>: <new target 0..1>}, "registry": '
                '{optional}, "note": "<one short reason>"}. Propose only SMALL, justified '
                "nudges to a few traits (omit ones you would not change); the value is a "
                "TARGET that gets smoothed over time. No prose outside the JSON.")
            user = f"Current traits: {tp}.\nRecent events:\n{ev}\n\nReflect and propose adjustments."
            obj = _extract_json(self.llm.complete(system, user, smart=True, json_object=True) or "")
            t = {k: max(0.0, min(1.0, float(obj["traits"][k]))) for k in TRAITS
                 if isinstance(obj.get("traits"), dict) and k in obj["traits"]}
            reg = obj.get("registry") if isinstance(obj.get("registry"), dict) else {}
            if t or reg:
                with self._lock_pe:
                    self._pending_evolve = (t, reg)
                self._log_decision("reflect", status="spoke", model=self.llm.smart_model,
                                   say=f"{obj.get('note','')} -> {t}")
                print(f"\n[reflect] {obj.get('note','')} -> {t}", file=sys.stderr)
            else:
                self._log_decision("reflect", status="no-reply", model=self.llm.smart_model)
        finally:
            self._busy_reflect = False

    def take_evolve(self):
        with self._lock_pe:
            ev, self._pending_evolve = self._pending_evolve, None
        return ev

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
    t0 = last_reflect = time.monotonic()
    while True:
        time.sleep(0.5)
        now = time.monotonic()
        clock.time = now - t0
        ev = state.take_evolve()               # apply reflection drift (queued single-threaded)
        if ev:
            interp.queue(Event("evolve", traits=ev[0], registry=ev[1]))
            print(f"[behavior] traits now {dict(interp.context['traits'])}", file=sys.stderr)
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
        if p == "/tts/config":
            return self._json(self.state.tts_config())
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
