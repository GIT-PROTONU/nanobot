"""The robot's cognition core — ALL the LLM-personality logic, ROS-free, in ONE place.

This is the single home for: generating a spoken line + face (`generate`), the on-demand
say/chat/observe/look paths, the statechart beat executor (`run_beat`), the skill library
(pick + invoke + the gated action tier), the phrase bank, the decision log, slow personality
reflection, and lifecycle speech. It deliberately has **no `rclpy` import**, so the *same*
code runs on the robot (`web_control.web_server`, a ROS node) and on a dev PC
(`scripts/dev_webui.py`, no ROS) — there is only one base to maintain.

Everything platform-specific is injected as a tiny **adapter** (a few callables), so the core
never knows whether a "face" is an OLED topic or a `print`, whether a camera frame comes from
V4L2 or a laptop webcam, or whether an action publishes a real ROS message:

    adapter callables (all required):
      face(mood)              -> show an OLED mood ("" clears to the dashboard)
      capture_frame()         -> one JPEG (bytes) or None
      sensor_snapshot()       -> a plain-English body description (str)
      sensor_signals()        -> the structured body dict for the phrase-bank classifier
      scan_summary()          -> one line about the latest lidar scan (str)
      publish_action(action)  -> (ok: bool, detail: str); the GATED topic tier (no-op off-robot)
      logger(msg)             -> log a line
      persist_settings(s)     -> persist the {enabled,model} UI settings (None = don't persist)

The TTS engine + the constructed `LlmClient` are passed in too (their construction differs by
platform — env key vs param key). Degrades to silence on every error: the brain is a garnish,
never load-bearing.
"""
import json
import os
import random
import threading
import time
from collections import deque

from .llm import MOODS, _extract_json
from .phrasebank import PhraseBank
from .skills import SkillLibrary

REFLECT_TRAITS = ("curiosity", "extraversion", "caution", "playfulness")  # personality axes
LLM_HISTORY_MAX = 8          # chat turns kept for context (user+assistant messages)
LLM_LOG_MAX = 50             # decision-log ring buffer length (also what the file tail loads)


def clamp01(v, lo=0.0, hi=1.0):
    try:
        return max(lo, min(hi, float(v)))
    except (TypeError, ValueError):
        return lo


class CognitionCore:
    """The shared LLM-personality brain. See the module docstring for the adapter contract."""

    def __init__(self, *, llm, tts, persona="", persona_name="Nano", traits=None,
                 settings=None, face=None, capture_frame=None, sensor_snapshot=None,
                 sensor_signals=None, scan_summary=None, publish_action=None,
                 logger=None, persist_settings=None, cog_log_path="", face_hold=10.0,
                 bank_path=None, bank_enable=True, bank_live_ratio=0.2, bank_drift=0.6,
                 bank_per_category=8, skills_dir="", skills_enable=True,
                 skills_allow_actions=False):
        self.llm = llm
        self.tts = tts
        self.persona = (persona or "").strip()
        self.persona_name = persona_name or "Nano"
        self.traits = {k: 0.5 for k in REFLECT_TRAITS}
        self.traits["caution"] = 0.6
        self.update_traits(traits or {})
        self.settings = dict(settings or {"enabled": llm.available(), "model": ""})
        # platform adapters
        self._face = face or (lambda _m: None)
        self._capture_frame = capture_frame or (lambda: None)
        self._sensor_snapshot = sensor_snapshot or (lambda: "no sensor data available")
        self._sensor_signals = sensor_signals or (lambda: {})
        self._scan_summary = scan_summary or (lambda: "no scan available")
        self._publish_action = publish_action or (lambda _a: (False, "no action backend"))
        self._log = logger or (lambda *_: None)
        self._persist_settings = persist_settings
        self._face_hold = float(face_hold)
        # decision log (ring buffer backed by a JSON-lines file, shared with the robot/dev)
        self._cog_log_path = cog_log_path or os.path.expanduser(
            "~/.local/state/nanobot/cognition.log")
        self._log_lock = threading.Lock()
        self._cog_log = deque(self._load_cog_log(), maxlen=LLM_LOG_MAX)
        # one-at-a-time LLM guard + chat history + offline streak (read by the health tick)
        self._llm_lock = threading.Lock()
        self._llm_busy = False
        self._llm_history = deque(maxlen=LLM_HISTORY_MAX)
        self.llm_fail_streak = 0
        # phrase bank + skill library (the shared modules)
        self._bank_enable = bool(bank_enable)
        self._bank_live_ratio = float(bank_live_ratio)
        self._bank_drift = float(bank_drift)
        self._bank_per_cat = int(bank_per_category)
        self._bank = PhraseBank(path=bank_path, logger=self._log)
        self.skills_enable = bool(skills_enable)
        self.skills_allow_actions = bool(skills_allow_actions)
        self.skills_dir = skills_dir
        self._skills = SkillLibrary(skills_dir if skills_enable else "", logger=self._log)
        self._reflect_busy = False

    def available(self):
        return self.llm.available()

    # ---- traits -------------------------------------------------------------
    def update_traits(self, traits):
        if isinstance(traits, dict):
            self.traits.update({k: clamp01(traits[k]) for k in REFLECT_TRAITS if k in traits})

    def traits_phrase(self):
        return ", ".join(f"{k} {self.traits.get(k, 0.5):.2f}" for k in REFLECT_TRAITS)

    # ---- decision log -------------------------------------------------------
    def _load_cog_log(self):
        """Seed the ring from the file's last LLM_LOG_MAX JSON lines (history across reboots
        / dev runs, shared file). Best-effort (returns [] on any problem)."""
        try:
            with open(self._cog_log_path, encoding="utf-8") as f:
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

    def log_decision(self, trigger, state="", camera=False, status="", model="",
                     prompt="", say="", mood="", ms=0, detail=""):
        """Record one cognition decision (+ outcome) to the ring buffer and append it as a
        JSON line to the log file. Log failures never block a decision."""
        entry = {"t": time.time(), "trigger": trigger, "state": state,
                 "camera": bool(camera), "model": model, "prompt": (prompt or "")[:160],
                 "say": say, "mood": mood, "status": status, "detail": detail, "ms": ms}
        with self._log_lock:
            self._cog_log.append(entry)
        try:
            os.makedirs(os.path.dirname(self._cog_log_path), exist_ok=True)
            with open(self._cog_log_path, "a", encoding="utf-8") as f:
                f.write(json.dumps(entry) + "\n")
        except Exception:
            pass
        return entry

    def get_cog_log(self):
        with self._log_lock:
            return {"entries": list(self._cog_log)[::-1]}   # newest first

    def recent_events_text(self, n=25):
        with self._log_lock:
            entries = list(self._cog_log)[-n:]
        lines = [f"- {e.get('trigger','')} [{e.get('status','')}] "
                 f"{e.get('say') or e.get('detail') or ''}".rstrip() for e in entries]
        return "\n".join(lines) or "(no recent events)"

    # ---- express + generate -------------------------------------------------
    def express(self, mood, say):
        """Show a mood on the face and speak the line. Optionally clear the face back to the
        dashboard after face_hold seconds (0 = leave it up like a manual mood)."""
        if mood and mood != "neutral":
            self._face(mood)
            if self._face_hold > 0:
                self._later(self._face_hold, lambda: self._face(""))
        if say and self.tts is not None and self.tts.available():
            self.tts.say(say)

    def generate(self, prompt, history=None, image_jpeg=None, trigger="manual",
                 state="", camera=False, smart=False):
        """Blocking generate + express, guarded so only one call runs at a time (little
        RAM/CPU; the API costs money). Records the decision + outcome. Returns the reply dict
        or None. Safe to call from any worker thread."""
        if not self.llm.available():
            self.log_decision(trigger, state, camera, status="llm-unavailable")
            return None
        with self._llm_lock:
            if self._llm_busy:
                self.log_decision(trigger, state, camera, status="skipped-busy")
                return None
            self._llm_busy = True
        t0 = time.monotonic()
        try:
            reply = self.llm.generate(prompt, history=history, image_jpeg=image_jpeg,
                                      smart=smart)
        finally:
            self._llm_busy = False
        model = self.llm.last_model or self.llm.model_for(smart=smart, image=bool(image_jpeg))
        ms = int((time.monotonic() - t0) * 1000)
        if reply:
            self.llm_fail_streak = 0                    # a real call succeeded -> online
            self.express(reply["mood"], reply["say"])
            self.log_decision(trigger, state, camera, status="spoke", model=model,
                              prompt=prompt, say=reply["say"], mood=reply["mood"], ms=ms)
        else:
            self.llm_fail_streak += 1                   # feeds the persistent offline indicator
            self.log_decision(trigger, state, camera, status="no-reply", model=model,
                              prompt=prompt, ms=ms)
        return reply

    # ---- on-demand interactions (the AI card) -------------------------------
    def llm_say(self, prompt=""):
        prompt = (prompt or "").strip() or (
            "Say one short, friendly, spontaneous line out loud to whoever is near you "
            "right now, and pick a fitting mood.")
        return self.generate(prompt, trigger="say")

    def llm_chat(self, message):
        message = (message or "").strip()
        if not message:
            return None
        history = list(self._llm_history)
        reply = self.generate(message, history=history, trigger="chat", smart=True)
        if reply:                                       # remember the exchange for context
            self._llm_history.append({"role": "user", "content": message})
            self._llm_history.append({"role": "assistant", "content": reply["say"]})
        return reply

    def llm_observe(self, trigger="observe", state=""):
        """Snapshot the body and have the robot comment on how it feels (bank-first)."""
        bank = self.bank_say(trigger, state)            # frequent -> prefer the cached bank
        if bank:
            return bank
        snap = self._sensor_snapshot()
        self._log(f"llm observe: {snap}")
        prompt = (f"Your own body's sensors report right now: {snap}. In character, say "
                  "one short spoken line reacting to how you physically feel or what your "
                  "sensors notice, and pick a fitting mood.")
        return self.generate(prompt, trigger=trigger, state=state)

    def llm_look(self, trigger="look", state=""):
        """Capture a frame (+ the sensor snapshot) and comment on what it SEES (vision)."""
        if not self.llm.can_call(image=True):           # capped: skip the capture entirely
            self.log_decision(trigger, state, True, status="rate-limited")
            return {"error": "vision hourly limit reached"}
        frame = self._capture_frame()
        if frame is None:
            self.log_decision(trigger, state, True, status="no-frame")
            return {"error": "no camera frame"}
        snap = self._sensor_snapshot()
        self._log(f"llm look: {len(frame)} byte frame; {snap}")
        prompt = ("This is the live view from your own camera. Your body also senses: "
                  f"{snap}. In character, say one short spoken line about what you can "
                  "see in front of you right now, and pick a fitting mood.")
        return self.generate(prompt, image_jpeg=frame, trigger=trigger, state=state, camera=True)

    # ---- statechart beat executor (the chart's /cognition/request) ----------
    def run_beat(self, trigger, state, prompt, camera):
        """Execute one enrichable beat: capture a frame if asked (else try the cached phrase
        bank for a body line), add the live sensors + personality, generate + express."""
        frame = None
        if camera:
            if not self.llm.can_call(image=True):       # don't spin up the camera if capped
                self.log_decision(trigger, state, camera, status="rate-limited")
                return
            frame = self._capture_frame()
            if frame is None:
                self.log_decision(trigger, state, camera, status="no-frame")
                return
        elif self.bank_say(trigger, state):             # frequent body beat -> cached line
            return                                       # (free/instant/offline; no LLM call)
        full = (prompt + " Your current personality (0..1) is " + self.traits_phrase()
                + ", and your body senses: " + self._sensor_snapshot())
        self.generate(full, image_jpeg=frame, trigger=trigger, state=state, camera=camera)

    # ---- phrase bank --------------------------------------------------------
    def bank_say(self, trigger, state="", camera=False):
        """Try the pre-generated phrase bank for a body-reaction line: classify the live
        sensors, pick + fill a cached line, speak/emote it, log it. Returns the reply dict if
        a line was used, else None (honours the live-LLM ratio for variety)."""
        if not self._bank_enable or camera:
            return None
        if random.random() < self._bank_live_ratio:     # occasionally go live for variety
            return None
        reply = self._bank.pick(self._sensor_signals())
        if not reply:
            return None
        self.express(reply["mood"], reply["say"])
        self.log_decision(trigger, state, camera, status="bank", model="phrasebank",
                          say=reply["say"], mood=reply["mood"], detail=reply["category"])
        return reply

    def bank_regen_check(self):
        """Regenerate the bank in the background if empty / soul drifted too far (no-op if
        disabled / LLM offline / drift small)."""
        if self._bank_enable and self.llm.available():
            self._bank.maybe_regenerate(self.llm, self.persona, self.traits,
                                        name=self.persona_name, threshold=self._bank_drift,
                                        per_category=self._bank_per_cat, background=True)

    def get_phrasebank(self):
        s = self._bank.stats()
        s["enabled"] = self._bank_enable
        s["live_ratio"] = self._bank_live_ratio
        s["needs_regen"] = self._bank.needs_regen(self.persona, self.traits, self._bank_drift)
        return s

    def regenerate_phrasebank(self):
        """Force a (background) regeneration regardless of drift — for the web UI button."""
        if not self.llm.available():
            return {"error": "llm unavailable"}
        self._bank.maybe_regenerate(self.llm, self.persona, self.traits,
                                    name=self.persona_name, threshold=-1.0,  # <0 => always
                                    per_category=self._bank_per_cat, background=True)
        return {"status": "regenerating"}

    # ---- skill library ------------------------------------------------------
    def get_skills(self):
        return {"enabled": self.skills_enable, "allow_actions": self.skills_allow_actions,
                "dir": self.skills_dir, "error": self._skills.error,
                "skills": self._skills.as_list()}

    def reload_skills(self):
        if not self.skills_enable:
            return {"error": "skills disabled"}
        self._skills.reload()
        return self.get_skills()

    def invoke_skill(self, name):
        """On-demand: run a named skill now (blocks on any LLM/express call, like the AI card)."""
        if not self.skills_enable:
            return {"error": "skills disabled"}
        skill = self._skills.get(name)
        if skill is None:
            return {"error": "unknown skill: %s" % name}
        return self._invoke_skill(skill, trigger="skill:" + skill.name, state="manual")

    def run_skill_beat(self, state="acting"):
        """Autonomous skill beat: ask the cheap model to pick the most fitting offered skill
        for this moment (or none), then perform it. Best-effort — no pick = a silent beat."""
        if not self.skills_enable:
            self.log_decision("beat:skill", state, status="skills-disabled")
            return
        cat = self._skills.format_catalogue(self.skills_allow_actions)
        if not cat:
            self.log_decision("beat:skill", state, status="no-skills")
            return
        system = ("You choose which ONE of a small robot's capabilities best fits this "
                  'moment, or none. Reply with ONLY compact JSON {"skill": "<name>"} using '
                  'an EXACT name from the list, or {"skill": ""} to do nothing. No prose.')
        user = ("Capabilities:\n%s\n\nYour body senses: %s.\nYour personality (0..1): %s.\n"
                "Pick the single most fitting capability to do now, or none."
                % (cat, self._sensor_snapshot(), self.traits_phrase()))
        content = self.llm.complete(system, user, json_object=True)
        skill = self._skills.choose(content or "", self.skills_allow_actions)
        if skill is None:
            self.log_decision("beat:skill", state, status="no-pick",
                              model=(self.llm.last_model or self.llm.model),
                              detail=(content or "")[:80])
            return
        self._invoke_skill(skill, trigger="skill:" + skill.name, state=state)

    def _skill_prompt(self, skill):
        """The user-turn steering a narrative skill: its body + any requested live context
        (sensors / lidar) + the current personality. {say,mood} shape is enforced by the LLM
        client's SYSTEM_BASE, so we just supply the steering."""
        parts = [skill.body or skill.description or skill.name]
        srcs = skill.sources
        if "sensors" in srcs:
            parts.append("Your body senses right now: " + self._sensor_snapshot() + ".")
        if "scan" in srcs:
            parts.append("Your lidar reports: " + self._scan_summary() + ".")
        parts.append("Your personality (0..1) is " + self.traits_phrase() + ".")
        parts.append("Reply with one short spoken line and a fitting mood.")
        return " ".join(p.strip() for p in parts if p and p.strip())

    def _invoke_skill(self, skill, trigger, state=""):
        """Perform a skill: narrative kinds generate+speak (optionally with a camera frame);
        the gated `topic` kind publishes a whitelisted ROS message. Returns a status dict."""
        if skill.is_action:
            return self._do_topic_skill(skill, trigger, state)
        frame = None
        if skill.camera:
            if not self.llm.can_call(image=True):
                self.log_decision(trigger, state, True, status="rate-limited")
                return {"error": "vision hourly limit reached"}
            frame = self._capture_frame()
            if frame is None:
                self.log_decision(trigger, state, True, status="no-frame")
                return {"error": "no camera frame"}
        reply = self.generate(self._skill_prompt(skill), image_jpeg=frame,
                              trigger=trigger, state=state, camera=bool(frame))
        return reply or {"error": "no reply"}

    def _do_topic_skill(self, skill, trigger, state):
        """Execute a gated topic-skill: publish a whitelisted, clamped message (+ an optional
        literal face/line). Refused unless actions are permitted AND the skill is enabled. The
        actual publish is the adapter's `publish_action` (a no-op off-robot)."""
        if not self.skills_allow_actions:
            self.log_decision(trigger, state, status="actions-disabled",
                              detail=str(skill.action.get("topic", "")))
            return {"error": "skill actions disabled (skills_allow_actions=false)"}
        if not skill.enabled:
            self.log_decision(trigger, state, status="skill-disabled")
            return {"error": "skill not enabled"}
        ok, detail = self._publish_action(skill.action)
        face = str(skill.action.get("face") or "")
        say = str(skill.action.get("say") or "")
        if ok and (face or say):
            self.express(face, say)                      # literal expression (no LLM call)
        self.log_decision(trigger, state, status=("acted" if ok else "error"),
                          model="action", say=say, mood=face, detail=detail)
        return {"status": "acted" if ok else "error", "detail": detail, "name": skill.name}

    # ---- lifecycle speech ---------------------------------------------------
    def speak_lifecycle(self, category, face=None):
        """Speak a pre-generated lifecycle line (greeting/farewell/restarting/offline) and
        optionally set a face. Offline-safe (phrase bank FALLBACK_LINES), best-effort."""
        try:
            reply = self._bank.pick(None, name=self.persona_name, category=category)
        except Exception:
            reply = None
        say = reply["say"] if reply else ""
        if face:
            self._face(face)
        if say and self.tts is not None and self.tts.available():
            self.tts.say(say)
        self.log_decision("life:" + category, status=("spoke" if say else "no-line"),
                          model="phrasebank", say=say, mood=(face or ""))
        return say

    # ---- slow personality reflection ----------------------------------------
    def reflect(self):
        """The deep/slow tier: review the recent decision log + current traits and propose
        SMALL smoothed trait/registry nudges. Returns {"traits","registry"} (already logged)
        or None — the caller delivers it (robot: /cognition/evolve; dev: the chart). Guarded
        so two reflections never overlap."""
        if self._reflect_busy or not self.llm.available():
            return None
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
            user = (f"Current traits: {self.traits_phrase()}.\nRecent events:\n"
                    f"{self.recent_events_text()}\n\nReflect and propose adjustments.")
            content = self.llm.complete(system, user, smart=True, json_object=True)
        finally:
            self._reflect_busy = False
        ms = int((time.monotonic() - t0) * 1000)
        obj = _extract_json(content or "")
        traits = {k: clamp01(obj["traits"][k]) for k in REFLECT_TRAITS
                  if isinstance(obj.get("traits"), dict) and k in obj["traits"]}
        registry = obj.get("registry") if isinstance(obj.get("registry"), dict) else {}
        rmodel = self.llm.last_model or self.llm.smart_model
        if not traits and not registry:
            self.log_decision("reflect", status="no-reply", model=rmodel, ms=ms)
            return None
        self.log_decision("reflect", status="spoke", model=rmodel,
                          say=f"{obj.get('note','')} -> {traits}", ms=ms)
        return {"traits": traits, "registry": registry, "note": obj.get("note", "")}

    # ---- LLM settings (web-tunable; persisted by the adapter) ----------------
    def get_llm_settings(self):
        s = dict(self.settings)
        s["available"] = self.llm.available()           # enabled AND a key is configured
        s["configured"] = self.llm.available()
        s["model_effective"] = self.llm.model
        s["smart_model"] = self.llm.smart_model
        s["vision_model"] = self.llm.vision_model
        s["free_model"] = self.llm.free_model           # free primaries (paid models are fallbacks)
        s["free_smart_model"] = self.llm.free_smart_model
        s["persona"] = self.persona                     # read-only: single-sourced from personality.json
        s["moods"] = list(MOODS)
        s["rate_limits"] = self.llm.rate_limits()       # {tier: [used_last_hour, cap]}; 0 cap = off
        return s

    def update_llm_settings(self, data):
        if "enabled" in data:
            self.settings["enabled"] = bool(data["enabled"])
        if "model" in data:
            self.settings["model"] = str(data["model"] or "")[:120]
        self.llm.configure(enabled=self.settings["enabled"], model=self.settings["model"])
        if self._persist_settings is not None:
            self._persist_settings(dict(self.settings))
        return self.get_llm_settings()

    # ---- util ---------------------------------------------------------------
    @staticmethod
    def _later(delay, fn):
        t = threading.Timer(max(0.0, float(delay)), fn)
        t.daemon = True
        t.start()
