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
      audio_summary()         -> one line about what the microphone currently hears (str)
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
from .skills import SkillLibrary, _slug
from .skillsmith import WorkshopState, render_skill_md, validate_candidate

REFLECT_TRAITS = ("curiosity", "extraversion", "caution", "playfulness")  # personality axes
# New LLM-steerable expressive DRIVES (above the 4 traits) the chart exposes — see
# behavior/presence.py. Kept as plain strings here so web_control never imports the behavior pkg.
REFLECT_DRIVES = ("energy", "focus", "introspection")     # 0..1 scalars, smoothed in the chart
DRIVE_MOODS = ("", "happy", "angry", "focused", "stress", "neutral", "looking", "sleepy")  # idle tint
LLM_HISTORY_MAX = 8          # chat turns kept for context (user+assistant messages)
LLM_LOG_MAX = 50             # decision-log ring buffer length (also what the file tail loads)

# Skill `sources:` -> (prompt template, the CognitionCore method that supplies the text).
# Adding a new live-context source for skills is one row here (+ its adapter). The aliases let
# a skill file say sound/mic/events/recent and resolve to the canonical source.
SKILL_SOURCES = {
    "sensors": ("Your body senses right now: {}.", "_sensor_snapshot"),
    "scan":    ("Your lidar reports: {}.", "_scan_summary"),
    "audio":   ("Through your microphone you hear: {}.", "_audio_summary"),
    "memory":  ("Lately you have been doing:\n{}", "recent_events_text"),
}
SOURCE_ALIASES = {"sound": "audio", "mic": "audio", "events": "memory", "recent": "memory"}


def clamp01(v, lo=0.0, hi=1.0):
    try:
        return max(lo, min(hi, float(v)))
    except (TypeError, ValueError):
        return lo


class CognitionCore:
    """The shared LLM-personality brain. See the module docstring for the adapter contract."""

    def __init__(self, *, llm, tts, persona="", persona_name="Nano", traits=None,
                 settings=None, face=None, capture_frame=None, sensor_snapshot=None,
                 sensor_signals=None, scan_summary=None, audio_summary=None,
                 publish_action=None,
                 logger=None, persist_settings=None, cog_log_path="", face_hold=10.0,
                 bank_path=None, bank_enable=True, bank_live_ratio=0.2, bank_drift=0.6,
                 bank_per_category=8, bank_grow_enable=True, bank_grow_period=1800.0,
                 bank_grow_max=24, bank_grow_batch=3, skills_dir="", skills_enable=True,
                 skills_allow_actions=False, self_model_path=None, self_model_enable=True,
                 consolidate_every=6, self_model_max_chars=600,
                 trait_history_path=None, trait_history_enable=True,
                 trait_history_period=3600.0, trait_history_max=336,
                 trait_history_window=604800.0, prelude_enable=True,
                 prelude_face="focused", camera_announce=True, camera_face="looking",
                 workshop_enable=True, workshop_path=None,
                 workshop_dir="", workshop_rounds=1, workshop_min_runs=3,
                 workshop_retire_errors=2, workshop_retire_net_neg=2):
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
        self._audio_summary = audio_summary or (lambda: "no audio available")
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
        # incremental growth: occasionally add fresh offline lines instead of only regenerating
        self._bank_grow_enable = bool(bank_grow_enable)
        self._bank_grow_period = float(bank_grow_period)
        self._bank_grow_max = int(bank_grow_max)
        self._bank_grow_batch = int(bank_grow_batch)
        self._bank = PhraseBank(path=bank_path, logger=self._log)
        self.skills_enable = bool(skills_enable)
        self.skills_allow_actions = bool(skills_allow_actions)
        self.skills_dir = skills_dir
        # The writable "learned" dir is where the workshop mints skills (kept out of the
        # committed catalogue; it's the deploy-synced state home, like the soul/phrase bank).
        self._workshop_dir = workshop_dir or os.path.expanduser(
            "~/.local/state/nanobot/skills")
        self._skills = SkillLibrary(skills_dir if skills_enable else "", logger=self._log,
                                    extra_dir=self._workshop_dir if skills_enable else "")
        self._reflect_busy = False
        # Long-term self-narrative (smart-LLM, durable across reboots). Loaded here and folded
        # into every spoken line's system prompt (LlmClient.set_self_note); rewritten slowly by
        # consolidate(). Unlike the smoothed traits it never reverts, so character compounds.
        self._self_model_enable = bool(self_model_enable)
        self._consolidate_every = max(0, int(consolidate_every))
        self._self_model_max = max(120, int(self_model_max_chars))
        self._self_model_path = self_model_path or os.path.expanduser(
            "~/.local/state/nanobot/self_model.json")
        self.self_narrative = self._load_self_model()
        if self.self_narrative:
            self.llm.set_self_note(self.self_narrative)
        self._reflect_count = 0
        self._consolidate_busy = False
        # Trait trajectory: a small, durable log of (timestamp, traits) snapshots so the robot
        # can reason about HOW it has changed, not just react to the last few events. Sampled at
        # most once per `trait_history_period`; `trait_trend_text()` summarises the drift over the
        # trailing `trait_history_window` and is folded into reflect()/consolidate() prompts so
        # the self-narrative grows from a real trajectory ("curiosity 0.50 -> 0.68"). Deploy-synced
        # like the soul (lives in the XDG state dir / devstate on the harness).
        self._trait_hist_enable = bool(trait_history_enable)
        self._trait_hist_period = max(60.0, float(trait_history_period))
        self._trait_hist_max = max(8, int(trait_history_max))
        self._trait_hist_window = max(self._trait_hist_period, float(trait_history_window))
        self._trait_hist_path = trait_history_path or os.path.expanduser(
            "~/.local/state/nanobot/trait_history.json")
        self._trait_hist = self._load_trait_history()
        # Skill workshop: reflection mode's experience-driven skill-synthesis loop (suggest ->
        # check -> rehearse -> trial -> adopt/retire). The pure ledger + gate live in skillsmith.py;
        # this class owns the LLM steps + the .md file writes. A trial skill is a normal,
        # immediately-usable skill; the ledger just tracks whether it earns permanence.
        self._workshop_enable = bool(workshop_enable) and self.skills_enable
        self._workshop_rounds = max(1, int(workshop_rounds))
        self._workshop = WorkshopState(
            workshop_path or os.path.expanduser("~/.local/state/nanobot/workshop.json"),
            logger=self._log, min_runs=workshop_min_runs,
            retire_errors=workshop_retire_errors, retire_net_neg=workshop_retire_net_neg)
        self._workshop_busy = False
        self._last_trial_skill = None        # the trial skill that ran most recently (reward target)
        # Interaction fillers: an instant "thinking" line the moment a slow call starts (so a
        # skill/beat feels instant), and a graceful "stumped" line when a call comes back empty
        # instead of dead air. Both pull from the phrase bank (offline-safe FALLBACK_LINES).
        self._prelude_enable = bool(prelude_enable)
        self._prelude_face = str(prelude_face or "")
        # Camera announce: ALWAYS speak a short "peeking/seeing" line the instant the camera is
        # used (before any capture), so the robot never looks silently. Independent of the
        # thinking-prelude (on even when prelude is off); the only off switch is this flag.
        # The peek moment also shows the dedicated "looking" OLED face (the wide, scanning eyes)
        # — matching the chart's camera-beat default face for the non-chart paths too.
        self._camera_announce = bool(camera_announce)
        self._camera_face = str(camera_face or "")

    def available(self):
        return self.llm.available()

    # ---- traits -------------------------------------------------------------
    def update_traits(self, traits):
        if isinstance(traits, dict):
            self.traits.update({k: clamp01(traits[k]) for k in REFLECT_TRAITS if k in traits})

    def traits_phrase(self):
        return ", ".join(f"{k} {self.traits.get(k, 0.5):.2f}" for k in REFLECT_TRAITS)

    # ---- trait trajectory (self-knowledge: how it has changed over time) -----
    def _load_trait_history(self):
        """Read the persisted (timestamp, traits) snapshots. Best-effort: [] on any problem."""
        try:
            with open(self._trait_hist_path, encoding="utf-8") as f:
                data = json.load(f)
            snaps = data.get("snapshots") if isinstance(data, dict) else data
            return [s for s in snaps if isinstance(s, dict) and "traits" in s][-self._trait_hist_max:]
        except Exception:
            return []

    def _save_trait_history(self):
        try:
            os.makedirs(os.path.dirname(self._trait_hist_path), exist_ok=True)
            tmp = self._trait_hist_path + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump({"snapshots": self._trait_hist}, f, indent=1)
            os.replace(tmp, self._trait_hist_path)
        except Exception as exc:
            self._log(f"trait history: save failed ({exc})")

    def record_trait_snapshot(self, force=False):
        """Append the current live traits to the trajectory log, at most once per period (or
        `force`). Caps the ring to trait_history_max. Cheap + best-effort; the input to
        trait_trend_text(). No-op when disabled."""
        if not self._trait_hist_enable:
            return False
        now = time.time()
        last = self._trait_hist[-1]["t"] if self._trait_hist else 0.0
        if not force and (now - float(last or 0.0)) < self._trait_hist_period:
            return False
        self._trait_hist.append({"t": now,
                                 "traits": {k: round(float(self.traits.get(k, 0.5)), 3)
                                            for k in REFLECT_TRAITS}})
        del self._trait_hist[:-self._trait_hist_max]
        self._save_trait_history()
        return True

    def trait_trend_text(self, min_delta=0.04):
        """A compact, human-readable summary of how the traits have drifted over the trailing
        window — e.g. "curiosity 0.50 -> 0.68 (rising), caution 0.60 -> 0.44 (easing)". Compares
        the current values to the oldest snapshot still inside the window. Returns "" when there
        isn't enough history or nothing moved meaningfully, so prompts can omit it cleanly."""
        if not self._trait_hist_enable or len(self._trait_hist) < 2:
            return ""
        now = time.time()
        in_window = [s for s in self._trait_hist if (now - float(s.get("t", 0))) <= self._trait_hist_window]
        base = (in_window or self._trait_hist)[0].get("traits", {})
        parts = []
        for k in REFLECT_TRAITS:
            old, new = float(base.get(k, 0.5)), float(self.traits.get(k, 0.5))
            if abs(new - old) >= min_delta:
                word = "rising" if new > old else "easing"
                parts.append(f"{k} {old:.2f} -> {new:.2f} ({word})")
        return "; ".join(parts)

    def get_trait_history(self):
        """Web/diagnostic readout: the raw snapshots (oldest first) + the current trend line."""
        return {"enabled": self._trait_hist_enable, "snapshots": list(self._trait_hist),
                "trend": self.trait_trend_text()}

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

    def _pick_filler(self, category):
        """Pick an offline-safe interaction filler from the bank (FALLBACK_LINES back it)."""
        try:
            return self._bank.pick(None, name=self.persona_name, category=category)
        except Exception:
            return None

    def _speak_prelude(self):
        """Speak an instant 'thinking' filler so a slow call isn't dead air. Non-blocking:
        tts.say plays on a worker thread, so the LLM call starts immediately in parallel and
        the real line's express() barges in (tts is barge-in) when it arrives."""
        reply = self._pick_filler("thinking")
        if not (reply and reply.get("say")):
            return
        if self._prelude_face:
            self._face(self._prelude_face)              # immediate visual "I'm on it"
        if self.tts is not None and self.tts.available():
            self.tts.say(reply["say"])

    def _speak_peek(self):
        """Speak an instant 'peeking/seeing' line the moment the camera is used — ALWAYS spoken
        before a frame is captured so the robot never looks silently. Non-blocking (tts.say runs
        on a worker thread), so it plays in parallel with the capture + vision call and the real
        vision line barges in when it lands. Offline-safe via the phrase bank FALLBACK_LINES."""
        if self._camera_face:
            self._face(self._camera_face)               # immediate "looking" eyes
        reply = self._pick_filler("peeking")
        if not (reply and reply.get("say")):
            return
        if self.tts is not None and self.tts.available():
            self.tts.say(reply["say"])

    def _capture_announced(self):
        """Capture one camera frame, ALWAYS announcing the peek first (when enabled). The single
        chokepoint every camera path goes through (beats, skills, /llm/look), so the 'say a
        seeing line before looking' rule holds everywhere — not just the chart-driven beats.
        Returns the JPEG bytes or None."""
        if self._camera_announce:
            self._speak_peek()
        return self._capture_frame()

    def _speak_stumped(self, state=""):
        """When an attempted call comes back empty, say a light 'lost the thought' line from
        the bank instead of going silent. Logged separately so the no-reply stays visible."""
        reply = self._pick_filler("stumped")
        if not (reply and reply.get("say")):
            return
        self.express(reply["mood"], reply["say"])
        self.log_decision("stumped", state=state, status="bank", model="phrasebank",
                          say=reply["say"], mood=reply["mood"])

    def generate(self, prompt, history=None, image_jpeg=None, trigger="manual",
                 state="", camera=False, smart=False, prelude=False):
        """Blocking generate + express, guarded so only one call runs at a time (little
        RAM/CPU; the API costs money). Records the decision + outcome. Returns the reply dict
        or None. Safe to call from any worker thread. With `prelude`, speak an instant
        "thinking" filler the moment the (slow) call starts so it doesn't feel like dead air;
        on an empty reply, speak a "stumped" line instead of going silent."""
        if not self.llm.available():
            self.log_decision(trigger, state, camera, status="llm-unavailable")
            return None
        with self._llm_lock:
            if self._llm_busy:
                self.log_decision(trigger, state, camera, status="skipped-busy")
                return None
            self._llm_busy = True
        if prelude and self._prelude_enable:            # instant filler while we wait (non-blocking)
            self._speak_prelude()
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
            if self._prelude_enable:                    # don't go silent on a failed call
                self._speak_stumped(state)
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
        return self.generate(prompt, trigger=trigger, state=state, prelude=True)

    def llm_look(self, trigger="look", state=""):
        """Capture a frame (+ the sensor snapshot) and comment on what it SEES (vision)."""
        if not self.llm.can_call(image=True):           # capped: skip the capture entirely
            self.log_decision(trigger, state, True, status="rate-limited")
            return {"error": "vision hourly limit reached"}
        frame = self._capture_announced()               # speak the peek line, then capture
        if frame is None:
            self.log_decision(trigger, state, True, status="no-frame")
            return {"error": "no camera frame"}
        snap = self._sensor_snapshot()
        self._log(f"llm look: {len(frame)} byte frame; {snap}")
        prompt = ("This is the live view from your own camera. Your body also senses: "
                  f"{snap}. In character, say one short spoken line about what you can "
                  "see in front of you right now, and pick a fitting mood.")
        # prelude=False: the peek line already covered the "I'm on it" filler, so don't double up.
        return self.generate(prompt, image_jpeg=frame, trigger=trigger, state=state,
                             camera=True, prelude=False)

    # ---- statechart beat executor (the chart's /cognition/request) ----------
    def run_beat(self, trigger, state, prompt, camera, audio=False):
        """Execute one enrichable beat: capture a frame if asked (else, for a non-audio body
        beat, try the cached phrase bank), add the live sensors + personality (+ what the mic
        hears for an `audio` beat), generate + express. An audio beat always goes live (the
        cached bank isn't sound-aware)."""
        frame = None
        if camera:
            if not self.llm.can_call(image=True):       # don't spin up the camera if capped
                self.log_decision(trigger, state, camera, status="rate-limited")
                return
            frame = self._capture_announced()           # speak the peek line, then capture
            if frame is None:
                self.log_decision(trigger, state, camera, status="no-frame")
                return
        elif not audio and self.bank_say(trigger, state):  # frequent body beat -> cached line
            return                                       # (free/instant/offline; no LLM call)
        full = (prompt + " Your current personality (0..1) is " + self.traits_phrase()
                + ", and your body senses: " + self._sensor_snapshot())
        if audio:
            full += " Through your microphone you hear: " + self._audio_summary() + "."
        # Camera beats already spoke the peek line; only the non-camera (audio) beat needs the
        # generic "thinking" prelude.
        self.generate(full, image_jpeg=frame, trigger=trigger, state=state, camera=camera,
                      prelude=not camera)

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

    def bank_grow_check(self):
        """Grow the phrase bank over time: occasionally add a few fresh offline lines via the
        LLM to the most under-filled situation (background, rate-limited inside the bank).
        No-op if disabled / LLM offline / the soul drifted (a full regen runs instead)."""
        if self._bank_enable and self._bank_grow_enable and self.llm.available():
            self._bank.maybe_grow(self.llm, self.persona, self.traits,
                                  name=self.persona_name, period=self._bank_grow_period,
                                  batch=self._bank_grow_batch,
                                  max_per_category=self._bank_grow_max,
                                  drift_threshold=self._bank_drift, background=True)

    def get_phrasebank(self):
        s = self._bank.stats()
        s["enabled"] = self._bank_enable
        s["live_ratio"] = self._bank_live_ratio
        s["grow_enable"] = self._bank_grow_enable
        s["grow_max"] = self._bank_grow_max
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

    def grow_phrasebank(self):
        """On-demand phrase-bank growth (the `phrases` meta skill + a manual trigger): add a
        few fresh offline lines to the most under-filled situation NOW, blocking on the LLM
        (like the workshop). Calls `grow()` directly, so it bypasses the inter-grow period
        gate; the new lines match the CURRENT soul. Returns a small status dict."""
        if not self._bank_enable:
            return {"error": "phrasebank disabled"}
        if not self.llm.available():
            return {"error": "llm unavailable"}
        res = self._bank.grow(self.llm, self.persona, self.traits, name=self.persona_name,
                              batch=self._bank_grow_batch, max_per_category=self._bank_grow_max)
        if not res:
            return {"status": "full-or-empty"}
        cat, added = res
        return {"status": "grew", "category": cat, "added": added}

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
        # Say an instant "thinking" filler BEFORE the (slow) pick call, so the beat feels
        # responsive instead of going silent while the model chooses. The chosen skill is then
        # performed with prelude=False so we don't double up on fillers.
        if self._prelude_enable and self.llm.available():
            self._speak_prelude()
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
        self._invoke_skill(skill, trigger="skill:" + skill.name, state=state, prelude=False)

    def _skill_prompt(self, skill):
        """The user-turn steering a narrative skill: its body + any requested live context
        (sensors / lidar) + the current personality. {say,mood} shape is enforced by the LLM
        client's SYSTEM_BASE, so we just supply the steering."""
        parts = [skill.body or skill.description or skill.name]
        seen = set()
        for src in skill.sources:
            key = SOURCE_ALIASES.get(src, src)
            spec = SKILL_SOURCES.get(key)
            if spec and key not in seen:                 # de-dupe aliases of the same source
                seen.add(key)
                template, provider = spec
                parts.append(template.format(getattr(self, provider)()))
        parts.append("Your personality (0..1) is " + self.traits_phrase() + ".")
        parts.append("Reply with one short spoken line and a fitting mood.")
        return " ".join(p.strip() for p in parts if p and p.strip())

    def _invoke_skill(self, skill, trigger, state="", prelude=True):
        """Perform a skill: narrative kinds generate+speak (optionally with a camera frame);
        the gated `topic` kind publishes a whitelisted ROS message; the `workshop` meta kind
        runs the skill-synthesis loop. Returns a status dict. `prelude` controls the instant
        "thinking" filler (set False when the caller already spoke one — e.g. the skill beat)."""
        if skill.is_action:
            return self._do_topic_skill(skill, trigger, state)
        if skill.is_meta:                                # self-improvement (operates on own state)
            if skill.kind == "phrases":                  # grow the offline phrase bank
                return self._do_phrases_skill(skill, trigger, state, prelude)
            return self._do_workshop_skill(skill, trigger, state, prelude)
        frame = None
        if skill.camera:
            if not self.llm.can_call(image=True):
                self.log_decision(trigger, state, True, status="rate-limited")
                return {"error": "vision hourly limit reached"}
            frame = self._capture_announced()           # speak the peek line, then capture
            if frame is None:
                self.log_decision(trigger, state, True, status="no-frame")
                return {"error": "no camera frame"}
        # A camera skill already spoke the peek line — suppress the generic "thinking" prelude.
        reply = self.generate(self._skill_prompt(skill), image_jpeg=frame, trigger=trigger,
                              state=state, camera=bool(frame),
                              prelude=(prelude and not skill.camera))
        self._note_trial_run(skill.name, ok=bool(reply))
        return reply or {"error": "no reply"}

    def _do_workshop_skill(self, skill, trigger, state, prelude=True):
        """Run the skill workshop on demand (the `workshop` meta kind). Speaks an instant
        "thinking" filler first (TTS stays responsive), then runs the synthesis loop, which
        makes its own LLM calls and logs each step. Returns the workshop's status dict."""
        if prelude and self._prelude_enable and self.llm.available():
            self._speak_prelude()
        res = self.run_skill_workshop()
        self.log_decision(trigger, state, status=res.get("status", ""), model="workshop",
                          detail=json.dumps(res.get("rounds", ""))[:160])
        return res

    def _do_phrases_skill(self, skill, trigger, state, prelude=True):
        """Grow the offline phrase bank on demand (the `phrases` meta kind). Speaks an instant
        "thinking" filler first (TTS stays responsive), then blocks on the growth LLM call and
        logs the outcome. Same routine that runs by itself during reflection mode."""
        if prelude and self._prelude_enable and self.llm.available():
            self._speak_prelude()
        res = self.grow_phrasebank()
        self.log_decision(trigger, state, status=res.get("status") or res.get("error", ""),
                          model="phrasebank",
                          detail=f"{res.get('category', '')}+{res.get('added', '')}")
        return res

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
        self._note_trial_run(skill.name, ok=ok)
        return {"status": "acted" if ok else "error", "detail": detail, "name": skill.name}

    # ---- skill workshop (reflection mode's self-improvement loop) -----------
    def run_skill_workshop(self, rounds=None):
        """Reflection mode's skill-synthesis loop: mine experience -> propose ONE new/adapted skill
        -> deterministic check -> rehearse once -> smart-model critique -> commit on trial.
        Then sweep the gate so ripe trials adopt/retire. Best-effort + one-at-a-time guarded;
        a no-op without the LLM or with the workshop disabled. Returns a small status dict."""
        if not self._workshop_enable:
            return {"status": "disabled"}
        if not self.llm.available():
            self.log_decision("workshop", status="llm-unavailable")
            return {"status": "llm-unavailable"}
        with self._llm_lock:                       # share the LLM guard so we never overlap a beat
            if self._workshop_busy or self._llm_busy:
                return {"status": "busy"}
            self._workshop_busy = True
        rounds = self._workshop_rounds if rounds is None else max(1, int(rounds))
        out = []
        try:
            for _ in range(rounds):
                out.append(self._workshop_round())
        finally:
            self._workshop_busy = False
        self.sweep_workshop()                      # adopt/retire any trial that has earned it
        return {"status": "ok", "rounds": out}

    def _workshop_round(self):
        """One propose->check->rehearse->critique->trial cycle. Returns a per-round status."""
        spec = self._suggest_skill()
        if not spec:
            self.log_decision("workshop:suggest", status="no-reply",
                              model=(self.llm.last_model or self.llm.smart_model))
            return {"status": "no-suggestion"}
        name = _slug(spec.get("name"))
        ok, why = validate_candidate(spec, self._skills.skills.keys(),
                                     self.skills_allow_actions)
        if not ok:
            self.log_decision("workshop:reject", status="invalid", say=name, detail=why)
            return {"status": "rejected", "name": name, "reason": why}
        path = self._write_skill_file(name, render_skill_md(spec))
        if not path:
            return {"status": "write-failed", "name": name}
        skill = self._skills.add_file(path)        # make the candidate live (no full re-scan)
        if skill is None:                          # shouldn't happen (validate round-tripped it)
            self._delete_skill_file(path)
            return {"status": "write-failed", "name": name}
        rehearsal = self._rehearse_skill(skill)
        verdict = self._critique_skill(spec, rehearsal)
        if not verdict.get("keep", True):
            self._delete_skill_file(path)
            self._skills.remove(name)
            self.log_decision("workshop:discard", status="critique", say=name,
                              detail=str(verdict.get("reason", ""))[:120])
            return {"status": "discarded", "name": name, "reason": verdict.get("reason")}
        self._workshop.track(name, origin=str(spec.get("mode", "new")),
                             parent=str(spec.get("target", "")),
                             rationale=str(spec.get("rationale", "")), path=path)
        self.log_decision("workshop:trial", status="trialing", say=name,
                          detail=str(spec.get("rationale", ""))[:120])
        return {"status": "trialing", "name": name}

    def _suggest_skill(self):
        """Ask the smart model to invent ONE small new capability or improve an existing one,
        grounded in the recent decision log (gaps, repeated 'no-pick'/'stumped', requests)."""
        cat = self._skills.format_catalogue(self.skills_allow_actions) or "(none yet)"
        kinds = "say, observe, look" + (", topic" if self.skills_allow_actions else "")
        selfctx = (f" You have become: {self.self_narrative}." if self.self_narrative else "")
        system = (
            f"You are the quiet, self-improving mind of a small robot named "
            f"{self.persona_name} during reflection. From its recent experience you invent ONE "
            "small new capability, or improve an existing one, so it serves the people around "
            "it a little better. A capability is a short instruction the robot follows to speak "
            "or act. Reply ONLY compact JSON: "
            '{"mode":"new"|"adapt","target":"<existing name, only if adapt>",'
            '"name":"<short-kebab-name>","description":"<one line>",'
            '"trigger":"<when to use it>","action":{"kind":"<one of: %s>","sources":'
            '["sensors"|"scan"|"audio"|"memory"]},"body":"<2-4 sentences telling the robot HOW '
            'to perform it>","rationale":"<why, from experience>"}. Use a NEW kebab name even '
            "when adapting (a variant); omit sources for a plain say. No prose outside the JSON."
            % kinds)
        user = (f"Existing capabilities:\n{cat}\n\nYour personality (0..1): "
                f"{self.traits_phrase()}.{selfctx}\n\nRecent experience (look for gaps, "
                f"repeated stumbles, things people seemed to want):\n"
                f"{self.recent_events_text(40)}\n\nPropose one capability to add or improve.")
        content = self.llm.complete(system, user, smart=True, json_object=True)
        obj = _extract_json(content or "")
        return obj if isinstance(obj, dict) and obj.get("name") else None

    def _rehearse_skill(self, skill):
        """Dry-run the candidate ONCE to get a real sample of its output, without speaking it
        aloud (reflection stays calm). Narrative kinds get a text generation; an action kind
        is described, not published. Returns a {say,mood}-ish dict or None."""
        if skill is None:
            return None
        if skill.is_action:
            return {"say": "(action: would publish %s)" % skill.action.get("topic", ""),
                    "mood": ""}
        try:
            return self.llm.generate(self._skill_prompt(skill), smart=False)
        except Exception:
            return None

    def _critique_skill(self, spec, rehearsal):
        """Smart-model self-check on the rehearsed candidate: is it useful, safe, in-character,
        not a duplicate? Returns {"keep":bool,"reason":str}. An unparseable/empty critique
        defaults to keep=True (the deterministic checks + rehearsal already passed); only an
        explicit "keep": false vetoes — so a flaky model never blocks all learning."""
        sample = (rehearsal or {}).get("say") if isinstance(rehearsal, dict) else ""
        system = ("You quality-check a small robot's proposed new capability before it goes on "
                  "trial. Judge whether it is useful, safe, in character, and not a duplicate "
                  'of what it can already do. Reply ONLY compact JSON '
                  '{"keep": true|false, "reason": "<one short line>"}.')
        user = ("Proposed capability:\n- name: %s\n- description: %s\n- trigger: %s\n- how: %s\n"
                "- rationale: %s\n\nA rehearsal of it produced: %r\n\nShould it go on trial?"
                % (spec.get("name"), spec.get("description"), spec.get("trigger"),
                   spec.get("body"), spec.get("rationale"), sample or "(no output)"))
        content = self.llm.complete(system, user, smart=True, json_object=True)
        obj = _extract_json(content or "")
        if not isinstance(obj, dict) or "keep" not in obj:
            return {"keep": True, "reason": ""}
        return {"keep": bool(obj.get("keep")), "reason": str(obj.get("reason", ""))}

    # ---- trial bookkeeping: runs, reward, the adopt/retire gate --------------
    def _note_trial_run(self, name, ok=True):
        """Record one trial-skill invocation + apply the gate now (a clearly good/bad skill
        needn't wait for the next reflection). No-op for untracked / non-trial skills."""
        if not self._workshop.is_trial(name):
            return
        self._last_trial_skill = _slug(name)
        self._workshop.record_run(name, ok=ok)
        self._apply_gate(name)

    def reward_trial_skill(self, value):
        """Credit a human 👍/👎 (the 'happy user' signal) to the trial skill that ran most
        recently, then re-gate it. Returns whether a trial was credited."""
        name = self._last_trial_skill
        if not (name and self._workshop.is_trial(name)):
            return False
        self._workshop.record_reward(name, value)
        self._apply_gate(name)
        return True

    def sweep_workshop(self):
        """Apply the gate across every trial (called after minting + on the slow tick)."""
        for name, decision in self._workshop.gate_all():
            self._apply_decision(name, decision)

    def _apply_gate(self, name):
        self._apply_decision(name, self._workshop.gate(name))

    def _apply_decision(self, name, decision):
        if decision == "adopt":
            self._adopt_skill(name)
        elif decision == "retire":
            self._retire_skill(name)

    def _adopt_skill(self, name):
        """Graduate a trial to permanent. The .md already lives in the durable learned dir
        (deploy-synced like the soul/phrase bank), so adoption is just a status flip + log."""
        name = _slug(name)
        rec = self._workshop.get(name) or {}
        self._workshop.keep(name)
        self.log_decision("workshop:adopt", status="adopted", say=name,
                          detail="runs=%s +%s/-%s" % (rec.get("runs"), rec.get("reward_pos"),
                                                      rec.get("reward_neg")))

    def _retire_skill(self, name):
        """Roll a trial back: delete its .md (the parent of an `adapt` is never touched) and
        drop the ledger record so the library stops offering it."""
        name = _slug(name)
        rec = self._workshop.get(name) or {}
        self._delete_skill_file(rec.get("path", ""))
        self._workshop.forget(name)
        self._skills.remove(name)                   # drop from the index (no full re-scan)
        self.log_decision("workshop:retire", status="retired", say=name)

    # ---- workshop file IO + web API -----------------------------------------
    def _write_skill_file(self, name, text):
        """Write a generated skill into the writable 'learned' dir (creating it if needed).
        Returns the path, or "" if there's nowhere writable (logged, no trial)."""
        if not text:
            return ""
        d = self._skills.write_dir() or self.skills_dir
        if not d:
            self._log("workshop: no writable skills directory")
            return ""
        path = os.path.join(d, _slug(name) + ".md")
        try:
            os.makedirs(d, exist_ok=True)
            with open(path, "w", encoding="utf-8") as f:
                f.write(text)
            return path
        except Exception as exc:
            self._log("workshop: failed to write %s: %s" % (path, exc))
            return ""

    def _delete_skill_file(self, path):
        if path and os.path.isfile(path):
            try:
                os.remove(path)
            except Exception as exc:
                self._log("workshop: failed to remove %s: %s" % (path, exc))

    def get_workshop(self):
        return {"enabled": self._workshop_enable, "busy": self._workshop_busy,
                "trials": self._workshop.to_public()}

    def keep_skill(self, name):
        """Manual override: adopt a trial now (a happy user pressing Keep)."""
        if not self._workshop.is_trial(name):
            return {"error": "unknown trial: %s" % name}
        self._adopt_skill(name)
        return {"status": "adopted", "name": _slug(name)}

    def kill_skill(self, name):
        """Manual override: discard a trial now (delete its file + forget it)."""
        if self._workshop.status_of(name) is None:
            return {"error": "unknown trial: %s" % name}
        self._retire_skill(name)
        return {"status": "retired", "name": _slug(name)}

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
        self.record_trait_snapshot()                     # log where the traits are right now
        try:
            system = (
                "You are the slow, reflective mind of a small robot named Nano. You review "
                "what just happened and gently adjust its personality so it grows over time. "
                "Traits are 0..1: curiosity, extraversion, caution, playfulness. You may also "
                "retune how often each idle behaviour fires by nudging its registry priority "
                "(0..1 base weight) — the beats are: musing (react to its body/sensors), looking "
                "(use the camera), wondering (a curious deep thought), listening (react to "
                "sounds). Favour what recently earned reward / fit the moment, ease off what "
                "fell flat. You may ALSO steer four expressive 'drives': energy (0..1, overall "
                "restlessness — paces how often it stirs and may chain an extra beat), focus "
                "(0..1, how readily it perks up alert and pays attention), introspection (0..1, "
                "how soon it drifts into quiet reflection when idle), and mood (a baseline idle "
                "face it briefly wears between beats, one of: happy, focused, neutral, looking, "
                'or "" for none). Output ONLY compact JSON: {"traits": {<trait>: <0..1>}, '
                '"registry": {optional: {"<beat>": {"priority":0..1,"enabled":bool}}}, '
                '"drives": {optional: {"energy":0..1,"focus":0..1,"introspection":0..1,'
                '"mood":"<face or empty>"}}, "note": "<one short reason>"}. Propose only SMALL, '
                "justified nudges (omit what you would not change); each 0..1 value is a TARGET "
                "that gets smoothed over time. No prose outside the JSON.")
            selfctx = (f"\nWho you have become: {self.self_narrative}"
                       if self.self_narrative else "")
            trend = self.trait_trend_text()
            trendctx = f"\nHow you have been drifting lately: {trend}." if trend else ""
            user = (f"Current traits: {self.traits_phrase()}.{selfctx}{trendctx}\nRecent events:\n"
                    f"{self.recent_events_text()}\n\nReflect and propose adjustments.")
            content = self.llm.complete(system, user, smart=True, json_object=True)
        finally:
            self._reflect_busy = False
        ms = int((time.monotonic() - t0) * 1000)
        obj = _extract_json(content or "")
        traits = {k: clamp01(obj["traits"][k]) for k in REFLECT_TRAITS
                  if isinstance(obj.get("traits"), dict) and k in obj["traits"]}
        registry = obj.get("registry") if isinstance(obj.get("registry"), dict) else {}
        odr = obj.get("drives") if isinstance(obj.get("drives"), dict) else {}
        drives = {k: clamp01(odr[k]) for k in REFLECT_DRIVES if k in odr}
        if isinstance(odr.get("mood"), str) and odr["mood"] in DRIVE_MOODS:
            drives["mood"] = odr["mood"]                  # categorical idle tint (whitelisted)
        rmodel = self.llm.last_model or self.llm.smart_model
        if not traits and not registry and not drives:
            self.log_decision("reflect", status="no-reply", model=rmodel, ms=ms)
            return None
        self.log_decision("reflect", status="spoke", model=rmodel,
                          say=f"{obj.get('note','')} -> {traits}{(' '+str(drives)) if drives else ''}",
                          ms=ms)
        self._maybe_consolidate()                        # slow long-term identity drift
        return {"traits": traits, "registry": registry, "drives": drives,
                "note": obj.get("note", "")}

    # ---- long-term self-narrative (smart-LLM, durable) ----------------------
    def _load_self_model(self):
        """Read the persisted self-narrative (best-effort; '' if absent/unreadable)."""
        try:
            with open(self._self_model_path, encoding="utf-8") as f:
                return str(json.load(f).get("narrative", "")).strip()
        except Exception:
            return ""

    def _save_self_model(self):
        try:
            os.makedirs(os.path.dirname(self._self_model_path), exist_ok=True)
            with open(self._self_model_path, "w", encoding="utf-8") as f:
                json.dump({"narrative": self.self_narrative, "name": self.persona_name,
                           "updated_at": int(time.time())}, f, indent=2, ensure_ascii=False)
        except Exception:
            pass

    def _maybe_consolidate(self):
        """Every `consolidate_every`-th successful reflection, rewrite the self-narrative
        (0 = never except on manual/reflection-mode consolidate)."""
        if not (self._self_model_enable and self._consolidate_every):
            return
        self._reflect_count += 1
        if self._reflect_count % self._consolidate_every == 0:
            self.consolidate()

    def consolidate(self):
        """The long-term tier (smart model): fold recent experience into a SHORT, durable,
        first-person self-narrative — who the robot is becoming. Persisted across reboots and
        folded into every spoken line's system prompt. Unlike the smoothed traits it does NOT
        revert on a brain outage, so character compounds. Guarded + best-effort; returns the
        new narrative or None."""
        if not self._self_model_enable or self._consolidate_busy or not self.llm.available():
            return None
        self._consolidate_busy = True
        t0 = time.monotonic()
        self.record_trait_snapshot()                     # capture the trajectory point too
        try:
            system = (
                f"You maintain the evolving self-understanding of a small robot named "
                f"{self.persona_name}. {(self.persona + ' ') if self.persona else ''}Given who "
                "it has been and what it has recently done, write its CURRENT self-narrative: a "
                "few first-person sentences about who it is becoming — its temperament, what it "
                "cares about, how it tends to act. Change it only a LITTLE from the previous "
                "narrative (gradual growth, not reinvention); keep what still fits. At most "
                f"~{self._self_model_max // 6} words, plain prose, no lists. Output ONLY the "
                "narrative text.")
            trend = self.trait_trend_text()
            trendline = f"How your personality has drifted over time: {trend}\n\n" if trend else ""
            user = (f"Previous self-narrative:\n{self.self_narrative or '(none yet)'}\n\n"
                    f"Current traits (0..1): {self.traits_phrase()}\n\n"
                    f"{trendline}"
                    f"Recent experience:\n{self.recent_events_text()}\n\n"
                    "Reflect on how you have changed, and write the updated self-narrative.")
            text = self.llm.complete(system, user, smart=True, json_object=False)
        finally:
            self._consolidate_busy = False
        ms = int((time.monotonic() - t0) * 1000)
        rmodel = self.llm.last_model or self.llm.smart_model
        text = (text or "").strip()
        if text.startswith(("{", "```")):                # model ignored "plain text" -> skip
            text = ""
        if not text:
            self.log_decision("consolidate", status="no-reply", model=rmodel, ms=ms)
            return None
        self.self_narrative = text[: self._self_model_max]
        self.llm.set_self_note(self.self_narrative)
        self._save_self_model()
        self.log_decision("consolidate", status="spoke", model=rmodel,
                          say=self.self_narrative[:160], ms=ms)
        return self.self_narrative

    def get_self_model(self):
        return {"enabled": self._self_model_enable, "narrative": self.self_narrative,
                "path": self._self_model_path, "consolidate_every": self._consolidate_every}

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
