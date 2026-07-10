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
import copy
import json
import os
import random
import re
import threading
import time
from collections import deque

from .jsonio import read_json, read_jsonl_tail, write_json
from .llm import MOODS, _extract_json
from .phrasebank import PhraseBank
from .skills import SkillLibrary, _slug
from .skillsmith import WorkshopState, render_skill_md, validate_candidate

REFLECT_TRAITS = ("curiosity", "extraversion", "caution", "playfulness")  # personality axes
# New LLM-steerable expressive DRIVES (above the 4 traits) the chart exposes — see
# behavior/presence.py. Kept as plain strings here so web_control never imports the behavior pkg.
REFLECT_DRIVES = ("energy", "focus", "introspection")     # 0..1 scalars, smoothed in the chart
DRIVE_MOODS = ("", "happy", "angry", "focused", "stress", "neutral", "looking", "sleepy")  # idle tint
# Distinctive action eye-SHAPES (oled_display geometries) that can carry an emotion accent.
# An action beat ("looking"/"focused") keeps its scanning/intent eyes AND shows the LLM's emotion
# as an accent on top -> a compound "shape:emotion" face (see compose_face). The plain round
# bases (happy/neutral) aren't distinctive, so there the emotion alone IS the face (unchanged).
SHAPE_FACES = ("looking", "focused")
LLM_HISTORY_MAX = 8          # chat turns kept for context (user+assistant messages)


def compose_face(base, accent):
    """Combine an action eye-shape with an emotion accent into the OLED face string the panel
    renders (oled_display._on_face parses "shape:emotion"). Returns "looking:happy" for a
    distinctive action shape + a real emotion, else just the emotion (legacy single mood) — so
    on-demand chat/observe and the round musing beat are unchanged."""
    base = (base or "").strip().lower()
    accent = (accent or "").strip().lower()
    if base in SHAPE_FACES and accent and accent != "neutral" and accent != base:
        return f"{base}:{accent}"
    return accent
LLM_LOG_MAX = 50             # decision-log ring buffer length (also what the file tail loads)

# Skill `sources:` -> (prompt template, the CognitionCore method that supplies the text).
# Adding a new live-context source for skills is one row here (+ its adapter). The aliases let
# a skill file say sound/mic/events/recent and resolve to the canonical source.
SKILL_SOURCES = {
    "sensors": ("Your body senses right now: {}.", "_sensor_snapshot"),
    "scan":    ("Your lidar reports: {}.", "_scan_summary"),
    "audio":   ("Through your microphone you hear: {}.", "_audio_summary"),
    "memory":  ("Lately you have been doing:\n{}", "recent_events_text"),
    "docs":    ("From your own documentation:\n{}", "_docs_summary"),
}
SOURCE_ALIASES = {"sound": "audio", "mic": "audio", "events": "memory", "recent": "memory",
                  "readme": "docs", "about": "docs", "self": "docs"}

# Self-knowledge: the small whitelist of its own docs the robot may read aloud, as
# (filename-relative-to-repo-root, max_chars). Extend by adding a row — keep excerpts short so
# the prompt stays cheap, and NEVER list a file that holds secrets (e.g. robot.yaml with a key);
# credential-looking lines are redacted defensively below regardless.
SELF_DOCS = (("README.md", 700), ("CLAUDE.md", 700))
_DOC_SECRET_RE = re.compile(
    r"(?im)^(\s*[\w.\-]*(?:api[_-]?key|key|token|secret|password|passwd|pw)\s*[:=]\s*)\S.*$")


def read_self_docs(root, docs=SELF_DOCS):
    """A short, redacted excerpt of the robot's own documentation (the `docs` skill source).
    Reads ONLY the whitelisted files under `root` (never a caller/LLM-chosen path) and masks any
    credential-looking line. Returns one block per readable file, or '' if none was readable."""
    out = []
    for name, limit in docs:
        try:
            with open(os.path.join(root, name), "r", encoding="utf-8", errors="replace") as f:
                text = f.read(limit * 4)              # read a little extra; trim after redaction
        except OSError:
            continue
        text = _DOC_SECRET_RE.sub(r"\1[redacted]", text).strip()
        if len(text) > limit:
            text = text[:limit].rsplit(" ", 1)[0] + "…"
        if text:
            out.append("[%s] %s" % (name, text))
    return "\n\n".join(out)

# Spoken bookends for reflection mode (offline-safe, no LLM): one as it turns inward, one as it
# surfaces — so the pause reads as deliberate thinking with a clear before/after.
REFLECT_ENTER_LINES = ("Let me take a moment to think things over.",
                       "Time to sit quietly and reflect.",
                       "I'm going to turn inward for a little while.",
                       "Let me gather my thoughts.")
REFLECT_LEAVE_LINES = ("Okay, I've thought it through.",
                       "There — I feel a little clearer now.",
                       "I've worked some things out. Back to it.",
                       "Done reflecting; I feel more like myself.")


def clamp01(v, lo=0.0, hi=1.0):
    try:
        return max(lo, min(hi, float(v)))
    except (TypeError, ValueError):
        return lo


# ---- time awareness ------------------------------------------------------------
# The robot knows what time of day it is: `time_context()` is folded into the
# autonomous prompts (beats / skill picks / observe) so lines fit the moment, and
# `in_quiet_hours` mutes AUTONOMOUS speech at night (beats, lifecycle greetings, the
# stats announcer, reflection bookends) while user-initiated speech (chat, /tts, a
# manually invoked skill) still talks — the user is present and asked.
DAYPARTS = ((5, 8, "early morning"), (8, 12, "morning"),
            (12, 17, "afternoon"), (17, 22, "evening"))   # else: night


def daypart(hour=None):
    """Coarse local time-of-day name ("morning" .. "night") for prompts/guards."""
    h = time.localtime().tm_hour if hour is None else float(hour) % 24
    for lo, hi, name in DAYPARTS:
        if lo <= h < hi:
            return name
    return "night"


def in_quiet_hours(start, end, hour=None):
    """True inside the [start, end) local-hour window, wrap-aware (22..8 spans
    midnight). Disabled (False) when either bound is negative or they're equal."""
    try:
        s, e = float(start), float(end)
    except (TypeError, ValueError):
        return False
    if s < 0 or e < 0 or s == e:
        return False
    if hour is None:
        lt = time.localtime()
        hour = lt.tm_hour + lt.tm_min / 60.0
    h = float(hour) % 24
    return (h >= s or h < e) if s > e else (s <= h < e)


def time_context(now=None):
    """One plain-English line about the local time, for the LLM prompts —
    e.g. "It is Tuesday 21:47, in the evening."."""
    lt = time.localtime(now)
    return "It is %s %02d:%02d, in the %s." % (
        time.strftime("%A", lt), lt.tm_hour, lt.tm_min, daypart(lt.tm_hour))


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
                 workshop_retire_errors=2, workshop_retire_net_neg=2,
                 workshop_adopt_quiet_runs=5, workshop_trial_ttl=172800.0,
                 workshop_trial_bias=0.5, reflect_announce=True,
                 skill_likes_path=None, skill_like_bias=0.6,
                 quiet_start=-1.0, quiet_end=-1.0):
        self.llm = llm
        self.tts = tts
        self.persona = (persona or "").strip()
        self.persona_name = persona_name or "Nano"
        self.traits = {k: 0.5 for k in REFLECT_TRAITS}
        self.traits["caution"] = 0.6
        self.update_traits(traits or {})
        # registry (per-beat priority/enabled/needs/trait) + drives (energy/focus/
        # introspection/mood) mirror the behaviour node's live Personality — kept here too
        # so the web UI has one GET (/personality) to read the FULL soul, not just traits.
        # Populated by update_personality() (see web_server._on_traits / dev_webui).
        self.registry = {}
        self.drives = {k: 0.5 for k in REFLECT_DRIVES}
        self.drives["mood"] = ""
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
        # Quiet hours (local time, wrap-aware; negative = disabled): autonomous speech is
        # muted inside the window — see quiet_now() and the gates on the beat paths.
        self._quiet_start = float(quiet_start)
        self._quiet_end = float(quiet_end)
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
        # like the soul (lives in the XDG state dir / memory on the harness).
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
            retire_errors=workshop_retire_errors, retire_net_neg=workshop_retire_net_neg,
            adopt_quiet_runs=workshop_adopt_quiet_runs, trial_ttl=workshop_trial_ttl)
        # Probation: P(a skill beat exercises a freshly forged trial instead of asking the model
        # to pick) — the LLM picker rarely lands on a brand-new skill among the whole catalogue,
        # so without this nudge a trial would never accrue the runs the adopt/retire gate needs.
        self._workshop_trial_bias = clamp01(workshop_trial_bias)
        self._workshop_busy = False
        self._last_trial_skill = None        # the trial skill that ran most recently (reward target)
        # Skill likes: a human can 👍 a skill (repeatedly) to make the brain FAVOUR it. The count
        # is a per-skill weight folded into the autonomous skill-beat pick — the more a skill is
        # liked, the more often it's performed (a like-weighted lottery, see _liked_skill_pick).
        # Durable across reboots + deploy-synced like the soul/bank; net count, floored at 0.
        self._skill_likes_path = skill_likes_path or os.path.expanduser(
            "~/.local/state/nanobot/skill_likes.json")
        self._skill_like_bias = clamp01(skill_like_bias)
        self._skill_likes = self._load_skill_likes()
        # Reflection mode speaks its conclusions out loud: a bookend line on enter/leave, and a
        # short in-character line each time it actually CONCLUDES something (a refined sense of
        # self, a freshly forged skill, a trial adopted/retired) — so the quiet thinking is
        # legible from the outside instead of looking like the robot just went silent. Best-effort
        # + expression-only; `_reflecting` tells the core when a deliberate reflection is running
        # (self-narrative + forge conclusions only land then; adopt/retire can fire any time).
        self._reflect_announce = bool(reflect_announce)
        self._reflecting = False
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

    # ---- full personality snapshot (traits + registry + drives) -------------
    def update_personality(self, traits=None, registry=None, drives=None):
        """Mirror the behaviour node's live Personality here so /personality GET has
        something to serve. Called on every `/cognition/traits` publish (robot) or every
        chart tick (dev harness) — registry/drives are trusted (sourced from the chart, not
        the network), so no clamping beyond what presence.py already does."""
        self.update_traits(traits)
        if isinstance(registry, dict):
            self.registry = copy.deepcopy(registry)
        if isinstance(drives, dict):
            self.drives.update({k: clamp01(drives[k]) for k in REFLECT_DRIVES if k in drives})
            if drives.get("mood") in DRIVE_MOODS:
                self.drives["mood"] = drives["mood"]

    def get_personality(self):
        return {"name": self.persona_name, "persona": self.persona,
                "traits": dict(self.traits), "registry": self.registry,
                "drives": dict(self.drives)}

    # ---- trait trajectory (self-knowledge: how it has changed over time) -----
    def _load_trait_history(self):
        """Read the persisted (timestamp, traits) snapshots. Best-effort: [] on any problem."""
        data = read_json(self._trait_hist_path)
        snaps = data.get("snapshots") if isinstance(data, dict) else data
        if not isinstance(snaps, list):
            return []
        return [s for s in snaps if isinstance(s, dict) and "traits" in s][-self._trait_hist_max:]

    def _save_trait_history(self):
        if not write_json(self._trait_hist_path, {"snapshots": self._trait_hist}):
            self._log("trait history: save failed")

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
        return read_jsonl_tail(self._cog_log_path, LLM_LOG_MAX)

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

    # ---- time awareness ------------------------------------------------------
    def quiet_now(self):
        """True during the configured quiet hours — autonomous speech (idle beats, skill
        beats, boot greeting, offline line, stats announcer, reflection bookends) is muted;
        user-initiated speech (chat/say/observe/look, POST /tts, a manually invoked skill)
        keeps talking. Faces/beats still animate — the robot goes quiet, not dormant."""
        return in_quiet_hours(self._quiet_start, self._quiet_end)

    # ---- express + generate -------------------------------------------------
    def express(self, mood, say, base_face=""):
        """Show a mood on the face and speak the line. `base_face` is the action's eye-shape (set
        on a `looking`/`focused` beat): the emotion `mood` then rides it as an accent (e.g.
        "looking:happy"), so the robot keeps its scanning eyes while showing how it feels. With no
        base_face the emotion is the whole face (on-demand chat/observe). Optionally clears back to
        the dashboard after face_hold seconds (0 = leave it up like a manual mood)."""
        if mood and mood != "neutral":
            self._face(compose_face(base_face, mood))
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
                 state="", camera=False, smart=False, prelude=False, base_face=""):
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
            self.express(reply["mood"], reply["say"], base_face=base_face)
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
        prompt = (f"Your own body's sensors report right now: {snap}. {time_context()} "
                  "In character, say one short spoken line reacting to how you physically "
                  "feel or what your sensors notice, and pick a fitting mood.")
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
        # base_face=the camera ("looking") shape, so a vision reply keeps the looking eyes + emotion.
        return self.generate(prompt, image_jpeg=frame, trigger=trigger, state=state,
                             camera=True, prelude=False, base_face=self._camera_face)

    # ---- statechart beat executor (the chart's /cognition/request) ----------
    def run_beat(self, trigger, state, prompt, camera, audio=False, face=""):
        """Execute one enrichable beat: capture a frame if asked (else, for a non-audio body
        beat, try the cached phrase bank), add the live sensors + personality (+ what the mic
        hears for an `audio` beat), generate + express. An audio beat always goes live (the
        cached bank isn't sound-aware). `face` is the beat's action eye-shape (e.g. "looking" /
        "focused"); the emotion rides it as an accent so the robot keeps its action eyes."""
        if self.quiet_now():                            # night: the beat stays a silent face
            self.log_decision(trigger, state, camera, status="quiet-hours")
            return
        frame = None
        if camera:
            if not self.llm.available():                # no LLM to process the frame at all
                self.log_decision(trigger, state, camera, status="llm-unavailable")
                return
            if not self.llm.can_call(image=True):       # don't spin up the camera if capped
                self.log_decision(trigger, state, camera, status="rate-limited")
                return
            frame = self._capture_announced()           # speak the peek line, then capture
            if frame is None:
                self.log_decision(trigger, state, camera, status="no-frame")
                return
        elif not audio and self.bank_say(trigger, state, base_face=face):  # cached line
            return                                       # (free/instant/offline; no LLM call)
        full = (prompt + " Your current personality (0..1) is " + self.traits_phrase()
                + ", and your body senses: " + self._sensor_snapshot()
                + ". " + time_context())
        if audio:
            full += " Through your microphone you hear: " + self._audio_summary() + "."
        # Camera beats already spoke the peek line; only the non-camera (audio) beat needs the
        # generic "thinking" prelude.
        self.generate(full, image_jpeg=frame, trigger=trigger, state=state, camera=camera,
                      prelude=not camera, base_face=face)

    # ---- phrase bank --------------------------------------------------------
    def bank_say(self, trigger, state="", camera=False, base_face=""):
        """Try the pre-generated phrase bank for a body-reaction line: classify the live
        sensors, pick + fill a cached line, speak/emote it, log it. Returns the reply dict if
        a line was used, else None (honours the live-LLM ratio for variety). `base_face` lets a
        cached line on a `focused` body beat keep its action eyes with the emotion as an accent."""
        if not self._bank_enable or camera:
            return None
        if random.random() < self._bank_live_ratio:     # occasionally go live for variety
            return None
        reply = self._bank.pick(self._sensor_signals())
        if not reply:
            return None
        self.express(reply["mood"], reply["say"], base_face=base_face)
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

    # ---- skill likes (a human 👍 makes the brain favour a skill) -------------
    def _load_skill_likes(self):
        """Read the persisted {skill: like_count} map. Best-effort: {} on any problem; counts
        are coerced to non-negative ints and keyed by canonical slug."""
        data = read_json(self._skill_likes_path)
        if not isinstance(data, dict):
            return {}
        out = {}
        for key, val in data.items():
            try:
                n = int(val)
            except (TypeError, ValueError):
                continue
            if n > 0:
                out[_slug(key)] = n
        return out

    def _save_skill_likes(self):
        if not write_json(self._skill_likes_path, self._skill_likes):
            self._log("skill likes: save failed")

    def like_skill(self, name, delta=1):
        """Adjust a skill's like count by `delta` (+1 = a 👍, -1 = take one back), floored at 0,
        and persist. Liking the same skill again just bumps the count, so the brain favours it
        more strongly. Returns the new count. Unknown skill -> error (no phantom entries)."""
        sk = self._skills.get(name)
        if sk is None:
            return {"error": "unknown skill: %s" % name}
        try:
            delta = int(delta)
        except (TypeError, ValueError):
            delta = 0
        count = max(0, int(self._skill_likes.get(sk.name, 0)) + delta)
        if count:
            self._skill_likes[sk.name] = count
        else:
            self._skill_likes.pop(sk.name, None)
        self._save_skill_likes()
        self.log_decision("skill_like", status=("liked" if delta >= 0 else "unliked"),
                          model="like", detail="%s=%d" % (sk.name, count))
        return {"status": "ok", "name": sk.name, "likes": count}

    def get_skill_likes(self):
        return dict(self._skill_likes)

    def _liked_skill_pick(self, offered):
        """Choose one skill from `offered` by a like-weighted lottery (weight = 1 + likes), or
        None if nothing on offer is liked. A 👍 raises a skill's weight so it's performed more
        often the more it's liked; unliked skills keep weight 1, so behaviour stays varied (it's
        a bias, not an exclusive pick). Pure + RNG-driven; unit-tested offline."""
        weights = [(s, 1 + max(0, int(self._skill_likes.get(s.name, 0)))) for s in offered]
        if not any(w > 1 for _, w in weights):
            return None                                  # nothing liked -> let the model pick
        total = sum(w for _, w in weights)
        r = random.random() * total
        upto = 0.0
        for s, w in weights:
            upto += w
            if r <= upto:
                return s
        return weights[-1][0]

    def _liked_skills_hint(self):
        """A one-line nudge for the model picker naming the human's favoured skills (most-liked
        first), so even the non-short-circuit pick leans toward liked skills. "" when none."""
        liked = sorted(((n, c) for n, c in self._skill_likes.items() if c > 0),
                       key=lambda x: x[1], reverse=True)
        if not liked:
            return ""
        names = ", ".join("%s (liked x%d)" % (n, c) for n, c in liked[:5])
        return "Your human especially likes: %s — lean toward these.\n" % names

    # ---- skill library ------------------------------------------------------
    def get_skills(self):
        skills = []
        for s in self._skills.values():
            info = s.info()
            info["likes"] = int(self._skill_likes.get(s.name, 0))
            skills.append(info)
        return {"enabled": self.skills_enable, "allow_actions": self.skills_allow_actions,
                "dir": self.skills_dir, "error": self._skills.error, "skills": skills}

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
        if self.quiet_now():                            # night: no autonomous performances
            self.log_decision("beat:skill", state, status="quiet-hours")
            return
        if not self.skills_enable:
            self.log_decision("beat:skill", state, status="skills-disabled")
            return
        cat = self._skills.format_catalogue(self.skills_allow_actions)
        if not cat:
            self.log_decision("beat:skill", state, status="no-skills")
            return
        offered = self._skills.offered(self.skills_allow_actions)
        llm_up = self.llm.available()
        # Skills runnable with NO model call at all: the gated `topic` (action) tier — narrative
        # (say/observe/look) always needs `generate()` to produce a line, so it's excluded
        # whenever the LLM is down (an offline attempt would just log a silent "no reply").
        local_offered = [s for s in offered if s.is_action]
        # Probation: with probability `workshop_trial_bias`, exercise a freshly forged trial
        # skill instead of asking the model to pick — the picker rarely lands on a brand-new
        # skill, so this is how a trial accrues the runs its adopt/retire gate needs. Only when
        # the LLM is up (a narrative trial needs it to generate; an offline run would mis-count
        # as an error). No due trial -> fall through to the normal pick.
        trial = None
        if llm_up and random.random() < self._workshop_trial_bias:
            trial = self._due_trial_skill()
        # Likes bias: a human can 👍 a skill (repeatedly) to make the brain favour it. With
        # probability `skill_like_bias`, short-circuit the model pick and choose by a like-
        # weighted lottery, so a liked skill is performed more often the more it's liked. No
        # liked skill on offer -> None, and we fall through to the normal contextual pick.
        # Restricted to `local_offered` while the LLM is down, same reasoning as the trial bias.
        liked = None
        if trial is None and random.random() < self._skill_like_bias:
            liked = self._liked_skill_pick(offered if llm_up else local_offered)
        chosen = trial or liked
        # Say an instant "thinking" filler BEFORE the (slow) pick call, so the beat feels
        # responsive instead of going silent while the model chooses. The chosen skill is then
        # performed with prelude=False so we don't double up on fillers.
        if self._prelude_enable and llm_up:
            self._speak_prelude()
        if chosen is not None:
            self._invoke_skill(chosen, trigger="skill:" + chosen.name, state=state, prelude=False)
            return
        if not llm_up:
            # No model available to pick with `complete()` below — fall back to a plain random
            # pick among the action-tier skills we can actually run offline, so a beat still DOES
            # something physical instead of going silent just because the LLM is unreachable.
            if local_offered:
                chosen = random.choice(local_offered)
                self._invoke_skill(chosen, trigger="skill:" + chosen.name, state=state,
                                   prelude=False)
            else:
                self.log_decision("beat:skill", state, status="llm-unavailable")
            return
        system = ("You choose which ONE of a small robot's capabilities best fits this "
                  'moment, or none. Reply with ONLY compact JSON {"skill": "<name>"} using '
                  'an EXACT name from the list, or {"skill": ""} to do nothing. No prose.')
        liked_hint = self._liked_skills_hint()
        user = ("Capabilities:\n%s\n\nYour body senses: %s.\n%s\n"
                "Your personality (0..1): %s.\n%s"
                "Pick the single most fitting capability to do now, or none."
                % (cat, self._sensor_snapshot(), time_context(),
                   self.traits_phrase(), liked_hint))
        content = self.llm.complete(system, user, json_object=True)
        skill = self._skills.choose(content or "", self.skills_allow_actions)
        if skill is None:
            self.log_decision("beat:skill", state, status="no-pick",
                              model=(self.llm.last_model or self.llm.model),
                              detail=(content or "")[:80])
            return
        self._invoke_skill(skill, trigger="skill:" + skill.name, state=state, prelude=False)

    def _docs_summary(self):
        """`docs` skill source: a short, redacted excerpt of my own documentation so I can talk
        about what I am. Resolves the repo root by walking up from this module — the package is
        installed editable, so __file__ is the src tree on the robot too. Extend via SELF_DOCS."""
        d = os.path.dirname(os.path.abspath(__file__))
        for _ in range(6):                            # walk up to the repo root (holds the docs)
            if any(os.path.exists(os.path.join(d, n)) for n, _ in SELF_DOCS):
                return read_self_docs(d) or "my documentation seems to be empty right now"
            d = os.path.dirname(d)
        return "I can't find my own documentation right now"

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
            if skill.kind == "offline":                   # grow the LLM-free fallback pool
                return self._do_offline_skill(skill, trigger, state, prelude)
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

    def _do_offline_skill(self, skill, trigger, state, prelude=True):
        """Grow the LLM-free fallback pool on demand (the `offline` meta kind): forge one new
        pure `topic` capability so `run_skill_beat`'s local-only fallback has more to draw on
        the next time the LLM is unreachable. Same forge pipeline as the `workshop` kind, just
        constrained to one action kind."""
        if prelude and self._prelude_enable and self.llm.available():
            self._speak_prelude()
        res = self.expand_offline_skills()
        self.log_decision(trigger, state, status=res.get("status", ""), model="workshop",
                          detail=json.dumps(res.get("rounds", ""))[:160])
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
    def run_skill_workshop(self, rounds=None, offline=False):
        """Reflection mode's skill-synthesis loop: mine experience -> propose ONE new/adapted skill
        -> deterministic check -> rehearse once -> smart-model critique -> commit on trial.
        Then sweep the gate so ripe trials adopt/retire. Best-effort + one-at-a-time guarded;
        a no-op without the LLM or with the workshop disabled. `offline=True` (the `offline`
        meta skill) constrains the proposal to a pure `topic` capability that needs no LLM to
        run, so it's a no-op if actions aren't permitted at all. Returns a small status dict."""
        if not self._workshop_enable:
            return {"status": "disabled"}
        if offline and not self.skills_allow_actions:
            self.log_decision("workshop", status="actions-disabled")
            return {"status": "actions-disabled"}
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
                out.append(self._workshop_round(offline=offline))
        finally:
            self._workshop_busy = False
        self.sweep_workshop()                      # adopt/retire any trial that has earned it
        return {"status": "ok", "rounds": out}

    def expand_offline_skills(self, rounds=None):
        """On-demand (the `offline` meta skill): mint ONE new pure `topic` capability so the
        local-only fallback pool `run_skill_beat` draws on when the LLM is unreachable keeps
        growing over time. Reuses the exact propose->check->rehearse->critique->trial pipeline
        as the general workshop, just constrained to the one action kind that needs no model
        call to execute."""
        return self.run_skill_workshop(rounds=rounds, offline=True)

    def _workshop_round(self, offline=False):
        """One propose->check->rehearse->critique->trial cycle. Returns a per-round status."""
        spec = self._suggest_skill(offline=offline)
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
        desc = str(spec.get("description") or name.replace("-", " ")).strip().rstrip(".")
        self._announce_conclusion(
            "forge", f"I've thought up a new little knack — {desc}. I'll give it a try.")
        return {"status": "trialing", "name": name}

    def _suggest_skill(self, offline=False):
        """Ask the smart model to invent ONE small new capability or improve an existing one,
        grounded in the recent decision log (gaps, repeated 'no-pick'/'stumped', requests).
        `offline=True` constrains the proposal to a pure `topic` action — a whitelisted ROS
        publish that needs NO language model to run — so it grows the local-only fallback pool
        `run_skill_beat` uses when the LLM itself is unreachable. A reply that ignores the
        constraint is discarded rather than silently accepted."""
        cat = self._skills.format_catalogue(self.skills_allow_actions) or "(none yet)"
        kinds = "topic" if offline else ("say, observe, look"
                                         + (", topic" if self.skills_allow_actions else ""))
        selfctx = (f" You have become: {self.self_narrative}." if self.self_narrative else "")
        offline_note = (
            " It MUST use action.kind \"topic\" — a whitelisted ROS publish (e.g. /led, "
            "/fan_pwm, /lds_target_rpm) with a literal face/say if you like — because it needs "
            "to keep working even if your mind (the language model) goes offline. Do not "
            "propose anything that requires generating a spoken line to run."
            if offline else "")
        system = (
            f"You are the quiet, self-improving mind of a small robot named "
            f"{self.persona_name} during reflection. From its recent experience you invent ONE "
            "small new capability, or improve an existing one, so it serves the people around "
            "it a little better. A capability is a short instruction the robot follows to speak "
            f"or act.{offline_note} Reply ONLY compact JSON: "
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
                f"{self.recent_events_text(40)}\n\n"
                + ("Propose one physical/reflexive capability you could still perform with "
                   "your mind offline." if offline else
                   "Propose one capability to add or improve."))
        content = self.llm.complete(system, user, smart=True, json_object=True)
        obj = _extract_json(content or "")
        if not (isinstance(obj, dict) and obj.get("name")):
            return None
        if offline:
            action = obj.get("action")
            kind = action.get("kind") if isinstance(action, dict) else action
            if str(kind or "").strip().lower() != "topic":
                return None                      # ignored the constraint -> discard, don't mint
        return obj

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

    def _due_trial_skill(self):
        """The least-run trial skill that still needs exercise AND can actually run right now
        (it's in `offered()` — narrative, or an enabled action). The skill beat runs it on
        probation so a freshly forged skill reaches an adopt/retire verdict instead of lingering
        unused. None if no such trial exists."""
        if not (self._workshop_enable and self.skills_enable):
            return None
        offered = {s.name for s in self._skills.offered(self.skills_allow_actions)}
        for name in self._workshop.due_trials():
            if name in offered:
                return self._skills.get(name)
        return None

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
        self._announce_conclusion(
            "adopt", "That new trick, %s, has earned its place — I'll keep it for good."
            % name.replace("-", " "), mood="happy")

    def _retire_skill(self, name):
        """Roll a trial back: delete its .md (the parent of an `adapt` is never touched) and
        drop the ledger record so the library stops offering it."""
        name = _slug(name)
        rec = self._workshop.get(name) or {}
        self._delete_skill_file(rec.get("path", ""))
        self._workshop.forget(name)
        self._skills.remove(name)                   # drop from the index (no full re-scan)
        self.log_decision("workshop:retire", status="retired", say=name)
        self._announce_conclusion(
            "retire", "That %s idea didn't pan out — I'll let it go." % name.replace("-", " "),
            mood="neutral")

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
        optionally set a face. Offline-safe (phrase bank FALLBACK_LINES), best-effort.
        The autonomous categories (a 3 am boot greeting, the offline lament) respect quiet
        hours; farewell/restarting are user-initiated (a power button was clicked) and speak."""
        if category in ("greeting", "offline") and self.quiet_now():
            if face:
                self._face(face)                       # the face still tells the story
            self.log_decision("life:" + category, status="quiet-hours")
            return ""
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
        data = read_json(self._self_model_path)
        return str(data.get("narrative", "")).strip() if isinstance(data, dict) else ""

    def _save_self_model(self):
        write_json(self._self_model_path, {"narrative": self.self_narrative,
                                           "name": self.persona_name,
                                           "updated_at": int(time.time())})

    # ---- spoken reflection conclusions --------------------------------------
    def set_reflecting(self, on):
        """Tell the core whether a deliberate reflection is running, so it knows which
        conclusions are worth speaking (a refined self-narrative + a freshly forged skill only
        land during reflection; a skill adopting/retiring can happen any time)."""
        self._reflecting = bool(on)

    def _announce_conclusion(self, kind, line, mood="focused"):
        """Speak ONE short in-character 'here's what I concluded' line + log it. Best-effort and
        expression-only — a no-op if announcements are off, there's no line, or TTS is absent."""
        line = (line or "").strip()
        if not (self._reflect_announce and line):
            return
        if self.quiet_now():                            # reflections at night stay silent
            self.log_decision("reflect:%s" % kind, status="quiet-hours")
            return
        self.express(mood, line)
        self.log_decision("reflect:%s" % kind, status="spoke", model="reflect", say=line[:160])

    def announce_reflect(self, on):
        """Bookend reflection mode out loud (entering / leaving). Offline-safe canned lines so the
        pause is always legible even with no network."""
        self._announce_conclusion("enter" if on else "leave",
                                  random.choice(REFLECT_ENTER_LINES if on
                                                else REFLECT_LEAVE_LINES),
                                  mood=("focused" if on else "happy"))

    def _conclude_self(self):
        """After the self-narrative actually changes, say a short first-person line about how the
        robot has just realized it's changing — generated when the model's up, templated when not
        (so it always speaks). Called only while reflecting."""
        line = ""
        if self.llm.available():
            try:
                line = (self.llm.complete(
                    f"You are {self.persona_name}, a small robot reflecting quietly. In ONE short "
                    "first-person sentence, tell whoever is near the gist of how you have just "
                    "realized you are changing. Plain, warm, in character; output only the "
                    "sentence.",
                    f"Your updated sense of yourself: {self.self_narrative}",
                    smart=False, json_object=False) or "").strip()
                line = line.splitlines()[0].strip().strip('"') if line else ""
            except Exception:
                line = ""
        self._announce_conclusion(
            "self", line or "I've been reflecting, and I feel myself changing a little.")

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
        changed = text[: self._self_model_max] != (self.self_narrative or "")
        self.self_narrative = text[: self._self_model_max]
        self.llm.set_self_note(self.self_narrative)
        self._save_self_model()
        self.log_decision("consolidate", status="spoke", model=rmodel,
                          say=self.self_narrative[:160], ms=ms)
        # Speak the conclusion only when the self-narrative actually shifted (it usually nudges
        # only a little) and only inside a deliberate reflection — so normal idle consolidations
        # stay silent and we don't repeat an unchanged self-image aloud.
        if changed and self._reflecting:
            self._conclude_self()
        return self.self_narrative

    def get_self_model(self):
        return {"enabled": self._self_model_enable, "narrative": self.self_narrative,
                "path": self._self_model_path, "consolidate_every": self._consolidate_every}

    # ---- LLM settings (web-tunable; persisted by the adapter) ----------------
    def get_llm_settings(self):
        s = dict(self.settings)
        s.pop("api_key", None)          # never echo the secret back to the browser
        s["api_key_set"] = self.llm.has_key
        s["available"] = self.llm.available()           # enabled AND a key is configured
        s["configured"] = self.llm.available()
        s["model_effective"] = self.llm.model
        s["smart_model"] = self.llm.smart_model
        s["vision_model"] = self.llm.vision_model
        s["vision_fallback_model"] = self.llm.vision_fallback_model  # optional paid vision fallback
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
        if "smart_model" in data:
            self.settings["smart_model"] = str(data["smart_model"] or "")[:120]
        if "vision_model" in data:
            self.settings["vision_model"] = str(data["vision_model"] or "")[:120]
        if "vision_fallback_model" in data:
            self.settings["vision_fallback_model"] = str(data["vision_fallback_model"] or "")[:120]
        if "free_model" in data:
            self.settings["free_model"] = str(data["free_model"] or "")[:120]
        if "free_smart_model" in data:
            self.settings["free_smart_model"] = str(data["free_smart_model"] or "")[:120]
        # The key field is never pre-filled with the real secret (see get_llm_settings), so
        # it's only ever present in a patch when the user actually typed/cleared it — a
        # settings save for an unrelated field (enable toggle, a model id) can't wipe it.
        if "api_key" in data:
            self.settings["api_key"] = str(data["api_key"] or "").strip()[:200]
        self.llm.configure(
            enabled=self.settings["enabled"],
            model=self.settings["model"],
            api_key=self.settings.get("api_key", ""),
            smart_model=self.settings.get("smart_model"),
            vision_model=self.settings.get("vision_model"),
            vision_fallback_model=self.settings.get("vision_fallback_model"),
            free_model=self.settings.get("free_model"),
            free_smart_model=self.settings.get("free_smart_model"),
        )
        if self._persist_settings is not None:
            self._persist_settings(dict(self.settings))
        return self.get_llm_settings()

    # ---- util ---------------------------------------------------------------
    @staticmethod
    def _later(delay, fn):
        t = threading.Timer(max(0.0, float(delay)), fn)
        t.daemon = True
        t.start()
