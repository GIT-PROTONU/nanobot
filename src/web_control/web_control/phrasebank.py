"""A pre-generated bank of spoken lines for the most frequent thing Nano does: react to
its own body (the autonomous `musing` beats + the manual "Observe"). Instead of paying a
live LLM call (latency + money + needs internet) every idle cycle, we pre-generate a
*bank* of in-character lines once — grouped by situation, with **placeholders** for live
values — then at runtime just classify the current sensor state, pick a line, and fill in
the numbers. Result: instant, free, works offline, still varied.

Deliberately ROS-free (like llm.py / tts.py) so it can be unit-tested or driven from the
dev harness / a CLI. It leans on `LlmClient` only for the one-off pre-generation.

  * **Placeholders** keep the cached lines flexible: ``"{temp} degrees. Warm enough to be
    alive."`` is generated once and reads correctly at any temperature. Available vars are
    {name} {cpu} {mem} {temp} {tilt}; a line uses only the ones that fit (the generator is
    told which are relevant per category). Unknown / missing placeholders fill to "".
  * **Soul drift re-generation**: the bank stores the *signature* (persona + traits) it was
    made with. When the personality has drifted "too much" (persona text changed, or the
    traits moved more than a threshold in total), `needs_regen()` is True and the caller
    re-runs `generate()` in the background — so the bank keeps sounding like who Nano has
    become, without regenerating on every tiny nudge.
  * **Incremental growth**: separately from full regeneration, the bank *grows over time*.
    `maybe_grow()` (rate-limited, called from the reflection / "learning" moment) asks the
    LLM for a few BRAND-NEW lines for the most under-filled situation and *appends* the
    non-duplicates (up to a per-category cap) — so the same offline-triggerable situations
    keep gaining fresh variety the more Nano runs, without throwing away what it already has.
    Growth only happens while the soul is stable (a drifted soul triggers a full regen first).

The bank is a JSON file (default ~/.local/state/nanobot/phrases.json), shared by the robot
(`web_server`) and the dev harness (`dev_webui.py`).
"""
import hashlib
import json
import os
import random
import re
import threading
import time

from .llm import MOODS, coerce_mood

# Situations Nano most often comments on, in *priority* order (first match wins), each with
# a plain-English description (fed to the generator) naming the placeholders that fit it.
# Keep these in sync with classify() below.
CATEGORIES = {
    "picked_up": "it has just been lifted off the ground and is being held in the air",
    "one_wheel": "one of its drive wheels has come off the ground",
    "tilted":    "it is tilted over a lot (about {tilt} degrees) and feels close to tipping",
    "jostled":   "it is being moved, pushed or jostled around",
    "leaning":   "it is leaning over slightly (about {tilt} degrees of tilt)",
    "hot":       "its main board is running hot (about {temp} degrees C)",
    "cold":      "its main board is quite cool (about {temp} degrees C)",
    "busy":      "its processor is working hard (about {cpu}% CPU load)",
    "idle":      "it is calm and idle — sitting level and still on the ground, nothing "
                 "notable happening",
    # --- lifecycle situations (NOT returned by classify(); the caller picks them by name at
    # the matching moment: startup / shutdown / restart / when the AI brain is unreachable).
    # Pre-generated like the rest so the line is instant + offline-safe; FALLBACK_LINES cover
    # a brand-new bank that has never been generated (e.g. the LLM was never configured).
    "greeting":   "it has just powered on / woken up and is booting to life, greeting whoever "
                  "is around",
    "farewell":   "it is shutting down and powering off for now, saying goodbye",
    "restarting": "it is restarting its own software to apply an update or to recover",
    "offline":    "its thinking AI brain is unreachable right now (no internet or no API key), "
                  "so it is running on simple built-in instincts only",
    # --- interaction fillers (also name-picked, never classified): a short line spoken the
    # INSTANT a slow LLM call starts (thinking) so there's no dead air, and a graceful line
    # when a call comes back empty (stumped) instead of going silent.
    "thinking":   "it has just been asked to do or say something and is taking a brief moment "
                  "to think before it answers. Reply with ONE-WORD fillers only (e.g. Hmm, "
                  "Thinking, Wait) — said out loud the instant it starts working",
    "stumped":    "it tried to think of something to say but its mind came up blank this time, "
                  "and it shrugs the lost thought off lightly",
    # --- camera peek: a SHORT line spoken the INSTANT before it points its camera and takes a
    # snapshot, so it always announces that it's looking (never silently peeks). Said out loud
    # the moment the camera is used, in parallel with the capture/vision call.
    "peeking":    "it is about to point its camera and take a quick look at what is in front of "
                  "it. Reply with ONE short spoken line announcing that it's taking a look (e.g. "
                  "Let me see, Peeking now, Taking a look) — said out loud the instant it looks",
}
# Built-in last-resort lines for the lifecycle categories, used when the bank has no entry for
# them yet (e.g. it was never generated because the LLM has never been online). Offline-safe.
# {name} is the only placeholder; the offline lines carry the "sleepy" mood (see oled_display).
FALLBACK_LINES = {
    "greeting":   [{"say": "Hi, {name} is awake.", "mood": "happy"},
                   {"say": "Good to be back. Hello there.", "mood": "happy"},
                   {"say": "Booting up and ready.", "mood": "happy"}],
    "farewell":   [{"say": "Powering down now. See you soon.", "mood": "neutral"},
                   {"say": "Going to sleep. Bye for now.", "mood": "neutral"}],
    "restarting": [{"say": "Restarting myself, one moment.", "mood": "focused"},
                   {"say": "Be right back, restarting.", "mood": "focused"}],
    "offline":    [{"say": "My thinking is offline right now.", "mood": "sleepy"},
                   {"say": "I can't reach my brain, running on instinct.", "mood": "sleepy"},
                   {"say": "No connection. Resting until my mind returns.", "mood": "sleepy"}],
    # The pre-call filler must be VERY short (one word) so it's spoken almost instantly.
    "thinking":   [{"say": "Hmm...", "mood": "focused"},
                   {"say": "Thinking...", "mood": "focused"},
                   {"say": "Wait...", "mood": "focused"},
                   {"say": "Working...", "mood": "focused"}],
    "stumped":    [{"say": "Hmm, my mind went blank there.", "mood": "neutral"},
                   {"say": "I lost my train of thought.", "mood": "neutral"},
                   {"say": "Sorry, the words escaped me.", "mood": "neutral"}],
    # The peek line must be SHORT (spoken in parallel with the capture, before the vision reply).
    "peeking":    [{"say": "Let me take a look.", "mood": "focused"},
                   {"say": "Peeking now...", "mood": "focused"},
                   {"say": "Let me see...", "mood": "focused"},
                   {"say": "Having a look around.", "mood": "focused"}],
}
# Lifecycle categories are picked by name, never by classify().
LIFECYCLE_CATEGORIES = tuple(FALLBACK_LINES.keys())
PLACEHOLDER_HELP = ("You may use these placeholders, which are filled with live values at "
                    "speak time: {name} (its name), {cpu} (CPU load percent), {mem} (memory "
                    "percent used), {temp} (main-board temperature in C), {tilt} (tilt angle "
                    "in degrees). Use a placeholder ONLY where it reads naturally for THIS "
                    "situation; do not invent any other placeholders.")
SIG_TRAITS = ("curiosity", "extraversion", "caution", "playfulness")
DEFAULT_PATH = os.path.expanduser("~/.local/state/nanobot/phrases.json")
_PLACEHOLDER_RE = re.compile(r"\{[a-zA-Z_]+\}")


class _SafeDict(dict):
    """str.format_map helper: any missing/unknown placeholder fills to '' instead of raising."""
    def __missing__(self, key):
        return ""


def classify(signals):
    """Map a structured sensor snapshot to (category, vars). `signals` is a dict with any of:
    cpu, mem, temp (floats or None), moving (bool), tilt (float degrees or None),
    pickup (0 none / 1 one wheel / 2 held). `vars` is the placeholder fill dict."""
    s = signals or {}
    cpu, mem, temp = s.get("cpu"), s.get("mem"), s.get("temp")
    tilt, pickup, moving = s.get("tilt"), int(s.get("pickup") or 0), bool(s.get("moving"))

    def num(v):
        return "" if v is None or v != v else f"{float(v):.0f}"      # NaN-safe -> ""
    vars_ = {"cpu": num(cpu), "mem": num(mem), "temp": num(temp), "tilt": num(tilt)}

    if pickup >= 2:
        cat = "picked_up"
    elif pickup == 1:
        cat = "one_wheel"
    elif tilt is not None and tilt == tilt and tilt > 25:
        cat = "tilted"
    elif moving:
        cat = "jostled"
    elif tilt is not None and tilt == tilt and tilt > 10:
        cat = "leaning"
    elif temp is not None and temp == temp and temp >= 60:
        cat = "hot"
    elif temp is not None and temp == temp and temp <= 38:
        cat = "cold"
    elif cpu is not None and cpu == cpu and cpu >= 75:
        cat = "busy"
    else:
        cat = "idle"
    return cat, vars_


def _fill(template, vars_):
    """Fill {placeholders} from vars_ (missing -> ''), then tidy any double spaces / spaces
    before punctuation left by an emptied placeholder."""
    out = str(template).format_map(_SafeDict(vars_))
    out = re.sub(r"\s+([,.!?])", r"\1", out)
    return re.sub(r"\s{2,}", " ", out).strip()


def soul_system(persona, traits, name="Nano"):
    """The shared system prompt describing Nano's spoken voice + the placeholder contract.
    Used by both full (re)generation and incremental growth so they share one voice."""
    traitline = ", ".join(f"{k} {float((traits or {}).get(k, 0.5)):.2f}" for k in SIG_TRAITS)
    return (
        f"You write short spoken one-liners for a small mobile robot named {name}. "
        f"{(persona + ' ') if persona else ''}Its personality on a 0..1 scale is: "
        f"{traitline}. Lines are spoken aloud through a tiny speaker: brief and natural "
        "to hear (at most ~20 words, no emoji, no markdown, no stage directions). "
        + PLACEHOLDER_HELP)


def signature(persona, traits):
    """A compact fingerprint of the 'soul' a bank was generated for: a hash of the persona
    text + the (rounded) trait vector. Drift is measured against the stored traits."""
    h = hashlib.sha1((persona or "").strip().encode("utf-8")).hexdigest()[:12]
    tr = {k: round(float((traits or {}).get(k, 0.5)), 2) for k in SIG_TRAITS}
    return {"persona": h, "traits": tr}


class PhraseBank:
    def __init__(self, path=None, logger=None):
        self.path = path or DEFAULT_PATH
        self._log = logger or (lambda *_: None)
        self._lock = threading.Lock()
        self._data = self._load()
        self._regen_busy = False

    # ---- persistence --------------------------------------------------------
    def _load(self):
        try:
            with open(self.path, encoding="utf-8") as f:
                d = json.load(f)
            if isinstance(d, dict) and isinstance(d.get("categories"), dict):
                return d
        except Exception:
            pass
        return {"version": 1, "signature": None, "generated_at": 0, "categories": {}}

    def _save(self):
        try:
            os.makedirs(os.path.dirname(self.path), exist_ok=True)
            with open(self.path, "w", encoding="utf-8") as f:
                json.dump(self._data, f, indent=1)
        except Exception as exc:
            self._log(f"phrasebank: save failed ({exc})")

    def is_empty(self):
        with self._lock:
            return not any(self._data.get("categories", {}).values())

    def stats(self):
        with self._lock:
            cats = self._data.get("categories", {})
            return {"signature": self._data.get("signature"),
                    "generated_at": self._data.get("generated_at", 0),
                    "grown_at": self._data.get("grown_at", 0),
                    "counts": {k: len(v) for k, v in cats.items()},
                    "total": sum(len(v) for v in cats.values())}

    # ---- runtime pick -------------------------------------------------------
    def pick(self, signals, name=None, category=None):
        """Pick a cached line and return {"say","mood","category"} — or None if the bank has
        nothing usable (caller then falls back to the live LLM / a default).

        If `category` is given, pick from THAT category by name (for the lifecycle lines —
        greeting / farewell / restarting / offline — which aren't sensor-classified), falling
        back to the built-in FALLBACK_LINES for it, then None. Otherwise classify `signals`
        and pick a body-reaction line (falling back to 'idle'). Prefers lines whose every
        placeholder can be filled right now, so a cached line that references e.g. {tilt}
        isn't chosen when tilt is unknown (which would read wrong)."""
        if category is not None:
            cat = str(category)
            with self._lock:
                entries = list((self._data.get("categories", {}) or {}).get(cat) or [])
                nm = name or self._data.get("name") or "Nano"
            entries = entries or FALLBACK_LINES.get(cat, [])
            vars_ = {"name": nm}
        else:
            cat, vars_ = classify(signals)
            with self._lock:
                cats = self._data.get("categories", {})
                entries = cats.get(cat) or cats.get("idle") or []
                nm = name or self._data.get("name") or "Nano"
            vars_["name"] = nm
        fillable = [e for e in entries if self._fillable(e.get("say", ""), vars_)]
        plain = [e for e in entries if not _PLACEHOLDER_RE.search(e.get("say", ""))]
        pool = fillable or plain or entries           # best -> safe -> last resort
        entry = random.choice(pool) if pool else None
        if not entry:
            return None
        say = _fill(entry.get("say", ""), vars_)
        if not say:
            return None
        return {"say": say[:240], "mood": coerce_mood(entry.get("mood")), "category": cat}

    @staticmethod
    def _fillable(template, vars_):
        """True if every {placeholder} in template has a non-empty live value."""
        for m in _PLACEHOLDER_RE.findall(template):
            if not str(vars_.get(m[1:-1], "")).strip():
                return False
        return True

    # ---- soul-drift detection ----------------------------------------------
    def needs_regen(self, persona, traits, threshold=0.6):
        """True if the bank is empty, was made for a different persona, or the traits have
        drifted more than `threshold` (summed absolute change) since it was generated."""
        with self._lock:
            sig = self._data.get("signature")
            has_lines = any(self._data.get("categories", {}).values())
        if not has_lines or not sig:
            return True
        cur = signature(persona, traits)
        if cur["persona"] != sig.get("persona"):
            return True
        old = sig.get("traits", {})
        drift = sum(abs(cur["traits"][k] - float(old.get(k, 0.5))) for k in SIG_TRAITS)
        return drift > threshold

    # ---- (re)generation -----------------------------------------------------
    def generate(self, llm, persona, traits, name="Nano", per_category=6):
        """Blocking: ask the *smart* model for `per_category` in-character lines per
        situation (using placeholders) and write the bank. Returns True on success. Best-
        effort: if the LLM is unavailable or every category fails, leaves the old bank.

        The SMART tier is used on purpose: small cheap models reliably return an empty
        ``{"lines": []}`` for this batch (multi-line, structured) request, whereas the smart
        model follows it. Generation is rare (once per persona/drift) and free-first, so the
        extra capability is nearly free — unlike the per-beat lines, which stay on the cheap
        tier. Single-line beat generation is unaffected."""
        if llm is None or not llm.available():
            self._log("phrasebank: LLM unavailable — cannot generate")
            return False
        system = soul_system(persona, traits, name)
        new_cats = {}
        for cat, desc in CATEGORIES.items():
            user = (
                f"Situation: {desc}.\nWrite {per_category} DIFFERENT in-character spoken "
                f"lines {name} might say in this situation. Vary them. For each, also pick a "
                'mood (face) from exactly: ' + ", ".join(MOODS) + ". Reply with ONLY compact "
                'JSON: {"lines": [{"say": "...", "mood": "..."}, ...]}.')
            lines = []
            for attempt in range(3):              # free models are flaky / return empty arrays;
                raw = llm.complete(system, user, smart=True, json_object=True)
                lines = self._parse_lines(raw)
                if lines:
                    break
                time.sleep(1.5)                   # pace the burst + let a rotating free slug recover
            if lines:
                new_cats[cat] = lines
                self._log(f"phrasebank: {cat}: {len(lines)} lines")
            else:
                self._log(f"phrasebank: {cat}: no lines (kept old)")
        if not new_cats:
            return False
        with self._lock:
            cats = dict(self._data.get("categories", {}))
            cats.update(new_cats)                       # keep any category that failed this run
            self._data = {"version": 1, "signature": signature(persona, traits),
                          "name": name, "generated_at": int(time.time()), "categories": cats}
            self._save()
        return True

    def maybe_regenerate(self, llm, persona, traits, name="Nano", threshold=0.6,
                         per_category=6, background=True):
        """Regenerate iff needs_regen(). When background=True runs in a daemon thread and
        returns immediately (the bank keeps serving old lines until the new ones land).
        One regeneration at a time."""
        if not self.needs_regen(persona, traits, threshold):
            return False
        if self._regen_busy:
            return False

        def work():
            self._regen_busy = True
            try:
                self._log("phrasebank: soul drifted / empty — regenerating…")
                self.generate(llm, persona, traits, name=name, per_category=per_category)
            finally:
                self._regen_busy = False
        if background:
            threading.Thread(target=work, daemon=True).start()
        else:
            work()
        return True

    # ---- incremental growth -------------------------------------------------
    @staticmethod
    def _norm(text):
        """Normalise a line for duplicate detection: lowercase, collapse whitespace, drop
        edge punctuation. Two lines that differ only in casing / trailing '.' are 'the same'."""
        return re.sub(r"\s+", " ", str(text or "").strip().lower()).strip(" .!?,")

    def grow(self, llm, persona, traits, name="Nano", batch=3, max_per_category=24,
             categories=None):
        """Incrementally ADD new, distinct lines to the most under-filled offline category,
        growing the bank over time instead of regenerating it wholesale. Picks the category
        with the fewest lines still below `max_per_category`, asks the LLM for `batch` fresh
        lines that AVOID the ones already cached (passed in the prompt), and appends the
        non-duplicates (capped). Returns (category, n_added) or None (LLM down / every
        category full / nothing usable). Best-effort: leaves the bank intact on failure, but
        always stamps `grown_at` so a failing/idle attempt doesn't hammer the LLM."""
        if llm is None or not llm.available():
            return None
        cats_src = list(categories) if categories else list(CATEGORIES.keys())
        with self._lock:
            existing = self._data.get("categories", {}) or {}
            below = [(len(existing.get(c) or []), i, c) for i, c in enumerate(cats_src)
                     if len(existing.get(c) or []) < max_per_category]
            if not below:
                return None
            below.sort()                                  # fewest lines first; ties -> reg. order
            cat = below[0][2]
            have = [str(e.get("say", "")) for e in (existing.get(cat) or [])]
        desc = CATEGORIES.get(cat, cat)
        system = soul_system(persona, traits, name)
        avoid = "\n".join(f"- {h}" for h in have[-12:])   # show the most recent to steer away
        user = (
            f"Situation: {desc}.\n"
            + (f"{name} already says these lines here — do NOT repeat or lightly reword "
               f"them:\n{avoid}\n\n" if avoid else "")
            + f"Write {batch} BRAND-NEW in-character spoken lines for this SAME situation, "
            "each clearly different from any above (fresh wording and a fresh angle). For "
            "each, pick a mood (face) from exactly: " + ", ".join(MOODS) + ". Reply with "
            'ONLY compact JSON: {"lines": [{"say": "...", "mood": "..."}, ...]}.')
        lines = []
        for _ in range(3):                                # free models are flaky / return []
            raw = llm.complete(system, user, smart=True, json_object=True)
            lines = self._parse_lines(raw)
            if lines:
                break
            time.sleep(1.5)
        added = 0
        with self._lock:
            cats = dict(self._data.get("categories", {}))
            pool = list(cats.get(cat) or [])
            seen = {self._norm(e.get("say", "")) for e in pool}
            for ln in lines:
                if len(pool) >= max_per_category:
                    break
                key = self._norm(ln.get("say", ""))
                if not key or key in seen:                # skip blanks + duplicates
                    continue
                seen.add(key)
                pool.append(ln)
                added += 1
            if added:
                cats[cat] = pool
                self._data["categories"] = cats
            self._data["grown_at"] = int(time.time())     # always stamp -> period gate resets
            self._save()
        if added:
            self._log(f"phrasebank: grew {cat} +{added} ({len(pool)}/{max_per_category})")
        return (cat, added) if added else None

    def maybe_grow(self, llm, persona, traits, name="Nano", period=1800.0, batch=3,
                   max_per_category=24, drift_threshold=0.6, background=True):
        """Grow the bank by one category at most once per `period` seconds. No-op if a
        regen/grow is already running, the soul has drifted (a full regen should run first so
        we don't accrete old-voice lines), or the period hasn't elapsed since the last grow /
        generation. Runs in a daemon thread when background=True. One growth at a time."""
        if self._regen_busy:
            return False
        if self.needs_regen(persona, traits, drift_threshold):
            return False
        with self._lock:
            last = float(self._data.get("grown_at") or self._data.get("generated_at") or 0)
        if (time.time() - last) < float(period):
            return False

        def work():
            self._regen_busy = True
            try:
                self.grow(llm, persona, traits, name=name, batch=batch,
                          max_per_category=max_per_category)
            finally:
                self._regen_busy = False
        if background:
            threading.Thread(target=work, daemon=True).start()
        else:
            work()
        return True

    @staticmethod
    def _parse_lines(raw):
        """Pull a list of {say,mood} from a model reply, tolerating fences / stray prose."""
        if not raw:
            return []
        t = str(raw).strip()
        if t.startswith("```"):
            t = re.sub(r"^```[a-zA-Z]*\s*", "", t)
            t = re.sub(r"\s*```$", "", t).strip()
        obj = None
        try:
            obj = json.loads(t)
        except Exception:
            m = re.search(r"\{.*\}", t, re.DOTALL)       # first {...} that parses
            if m:
                try:
                    obj = json.loads(m.group(0))
                except Exception:
                    obj = None
        items = obj.get("lines") if isinstance(obj, dict) else (obj if isinstance(obj, list) else None)
        if not isinstance(items, list):
            return []
        out = []
        for it in items:
            if isinstance(it, dict):
                say = str(it.get("say") or "").strip()
            elif isinstance(it, str):
                say = it.strip()
            else:
                continue
            if not say:
                continue
            mood = coerce_mood(it.get("mood") if isinstance(it, dict) else "")
            out.append({"say": say[:240], "mood": mood})
        return out
