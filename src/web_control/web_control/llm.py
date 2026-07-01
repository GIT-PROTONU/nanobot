"""OpenRouter-backed speech + expression generation for the robot's personality.

Deliberately ROS-free (like tts.py) so it has no rclpy import and can be unit-tested
or driven from a plain CLI on a dev PC (see scripts/dev_tts_test.py). One *blocking*
`generate()` call hits OpenRouter's chat-completions endpoint over stdlib urllib (no
new dependency, no SDK) and returns a tiny ``{"say": str, "mood": str}`` the caller
speaks (TTS) and shows on the OLED (`/oled_face`).

Everything degrades to ``None`` — meaning "the caller does nothing" — when the
feature is disabled, no API key is configured, or the network/API call fails. The
robot must keep working perfectly with no internet, so an LLM line is always a
best-effort garnish, never load-bearing.

The model is constrained to the OLED's small fixed face vocabulary (see
oled_display KNOWN_MOODS + "neutral"); anything else it returns is coerced to
"neutral". Output is parsed as JSON, robustly (code fences / stray prose stripped),
so a chatty model still works.

Config (model, key, persona, timeout) is injected by the web layer from robot.yaml;
the API key may instead come from the OPENROUTER_API_KEY env var (which wins when the
robot.yaml value is left blank, so you never have to commit a secret).
"""
import base64
import json
import os
import re
import threading
import time
import urllib.error
import urllib.request
from collections import deque

# The OLED can only render these expressions. "neutral" is the calm default face;
# "" (dashboard) is never chosen by the model — clearing is the caller's job.
MOODS = ("happy", "angry", "focused", "stress", "neutral")
DEFAULT_BASE_URL = "https://openrouter.ai/api/v1/chat/completions"
# FREE-FIRST: each text tier tries one or more FREE OpenRouter models (comma-separated,
# in order) and only falls back to the PAID DeepSeek model below when ALL the free ones are
# over their (shared, ~daily / upstream) rate limit — so routine chatter costs nothing and
# we only pay when the free quota is exhausted. Free `:free` slugs rotate and the popular
# ones get upstream-throttled, so we list a couple per tier (verified working 2026-06-28);
# pick current ones via OpenRouter's /models API if these stop responding.
DEFAULT_FREE_MODEL = "deepseek/deepseek-v4-flash"   # free, cheap tier
DEFAULT_FREE_SMART_MODEL = "deepseek/deepseek-v4-flash"  # free, smart/chat tier
# Fallbacks — used only when the matching free model is rate-limited.
DEFAULT_MODEL = "deepseek/deepseek-v4-flash"        # any OpenRouter model id works
DEFAULT_SMART_MODEL = "deepseek/deepseek-v4-flash"
# A *vision* (multimodal) call used only when an image is attached. openrouter/free routes
# the request to a current free model on OpenRouter's side.
DEFAULT_VISION_MODEL = "openrouter/free"
MAX_SAY = 240          # chars; keep spoken lines short (TTS hard-caps at 300 anyway)

# The fixed framing every request gets, on top of the user-tunable persona. We force a
# strict, tiny JSON shape so the reply is trivial + safe to parse and act on.
SYSTEM_BASE = (
    "You are the voice and face of a small mobile robot named Nano. You speak short "
    "spoken lines out loud through a tiny speaker and show a simple face on a small "
    "OLED screen. Stay in character and keep replies brief and natural to hear aloud "
    "(at most ~30 words, one or two sentences, no emoji, no markdown, no stage "
    "directions). Reply with ONLY a compact JSON object and nothing else, of the form "
    '{"say": "the spoken line", "mood": "MOOD"} where MOOD is exactly one of: '
    + ", ".join(MOODS)
    + ". The mood is the face you show while saying the line; pick the one that "
    "best fits it."
)


def coerce_mood(mood):
    """Map whatever the model said to a renderable OLED mood (default 'neutral')."""
    m = str(mood or "").strip().lower()
    return m if m in MOODS else "neutral"


def _strip_code_fence(text):
    """Drop a leading/trailing ```json … ``` (or bare ```) fence a model may wrap its
    answer in, and surrounding whitespace. Returns '' for falsy input."""
    if not text:
        return ""
    t = str(text).strip()
    if t.startswith("```"):
        t = re.sub(r"^```[a-zA-Z]*\s*", "", t)
        t = re.sub(r"\s*```$", "", t).strip()
    return t


def _extract_json(text, keys=("say", "mood")):
    """Pull a flat JSON object out of a model reply, tolerating code fences, a stray
    sentence, or a *reasoning model* that narrates its thinking first and emits the JSON
    answer LAST. Prefers the last ``{...}`` span carrying any of ``keys`` (the real answer
    follows the prose). Returns a dict, or {} if nothing usable parses."""
    t = _strip_code_fence(text)
    if not t:
        return {}
    try:
        obj = json.loads(t)
        if isinstance(obj, dict):
            return obj
    except Exception:
        pass
    for cand in reversed(re.findall(r"\{[^{}]*\}", t, re.DOTALL)):
        try:
            obj = json.loads(cand)
        except Exception:
            continue
        if isinstance(obj, dict) and any(k in obj for k in keys):
            return obj
    return {}


def _extract_json_list(text, key="lines"):
    """Pull a JSON list out of a model reply: either a top-level array, or an object whose
    ``key`` holds the array (tolerating fences / stray prose). Returns a list, or []."""
    t = _strip_code_fence(text)
    if not t:
        return []
    obj = None
    try:
        obj = json.loads(t)
    except Exception:
        m = re.search(r"\{.*\}|\[.*\]", t, re.DOTALL)   # first object/array that parses
        if m:
            try:
                obj = json.loads(m.group(0))
            except Exception:
                obj = None
    if isinstance(obj, dict):
        obj = obj.get(key)
    return obj if isinstance(obj, list) else []


class LlmClient:
    """Stateless OpenRouter caller. `generate()` blocks on the HTTP request, so call it
    from a worker thread / request handler thread — never the ROS executor thread."""

    def __init__(self, enabled=True, api_key="", model=None, base_url=None,
                 persona="", timeout=30.0, max_tokens=1024, vision_model=None,
                 smart_model=None, free_model=None, free_smart_model=None,
                 vision_fallback_model=None, smart_max_per_hour=0, vision_max_per_hour=0,
                 hard_deadline=45.0, logger=None):
        # Env wins when the explicit key is blank, so the secret can stay out of the
        # (version-controlled) robot.yaml entirely.
        self._key = (api_key or "").strip() or os.environ.get("OPENROUTER_API_KEY", "").strip()
        self._enabled = bool(enabled)
        self._model = (model or "").strip() or DEFAULT_MODEL          # paid cheap fallback
        self._smart_model = (smart_model or "").strip() or DEFAULT_SMART_MODEL  # paid smart fallback
        self._vision_model = (vision_model or "").strip() or DEFAULT_VISION_MODEL
        # Free primaries (tried before the paid fallbacks). "" disables free-first for that
        # tier (then it goes straight to the paid model). vision is already a free model, so
        # its (optional) fallback is empty by default — DeepSeek can't see.
        self._free_model = "" if free_model == "" else ((free_model or "").strip() or DEFAULT_FREE_MODEL)
        self._free_smart_model = ("" if free_smart_model == ""
                                  else (free_smart_model or "").strip() or DEFAULT_FREE_SMART_MODEL)
        self._vision_fallback = (vision_fallback_model or "").strip()
        self._last_model = ""                              # slug that produced the last reply
        self._url = (base_url or "").strip() or DEFAULT_BASE_URL
        self._persona = (persona or "").strip()
        # A durable, smart-LLM-maintained "self-narrative" (who the robot is becoming), folded
        # into the system prompt under the fixed persona. Set by CognitionCore.consolidate();
        # empty until the first consolidation. Long-term identity drift, vs the volatile traits.
        self._self_note = ""
        self._timeout = float(timeout) if timeout else 20.0
        # HARD wall-clock cap on a single call. urlopen's `timeout` is only a per-socket-operation
        # timeout, so a gateway that trickles keep-alive bytes (OpenRouter's free tier does this
        # while a model is queued) never trips it and a call can hang for ~2 min — holding the
        # caller's one-at-a-time guard and starving every other beat. A watchdog abandons the call
        # past this deadline. Kept comfortably above the slowest real reflection (~25 s).
        self._hard_deadline = max(float(hard_deadline) if hard_deadline else 45.0,
                                  self._timeout + 5.0)
        self._max_tokens = int(max_tokens) if max_tokens else 160
        self._log = logger or (lambda *_: None)
        # Hourly caps on the *expensive* tiers so autonomous beats/reflection can't run up
        # the bill: the smart/pro text model and the vision/camera model. 0 = unlimited.
        # The cheap default (flash) model is never capped. A sliding 1-hour window of call
        # timestamps per tier; calls over the cap return None (the caller stays silent).
        self._max = {"smart": max(0, int(smart_max_per_hour or 0)),
                     "vision": max(0, int(vision_max_per_hour or 0))}
        self._calls = {"smart": deque(), "vision": deque()}
        self._rate_lock = threading.Lock()

    # ---- config (web-tunable at runtime) ------------------------------------
    def configure(self, enabled=None, model=None, persona=None, api_key=None,
                  vision_model=None, smart_model=None, free_model=None,
                  free_smart_model=None, vision_fallback_model=None,
                  smart_max_per_hour=None, vision_max_per_hour=None):
        if enabled is not None:
            self._enabled = bool(enabled)
        if model is not None:
            self._model = model.strip() or DEFAULT_MODEL
        if smart_model is not None:
            self._smart_model = smart_model.strip() or DEFAULT_SMART_MODEL
        if vision_model is not None:
            self._vision_model = vision_model.strip() or DEFAULT_VISION_MODEL
        if vision_fallback_model is not None:
            self._vision_fallback = vision_fallback_model.strip()  # "" => no paid vision fallback
        if free_model is not None:
            self._free_model = free_model.strip()          # "" => paid-only for cheap tier
        if free_smart_model is not None:
            self._free_smart_model = free_smart_model.strip()
        if persona is not None:
            self._persona = persona.strip()
        if api_key is not None:
            self._key = api_key.strip() or os.environ.get("OPENROUTER_API_KEY", "").strip()
        if smart_max_per_hour is not None:
            self._max["smart"] = max(0, int(smart_max_per_hour))
        if vision_max_per_hour is not None:
            self._max["vision"] = max(0, int(vision_max_per_hour))

    # ---- hourly rate limiting (expensive tiers only) ------------------------
    @staticmethod
    def _tier(smart=False, image=False):
        """The cost tier a call falls in: 'vision' (image), 'smart' (pro text), or None
        (the cheap default model — never capped). Image wins, matching model_for()."""
        return "vision" if image else ("smart" if smart else None)

    def _prune(self, tier, now):
        dq = self._calls[tier]
        while dq and now - dq[0] >= 3600.0:
            dq.popleft()
        return dq

    def can_call(self, smart=False, image=False):
        """Peek (no slot consumed) whether SOME model for this tier is usable right now: a
        free primary is always usable; the paid fallback is usable only under its hourly cap.
        Lets a caller skip expensive prep (e.g. a webcam capture) only when even the fallback
        is capped out. (With a free primary configured this is essentially always True.)"""
        tier = self._tier(smart, image)
        for _model, is_paid in self._candidates(smart, image):
            if not is_paid or self._max.get(tier, 0) <= 0:
                return True
            with self._rate_lock:
                if len(self._prune(tier, time.monotonic())) < self._max[tier]:
                    return True
        return False

    def _rate_consume(self, tier):
        """Record a call against its hourly cap; return False (and record nothing) if it
        would exceed the cap. No-op/True for the uncapped cheap tier."""
        if tier is None or self._max[tier] <= 0:
            return True
        with self._rate_lock:
            now = time.monotonic()
            dq = self._prune(tier, now)
            if len(dq) >= self._max[tier]:
                return False
            dq.append(now)
            return True

    def rate_limits(self):
        """Current caps + usage in the last hour, for config display. {tier: [used, max]}."""
        with self._rate_lock:
            now = time.monotonic()
            return {t: [len(self._prune(t, now)), self._max[t]] for t in self._max}

    @property
    def model(self):
        return self._model

    @property
    def smart_model(self):
        return self._smart_model

    @property
    def vision_model(self):
        return self._vision_model

    @property
    def free_model(self):
        return self._free_model

    @property
    def free_smart_model(self):
        return self._free_smart_model

    @property
    def vision_fallback_model(self):
        return self._vision_fallback

    @property
    def last_model(self):
        """The slug that produced the most recent reply (free primary or paid fallback)."""
        return self._last_model

    @staticmethod
    def _is_free(model):
        m = str(model).strip().lower()
        # `:free` slugs and the openrouter/free meta-id are free (never count against caps).
        return m.endswith(":free") or m == "openrouter/free"

    def _candidates(self, smart=False, image=False):
        """Ordered (model, is_paid) to try for a call: the FREE model(s) first, the PAID
        fallback last (used only when ALL the free ones are over their rate/daily limit).
        The free fields may be comma-separated lists (tried in order). is_paid (slug ends in
        ':free' => free) flags which entries count against the hourly cap. Deduped; a blank
        free field => the paid model is the only candidate (no free-first for that tier)."""
        def split(s):
            return [x.strip() for x in str(s or "").split(",") if x.strip()]
        if image:
            models = split(self._vision_model) + split(self._vision_fallback)
        elif smart:
            models = split(self._free_smart_model) + [self._smart_model]
        else:
            models = split(self._free_model) + [self._model]
        out, seen = [], set()
        for m in models:
            m = (m or "").strip()
            if m and m not in seen:
                seen.add(m)
                out.append((m, not self._is_free(m)))
        return out

    def model_for(self, smart=False, image=False):
        """The model a call will TRY FIRST (the free primary) — handy for logging intent.
        Use `last_model` after a call for the one that actually answered."""
        c = self._candidates(smart=smart, image=image)
        return c[0][0] if c else self._model

    @property
    def persona(self):
        return self._persona

    def set_self_note(self, text):
        """Set the durable self-narrative folded into the system prompt (see `_self_note`)."""
        self._self_note = (text or "").strip()

    def available(self):
        """True if the feature is on AND a key is configured. Cheap — no network."""
        return bool(self._enabled and self._key)

    # ---- generation ----------------------------------------------------------
    def generate(self, prompt, history=None, image_jpeg=None, smart=False):
        """Ask the model for one spoken line + a mood. `prompt` is the user/trigger
        turn (e.g. a chat message, or "say something cheery to whoever's nearby").
        `history` is an optional list of prior ``{"role","content"}`` turns (for chat).
        `image_jpeg` (bytes) attaches a camera frame — when present the request goes to
        the *vision* model and the frame is sent as a base64 data-URI image part.
        `smart=True` (text only) routes to the slightly-better model (used for chat).

        Returns ``{"say": str, "mood": str}`` on success, or ``None`` — the caller
        treats None as "stay silent". Never raises."""
        if not self.available():
            return None
        system = SYSTEM_BASE + (("\n\n" + self._persona) if self._persona else "")
        if self._self_note:                                # durable, evolving self-narrative
            system += "\n\nWhat you have come to understand about yourself: " + self._self_note
        messages = [{"role": "system", "content": system}]
        if history:
            messages.extend(history)
        if image_jpeg:
            # Multimodal content: text + the frame as a data URI. Routes to the vision
            # model (the default text model can't see).
            b64 = base64.b64encode(image_jpeg).decode("ascii")
            messages.append({"role": "user", "content": [
                {"type": "text", "text": str(prompt)},
                {"type": "image_url",
                 "image_url": {"url": "data:image/jpeg;base64," + b64}},
            ]})
        else:
            messages.append({"role": "user", "content": str(prompt)})

        # JSON-object hint for text; skipped for vision (some multimodal models reject it).
        content = self._chat(messages, smart=smart, image=bool(image_jpeg),
                             json_object=not image_jpeg)
        if content is None:
            return None
        obj = _extract_json(content)
        say = str(obj.get("say") or "").strip()[:MAX_SAY]
        if not say:
            # No usable JSON line. We deliberately do NOT speak the raw content as a
            # fallback: for a reasoning model that would read its chain-of-thought aloud.
            # Staying silent (None) is the safe best-effort behaviour.
            self._log(f"llm: no JSON line in reply: {str(content)[:160]}")
            return None
        return {"say": say, "mood": coerce_mood(obj.get("mood"))}

    def complete(self, system, user, smart=False, max_tokens=None, json_object=True):
        """General-purpose completion (NOT framed as the {say,mood} face line): returns the
        model's raw text content, or None. Used by tools that need other JSON shapes — the
        personality creator (smart=True) and, later, the reflection loop. `json_object`
        hints the provider to emit a JSON object."""
        if not self.available():
            return None
        messages = [{"role": "system", "content": str(system)},
                    {"role": "user", "content": str(user)}]
        return self._chat(messages, smart=smart, max_tokens=max_tokens, json_object=json_object)

    def _chat(self, messages, smart=False, image=False, max_tokens=None, json_object=False):
        """Try the tier's models in order (free primary -> paid fallback), returning the
        first reply. Falls through to the paid model ONLY when the free one is over its
        rate/daily limit; any other failure (bad request, network, odd shape) stops and
        returns None. The paid fallback is gated by the tier's hourly cap. Records the slug
        that answered in `last_model`. Never raises."""
        tier = self._tier(smart=smart, image=image)
        candidates = self._candidates(smart=smart, image=image)
        for model, is_paid in candidates:
            if is_paid and not self._rate_consume(tier):
                self._log(f"llm: {tier} paid cap reached ({self._max.get(tier, 0)}/h) "
                          f"— not falling back to {model}")
                break                                      # don't try the (capped) paid model
            content, kind = self._call_one(messages, model, max_tokens=max_tokens,
                                           json_object=json_object)
            self._last_model = model
            if content is not None:
                return content
            if kind != "ratelimit":
                return None                                # not a quota issue -> no fallback
            self._log(f"llm: {model} over rate/daily limit — trying fallback")
        return None

    @staticmethod
    def _looks_ratelimited(text):
        t = str(text or "").lower()
        return any(s in t for s in ("rate limit", "rate-limit", "ratelimit", "quota",
                                    "exhausted", "too many requests", "daily limit",
                                    "per-day", "limit reached", "429"))

    def _call_one(self, messages, model, max_tokens=None, json_object=False):
        """One OpenRouter call to a SPECIFIC model. Returns (content, kind): (str, None) on
        success; (None, "ratelimit") when the model is over its rate/daily limit (the caller
        may fall back); (None, "other") on any other failure. Never raises."""
        payload = {
            "model": model,
            "messages": messages,
            "max_tokens": int(max_tokens or self._max_tokens),
            "temperature": 0.8,
            # Keep reasoning out of the response (reasoning models otherwise narrate their
            # thinking into content + blow the token budget). Ignored by non-reasoning models.
            "reasoning": {"exclude": True},
        }
        if json_object:
            payload["response_format"] = {"type": "json_object"}
        body = json.dumps(payload).encode("utf-8")
        req = urllib.request.Request(self._url, data=body, method="POST", headers={
            "Authorization": f"Bearer {self._key}",
            "Content-Type": "application/json",
            # OpenRouter asks for these for attribution/routing; harmless if generic.
            "HTTP-Referer": "https://github.com/nanobot",
            "X-Title": "Nano robot",
        })
        # Run the blocking request on a worker thread and enforce a HARD wall-clock deadline on
        # top of the per-socket timeout (see _hard_deadline). If the deadline passes we abandon
        # the call (return "other"); the orphan thread dies on its own when its socket read
        # finally times out, and it touches only local `result`, so it can't corrupt anything.
        result = {}

        def _do_request():
            try:
                with urllib.request.urlopen(req, timeout=self._timeout) as resp:
                    result["data"] = json.loads(resp.read().decode("utf-8", "replace"))
            except urllib.error.HTTPError as exc:
                result["http"] = (exc.code, exc.read().decode("utf-8", "replace")[:200]
                                  if hasattr(exc, "read") else "")
            except Exception as exc:                    # URLError / TimeoutError / OSError / parse
                result["err"] = exc

        worker = threading.Thread(target=_do_request, daemon=True)
        worker.start()
        worker.join(self._hard_deadline)
        if worker.is_alive():
            self._log(f"llm: hard deadline {self._hard_deadline:.0f}s exceeded ({model}) — abandoned")
            return None, "other"
        if "http" in result:
            code, detail = result["http"]
            kind = "ratelimit" if (code in (429, 402) or self._looks_ratelimited(detail)) else "other"
            self._log(f"llm: HTTP {code} ({model}) {detail}")
            return None, kind
        if "err" in result:
            self._log(f"llm: request failed ({model}: {result['err']})")  # offline / DNS / timeout
            return None, "other"
        data = result.get("data")
        # OpenRouter sometimes returns HTTP 200 with an error object (esp. for free models).
        if isinstance(data, dict) and data.get("error"):
            err = data["error"] if isinstance(data["error"], dict) else {"message": data["error"]}
            msg = json.dumps(err)[:200]
            kind = ("ratelimit" if (err.get("code") in (429, 402)
                                    or self._looks_ratelimited(msg)) else "other")
            self._log(f"llm: error body ({model}) {msg}")
            return None, kind
        try:
            return data["choices"][0]["message"]["content"], None
        except (KeyError, IndexError, TypeError):
            self._log(f"llm: unexpected response shape ({model}): {str(data)[:200]}")
            return None, "other"
