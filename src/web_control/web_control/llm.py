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
import urllib.error
import urllib.request

# The OLED can only render these expressions. "neutral" is the calm default face;
# "" (dashboard) is never chosen by the model — clearing is the caller's job.
MOODS = ("happy", "angry", "focused", "stress", "neutral")
DEFAULT_BASE_URL = "https://openrouter.ai/api/v1/chat/completions"
# Cheap default text model used for *everything* except the smarter chat path: the
# autonomous beats, observe, say, and the camera-look's text reasoning all run on this.
DEFAULT_MODEL = "deepseek/deepseek-v4-flash"        # any OpenRouter model id works
# A slightly-better text model used ONLY for the conversational chat (generate(smart=True)),
# where coherence over a back-and-forth matters more. Same family, ~5x the per-call cost.
DEFAULT_SMART_MODEL = "deepseek/deepseek-v4-pro"
# A *vision* (multimodal) model used only when an image is attached (the text models can't
# see). DeepSeek has no vision model, so this stays a separate id. Free + verified working;
# swap for a paid multimodal id (Gemini/GPT/Claude vision) for sharper descriptions.
DEFAULT_VISION_MODEL = "nvidia/nemotron-nano-12b-v2-vl:free"
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


def _extract_json(text):
    """Pull our ``{"say","mood"}`` object out of a model reply, tolerating code fences,
    a stray sentence, or a *reasoning model* that narrates its thinking first and emits
    the JSON answer LAST. Returns a dict, or {} if nothing usable parses."""
    if not text:
        return {}
    t = text.strip()
    # Strip a ```json ... ``` (or bare ```) fence if the model wrapped its answer.
    if t.startswith("```"):
        t = re.sub(r"^```[a-zA-Z]*\s*", "", t)
        t = re.sub(r"\s*```$", "", t).strip()
    try:
        obj = json.loads(t)
        if isinstance(obj, dict):
            return obj
    except Exception:
        pass
    # Our object is flat (no nesting), so match each {...} span and prefer the LAST one
    # carrying our keys — reasoning models put prose first and the real answer at the end.
    for cand in reversed(re.findall(r"\{[^{}]*\}", t, re.DOTALL)):
        try:
            obj = json.loads(cand)
        except Exception:
            continue
        if isinstance(obj, dict) and ("say" in obj or "mood" in obj):
            return obj
    return {}


class LlmClient:
    """Stateless OpenRouter caller. `generate()` blocks on the HTTP request, so call it
    from a worker thread / request handler thread — never the ROS executor thread."""

    def __init__(self, enabled=True, api_key="", model=None, base_url=None,
                 persona="", timeout=30.0, max_tokens=1024, vision_model=None,
                 smart_model=None, logger=None):
        # Env wins when the explicit key is blank, so the secret can stay out of the
        # (version-controlled) robot.yaml entirely.
        self._key = (api_key or "").strip() or os.environ.get("OPENROUTER_API_KEY", "").strip()
        self._enabled = bool(enabled)
        self._model = (model or "").strip() or DEFAULT_MODEL
        self._smart_model = (smart_model or "").strip() or DEFAULT_SMART_MODEL
        self._vision_model = (vision_model or "").strip() or DEFAULT_VISION_MODEL
        self._url = (base_url or "").strip() or DEFAULT_BASE_URL
        self._persona = (persona or "").strip()
        self._timeout = float(timeout) if timeout else 20.0
        self._max_tokens = int(max_tokens) if max_tokens else 160
        self._log = logger or (lambda *_: None)

    # ---- config (web-tunable at runtime) ------------------------------------
    def configure(self, enabled=None, model=None, persona=None, api_key=None,
                  vision_model=None, smart_model=None):
        if enabled is not None:
            self._enabled = bool(enabled)
        if model is not None:
            self._model = model.strip() or DEFAULT_MODEL
        if smart_model is not None:
            self._smart_model = smart_model.strip() or DEFAULT_SMART_MODEL
        if vision_model is not None:
            self._vision_model = vision_model.strip() or DEFAULT_VISION_MODEL
        if persona is not None:
            self._persona = persona.strip()
        if api_key is not None:
            self._key = api_key.strip() or os.environ.get("OPENROUTER_API_KEY", "").strip()

    @property
    def model(self):
        return self._model

    @property
    def smart_model(self):
        return self._smart_model

    @property
    def vision_model(self):
        return self._vision_model

    def model_for(self, smart=False, image=False):
        """Which model a given call will use — handy for logging the decision."""
        if image:
            return self._vision_model
        return self._smart_model if smart else self._model

    @property
    def persona(self):
        return self._persona

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
        content = self._chat(messages, self.model_for(smart=smart, image=bool(image_jpeg)),
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
        model = self._smart_model if smart else self._model
        return self._chat(messages, model, max_tokens=max_tokens, json_object=json_object)

    def _chat(self, messages, model, max_tokens=None, json_object=False):
        """Low-level OpenRouter chat call. Returns the assistant message content (str) or
        None. Never raises — logs + returns None on any HTTP/network/shape error."""
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
        try:
            with urllib.request.urlopen(req, timeout=self._timeout) as resp:
                data = json.loads(resp.read().decode("utf-8", "replace"))
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", "replace")[:200] if hasattr(exc, "read") else ""
            self._log(f"llm: HTTP {exc.code} {detail}")
            return None
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            self._log(f"llm: request failed ({exc})")     # offline / DNS / timeout
            return None
        except Exception as exc:
            self._log(f"llm: unexpected error ({exc})")
            return None
        try:
            return data["choices"][0]["message"]["content"]
        except (KeyError, IndexError, TypeError):
            self._log(f"llm: unexpected response shape: {str(data)[:200]}")
            return None
