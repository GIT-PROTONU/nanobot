"""Offline tests for the two fixes that came out of the last-run analysis (ROS-free):

  1. **Always announce the peek** — every camera path (beats / skills / the Look button) speaks a
     short "peeking/seeing" line BEFORE it captures a frame, so the robot never looks silently.
  2. **Hard wall-clock deadline** on an LLM call — urlopen's per-socket timeout can't bound a
     gateway that trickles keep-alive bytes, so a watchdog abandons the call past _hard_deadline
     (the root cause of the ~121 s vision hangs that starved every beat in the log).

    pixi run python -m pytest src/web_control/test
"""
import json
import time

from web_control.cognition import CognitionCore
from web_control import llm as llmmod
from web_control.llm import LlmClient


class _LLM:
    """Inert LlmClient stand-in (the peek is independent of the LLM)."""
    def __init__(self):
        self.last_model, self.smart_model = "", "x"

    def available(self):
        return False

    def set_self_note(self, _):
        pass


class _TTS:
    """Records spoken lines into a shared events list (for ordering assertions)."""
    def __init__(self, events):
        self.events = events

    def available(self):
        return True

    def say(self, text):
        self.events.append(("say", text))


def _core(tmp_path, events, **kw):
    s = lambda n: str(tmp_path / n)

    def capture():
        events.append(("capture", None))
        return b"JPEGBYTES"

    return CognitionCore(
        llm=_LLM(), tts=_TTS(events), persona="", persona_name="Nano",
        face=lambda m: events.append(("face", m)), capture_frame=capture,
        cog_log_path=s("c.log"), bank_path=s("p.json"), skills_dir="", skills_enable=False,
        self_model_path=s("sm.json"), workshop_path=s("w.json"), workshop_dir=s("sk"),
        trait_history_path=s("th.json"), **kw)


# ---- 1. peek-before-capture --------------------------------------------------
def test_peek_spoken_before_capture(tmp_path):
    events = []
    core = _core(tmp_path, events)
    frame = core._capture_announced()
    assert frame == b"JPEGBYTES"                       # the frame still comes back
    kinds = [e[0] for e in events]
    assert "say" in kinds and "capture" in kinds
    assert kinds.index("say") < kinds.index("capture")  # the peek is spoken FIRST
    # the spoken line is one of the offline-safe peeking fallbacks
    said = next(text for k, text in events if k == "say")
    assert said and len(said) < 60


def test_peek_shows_looking_face_before_capture(tmp_path):
    events = []
    core = _core(tmp_path, events)
    core._capture_announced()
    assert ("face", "looking") in events               # the dedicated peek face is shown
    kinds = [e[0] for e in events]
    assert kinds.index("face") < kinds.index("capture")  # ...before the capture


def test_camera_face_is_configurable(tmp_path):
    events = []
    core = _core(tmp_path, events, camera_face="focused")
    core._capture_announced()
    assert ("face", "focused") in events and ("face", "looking") not in events


def test_announce_can_be_disabled(tmp_path):
    events = []
    core = _core(tmp_path, events, camera_announce=False)
    frame = core._capture_announced()
    assert frame == b"JPEGBYTES"
    assert [e[0] for e in events] == ["capture"]       # captured silently, no peek line


def test_peeking_category_has_offline_fallback():
    from web_control.phrasebank import FALLBACK_LINES, CATEGORIES
    assert "peeking" in CATEGORIES                      # generator knows the situation
    assert FALLBACK_LINES.get("peeking")               # offline-safe even with an empty bank


# ---- 2. hard wall-clock deadline --------------------------------------------
def test_hard_deadline_abandons_a_hung_call(monkeypatch):
    c = LlmClient(enabled=True, api_key="x", timeout=10.0)
    c._hard_deadline = 0.3                              # bypass the production floor for the test

    def hung_urlopen(req, timeout=None):               # never returns in time (simulates a trickle)
        time.sleep(3.0)
        raise AssertionError("the watchdog should have abandoned this call")

    monkeypatch.setattr(llmmod.urllib.request, "urlopen", hung_urlopen)
    t0 = time.monotonic()
    content, kind = c._call_one([{"role": "user", "content": "hi"}], "some-model")
    dt = time.monotonic() - t0
    assert (content, kind) == (None, "other")
    assert dt < 1.5                                     # returned ~at the deadline, not after 3 s


def test_call_one_success_path_still_works(monkeypatch):
    c = LlmClient(enabled=True, api_key="x")

    class _Resp:
        def __init__(self, payload):
            self._b = json.dumps(payload).encode("utf-8")

        def read(self):
            return self._b

        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

    def ok_urlopen(req, timeout=None):
        return _Resp({"choices": [{"message": {"content": "hello world"}}]})

    monkeypatch.setattr(llmmod.urllib.request, "urlopen", ok_urlopen)
    content, kind = c._call_one([{"role": "user", "content": "hi"}], "m")
    assert kind is None and content == "hello world"
