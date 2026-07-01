"""Offline tests for reflection mode speaking its conclusions out loud (ROS-free):

  - bookend lines when reflection turns inward / surfaces (`announce_reflect`),
  - a short conclusion line when the robot refines its sense of self (`_conclude_self`),
  - everything gated by `reflect_announce` and (for self/forge) by `set_reflecting`.

The point: a reflection no longer looks like the robot just going silent — each conclusion is
audible + logged as `reflect:*`.

    pixi run python -m pytest src/web_control/test
"""
from web_control.cognition import CognitionCore


class _LLM:
    """Inert LlmClient stand-in — forces the offline/templated path."""
    def __init__(self):
        self.last_model, self.smart_model = "", "x"

    def available(self):
        return False

    def set_self_note(self, _):
        pass


class _TTS:
    def __init__(self, events):
        self.events = events

    def available(self):
        return True

    def say(self, text):
        self.events.append(("say", text))


def _core(tmp_path, events, **kw):
    s = lambda n: str(tmp_path / n)
    return CognitionCore(
        llm=_LLM(), tts=_TTS(events), persona="", persona_name="Nano",
        face=lambda m: events.append(("face", m)),
        cog_log_path=s("c.log"), bank_path=s("p.json"), skills_dir="", skills_enable=False,
        self_model_path=s("sm.json"), workshop_path=s("w.json"), workshop_dir=s("sk"),
        trait_history_path=s("th.json"), **kw)


def _spoken(events):
    return [t for k, t in events if k == "say"]


def _logged(core, trigger):
    return [e for e in core._cog_log if e.get("trigger") == trigger]


# ---- bookends ----------------------------------------------------------------
def test_enter_speaks_and_logs(tmp_path):
    events = []
    core = _core(tmp_path, events)
    core.announce_reflect(True)
    assert _spoken(events)                              # a "turning inward" line was spoken
    assert _logged(core, "reflect:enter")


def test_leave_speaks_a_different_bookend(tmp_path):
    events = []
    core = _core(tmp_path, events)
    core.announce_reflect(False)
    assert _spoken(events)
    assert _logged(core, "reflect:leave")
    assert not _logged(core, "reflect:enter")


def test_announce_can_be_disabled(tmp_path):
    events = []
    core = _core(tmp_path, events, reflect_announce=False)
    core.announce_reflect(True)
    core._conclude_self()
    core._announce_conclusion("forge", "some line")
    assert _spoken(events) == []                        # silent reflection when the knob is off
    assert list(core._cog_log) == []


# ---- self-understanding conclusion ------------------------------------------
def test_conclude_self_speaks_offline_fallback(tmp_path):
    events = []
    core = _core(tmp_path, events)
    core._conclude_self()                               # LLM inert -> templated line
    said = _spoken(events)
    assert said and said[0]
    assert _logged(core, "reflect:self")


# ---- gating of self/forge on actually-reflecting ----------------------------
def test_set_reflecting_toggles_flag(tmp_path):
    core = _core(tmp_path, [])
    assert core._reflecting is False
    core.set_reflecting(True)
    assert core._reflecting is True
    core.set_reflecting(False)
    assert core._reflecting is False


def test_empty_line_is_a_noop(tmp_path):
    events = []
    core = _core(tmp_path, events)
    core._announce_conclusion("self", "")               # nothing to say
    assert _spoken(events) == []
    assert list(core._cog_log) == []
