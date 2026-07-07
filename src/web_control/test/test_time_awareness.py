"""Offline tests for time awareness (ROS-free):

  - the pure helpers: `daypart`, `in_quiet_hours` (wrap-aware), `time_context`;
  - quiet hours mute AUTONOMOUS speech (beats, skill beats, boot greeting, offline
    line, reflection bookends) and log `quiet-hours` — while user-initiated speech
    (llm_say path etc.) is untouched;
  - the defaults (negative bounds) leave everything exactly as before.

    pixi run python -m pytest src/web_control/test
"""
from web_control.cognition import (CognitionCore, daypart, in_quiet_hours,
                                   time_context)


class _LLM:
    """Inert LlmClient stand-in — forces the offline path (no network)."""
    def __init__(self):
        self.last_model, self.model, self.smart_model = "", "x", "x"

    def available(self):
        return False

    def can_call(self, image=False):
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


ALL_DAY = {"quiet_start": 0.0, "quiet_end": 24.0}       # quiet_now() always True


def _spoken(events):
    return [t for k, t in events if k == "say"]


def _statuses(core, trigger):
    return [e["status"] for e in core._cog_log if e.get("trigger") == trigger]


# ---- pure helpers --------------------------------------------------------------
def test_daypart_covers_the_clock():
    assert daypart(6) == "early morning"
    assert daypart(9) == "morning"
    assert daypart(13) == "afternoon"
    assert daypart(19) == "evening"
    assert daypart(23) == "night" and daypart(3) == "night"


def test_in_quiet_hours_wraps_midnight():
    assert in_quiet_hours(22, 8, hour=23)
    assert in_quiet_hours(22, 8, hour=3)
    assert not in_quiet_hours(22, 8, hour=12)
    assert in_quiet_hours(1, 5, hour=3)                 # non-wrapping window
    assert not in_quiet_hours(1, 5, hour=6)


def test_in_quiet_hours_disabled_forms():
    assert not in_quiet_hours(-1, 8, hour=3)            # negative bound = off
    assert not in_quiet_hours(22, -1, hour=3)
    assert not in_quiet_hours(9, 9, hour=9)             # equal bounds = off
    assert not in_quiet_hours("junk", 8, hour=3)        # unparseable = off


def test_time_context_mentions_the_daypart():
    line = time_context()
    assert line.startswith("It is ") and line.endswith(".")
    assert any(p in line for p in
               ("early morning", "morning", "afternoon", "evening", "night"))


# ---- quiet-hours gating ---------------------------------------------------------
def test_beat_is_muted_and_logged_in_quiet_hours(tmp_path):
    events = []
    core = _core(tmp_path, events, **ALL_DAY)
    core.run_beat("beat:musing", "musing", "say something", camera=False)
    assert not _spoken(events)
    assert _statuses(core, "beat:musing") == ["quiet-hours"]


def test_skill_beat_is_muted_in_quiet_hours(tmp_path):
    events = []
    core = _core(tmp_path, events, **ALL_DAY)
    core.run_skill_beat("acting")
    assert not _spoken(events)
    assert _statuses(core, "beat:skill") == ["quiet-hours"]


def test_greeting_and_offline_muted_but_face_still_shows(tmp_path):
    events = []
    core = _core(tmp_path, events, **ALL_DAY)
    assert core.speak_lifecycle("greeting") == ""
    assert core.speak_lifecycle("offline", face="sleepy") == ""
    assert not _spoken(events)
    assert ("face", "sleepy") in events                 # the face still tells the story
    assert _statuses(core, "life:greeting") == ["quiet-hours"]


def test_farewell_still_speaks_in_quiet_hours(tmp_path):
    """Farewell/restarting come from a user clicking a power button — never muted."""
    events = []
    core = _core(tmp_path, events, **ALL_DAY)
    core.speak_lifecycle("farewell")
    assert _spoken(events)


def test_reflect_bookends_muted_in_quiet_hours(tmp_path):
    events = []
    core = _core(tmp_path, events, **ALL_DAY)
    core.announce_reflect(True)
    assert not _spoken(events)
    assert _statuses(core, "reflect:enter") == ["quiet-hours"]


def test_defaults_leave_everything_unmuted(tmp_path):
    """With the default (disabled) window nothing is gated — pre-change behaviour."""
    events = []
    core = _core(tmp_path, events)                      # no quiet_* kwargs
    assert not core.quiet_now()
    core.speak_lifecycle("greeting")
    assert _spoken(events)                              # greeting speaks as before
    core.run_beat("beat:musing", "musing", "say something", camera=False)
    assert "quiet-hours" not in _statuses(core, "beat:musing")
