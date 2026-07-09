"""Offline test (ROS-free): the autonomous skill beat must still be able to DO something when
the LLM is unavailable. `run_skill_beat` picks via `llm.complete()`, which returns None whenever
`llm.available()` is False -- previously that meant the beat always logged "no-pick" and ran
NOTHING, even a pure `topic` (action) skill that needs no model call at all to execute. It should
instead fall back to a local, LLM-free random pick among the runnable action skills.

    pixi run python -m pytest src/web_control/test
"""
import random

from web_control.cognition import CognitionCore


class _OfflineLLM:
    """Unavailable stub — `complete`/`generate` must never be relied on to produce a pick."""
    def __init__(self):
        self.last_model, self.smart_model, self.model = "m", "x", "m"
        self.complete_calls = 0

    def available(self):
        return False

    def set_self_note(self, _):
        pass

    def complete(self, *a, **kw):
        self.complete_calls += 1
        return None                                    # exactly what a down/capped LLM returns

    def generate(self, *a, **kw):
        return None

    def can_call(self, *a, **kw):
        return False


def _say_md(name):
    return ("---\nname: %s\ndescription: %s\naction: {kind: say}\n---\n\n# %s\nSay a line.\n"
            % (name, name, name.title()))


def _topic_md(name):
    return ("---\nname: %s\ndescription: %s\n"
            "action: {kind: topic, topic: /led, enabled: true}\n---\n\n# %s\nBlink.\n"
            % (name, name, name.title()))


def _core(tmp_path, llm, **kw):
    s = lambda n: str(tmp_path / n)
    skdir = tmp_path / "sk"
    skdir.mkdir()
    (skdir / "chatty.md").write_text(_say_md("chatty"), encoding="utf-8")
    (skdir / "blink.md").write_text(_topic_md("blink"), encoding="utf-8")
    return CognitionCore(
        llm=llm, tts=None, persona="", persona_name="Nano",
        cog_log_path=s("c.log"), bank_path=s("p.json"), skills_dir=str(skdir),
        skills_enable=True, skills_allow_actions=True, self_model_path=s("sm.json"),
        workshop_path=s("w.json"), workshop_dir=str(tmp_path / "learned"),
        trait_history_path=s("th.json"), skill_likes_path=s("likes.json"),
        workshop_trial_bias=0.0, skill_like_bias=0.0, **kw)


def test_skill_beat_falls_back_to_local_pick_when_llm_down(tmp_path):
    llm = _OfflineLLM()
    core = _core(tmp_path, llm)
    ran = []
    core._invoke_skill = lambda skill, **kw: ran.append(skill.name)
    random.seed(0)
    core.run_skill_beat()
    assert ran == ["blink"]                             # only the LLM-free action skill can run
    assert llm.complete_calls == 0                       # never wasted a call on a dead model


def test_skill_beat_offline_is_a_noop_with_no_action_skills(tmp_path):
    llm = _OfflineLLM()
    s = lambda n: str(tmp_path / n)
    skdir = tmp_path / "sk"
    skdir.mkdir()
    (skdir / "chatty.md").write_text(_say_md("chatty"), encoding="utf-8")
    core = CognitionCore(
        llm=llm, tts=None, persona="", persona_name="Nano",
        cog_log_path=s("c.log"), bank_path=s("p.json"), skills_dir=str(skdir),
        skills_enable=True, skills_allow_actions=True, self_model_path=s("sm.json"),
        workshop_path=s("w.json"), workshop_dir=str(tmp_path / "learned"),
        trait_history_path=s("th.json"), skill_likes_path=s("likes.json"),
        workshop_trial_bias=0.0, skill_like_bias=0.0)
    ran = []
    core._invoke_skill = lambda skill, **kw: ran.append(skill.name)
    core.run_skill_beat()
    assert ran == []                                    # nothing runnable offline -> silent beat
    assert llm.complete_calls == 0
