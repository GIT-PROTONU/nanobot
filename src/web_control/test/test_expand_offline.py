"""Offline test (ROS-free) for the `offline` meta skill (`expand-offline.md`): it should mint a
new pure `topic` capability -- one that needs NO language model to run -- so the local-only
fallback pool `run_skill_beat` draws on when the LLM is unreachable (see test_skill_beat_offline)
keeps growing over time. Reuses the workshop's propose->check->rehearse->critique->trial
pipeline, just constrained to one action kind.

    pixi run python -m pytest src/web_control/test
"""
from web_control.cognition import CognitionCore

_SUGGEST_TOPIC = ('{"mode":"new","name":"wag-tail","description":"a little wiggle",'
                  '"trigger":"when happy","action":{"kind":"topic","topic":"/led"},'
                  '"body":"blink the LED twice","rationale":"people smiled at the wiggle"}')
_SUGGEST_SAY = ('{"mode":"new","name":"say-thanks","description":"say thanks",'
               '"trigger":"when helped","action":{"kind":"say"},'
               '"body":"say a warm thank you","rationale":"people helped recently"}')
_KEEP = '{"keep": true, "reason": "safe and useful"}'


class _LLM:
    """Available stub: `complete()` replies suggest-then-critique in sequence."""
    def __init__(self, suggest=_SUGGEST_TOPIC):
        self.last_model, self.smart_model, self.model = "m", "x", "m"
        self.complete_calls = 0
        self._suggest = suggest

    def available(self):
        return True

    def set_self_note(self, _):
        pass

    def complete(self, system, user, **kw):
        self.complete_calls += 1
        return self._suggest if self.complete_calls == 1 else _KEEP


def _core(tmp_path, llm, allow_actions=True, **kw):
    s = lambda n: str(tmp_path / n)
    skdir = tmp_path / "sk"
    skdir.mkdir()
    return CognitionCore(
        llm=llm, tts=None, persona="", persona_name="Nano",
        cog_log_path=s("c.log"), bank_path=s("p.json"), skills_dir=str(skdir),
        skills_enable=True, skills_allow_actions=allow_actions, self_model_path=s("sm.json"),
        workshop_path=s("w.json"), workshop_dir=str(tmp_path / "learned"),
        trait_history_path=s("th.json"), skill_likes_path=s("likes.json"), **kw)


def test_expand_offline_mints_a_topic_skill(tmp_path):
    llm = _LLM()
    core = _core(tmp_path, llm)
    res = core.expand_offline_skills()
    assert res["status"] == "ok"
    assert res["rounds"][0]["status"] == "trialing"
    assert res["rounds"][0]["name"] == "wag-tail"
    sk = core._skills.get("wag-tail")
    assert sk is not None and sk.kind == "topic" and sk.is_action
    assert core._workshop.is_trial("wag-tail")


def test_expand_offline_discards_a_non_topic_suggestion(tmp_path):
    # The model ignored the offline constraint and proposed a narrative skill -> discarded,
    # nothing minted, rather than silently growing the wrong (LLM-dependent) tier.
    llm = _LLM(suggest=_SUGGEST_SAY)
    core = _core(tmp_path, llm)
    res = core.expand_offline_skills()
    assert res["rounds"][0]["status"] == "no-suggestion"
    assert core._skills.get("say-thanks") is None


def test_expand_offline_noop_when_actions_disabled(tmp_path):
    llm = _LLM()
    core = _core(tmp_path, llm, allow_actions=False)
    res = core.expand_offline_skills()
    assert res == {"status": "actions-disabled"}
    assert llm.complete_calls == 0                       # never even asked the model


def test_offline_skill_kind_dispatches_to_expand_offline(tmp_path):
    llm = _LLM()
    core = _core(tmp_path, llm)
    calls = []
    core.expand_offline_skills = lambda *a, **kw: (calls.append(1) or {"status": "ok",
                                                                        "rounds": []})
    (tmp_path / "sk" / "expand-offline.md").write_text(
        "---\nname: expand-offline\ndescription: grow offline skills\n"
        "action: {kind: offline}\n---\n\n# Expand\nMint one.\n", encoding="utf-8")
    core.reload_skills()
    res = core.invoke_skill("expand-offline")
    assert calls == [1]
    assert res["status"] == "ok"
