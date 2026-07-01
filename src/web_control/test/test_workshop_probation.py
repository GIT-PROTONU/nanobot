"""Offline test for closing the trial loop (ROS-free): the skill beat puts a freshly forged
trial on PROBATION — it runs the trial directly (so it accrues the runs the gate needs) instead
of asking the model to pick, which rarely lands on a brand-new skill. Paired with the quiet-adopt
/ stale-retire gate changes in test_skillsmith.py, this is what lets a forged skill graduate or
get culled instead of sitting at runs=0 forever.

    pixi run python -m pytest src/web_control/test
"""
from web_control.cognition import CognitionCore


class _LLM:
    """Available stub — records whether the picker (`complete`) was consulted."""
    def __init__(self):
        self.last_model, self.smart_model, self.model = "m", "x", "m"
        self.complete_calls = 0

    def available(self):
        return True

    def set_self_note(self, _):
        pass

    def complete(self, *a, **kw):
        self.complete_calls += 1
        return '{"skill": ""}'                       # "do nothing" if ever consulted


_TRIAL_MD = ("---\nname: quip\ndescription: a tiny quip\naction: {kind: say}\n---\n\n"
             "# Quip\nSay a tiny quip.\n")


def _core(tmp_path, llm, **kw):
    s = lambda n: str(tmp_path / n)
    skdir = tmp_path / "sk"
    skdir.mkdir()
    (skdir / "quip.md").write_text(_TRIAL_MD, encoding="utf-8")
    core = CognitionCore(
        llm=llm, tts=None, persona="", persona_name="Nano",
        cog_log_path=s("c.log"), bank_path=s("p.json"), skills_dir="", skills_enable=True,
        self_model_path=s("sm.json"), workshop_path=s("w.json"), workshop_dir=str(skdir),
        trait_history_path=s("th.json"), **kw)
    core._workshop.track("quip", origin="new", path=str(skdir / "quip.md"))  # mark it a trial
    return core


def test_due_trial_is_the_forged_one(tmp_path):
    core = _core(tmp_path, _LLM())
    sk = core._due_trial_skill()
    assert sk is not None and sk.name == "quip"      # offered + under the run bar -> due


def test_skill_beat_runs_the_trial_under_probation(tmp_path):
    llm = _LLM()
    core = _core(tmp_path, llm, workshop_trial_bias=1.0)  # always probation
    ran = []
    core._invoke_skill = lambda skill, **kw: ran.append(skill.name)
    core.run_skill_beat()
    assert ran == ["quip"]                            # the trial was exercised...
    assert llm.complete_calls == 0                    # ...without consulting the LLM picker


def test_zero_bias_uses_the_llm_picker(tmp_path):
    llm = _LLM()
    core = _core(tmp_path, llm, workshop_trial_bias=0.0)  # never probation
    ran = []
    core._invoke_skill = lambda skill, **kw: ran.append(skill.name)
    core.run_skill_beat()
    assert ran == []                                  # no probation run
    assert llm.complete_calls == 1                    # the model was asked to pick instead


def test_adopted_trial_is_no_longer_due(tmp_path):
    core = _core(tmp_path, _LLM())
    core._workshop.keep("quip")                       # graduate it
    assert core._due_trial_skill() is None            # adopted -> off probation
