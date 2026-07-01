"""Offline test for skill LIKES (ROS-free): a human can 👍 a skill (repeatedly) to make the
autonomous skill beat favour it. The like count is a per-skill weight in a like-weighted lottery
(`_liked_skill_pick`); `run_skill_beat` short-circuits the model pick to it with probability
`skill_like_bias`. Liking is persisted and surfaced in `get_skills()`.

    pixi run python -m pytest src/web_control/test
"""
import random

from web_control.cognition import CognitionCore


class _LLM:
    """Available stub — records whether the contextual picker (`complete`) was consulted."""
    def __init__(self):
        self.last_model, self.smart_model, self.model = "m", "x", "m"
        self.complete_calls = 0

    def available(self):
        return True

    def set_self_note(self, _):
        pass

    def complete(self, *a, **kw):
        self.complete_calls += 1
        return '{"skill": ""}'                        # "do nothing" if ever consulted


def _md(name):
    return ("---\nname: %s\ndescription: %s\naction: {kind: say}\n---\n\n# %s\nSay a line.\n"
            % (name, name, name.title()))


def _core(tmp_path, llm, **kw):
    s = lambda n: str(tmp_path / n)
    skdir = tmp_path / "sk"
    skdir.mkdir()
    for n in ("alpha", "beta"):
        (skdir / (n + ".md")).write_text(_md(n), encoding="utf-8")
    # workshop_trial_bias=0 so trial probation never competes with the likes path under test.
    return CognitionCore(
        llm=llm, tts=None, persona="", persona_name="Nano",
        cog_log_path=s("c.log"), bank_path=s("p.json"), skills_dir=str(skdir),
        skills_enable=True, self_model_path=s("sm.json"), workshop_path=s("w.json"),
        workshop_dir=str(tmp_path / "learned"), trait_history_path=s("th.json"),
        skill_likes_path=s("likes.json"), workshop_trial_bias=0.0, **kw)


def test_like_increments_and_persists(tmp_path):
    core = _core(tmp_path, _LLM())
    assert core.like_skill("alpha")["likes"] == 1
    assert core.like_skill("alpha")["likes"] == 2     # repeatable -> stronger preference
    assert core.get_skill_likes() == {"alpha": 2}
    # get_skills() surfaces the count for the UI.
    likes = {s["name"]: s["likes"] for s in core.get_skills()["skills"]}
    assert likes == {"alpha": 2, "beta": 0}
    # A fresh core reads the persisted ledger back.
    core2 = CognitionCore(
        llm=_LLM(), tts=None, persona="", persona_name="Nano",
        cog_log_path=str(tmp_path / "c.log"), bank_path=str(tmp_path / "p.json"),
        skills_dir=str(tmp_path / "sk"), skills_enable=True,
        self_model_path=str(tmp_path / "sm.json"), workshop_path=str(tmp_path / "w.json"),
        workshop_dir=str(tmp_path / "learned"), trait_history_path=str(tmp_path / "th.json"),
        skill_likes_path=str(tmp_path / "likes.json"))
    assert core2.get_skill_likes() == {"alpha": 2}


def test_unlike_floors_at_zero(tmp_path):
    core = _core(tmp_path, _LLM())
    core.like_skill("alpha")
    assert core.like_skill("alpha", -1)["likes"] == 0
    assert core.like_skill("alpha", -1)["likes"] == 0  # never goes negative
    assert core.get_skill_likes() == {}                # zeroed entries are dropped


def test_like_unknown_skill_is_an_error(tmp_path):
    core = _core(tmp_path, _LLM())
    assert "error" in core.like_skill("nope")
    assert core.get_skill_likes() == {}


def test_liked_pick_is_none_when_nothing_liked(tmp_path):
    core = _core(tmp_path, _LLM())
    offered = core._skills.offered(False)
    assert core._liked_skill_pick(offered) is None     # -> fall through to the model pick


def test_liked_pick_favours_the_liked_skill(tmp_path):
    core = _core(tmp_path, _LLM())
    core.like_skill("alpha", 99)                        # heavily weighted
    offered = core._skills.offered(False)
    random.seed(1)
    picks = [core._liked_skill_pick(offered).name for _ in range(200)]
    assert picks.count("alpha") > picks.count("beta")   # weight = 1 + likes dominates


def test_skill_beat_short_circuits_to_a_liked_skill(tmp_path):
    llm = _LLM()
    core = _core(tmp_path, llm, skill_like_bias=1.0)    # always take the likes path
    core.like_skill("beta", 5)
    ran = []
    core._invoke_skill = lambda skill, **kw: ran.append(skill.name)
    random.seed(0)
    core.run_skill_beat()
    assert ran and ran[0] in ("alpha", "beta")          # a liked-lottery pick was performed...
    assert llm.complete_calls == 0                      # ...without consulting the model picker


def test_skill_beat_uses_llm_picker_with_no_likes(tmp_path):
    llm = _LLM()
    core = _core(tmp_path, llm, skill_like_bias=1.0)    # bias high, but nothing is liked
    ran = []
    core._invoke_skill = lambda skill, **kw: ran.append(skill.name)
    core.run_skill_beat()
    assert ran == []                                    # no liked short-circuit
    assert llm.complete_calls == 1                      # the model was asked to pick instead
