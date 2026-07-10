"""Offline unit tests for the phrase bank, focused on incremental growth (ROS-free):

    pixi run python -m pytest src/web_control/test

Pure logic, no rclpy, no network — a fake LLM stands in for OpenRouter.
"""
import json

from web_control.phrasebank import PhraseBank, CATEGORIES, strip_em_dash


class FakeLLM:
    """Minimal LlmClient stand-in: returns a fixed batch of lines, counts calls. Each call
    yields `n` lines tagged with the call index so successive grows produce *new* text."""
    def __init__(self, n=3, available=True):
        self.n = n
        self._available = available
        self.calls = 0

    def available(self):
        return self._available

    def complete(self, system, user, smart=False, json_object=False):
        self.calls += 1
        c = self.calls
        lines = [{"say": f"fresh line {c}-{i}", "mood": "happy"} for i in range(self.n)]
        return json.dumps({"lines": lines})


class EmDashLLM:
    """Stands in for a model that reaches for '—' by default, to prove the bank strips it."""
    def available(self):
        return True

    def complete(self, system, user, smart=False, json_object=False):
        return json.dumps({"lines": [
            {"say": "Sensors nominal — all good.", "mood": "neutral"},
            {"say": "Tilt rising — hold steady.", "mood": "stress"},
        ]})


def _bank(tmp_path):
    return PhraseBank(path=str(tmp_path / "phrases.json"))


def test_strip_em_dash():
    assert strip_em_dash("Sensors nominal — all good.") == "Sensors nominal, all good."
    assert strip_em_dash("one — two — three") == "one, two, three"
    assert "—" not in strip_em_dash("trailing dash—")
    assert strip_em_dash("no dash here.") == "no dash here."


def test_grow_never_admits_em_dash(tmp_path):
    bank = _bank(tmp_path)
    bank._data["categories"] = {"idle": []}
    cat, added = bank.grow(EmDashLLM(), persona="", traits={}, max_per_category=24,
                           categories=["idle"])
    assert added == 2
    says = [e["say"] for e in bank._data["categories"][cat]]
    assert says and all("—" not in s for s in says)


def test_grow_appends_without_replacing(tmp_path):
    bank = _bank(tmp_path)
    bank._data["categories"] = {"idle": [{"say": "old line", "mood": "neutral"}]}
    res = bank.grow(FakeLLM(n=2), persona="", traits={}, name="Nano", max_per_category=24,
                    categories=["idle"])
    assert res is not None
    cat, added = res
    assert added == 2
    says = [e["say"] for e in bank._data["categories"][cat]]
    assert "old line" in says                       # the pre-existing line is preserved
    assert len([s for s in says if s.startswith("fresh")]) == 2


def test_grow_picks_most_underfilled(tmp_path):
    bank = _bank(tmp_path)
    # 'idle' is full to the cap; another category is empty -> growth must target the empty one.
    bank._data["categories"] = {"idle": [{"say": f"x{i}", "mood": "neutral"} for i in range(24)]}
    cat, added = bank.grow(FakeLLM(n=3), persona="", traits={}, max_per_category=24)
    assert cat != "idle" and added == 3


def test_grow_dedupes(tmp_path):
    bank = _bank(tmp_path)
    # Pre-seed with a line the fake LLM will also produce (case/punct-insensitive match).
    bank._data["categories"] = {"idle": [{"say": "Fresh line 1-0.", "mood": "happy"}]}
    cat, added = bank.grow(FakeLLM(n=2), persona="", traits={}, max_per_category=24,
                           categories=["idle"])
    assert added == 1                               # only the second of the two is novel


def test_grow_respects_cap(tmp_path):
    bank = _bank(tmp_path)
    bank._data["categories"] = {c: [] for c in CATEGORIES}
    bank._data["categories"]["idle"] = [{"say": f"x{i}", "mood": "neutral"} for i in range(23)]
    # idle is the most-filled, so growth won't pick it; force-target via a single-category list.
    cat, added = bank.grow(FakeLLM(n=5), persona="", traits={}, max_per_category=24,
                           categories=["idle"])
    assert cat == "idle" and added == 1             # capped at 24 (had 23)
    assert len(bank._data["categories"]["idle"]) == 24


def test_grow_all_full_is_noop(tmp_path):
    bank = _bank(tmp_path)
    bank._data["categories"] = {c: [{"say": f"x{i}", "mood": "neutral"} for i in range(24)]
                                for c in CATEGORIES}
    llm = FakeLLM()
    assert bank.grow(llm, persona="", traits={}, max_per_category=24) is None
    assert llm.calls == 0                           # no LLM call when nothing to fill


def test_grow_unavailable_llm_is_noop(tmp_path):
    bank = _bank(tmp_path)
    assert bank.grow(FakeLLM(available=False), persona="", traits={}) is None


def test_maybe_grow_period_gate(tmp_path):
    import time
    from web_control.phrasebank import signature
    bank = _bank(tmp_path)
    bank._data["categories"] = {"idle": [{"say": "old", "mood": "neutral"}]}
    bank._data["signature"] = signature("", {})      # stable soul -> needs_regen() False
    bank._data["grown_at"] = int(time.time())        # just grew -> period gate should block
    assert bank.maybe_grow(FakeLLM(), "", {}, period=1800.0, background=False) is False
    bank._data["grown_at"] = int(time.time()) - 3600  # long ago -> allowed
    assert bank.maybe_grow(FakeLLM(), "", {}, period=1800.0, background=False) is True


def test_maybe_grow_blocks_on_drift(tmp_path):
    bank = _bank(tmp_path)
    from web_control.phrasebank import signature
    bank._data["categories"] = {"idle": [{"say": "old", "mood": "neutral"}]}
    bank._data["signature"] = signature("persona A", {"curiosity": 0.1})
    bank._data["grown_at"] = 0                        # period elapsed, but soul drifted
    # Very different soul -> needs_regen() True -> growth must defer to a full regen.
    assert bank.maybe_grow(FakeLLM(), "persona B", {"curiosity": 0.9},
                           period=1800.0, background=False) is False
