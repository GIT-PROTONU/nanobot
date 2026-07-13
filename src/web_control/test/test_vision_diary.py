"""Offline unit tests for the visual diary (ROS-free):

    pixi run python -m pytest src/web_control/test

Covers record_vision_snapshot (period gating + cap + persistence) and vision_trend_text
(plain-English scene-drift summary folded into the reflection prompts). Same fake-LLM /
tmp-path pattern as test_trait_history.py.
"""
import json

from web_control.cognition import CognitionCore


class FakeLLM:
    def __init__(self):
        self.last_model = ""
        self.smart_model = "fake-smart"

    def available(self):
        return False

    def set_self_note(self, _note):
        pass


def _core(tmp_path, **kw):
    s = lambda name: str(tmp_path / name)
    return CognitionCore(
        llm=FakeLLM(), tts=None, persona="", persona_name="Nano",
        cog_log_path=s("cognition.log"), bank_path=s("phrases.json"),
        skills_dir="", skills_enable=False, self_model_path=s("self_model.json"),
        workshop_path=s("workshop.json"), workshop_dir=s("skills"),
        trait_history_path=s("trait_history.json"),
        vision_diary_path=s("vision_diary.json"), **kw)


def test_snapshot_is_period_gated(tmp_path):
    core = _core(tmp_path, vision_diary_period=600.0)
    sig = {"luma": 0.4, "motion": 0.02}
    assert core.record_vision_snapshot(sig) is True         # first snapshot always lands
    assert core.record_vision_snapshot(sig) is False        # too soon -> gated
    assert len(core._vision_diary) == 1
    assert core.record_vision_snapshot(sig, force=True) is True
    assert len(core._vision_diary) == 2


def test_snapshot_persists_and_reloads(tmp_path):
    core = _core(tmp_path)
    core.record_vision_snapshot({"luma": 0.42, "edge": 0.1}, force=True)
    on_disk = json.loads((tmp_path / "vision_diary.json").read_text())
    assert on_disk["snapshots"][0]["luma"] == 0.42
    core2 = _core(tmp_path)
    assert len(core2._vision_diary) == 1


def test_snapshot_respects_cap_and_rejects_junk(tmp_path):
    core = _core(tmp_path, vision_diary_max=8)
    for _ in range(20):
        core.record_vision_snapshot({"luma": 0.5}, force=True)
    assert len(core._vision_diary) == 8
    assert core.record_vision_snapshot({"luma": "not a number"}, force=True) is False
    assert core.record_vision_snapshot("nonsense", force=True) is False


def test_trend_text_reports_scene_drift(tmp_path):
    core = _core(tmp_path)
    core.record_vision_snapshot({"luma": 0.6, "motion": 0.3}, force=True)
    core.record_vision_snapshot({"luma": 0.15, "motion": 0.02}, force=True)
    trend = core.vision_trend_text()
    assert "darker" in trend and "60% -> 15%" in trend
    assert "calmer" in trend


def test_trend_text_empty_cases(tmp_path):
    core = _core(tmp_path)
    assert core.vision_trend_text() == ""                   # nothing logged yet
    core.record_vision_snapshot({"luma": 0.5}, force=True)
    assert core.vision_trend_text() == ""                   # one point -> no trajectory
    core.record_vision_snapshot({"luma": 0.52}, force=True)
    assert core.vision_trend_text() == ""                   # below min_delta


def test_disabled_diary_is_inert(tmp_path):
    core = _core(tmp_path, vision_diary_enable=False)
    assert core.record_vision_snapshot({"luma": 0.5}, force=True) is False
    assert core.vision_trend_text() == ""
    assert not (tmp_path / "vision_diary.json").exists()


def test_readout_shape(tmp_path):
    core = _core(tmp_path)
    core.record_vision_snapshot({"luma": 0.5, "warmth": 0.2}, force=True)
    out = core.get_vision_diary()
    assert out["enabled"] is True
    assert out["snapshots"][0]["warmth"] == 0.2
    assert "trend" in out
