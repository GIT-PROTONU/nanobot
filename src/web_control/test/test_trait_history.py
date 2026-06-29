"""Offline unit tests for the trait-trajectory self-knowledge log (ROS-free):

    pixi run python -m pytest src/web_control/test

Covers record_trait_snapshot (period gating + cap + persistence) and trait_trend_text
(per-trait drift summary over the trailing window). A fake LLM stands in for OpenRouter;
all state paths are redirected to a tmp dir so the test never touches ~/.local/state.
"""
import json

from web_control.cognition import CognitionCore


class FakeLLM:
    """Minimal LlmClient stand-in: never available (so reflect/consolidate no-op) but exposes
    the few attributes/methods CognitionCore touches at construction."""
    def __init__(self):
        self.last_model = ""
        self.smart_model = "fake-smart"

    def available(self):
        return False

    def set_self_note(self, _note):
        pass


def _core(tmp_path, **kw):
    """A CognitionCore with every state path under tmp and the LLM/skills inert."""
    s = lambda name: str(tmp_path / name)
    return CognitionCore(
        llm=FakeLLM(), tts=None, persona="", persona_name="Nano",
        cog_log_path=s("cognition.log"), bank_path=s("phrases.json"),
        skills_dir="", skills_enable=False, self_model_path=s("self_model.json"),
        workshop_path=s("workshop.json"), workshop_dir=s("skills"),
        trait_history_path=s("trait_history.json"), **kw)


def test_snapshot_is_period_gated(tmp_path):
    core = _core(tmp_path, trait_history_period=3600.0)
    assert core.record_trait_snapshot() is True            # first snapshot always lands
    assert core.record_trait_snapshot() is False           # too soon -> gated
    assert len(core._trait_hist) == 1
    assert core.record_trait_snapshot(force=True) is True   # force bypasses the gate
    assert len(core._trait_hist) == 2


def test_snapshot_persists_and_reloads(tmp_path):
    core = _core(tmp_path)
    core.record_trait_snapshot(force=True)
    on_disk = json.loads((tmp_path / "trait_history.json").read_text())
    assert on_disk["snapshots"][0]["traits"]["caution"] == 0.6   # the seeded default
    # A fresh core over the same path reloads the history.
    core2 = _core(tmp_path)
    assert len(core2._trait_hist) == 1


def test_snapshot_respects_cap(tmp_path):
    core = _core(tmp_path, trait_history_max=8)
    for _ in range(20):
        core.record_trait_snapshot(force=True)
    assert len(core._trait_hist) == 8                      # ring trimmed to the cap


def test_trend_text_reports_drift(tmp_path):
    core = _core(tmp_path)
    core.record_trait_snapshot(force=True)                 # baseline (curiosity 0.50)
    core.update_traits({"curiosity": 0.80})                # the robot has grown more curious
    core.record_trait_snapshot(force=True)
    trend = core.trait_trend_text()
    assert "curiosity 0.50 -> 0.80 (rising)" in trend
    assert "caution" not in trend                          # unchanged traits are omitted


def test_trend_text_empty_without_history(tmp_path):
    core = _core(tmp_path)
    assert core.trait_trend_text() == ""                   # nothing logged yet
    core.record_trait_snapshot(force=True)
    assert core.trait_trend_text() == ""                   # one point -> no trajectory


def test_trend_ignores_tiny_moves(tmp_path):
    core = _core(tmp_path)
    core.record_trait_snapshot(force=True)
    core.update_traits({"curiosity": 0.51})                # below the min_delta threshold
    core.record_trait_snapshot(force=True)
    assert core.trait_trend_text() == ""


def test_disabled_history_is_inert(tmp_path):
    core = _core(tmp_path, trait_history_enable=False)
    assert core.record_trait_snapshot(force=True) is False
    assert core.trait_trend_text() == ""
    assert not (tmp_path / "trait_history.json").exists()
