"""Offline tests for compound "shape:emotion" OLED faces (ROS-free):

The behaviour layer publishes a beat's action eye-SHAPE ("looking"/"focused"); the LLM picks an
EMOTION. Instead of the emotion replacing the action face, cognition composes them into a single
"shape:emotion" string (e.g. "looking:happy") so the robot keeps its scanning/intent eyes AND
shows how it feels. oled_display / the web mirror render the shape + an accent overlay on top.

    pixi run python -m pytest src/web_control/test
"""
from web_control.cognition import CognitionCore, compose_face


class _LLM:
    def __init__(self):
        self.last_model, self.smart_model = "", "x"

    def available(self):
        return False


class _TTS:
    def available(self):
        return True

    def say(self, _text):
        pass


def _core(tmp_path, faces):
    s = lambda n: str(tmp_path / n)
    return CognitionCore(
        llm=_LLM(), tts=_TTS(), persona="", persona_name="Nano",
        face=lambda m: faces.append(m),
        cog_log_path=s("c.log"), bank_path=s("p.json"), skills_dir="", skills_enable=False,
        self_model_path=s("sm.json"), workshop_path=s("w.json"), workshop_dir=s("sk"),
        trait_history_path=s("th.json"))


# ---- compose_face (pure) -----------------------------------------------------
def test_action_shape_carries_emotion_accent():
    assert compose_face("looking", "happy") == "looking:happy"
    assert compose_face("focused", "angry") == "focused:angry"


def test_round_bases_keep_emotion_alone():
    # happy/neutral aren't distinctive shapes, so the emotion is the whole face (legacy).
    assert compose_face("happy", "angry") == "angry"
    assert compose_face("neutral", "happy") == "happy"
    assert compose_face("", "happy") == "happy"


def test_redundant_or_empty_accent_collapses():
    assert compose_face("looking", "looking") == "looking"   # accent == shape -> no compound
    assert compose_face("looking", "neutral") == "neutral"   # neutral isn't an accent
    assert compose_face("looking", "") == ""


def test_case_and_whitespace_normalised():
    assert compose_face(" Looking ", " Happy ") == "looking:happy"


# ---- express() emits the compound string -------------------------------------
def test_express_composes_on_action_beat(tmp_path):
    faces = []
    core = _core(tmp_path, faces)
    core.express("happy", "hi", base_face="looking")
    assert faces == ["looking:happy"]


def test_express_without_base_is_plain_emotion(tmp_path):
    faces = []
    core = _core(tmp_path, faces)
    core.express("happy", "hi")                              # on-demand chat/observe: no action shape
    assert faces == ["happy"]


def test_express_neutral_shows_nothing(tmp_path):
    faces = []
    core = _core(tmp_path, faces)
    core.express("neutral", "hi", base_face="looking")       # neutral never paints the panel
    assert faces == []
