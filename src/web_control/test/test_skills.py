"""Offline unit tests for the skill library loader (ROS-free):

    pixi run python -m pytest src/web_control/test

Mirrors src/behavior/test: pure logic, no rclpy, no network.
"""
import os
import textwrap

from web_control.skills import (Skill, SkillLibrary, parse_skill_text,
                                 resolve_skills_dir, _slug)


def _write(d, name, text):
    p = os.path.join(d, name)
    with open(p, "w", encoding="utf-8") as f:
        f.write(textwrap.dedent(text))
    return p


def test_slug_normalisation():
    assert _slug("Read LiDAR") == "read-lidar"
    assert _slug("say_hello") == "say-hello"
    assert _slug("  Wiggle!  ") == "wiggle"


def test_parse_narrative_skill():
    sk = parse_skill_text("""\
---
name: say-hello
description: Greet whoever is nearby.
trigger: when someone appears
action:
  kind: say
---
# Say Hello
Say a warm hello.
""")
    assert sk.name == "say-hello"
    assert sk.kind == "say"
    assert sk.is_action is False
    assert sk.enabled is True                 # narrative skills are always enabled
    assert sk.camera is False
    assert "warm hello" in sk.body
    assert sk.info()["topic"] is None


def test_name_defaults_to_filename():
    sk = parse_skill_text("# no frontmatter here\nbody", default_name="look-around")
    assert sk.name == "look-around"
    assert sk.kind == "say"                    # default action


def test_observe_sources_and_look_camera():
    obs = parse_skill_text("---\nname: read-lidar\naction:\n  kind: observe\n"
                           "  sources: [scan]\n---\nbody")
    assert obs.kind == "observe"
    assert obs.sources == ["scan"]
    look = parse_skill_text("---\nname: look-around\naction: {kind: look}\n---\nbody")
    assert look.kind == "look"
    assert look.camera is True


def test_topic_action_is_gated_until_enabled():
    off = parse_skill_text("---\nname: wiggle\naction:\n  kind: topic\n"
                           "  topic: /cmd_vel\n  enabled: false\n---\nbody")
    assert off.is_action is True
    assert off.enabled is False                # opt-in required
    assert off.info()["topic"] == "/cmd_vel"
    on = parse_skill_text("---\nname: blink\naction:\n  kind: topic\n"
                          "  topic: /led\n  enabled: true\n---\nbody")
    assert on.enabled is True


def test_library_load_and_offered(tmp_path):
    d = str(tmp_path)
    _write(d, "say-hi.md", "---\nname: say-hi\naction: {kind: say}\n---\nhi")
    _write(d, "wiggle.md", "---\nname: wiggle\naction:\n  kind: topic\n"
                           "  topic: /cmd_vel\n  enabled: false\n---\nw")
    _write(d, "blink.md", "---\nname: blink\naction:\n  kind: topic\n"
                          "  topic: /led\n  enabled: true\n---\nb")
    _write(d, "README.md", "# docs, must be ignored")
    lib = SkillLibrary(d)
    assert set(lib.skills) == {"say-hi", "wiggle", "blink"}   # README skipped
    # Actions disallowed -> only the narrative skill is offered.
    assert {s.name for s in lib.offered(allow_actions=False)} == {"say-hi"}
    # Actions allowed -> the narrative skill + the *enabled* action skill (not the off one).
    assert {s.name for s in lib.offered(allow_actions=True)} == {"say-hi", "blink"}


def test_choose_from_json_and_text(tmp_path):
    d = str(tmp_path)
    _write(d, "say-hi.md", "---\nname: say-hi\naction: {kind: say}\n---\nhi")
    _write(d, "look-around.md", "---\nname: look-around\naction: {kind: look}\n---\nl")
    lib = SkillLibrary(d)
    assert lib.choose('{"skill": "look-around"}').name == "look-around"
    assert lib.choose('I think {"skill":"say-hi"} fits.').name == "say-hi"
    assert lib.choose("let's do look-around now").name == "look-around"   # loose text
    assert lib.choose('{"skill": ""}') is None                            # do nothing
    assert lib.choose("nothing relevant") is None


def test_bad_file_is_skipped_not_fatal(tmp_path):
    d = str(tmp_path)
    _write(d, "ok.md", "---\nname: ok\naction: {kind: say}\n---\nok")
    _write(d, "broken.md", "---\nname: : : not yaml [unclosed\naction\n---\nx")
    lib = SkillLibrary(d)
    assert "ok" in lib.skills                  # the good one still loads


def test_resolve_skills_dir_prefers_existing(tmp_path):
    real = str(tmp_path)
    # an explicit existing param wins
    assert resolve_skills_dir(param=real) == os.path.abspath(real)
    # a non-existent param falls through to the source-tree default (../skills next to module)
    got = resolve_skills_dir(param=os.path.join(real, "nope"))
    assert got.endswith("skills")
