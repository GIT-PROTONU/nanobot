"""Offline tests for the `docs` skill source — the robot reading its own documentation:

    pixi run python -m pytest src/web_control/test

ROS-free: exercises the pure redact/excerpt helper, no rclpy, no network.
"""
from web_control.cognition import (SELF_DOCS, SKILL_SOURCES, SOURCE_ALIASES,
                                   read_self_docs)


def _write(d, name, text):
    p = d / name
    p.write_text(text, encoding="utf-8")
    return p


def test_docs_source_is_wired():
    assert SKILL_SOURCES["docs"][1] == "_docs_summary"   # row -> provider method
    assert SOURCE_ALIASES["readme"] == "docs"            # aliases resolve to it


def test_reads_whitelisted_files_only(tmp_path):
    _write(tmp_path, "README.md", "I am Nano, a small robot.")
    _write(tmp_path, "CLAUDE.md", "I run ROS 2 on a NanoPi.")
    _write(tmp_path, "secrets.txt", "do not read me")
    out = read_self_docs(str(tmp_path))
    assert "I am Nano" in out and "I run ROS 2" in out
    assert "do not read me" not in out                   # not in the whitelist
    assert "[README.md]" in out and "[CLAUDE.md]" in out


def test_redacts_credential_lines(tmp_path):
    _write(tmp_path, "README.md", "intro\nllm_api_key: sk-secret-value-123\noutro")
    out = read_self_docs(str(tmp_path))
    assert "sk-secret-value-123" not in out
    assert "[redacted]" in out
    assert "intro" in out and "outro" in out             # surrounding text survives


def test_excerpt_is_truncated(tmp_path):
    _write(tmp_path, "README.md", "word " * 500)
    out = read_self_docs(str(tmp_path), docs=(("README.md", 100),))
    assert out.endswith("…")
    assert len(out) < 200                                # ~limit + the "[README.md] " prefix


def test_missing_files_degrade_to_empty(tmp_path):
    assert read_self_docs(str(tmp_path)) == ""           # nothing readable -> empty, not an error
    assert isinstance(SELF_DOCS, tuple) and SELF_DOCS    # whitelist is non-empty
