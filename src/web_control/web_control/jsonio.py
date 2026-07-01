"""Tiny shared helpers for web_control's persisted JSON state.

Every state file here (`tts.json`, `llm.json`, `phrases.json`, `workshop.json`,
`trait_history.json`, `self_model.json`, the decision log, …) is small and
hand-editable, so we keep it as indented UTF-8 text — the goal is *one* atomic,
concurrency-safe writer, not a faster parser (a few KB of JSON parses in
microseconds; a binary format would only cost us the readability we rely on).

Standard solution, no new dependency: write to a unique temp file in the target
directory, then `os.replace` (an atomic rename on the same filesystem) so a reader
— or a crash — never sees a half-written file, and two writer threads can't clobber
each other. Mirrors ``behavior.brain.{load_json,save_json}``; kept local because
``web_control`` and ``behavior`` are separate ROS packages that can't import each
other.
"""
import json
import os
import tempfile


def read_json(path, default=None):
    """Best-effort parse of a JSON file. Returns ``default`` on any problem (missing,
    unreadable, malformed) — never raises."""
    try:
        with open(path, encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return default


def write_json(path, obj, indent=2):
    """Atomically persist ``obj`` as indented JSON (unique temp + ``os.replace``).
    Concurrency-safe and best-effort: returns True on success, False on any failure
    (never raises). Leaves no stray temp file behind on error."""
    try:
        d = os.path.dirname(path) or "."
        os.makedirs(d, exist_ok=True)
        fd, tmp = tempfile.mkstemp(dir=d, prefix=os.path.basename(path) + ".", suffix=".tmp")
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as f:
                json.dump(obj, f, indent=indent, ensure_ascii=False)
            os.replace(tmp, path)
        except Exception:
            try:
                os.remove(tmp)
            except OSError:
                pass
            raise
        return True
    except Exception:
        return False


def read_jsonl_tail(path, n):
    """Parse the last ``n`` JSON-lines of an append-only log into a list (oldest first).
    Best-effort: skips blank/unparseable lines, returns [] if the file is absent."""
    try:
        with open(path, encoding="utf-8") as f:
            lines = f.readlines()[-n:]
    except Exception:
        return []
    out = []
    for ln in lines:
        ln = ln.strip()
        if ln:
            try:
                out.append(json.loads(ln))
            except Exception:
                pass
    return out
