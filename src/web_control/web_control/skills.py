"""Skill library — a portable, self-documenting capability catalogue (ROS-free).

Each capability Nano can perform is ONE Markdown file in the skills directory, e.g.

    ---
    name: read-lidar
    description: Report the nearest obstacle from the lidar scan.
    trigger: when asked what's around or how close things are
    action: {kind: observe, sources: [scan]}
    ---
    # Read LiDAR
    Free-text instructions to the brain on how to perform / narrate this skill.

The YAML frontmatter is the machine-readable contract; the Markdown body is the human-
(and LLM-) readable "how", folded into the prompt that steers the spoken line. Drop a new
`.md` in the directory and the robot gains a capability — no code change. This module just
*parses + indexes* the files; the web layer (web_server.py) decides which to run and
executes the action. Keeping it ROS-free means it unit-tests offline, like llm.py /
phrasebank.py:

    pixi run python -m pytest src/web_control/test

Action kinds (the ``action.kind`` field):
  - ``say``     : speak one in-character line steered by the body text (cheap text model)
  - ``observe`` : like say, but appends live context (``sources: [sensors]`` and/or
                  ``[scan]``) so the robot reacts to its own body / the room
  - ``look``    : capture a camera frame + narrate what it sees (vision model)
  - ``topic``   : publish a WHITELISTED ROS message — the GATED "physical" tier. Runs only
                  when the skill sets ``enabled: true`` AND the node's
                  ``skills_allow_actions`` master switch is on. Motion is also clamped
                  reflexively by slam_nav, so this can never be load-bearing/unsafe.
  - ``workshop``: run the skill workshop (the reflection-mode skill-synthesis loop) on
                  demand — mine recent experience and forge ONE new/improved capability.
                  A "meta" capability (it operates on the library itself); never auto-picked
                  on a skill beat (that would mint constantly), only invoked deliberately.
  - ``phrases`` : grow the offline phrase bank on demand — add a few BRAND-NEW in-character
                  lines (LLM) to the most under-filled situation. Like ``workshop`` it's a
                  self-improvement "meta" kind (operates on the robot's own state, not picked
                  autonomously); the same growth runs by itself during reflection mode.

Everything degrades safely: a missing directory, a malformed file, or no PyYAML => that
skill (or the whole library) is simply absent. The brain is a garnish, never load-bearing.
"""
import glob
import json
import os
import re

from .llm import _extract_json

NARRATIVE_KINDS = ("say", "observe", "look")
ACTION_KINDS = ("topic",)
# "Meta" kinds run an internal cognition routine rather than speaking or publishing. They're
# always enabled (not gated like the topic tier) but excluded from autonomous selection.
META_KINDS = ("workshop", "phrases")
KNOWN_KINDS = NARRATIVE_KINDS + ACTION_KINDS + META_KINDS


def _slug(name):
    """Canonical skill id: lowercase, spaces/underscores -> dashes, trimmed."""
    s = re.sub(r"[\s_]+", "-", str(name or "").strip().lower())
    return re.sub(r"[^a-z0-9-]", "", s).strip("-")


def _split_frontmatter(text):
    """Split a Markdown string into (frontmatter_text, body). Frontmatter is the block
    between a leading ``---`` line and the next ``---`` line. Returns ("", text) if absent."""
    if not text:
        return "", ""
    # Tolerate a UTF-8 BOM / leading blank lines before the opening fence.
    t = text.lstrip("﻿")
    m = re.match(r"^\s*---[ \t]*\r?\n(.*?)\r?\n---[ \t]*\r?\n?(.*)$", t, re.DOTALL)
    if not m:
        return "", text
    return m.group(1), m.group(2)


def _parse_frontmatter(fm_text):
    """Parse the frontmatter block to a dict. Uses PyYAML when available (handles nested
    ``action:`` maps + flow syntax); falls back to a tiny flat ``key: value`` parser so a
    simple narrative skill still loads with no PyYAML. Returns {} on anything unparseable."""
    fm_text = fm_text or ""
    if not fm_text.strip():
        return {}
    try:
        import yaml  # almost always present in a ROS env (rclpy depends on it)
        data = yaml.safe_load(fm_text)
        return data if isinstance(data, dict) else {}
    except ImportError:
        pass
    except Exception:
        return {}
    # Minimal fallback: flat "key: value" lines only (no nesting). An inline-flow `action:`
    # is parsed leniently; a block-style action falls back to its kind word if present.
    out = {}
    for line in fm_text.splitlines():
        line = line.strip()
        if not line or line.startswith("#") or ":" not in line:
            continue
        key, _, val = line.partition(":")
        key, val = key.strip(), val.strip()
        if key == "action" and val.startswith("{"):
            try:
                out["action"] = json.loads(val.replace("'", '"'))
            except Exception:
                out["action"] = {"kind": "say"}
        elif val and not val.startswith(("{", "[")):
            out[key] = val.strip().strip("'\"")
    return out


class Skill:
    """One parsed capability file."""

    def __init__(self, name, description="", trigger="", body="", action=None, path=""):
        self.name = _slug(name)
        self.description = str(description or "").strip()
        self.trigger = str(trigger or "").strip()
        self.body = str(body or "").strip()
        self.action = action if isinstance(action, dict) else {"kind": "say"}
        self.path = path

    @property
    def kind(self):
        k = str(self.action.get("kind", "say")).strip().lower()
        return k if k in KNOWN_KINDS else "say"

    @property
    def is_action(self):
        """A 'physical' (topic-publishing) skill, vs a pure narrative one."""
        return self.kind in ACTION_KINDS

    @property
    def is_meta(self):
        """A 'meta' skill that runs an internal cognition routine (e.g. the workshop)."""
        return self.kind in META_KINDS

    @property
    def camera(self):
        return self.kind == "look" or bool(self.action.get("camera"))

    @property
    def sources(self):
        """Which live context an `observe` skill wants appended: a list of 'sensors'/'scan'."""
        src = self.action.get("sources", self.action.get("source"))
        if isinstance(src, str):
            return [src.strip().lower()]
        if isinstance(src, (list, tuple)):
            return [str(s).strip().lower() for s in src]
        return ["sensors"] if self.kind == "observe" else []

    @property
    def enabled(self):
        """Narrative skills are always enabled. A `topic` (action) skill must opt in."""
        return (not self.is_action) or bool(self.action.get("enabled", False))

    def info(self):
        """The dict the web UI + selection catalogue use (no body)."""
        return {"name": self.name, "description": self.description, "trigger": self.trigger,
                "kind": self.kind, "is_action": self.is_action, "camera": self.camera,
                "enabled": self.enabled, "topic": str(self.action.get("topic", "")) or None}


def parse_skill_text(text, default_name="", path=""):
    """Parse a full skill-file string into a Skill (or None if it has no usable name)."""
    fm_text, body = _split_frontmatter(text)
    meta = _parse_frontmatter(fm_text)
    name = meta.get("name") or default_name
    if not _slug(name):
        return None
    action = meta.get("action")
    if isinstance(action, str):                      # `action: observe` shorthand
        action = {"kind": action}
    elif not isinstance(action, dict):
        action = {"kind": "say"}
    return Skill(name=name, description=meta.get("description", ""),
                 trigger=meta.get("trigger", ""), body=body, action=action, path=path)


def parse_skill_file(path):
    with open(path, encoding="utf-8") as f:
        text = f.read()
    return parse_skill_text(text, default_name=os.path.splitext(os.path.basename(path))[0],
                            path=path)


class SkillLibrary:
    """Loads + indexes the ``*.md`` skill files in a directory. Tolerant: a bad file is
    skipped (logged), a missing directory yields an empty (but usable) library."""

    def __init__(self, directory, logger=None, extra_dir=""):
        self.directory = directory or ""
        # An optional second, WRITABLE directory loaded alongside the built-ins — the home for
        # skills the robot mints itself (the workshop's trial/adopted .md files). Kept separate
        # so the committed catalogue stays read-only and the learned ones live in the synced
        # state area. A learned skill with a new name adds a capability; a name collision lets
        # the learned file override the built-in (it loads last).
        self.extra_dir = extra_dir or ""
        self._log = logger or (lambda *_: None)
        self.skills = {}                              # slug -> Skill (sorted by filename)
        self.error = ""
        self.load()

    def load(self):
        self.skills, self.error = {}, ""
        if not self.directory or not os.path.isdir(self.directory):
            self.error = "no skills directory at %r" % (self.directory,)
            self._log("skills: " + self.error)
        for d in (self.directory, self.extra_dir):    # built-ins first, then learned (override)
            if not d or not os.path.isdir(d):
                continue
            for path in sorted(glob.glob(os.path.join(d, "*.md"))):
                if os.path.basename(path).lower() == "readme.md":
                    continue                          # docs, not a capability
                try:
                    sk = parse_skill_file(path)
                except Exception as exc:
                    self._log("skills: failed to parse %s: %s"
                              % (os.path.basename(path), exc))
                    continue
                if sk is not None:
                    self.skills[sk.name] = sk
        self._log("skills: loaded %d from %s%s" % (
            len(self.skills), self.directory,
            (" + " + self.extra_dir) if self.extra_dir else ""))
        return self

    def write_dir(self):
        """Where newly-minted skills are written: the learned dir if set, else the main dir."""
        return self.extra_dir or self.directory

    reload = load

    def add_file(self, path):
        """Parse ONE .md and add/replace it in the index, without re-scanning the directory.
        Lets the workshop make a freshly-written candidate live cheaply (a full reload would
        re-glob + re-parse the whole catalogue). Returns the Skill, or None on a parse failure."""
        try:
            sk = parse_skill_file(path)
        except Exception as exc:
            self._log("skills: failed to parse %s: %s" % (os.path.basename(path), exc))
            return None
        if sk is not None:
            self.skills[sk.name] = sk
        return sk

    def remove(self, name):
        """Drop a skill from the index (no file IO). Returns True if it was present."""
        return self.skills.pop(_slug(name), None) is not None

    def __len__(self):
        return len(self.skills)

    def get(self, name):
        return self.skills.get(_slug(name))

    def values(self):
        return list(self.skills.values())

    def offered(self, allow_actions):
        """The skills the brain may pick autonomously: all narrative ones, plus enabled
        action skills only when the node permits actions. Meta skills (the workshop) are
        excluded — they're deliberate, on-demand only, never an autonomous skill-beat pick."""
        return [s for s in self.skills.values()
                if not s.is_meta and (not s.is_action or (allow_actions and s.enabled))]

    def catalogue(self, allow_actions):
        """The (name, description, trigger) list shown to the model for selection."""
        return [{"name": s.name, "description": s.description, "trigger": s.trigger}
                for s in self.offered(allow_actions)]

    def format_catalogue(self, allow_actions):
        """A compact numbered text block of the offered skills, for the selection prompt."""
        rows = []
        for s in self.offered(allow_actions):
            line = "- %s: %s" % (s.name, s.description or "(no description)")
            if s.trigger:
                line += " [use %s]" % s.trigger
            rows.append(line)
        return "\n".join(rows)

    def as_list(self):
        """Everything, for the web UI panel (includes disabled/action skills)."""
        return [s.info() for s in self.skills.values()]

    def choose(self, reply_text, allow_actions=True):
        """Map a model's selection reply to a Skill. Accepts JSON ``{"skill": name}`` or
        loose text that names a known skill. Returns a Skill, or None for 'do nothing'."""
        if not reply_text:
            return None
        offered = {s.name: s for s in self.offered(allow_actions)}
        # Preferred: a JSON object naming the skill.
        obj = _extract_json(reply_text, keys=("skill",))
        if "skill" in obj:
            return offered.get(_slug(obj.get("skill")))
        # Fallback: the reply just contains a known slug somewhere.
        low = reply_text.lower()
        for name, sk in offered.items():
            if name and name in low:
                return sk
        return None


def resolve_skills_dir(param="", share_dir=None):
    """Pick the skills directory: an explicit param, else $NANOBOT_SKILLS_DIR, else the
    installed package share (``<share>/web_control/skills``), else the source tree next to
    this module (``src/web_control/skills``). Returns the first that EXISTS; if none do,
    returns the most-preferred candidate anyway (so the UI can show where to put files)."""
    cands = []
    if param:
        cands.append(param)
    env = os.environ.get("NANOBOT_SKILLS_DIR")
    if env:
        cands.append(env)
    if share_dir:
        cands.append(os.path.join(share_dir, "skills"))
    cands.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "skills"))
    resolved = [os.path.abspath(os.path.expanduser(c)) for c in cands]
    for c in resolved:
        if os.path.isdir(c):
            return c
    return resolved[0] if resolved else ""
