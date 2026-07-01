"""Skill workshop — the pure (ROS-free) core of reflection mode's skill-synthesis loop.

During reflection the brain mines its own experience (the decision log) and proposes a
**new or adapted** capability, checks it, rehearses it, and — if it survives — writes it
into the skill library on **trial**. A trial skill is a normal, immediately-usable skill
(see [[skill-library]]); the only difference is that this module tracks how it performs and
decides when it graduates to **permanent** (adopted) or gets rolled back (retired).

This file holds only the deterministic, unit-testable pieces — no LLM, no ROS, no files
beyond the sidecar state JSON:

  * ``render_skill_md(spec)``     : a candidate dict -> a ``*.md`` skill-file string.
  * ``validate_candidate(...)``   : deterministic safety/sanity checks before anything is
                                    written (parse round-trip, kind whitelist, dup/collision).
  * ``WorkshopState``             : the sidecar ledger (``workshop.json``) — per trial skill
                                    counters (runs / reward / errors) + the adopt/retire gate.

The LLM-driven suggest / rehearse / critique steps and the file writes live in
``CognitionCore`` (cognition.py), so they run identically on the robot and the dev harness.
Everything degrades safely: no PyYAML, a bad spec, or no state file => the workshop is simply
inert, never load-bearing.

    pixi run python -m pytest src/web_control/test
"""
import json
import time

from .jsonio import read_json, write_json
from .skills import KNOWN_KINDS, NARRATIVE_KINDS, _slug, parse_skill_text

# Status values a tracked skill moves through.
TRIAL, ADOPTED, RETIRED = "trial", "adopted", "retired"


def render_skill_md(spec):
    """Render a candidate spec dict into a complete skill ``*.md`` string (YAML frontmatter +
    Markdown body). Tolerant of missing keys; the result is meant to be fed straight back
    through ``parse_skill_text`` for validation. Returns "" if there is no usable name."""
    name = _slug(spec.get("name"))
    if not name:
        return ""
    action = spec.get("action")
    if isinstance(action, str):
        action = {"kind": action}
    if not isinstance(action, dict):
        action = {"kind": "say"}
    kind = str(action.get("kind", "say")).strip().lower()
    if kind not in KNOWN_KINDS:
        kind = "say"
    action["kind"] = kind
    # A generated action (topic) skill is ALWAYS born disabled — it can never publish until a
    # human flips skills_allow_actions and enables it. Trials are still auto-eligible to the
    # skill beat for the narrative tiers; the topic tier stays gated regardless.
    if kind not in NARRATIVE_KINDS:
        action["enabled"] = False
    fm = {"name": name}
    if spec.get("description"):
        fm["description"] = str(spec["description"]).strip()
    if spec.get("trigger"):
        fm["trigger"] = str(spec["trigger"]).strip()
    fm["action"] = action
    header = _dump_frontmatter(fm)
    body = str(spec.get("body") or spec.get("description") or "").strip()
    title = spec.get("title") or name.replace("-", " ").title()
    return "---\n%s---\n\n# %s\n%s\n" % (header, title, body)


def _dump_frontmatter(fm):
    """YAML frontmatter text for the dict. Uses PyYAML when present; else a tiny hand-roller
    (flat scalars + an inline-flow ``action:``) that ``skills._parse_frontmatter`` can read."""
    try:
        import yaml
        return yaml.safe_dump(fm, sort_keys=False, default_flow_style=False,
                              allow_unicode=True)
    except Exception:
        pass
    rows = []
    for key, val in fm.items():
        if isinstance(val, dict):
            inline = ", ".join("%s: %s" % (k, json.dumps(v) if isinstance(v, (dict, list))
                                           else v) for k, v in val.items())
            rows.append("%s: {%s}" % (key, inline))
        else:
            rows.append("%s: %s" % (key, val))
    return "\n".join(rows) + "\n"


def validate_candidate(spec, existing_names, allow_actions=False):
    """Deterministic pre-flight on a proposed skill. Returns ``(ok, reason)``. Checks: a usable
    name; mode/collision rules (``adapt`` must target an existing skill, ``new`` must not
    collide); a known action kind; an action-tier skill only when actions are permitted; and a
    full render -> parse round-trip (so a malformed body can't slip through). ``existing_names``
    is the set/iterable of current skill slugs."""
    existing = {_slug(n) for n in (existing_names or [])}
    name = _slug(spec.get("name"))
    if not name:
        return False, "no usable name"
    mode = str(spec.get("mode") or "new").strip().lower()
    target = _slug(spec.get("target"))
    # A trial is ALWAYS a fresh file (even when adapting), so a retire/rollback can delete it
    # without ever touching an existing skill. An `adapt` is a new *variant* of its parent.
    if name in existing:
        return False, "name %r already exists" % (name,)
    if mode == "adapt" and (not target or target not in existing):
        return False, "adapt target %r not found" % (spec.get("target"),)
    action = spec.get("action")
    kind = (action.get("kind") if isinstance(action, dict)
            else action if isinstance(action, str) else "say")
    kind = str(kind or "say").strip().lower()
    if kind not in KNOWN_KINDS:
        return False, "unknown action kind %r" % (kind,)
    if kind not in NARRATIVE_KINDS and not allow_actions:
        return False, "action-tier skill but actions are disabled"
    text = render_skill_md(spec)
    sk = parse_skill_text(text) if text else None
    if sk is None or sk.name != name or sk.kind != kind:
        return False, "render/parse round-trip failed"
    return True, "ok"


class WorkshopState:
    """The sidecar ledger for trial skills (``workshop.json``). Pure bookkeeping — the caller
    owns the actual ``.md`` files. Each record:

        {origin: new|adapt, parent: <slug|"">, created: ts, runs, reward_pos, reward_neg,
         errors, status: trial|adopted|retired, last_run, rationale}

    The adopt/retire decision is ``gate()`` — deterministic, config-driven."""

    def __init__(self, path, logger=None, min_runs=3, retire_errors=2, retire_net_neg=2,
                 adopt_quiet_runs=5, trial_ttl=0.0):
        self.path = path or ""
        self._log = logger or (lambda *_: None)
        self.min_runs = max(1, int(min_runs))
        self.retire_errors = max(1, int(retire_errors))
        self.retire_net_neg = max(1, int(retire_net_neg))
        # An autonomous robot rarely gets an explicit 👍, so a trial that simply runs cleanly
        # a few more times (and draws no 👎) graduates on its own — else trials pile up forever.
        self.adopt_quiet_runs = max(self.min_runs, int(adopt_quiet_runs))
        # Backstop: a trial that lingers this long (s) without earning adoption is rolled back
        # (e.g. a disabled action trial that can never accrue runs). 0 = no TTL.
        self.trial_ttl = max(0.0, float(trial_ttl))
        self.skills = {}
        self.load()

    # ---- persistence --------------------------------------------------------
    def load(self):
        self.skills = {}
        if not self.path:
            return self
        data = read_json(self.path)
        skills = data.get("skills", data) if isinstance(data, dict) else {}
        if isinstance(skills, dict):
            self.skills = {_slug(k): v for k, v in skills.items()
                           if isinstance(v, dict)}
        return self

    def save(self):
        # The ledger is mutated from two threads — the background workshop (reflection) and
        # the executor (a trial skill running / a reward). write_json's unique temp + atomic
        # os.replace keeps concurrent writers from clobbering each other (no lock held over IO).
        if self.path and not write_json(self.path, {"skills": self.skills}):
            self._log("workshop: failed to save %s" % self.path)

    # ---- mutation -----------------------------------------------------------
    def track(self, name, origin="new", parent="", rationale="", path=""):
        """Begin tracking a freshly-minted trial skill (overwrites any prior record)."""
        name = _slug(name)
        if not name:
            return None
        rec = {"origin": str(origin or "new"), "parent": _slug(parent), "rationale":
               str(rationale or "")[:240], "created": time.time(), "runs": 0,
               "reward_pos": 0, "reward_neg": 0, "errors": 0, "status": TRIAL,
               "last_run": 0.0, "path": path}
        self.skills[name] = rec
        self.save()
        return rec

    def record_run(self, name, ok=True):
        """Note one invocation of a tracked skill. Untracked / non-trial names are ignored."""
        rec = self.skills.get(_slug(name))
        if not rec or rec.get("status") != TRIAL:
            return False
        rec["runs"] = int(rec.get("runs", 0)) + 1
        if not ok:
            rec["errors"] = int(rec.get("errors", 0)) + 1
        rec["last_run"] = time.time()
        self.save()
        return True

    def record_reward(self, name, value):
        """Credit a human 👍/👎 to a tracked trial skill (the 'happy user' signal)."""
        rec = self.skills.get(_slug(name))
        if not rec or rec.get("status") != TRIAL:
            return False
        try:
            value = float(value)
        except (TypeError, ValueError):
            return False
        if value > 0:
            rec["reward_pos"] = int(rec.get("reward_pos", 0)) + 1
        elif value < 0:
            rec["reward_neg"] = int(rec.get("reward_neg", 0)) + 1
        self.save()
        return True

    def keep(self, name):
        """Manual override: adopt a trial now (a happy user pressing Keep)."""
        return self._set_status(name, ADOPTED)

    def kill(self, name):
        """Manual override: retire a trial now (Discard/Kill)."""
        return self._set_status(name, RETIRED)

    def _set_status(self, name, status):
        rec = self.skills.get(_slug(name))
        if not rec:
            return False
        rec["status"] = status
        self.save()
        return True

    # ---- the gate -----------------------------------------------------------
    def gate(self, name):
        """Decide a trial's fate from its evidence. Returns 'adopt', 'retire', or None
        (keep trialing). Deterministic; the caller performs the file move/rollback."""
        rec = self.skills.get(_slug(name))
        if not rec or rec.get("status") != TRIAL:
            return None
        runs = int(rec.get("runs", 0))
        pos, neg = int(rec.get("reward_pos", 0)), int(rec.get("reward_neg", 0))
        errors = int(rec.get("errors", 0))
        # Clear failures retire first.
        if errors >= self.retire_errors or (neg - pos) >= self.retire_net_neg:
            return "retire"
        if errors == 0:
            if runs >= self.min_runs and pos > neg:
                return "adopt"                       # happy user (fast path)
            if runs >= self.adopt_quiet_runs and neg == 0:
                return "adopt"                       # clean, uncomplained-about track record
        # Stale backstop: lingered past its TTL without earning adoption -> roll it back.
        if self.trial_ttl and (time.time() - float(rec.get("created", 0.0))) > self.trial_ttl:
            return "retire"
        return None

    def due_trials(self, bar=None):
        """Trial skills that still need exercise to be judged (runs < ``bar``, default the
        quiet-adopt threshold), least-run first — the probation queue the skill beat draws from
        so a freshly forged skill actually accrues runs instead of sitting unused."""
        bar = self.adopt_quiet_runs if bar is None else max(1, int(bar))
        rows = [(int(r.get("runs", 0)), n) for n, r in self.skills.items()
                if r.get("status") == TRIAL and int(r.get("runs", 0)) < bar]
        rows.sort()
        return [n for _, n in rows]

    def gate_all(self):
        """Run the gate over every trial; return ``[(name, 'adopt'|'retire'), ...]``."""
        out = []
        for name in list(self.skills):
            decision = self.gate(name)
            if decision:
                out.append((name, decision))
        return out

    # ---- readers ------------------------------------------------------------
    def get(self, name):
        """The raw record for a tracked skill (or None). Read-only — callers must not mutate."""
        return self.skills.get(_slug(name))

    def status_of(self, name):
        rec = self.skills.get(_slug(name))
        return rec.get("status") if rec else None

    def is_trial(self, name):
        return self.status_of(name) == TRIAL

    def trials(self):
        return [n for n, r in self.skills.items() if r.get("status") == TRIAL]

    def forget(self, name):
        """Drop a record entirely (after a retired file is deleted)."""
        if _slug(name) in self.skills:
            del self.skills[_slug(name)]
            self.save()
            return True
        return False

    def to_public(self):
        """A list for the web UI / decision-log readout (newest first)."""
        rows = []
        for name, r in self.skills.items():
            rows.append({"name": name, "origin": r.get("origin"), "parent": r.get("parent"),
                         "status": r.get("status"), "runs": int(r.get("runs", 0)),
                         "reward_pos": int(r.get("reward_pos", 0)),
                         "reward_neg": int(r.get("reward_neg", 0)),
                         "errors": int(r.get("errors", 0)),
                         "created": r.get("created", 0.0),
                         "rationale": r.get("rationale", "")})
        rows.sort(key=lambda x: x["created"], reverse=True)
        return rows
