#!/usr/bin/env python3
"""Pre-generate (or refresh) Nano's phrase bank — the cached, in-character one-liners it
speaks when reacting to its own body (the frequent "musing" beats). ROS-free: runs on the
robot or a dev PC, using the same `web_control.phrasebank` + `web_control.llm` the stack
uses, so the file it writes is exactly what the robot reads at runtime.

Why: a live LLM call every idle cycle costs latency + money + needs internet. The bank is
generated once (here), then runtime just classifies the sensors, picks a line, and fills in
the live values ({temp}/{cpu}/{tilt}/…). Re-run this when you change the persona/traits, or
just let the stack auto-regenerate when the personality drifts too far.

    set OPENROUTER_API_KEY first, then:
    python scripts/pregenerate_phrases.py            # regenerate from robot.yaml + personality.json
    python scripts/pregenerate_phrases.py --show     # just print the current bank, don't generate
    python scripts/pregenerate_phrases.py --if-needed # regenerate ONLY if missing/drifted (else no-op)
    python scripts/pregenerate_phrases.py --per-category 8

--if-needed uses the SAME PhraseBank.needs_regen() the runtime uses (empty bank, persona
change, or trait drift past phrasebank_drift), and degrades to a non-fatal warning (exit 0)
if it can't build — so a launcher can call it as a pre-step without ever blocking startup.
"""
import argparse
import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.normpath(os.path.join(_HERE, ".."))
sys.path.insert(0, os.path.join(_ROOT, "src", "web_control"))

from nanobot_brain.cognition import LlmClient                       # noqa: E402
from nanobot_brain.cognition import PhraseBank, CATEGORIES   # noqa: E402

ROBOT_YAML = os.path.join(_ROOT, "src", "robot_bringup", "config", "robot.yaml")
# Dev tooling keeps its state in the project-local devstate/ folder (same as dev_webui.py),
# so the soul + phrase bank are visible/editable in the repo. A robot.yaml *_path override wins.
DEV_STATE_DIR = os.path.join(_ROOT, "devstate")


def _cfg(section):
    try:
        import yaml
        with open(ROBOT_YAML, encoding="utf-8") as f:
            return yaml.safe_load(f)[section]["ros__parameters"]
    except Exception as exc:
        print(f"(couldn't read robot.yaml [{section}]: {exc} — using defaults)", file=sys.stderr)
        return {}


def _personality():
    base = {"name": "Nano", "persona": "", "traits": {}}
    try:
        with open(os.path.join(DEV_STATE_DIR, "personality.json"), encoding="utf-8") as f:
            saved = json.load(f)
        for k in base:
            if k in saved:
                base[k] = saved[k]
    except Exception:
        pass
    return base


def main():
    ap = argparse.ArgumentParser(description="Pre-generate Nano's spoken-line phrase bank.")
    ap.add_argument("--show", action="store_true", help="print the current bank and exit")
    ap.add_argument("--if-needed", action="store_true",
                    help="regenerate only if the bank is missing/drifted; else no-op (exit 0)")
    ap.add_argument("--per-category", type=int, default=None, help="lines per situation")
    args = ap.parse_args()

    cfg = _cfg("web_control")
    pers = _personality()
    persona = pers.get("persona") or cfg.get("llm_persona", "")
    bank = PhraseBank(path=(cfg.get("phrasebank_path") or os.path.join(DEV_STATE_DIR,
                                                                       "phrases.json")),
                      logger=lambda m: print(m, file=sys.stderr))

    if args.show:
        print(json.dumps(bank.stats(), indent=2))
        return

    # --if-needed is a launcher pre-step: skip if the bank is already current, and never abort
    # startup on a missing key / failed generation (warn + exit 0 — runtime can still regen).
    fatal = not args.if_needed
    if args.if_needed:
        if not bool(cfg.get("phrasebank_enable", True)):
            print("Phrase bank disabled (phrasebank_enable=false) — skipping.", file=sys.stderr)
            return
        threshold = float(cfg.get("phrasebank_drift", 0.6))
        if not bank.needs_regen(persona, pers.get("traits"), threshold):
            print(f"Phrase bank is current ({bank.stats()['total']} lines at {bank.path}) "
                  "— no rebuild needed.", file=sys.stderr)
            return
        print("Phrase bank missing or drifted — rebuilding…", file=sys.stderr)

    llm = LlmClient(enabled=True, api_key="", model=cfg.get("llm_model", ""),
                    persona=persona, smart_model=cfg.get("llm_smart_model", ""),
                    free_model=cfg.get("llm_free_model", ""),
                    free_smart_model=cfg.get("llm_free_smart_model", ""),
                    logger=lambda m: print(f"[llm] {m}", file=sys.stderr))
    if not llm.available():
        print("! No OPENROUTER_API_KEY (or llm_api_key) — cannot generate.", file=sys.stderr)
        sys.exit(1 if fatal else 0)

    per_cat = args.per_category or int(cfg.get("phrasebank_per_category", 6))
    print(f"Generating {per_cat} lines for each of {len(CATEGORIES)} situations as "
          f"'{pers.get('name', 'Nano')}' (model {llm.model})…", file=sys.stderr)
    ok = bank.generate(llm, persona, pers.get("traits"), name=pers.get("name", "Nano"),
                       per_category=per_cat)
    if not ok:
        print("! Generation failed (no lines produced).", file=sys.stderr)
        sys.exit(1 if fatal else 0)
    st = bank.stats()
    print(f"\nWrote {st['total']} lines to {bank.path}", file=sys.stderr)
    print(json.dumps(st["counts"], indent=2))


if __name__ == "__main__":
    main()
