#!/usr/bin/env python3
"""Personality creator — a SHORT questionnaire that the smart model expands into Nano's
first-version personality (persona + traits + behaviour registry), written to
`personality.json` (the seed the reflex chart / evolution loop will load).

ROS-free, so you can run it on a dev PC. It asks ~6 quick questions, hands them to the
*smart* model (deepseek-v4-pro by default — the one tuned for the deeper, nuanced work),
and gets back a coherent personality. The point of using the smart model is that you give
it a few sliders + a sentence and it fills in a rich, consistent character — the
questionnaire stays short, the result is useful.

    set OPENROUTER_API_KEY first, then:
    python scripts/personality_creator.py                 # interactive
    python scripts/personality_creator.py --out my.json
    # non-interactive (answers piped, one per line):
    printf 'Pip, a shy archivist robot\n2\n4\n2\n5\n loves old maps\n' | python scripts/personality_creator.py

Prompts go to stderr; the resulting JSON + a ready-to-paste robot.yaml snippet go to
stdout, so you can redirect stdout to a file if you like.
"""
import argparse
import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.normpath(os.path.join(_HERE, ".."))
sys.path.insert(0, os.path.join(_ROOT, "src", "web_control"))

from web_control.llm import LlmClient, _extract_json      # noqa: E402

TRAITS = ("curiosity", "extraversion", "caution", "playfulness")
DEFAULT_OUT = os.path.expanduser("~/.local/state/nanobot/personality.json")

DESIGNER_SYSTEM = (
    "You design the initial personality for Nano, a small expressive mobile robot that "
    "speaks short lines aloud and shows a simple face on a tiny screen. From a brief "
    "intake, output ONLY a compact JSON object — no prose, no code fence — with EXACTLY "
    "these keys:\n"
    '{"name": "<short name>",\n'
    ' "persona": "<2 to 4 sentences, second person (\'You are ...\'), describing voice, '
    "temperament and any signature quirk. It is appended to a base prompt that ALREADY "
    "forces a short spoken line plus a face mood, so DO NOT mention output format, JSON, "
    'moods, or screens here — character and voice only. No emoji, no markdown.>",\n'
    ' "traits": {"curiosity":0.0-1.0,"extraversion":0.0-1.0,"caution":0.0-1.0,'
    '"playfulness":0.0-1.0},\n'
    ' "registry": {"musing":{"priority":0.0-1.0,"enabled":true},'
    '"looking":{"priority":0.0-1.0,"enabled":true,"needs":{"curiosity":0.0-1.0}}}}\n'
    "Use the 1-5 sliders as guidance but refine the trait numbers for a coherent "
    "character. The registry tunes idle behaviour: 'musing' = remark on its own sensors, "
    "'looking' = use its camera to comment on what it sees. A curious/outgoing robot "
    "weights 'looking' higher and may lower its 'needs.curiosity' gate; a cautious/"
    "reserved one weights it lower. Keep everything internally consistent."
)


def _ask(prompt, default=""):
    sys.stderr.write(prompt)
    sys.stderr.flush()
    try:
        line = input()
    except EOFError:
        return default
    return line.strip() or default


def _ask_scale(label):
    raw = _ask(f"  {label} [1-5, Enter=3]: ", "3")
    try:
        return max(1, min(5, int(float(raw))))
    except ValueError:
        return 3


def gather_intake():
    sys.stderr.write("\n-- Nano personality creator: a few quick questions --\n")
    concept = _ask("  Name + one-sentence concept "
                   "(e.g. 'Pip, a shy archivist robot who loves old maps'): ",
                   "Nano, a curious little helper robot")
    social = _ask_scale("Social energy   (1 reserved … 5 outgoing)")
    caution = _ask_scale("Caution         (1 daring … 5 very careful)")
    playful = _ask_scale("Playfulness     (1 serious … 5 very playful)")
    curiosity = _ask_scale("Curiosity       (1 incurious … 5 insatiable)")
    quirk = _ask("  One signature quirk / speech style (optional, Enter to skip): ", "")
    return {"concept": concept, "social": social, "caution": caution,
            "playful": playful, "curiosity": curiosity, "quirk": quirk}


def intake_to_prompt(a):
    return (f"Concept: {a['concept']}\n"
            f"Social energy: {a['social']}/5 (1 reserved, 5 outgoing)\n"
            f"Caution: {a['caution']}/5 (1 daring, 5 careful)\n"
            f"Playfulness: {a['playful']}/5 (1 serious, 5 playful)\n"
            f"Curiosity: {a['curiosity']}/5 (1 incurious, 5 insatiable)\n"
            f"Signature quirk / speech style: {a['quirk'] or '(none given — choose one that fits)'}")


def _clamp01(v, default=0.5):
    try:
        return round(max(0.0, min(1.0, float(v))), 3)
    except (TypeError, ValueError):
        return default


def sanitize(obj):
    """Coerce the model's JSON into a valid, fully-populated personality (model output is
    untrusted): clamp traits, ensure the registry beats + fields, cap the persona."""
    traits = obj.get("traits") or {}
    out_traits = {k: _clamp01(traits.get(k), 0.5) for k in TRAITS}
    reg = obj.get("registry") or {}
    musing = reg.get("musing") or {}
    looking = reg.get("looking") or {}
    out_reg = {
        "musing": {"priority": _clamp01(musing.get("priority"), 0.5),
                   "enabled": bool(musing.get("enabled", True))},
        "looking": {"priority": _clamp01(looking.get("priority"), 0.4),
                    "enabled": bool(looking.get("enabled", True)),
                    "needs": {"curiosity": _clamp01((looking.get("needs") or {}).get("curiosity"), 0.3)}},
    }
    return {
        "name": str(obj.get("name") or "Nano")[:40],
        "persona": " ".join(str(obj.get("persona") or "").split())[:700],
        "traits": out_traits,
        "registry": out_reg,
    }


def to_yaml_snippet(p):
    """A ready-to-paste robot.yaml fragment (hand-formatted; no PyYAML needed)."""
    persona = p["persona"]
    lines = ["# --- paste into robot.yaml ---",
             "# web_control: ros__parameters:",
             "    llm_persona: >-"]
    # wrap the persona at ~78 cols under a folded scalar
    words, row = persona.split(), "     "
    for w in words:
        if len(row) + len(w) + 1 > 78:
            lines.append(row)
            row = "     "
        row += " " + w
    lines.append(row)
    lines += ["# behavior: ros__parameters:",
              "    personality:",
              "      traits: { " + ", ".join(f"{k}: {p['traits'][k]}" for k in TRAITS) + " }",
              "      registry:",
              f"        musing:  {{ priority: {p['registry']['musing']['priority']}, "
              f"enabled: {str(p['registry']['musing']['enabled']).lower()} }}",
              f"        looking: {{ priority: {p['registry']['looking']['priority']}, "
              f"enabled: {str(p['registry']['looking']['enabled']).lower()}, "
              f"needs: {{ curiosity: {p['registry']['looking']['needs']['curiosity']} }} }}"]
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description="Generate Nano's first-version personality via the smart model.")
    ap.add_argument("--out", default=DEFAULT_OUT, help=f"where to write personality.json (default {DEFAULT_OUT})")
    ap.add_argument("--model", default="", help="override the smart model id")
    args = ap.parse_args()

    client = LlmClient(enabled=True, api_key="", smart_model=args.model or None,
                       max_tokens=1500, logger=lambda m: print(f"[llm] {m}", file=sys.stderr))
    if not client.available():
        print("! Set OPENROUTER_API_KEY in your environment first.", file=sys.stderr)
        return 2

    intake = gather_intake()
    sys.stderr.write(f"\n... designing personality with {client.smart_model} ...\n")
    raw = client.complete(DESIGNER_SYSTEM, intake_to_prompt(intake), smart=True,
                          max_tokens=1500, json_object=True)
    obj = _extract_json(raw or "")
    if not obj:
        print(f"! The model did not return usable JSON. Raw:\n{raw}", file=sys.stderr)
        return 1
    p = sanitize(obj)

    # Persist the seed personality.
    try:
        os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
        with open(args.out, "w", encoding="utf-8") as f:
            json.dump(p, f, indent=2, ensure_ascii=False)
        sys.stderr.write(f"\n[ok] wrote {args.out}\n")
    except Exception as exc:
        sys.stderr.write(f"\n! could not write {args.out}: {exc}\n")

    # Human summary to stderr; machine-usable JSON + yaml snippet to stdout.
    sys.stderr.write(f"\n  name : {p['name']}\n  persona: {p['persona']}\n"
                     f"  traits : {p['traits']}\n  registry: {p['registry']}\n\n")
    print(json.dumps(p, indent=2, ensure_ascii=False))
    print("\n" + to_yaml_snippet(p))
    return 0


if __name__ == "__main__":
    sys.exit(main())
