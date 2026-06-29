#!/usr/bin/env python3
"""Export the presence Sismic statechart to PlantUML (for visualising the brain).

The chart in `behavior/presence.py` is written in **Sismic's YAML format**, which is NOT
PlantUML — pasting it into a PlantUML renderer fails with "directive `statechart:` is not
recognized". Sismic ships the right converter (`sismic.io.export_to_plantuml`); this wraps it
so a single command always emits PlantUML that matches the live chart.

    pixi run python scripts/export_statechart_puml.py            # -> docs/presence.puml
    pixi run python scripts/export_statechart_puml.py --stdout   # print instead of writing
    python scripts/export_statechart_puml.py --out /tmp/x.puml   # (bare python works too)

Render the result with any PlantUML tool (the public server, the VS Code PlantUML extension,
or `plantuml docs/presence.puml`). ROS-free; needs only `sismic` (already a dev dep).
"""
import argparse
import os
import sys

# Make `behavior` importable whether or not the package is installed (e.g. a bare dev python):
# add <repo>/src/behavior to sys.path relative to this script.
_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_REPO, "src", "behavior"))


def main():
    ap = argparse.ArgumentParser(description="Export the presence statechart to PlantUML.")
    ap.add_argument("--out", default=os.path.join(_REPO, "docs", "presence.puml"),
                    help="output .puml path (default: docs/presence.puml)")
    ap.add_argument("--stdout", action="store_true", help="print to stdout instead of writing")
    args = ap.parse_args()

    import sismic.io as sio
    from behavior.presence import PRESENCE_YAML

    statechart = sio.import_from_yaml(PRESENCE_YAML)
    puml = sio.export_to_plantuml(statechart)
    if not isinstance(puml, str):                       # older sismic may return a filepath
        with open(puml, encoding="utf-8") as f:
            puml = f.read()

    if args.stdout:
        print(puml)
        return
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        f.write(puml.rstrip("\n") + "\n")
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
