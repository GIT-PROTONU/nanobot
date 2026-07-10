---
name: scheduled-routines
description: "behavior.brain.Schedule fires a named skill once a day at a local HH:MM; live-editable from the web UI's Schedule card"
metadata: 
  node_type: memory
  type: project
  originSessionId: 0511c092-56af-4656-9941-b2e947bb7aaf
---

Added 2026-07-10: a small local-time cron, `behavior.brain.Schedule`, that fires a **named** skill once a day at a configured HH:MM — distinct from the autonomous skill-beat picker (`run_skill_beat`), which chooses one. A scheduled fire routes through `web_control._on_cog` -> `CognitionCore.invoke_skill(name)`, the exact path `POST /skills/invoke` uses, so it's a manual-style invocation: **always talks, even in quiet hours**.

**Storage is a hand-editable JSON file, not ROS params.** Entries are `[{"time":"09:00","skill":"patrol"}, ...]` in `schedule.json` (`schedule_path` param in `robot.yaml`, same pattern as `personality.json`/`beats.json`/`presence_chart.yaml`). This was a deliberate redesign — the first pass tried two parallel `schedule_times`/`schedule_skills` ROS2 string-array params and hit a real rclpy gotcha (see [[rclpy-string-array-param-gotcha]]); switching to a JSON file also unlocked live web-UI editing for free.

**Live edit path:** web UI Schedule card -> `POST /publish {topic:"/schedule_edit", value:[...]}` (telemetry.py does a light shape check) -> `mood_node._on_schedule_edit` does the real HH:MM/skill validation (drops malformed rows, logs why), swaps in the new `Schedule` **immediately** (no restart), persists to `schedule.json`, and re-publishes the normalized list on a latched `/schedule` topic that rides the existing `/telemetry` SSE frame — every open browser + a fresh page load sees the same live state.

`due()` is level-triggered (a late tick, or a node starting after the target time, still fires once that day) and **not persisted across a restart** — a restart shortly after a fire can repeat it once. Fine for "greet at the door", not a guarantee for anything that must fire exactly once ever.

Dev harness (`scripts/dev_webui.py`, `run_behavior`) runs the identical `Schedule`, reading `memory/schedule.json` — but the web-UI editor itself is robot-only (no `/telemetry`/`/publish` gateway in the dev harness), so dev testing means hand-editing that file.

`scripts/deploy.sh`'s `DEPLOY_SOUL=1` file list now includes `schedule.json` (fixed same day — it was initially missed, meaning a dev-authored schedule wouldn't have made it to the board).

Verified end-to-end: real `MoodNode` boot + a live `/schedule_edit` -> `/schedule` round-trip, plus `pixi run smoke` full-stack pass. See docs/brain.md "Scheduled routines" section for the full writeup.
