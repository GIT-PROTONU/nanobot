# Nano's skill library

Each file in this directory is **one self-documenting capability** — a portable
"SKILL.md" (inspired by OpenClaw). Drop a new `.md` here and Nano gains a capability;
no code change. The robot loads them at start and on `POST /skills/reload`, the brain can
pick one autonomously on a `skill` beat, and the web UI's **Skills** panel lists them with
an *Invoke* button.

> `README.md` (this file) is ignored by the loader — only capability files are indexed.

## Anatomy of a skill file

YAML frontmatter (the machine-readable contract) + a Markdown body (the human/LLM-readable
"how", folded into the prompt that steers the spoken line):

```markdown
---
name: read-lidar                      # unique slug; defaults to the filename
description: Report the nearest obstacle from the lidar scan.   # shown to the brain + UI
trigger: when asked what's around or how close things are       # when to pick it (selection hint)
action:
  kind: observe                       # say | observe | look | topic
  sources: [scan]                     # observe: which context to append (sensors / scan)
---
# Read LiDAR
Free-text instructions to the brain on how to perform / narrate this skill.
```

## Action kinds

| kind | what it does | model |
|---|---|---|
| `say` | speak one in-character line steered by the body text | cheap text |
| `observe` | like `say`, plus appends context (`sources: [sensors]` and/or `[scan]`) | cheap text |
| `look` | grabs one camera frame and describes what it sees | vision |
| `topic` | publishes a **whitelisted** ROS message — the gated "physical" tier | — |

## The gated action tier (`kind: topic`)

Narrative skills (`say`/`observe`/`look`) are always safe — pure expression. A `topic`
skill actually *does* something, so it's gated **twice**:

1. the skill file must set `action.enabled: true`, **and**
2. the node's `skills_allow_actions` param (`web_control` in `robot.yaml`) must be on
   (it's **off by default**).

Whitelisted topics + their clamps (anything else is refused):

| topic | type | `value` | clamp |
|---|---|---|---|
| `/led` | `std_msgs/Bool` | `true`/`false` | — (use `off_after: <s>` to auto-revert) |
| `/fan_pwm` | `std_msgs/Float32` | `0..1` | clamped to `0..1` |
| `/lds_target_rpm` | `std_msgs/Float32` | rpm | clamped to `0..400` |
| `/cmd_vel` | `geometry_msgs/Twist` | `{lin, ang}` | `|lin|≤0.15`, `|ang|≤0.8`, `duration≤3s`, then auto-stop |

Motion is **also** clamped reflexively by `slam_nav` downstream, so a skill can never push
the robot into an unsafe state. Optional `face:` (an OLED mood) and `say:` (a literal
spoken line — no LLM) make a topic skill expressive without a model call.

See `wiggle.md` for a motion template and `blink-led.md` for a safe indicator action.
