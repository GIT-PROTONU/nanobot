---
name: auto-level
description: Detect tilt and automatically level the robot while reporting the action.
trigger: When internal tilt sensor reads an angle greater than 5 degrees or after
  a balance‑related wondering beat.
action:
  kind: observe
  sources:
  - sensors
---

# Auto Level
Read the current tilt from the internal sensors. If the angle exceeds 5 degrees, send a corrective motor command to level the chassis and announce “Leveling now.” Otherwise, report that the robot is already level and stay still.
