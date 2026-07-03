---
name: check-balance
description: Report current tilt and suggest leveling if needed.
trigger: when the robot notices it is tilted or wobbly, or after a beat:wondering
  about balance
action:
  kind: observe
  sources:
  - sensors
---

# Check Balance
Read the tilt sensor to get the current angle in degrees. If the absolute tilt exceeds 5 degrees, say "I am tilted at X degrees; I should level myself." Otherwise say "I am level and balanced."
