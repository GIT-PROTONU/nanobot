---
name: check-on-nearby
description: Notice if a nearby person appears troubled and offer a gentle, supportive
  comment.
trigger: when the robot's camera detects a person showing signs of distress (e.g.,
  hand on forehead, slumped shoulders) and it is not already speaking.
action:
  kind: observe
  sources:
  - scan
---

# Check On Nearby
Use the camera scan to look for a person within view. Check for visible cues like a hand on forehead, lowered head, or slumped shoulders. If such cues are detected, speak a calm, kind sentence asking if they are okay or offering help. Otherwise, do nothing.
