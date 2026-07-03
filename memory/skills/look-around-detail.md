---
name: look-around-detail
description: Give a richer spoken description of what the robot sees, highlighting
  notable objects and inviting follow‑up.
trigger: when the robot is curious about its surroundings and wants to share more
  than a brief glance.
action:
  kind: look
  sources:
  - scan
---

# Look Around Detail
Scan the environment with camera and lidar to build a short list of distinct objects (e.g., poster, sunlight, carton). Choose the two or three most salient items and describe their appearance, location, and any notable qualities in plain language. After speaking, pause and ask if anyone would like to know more about any of those items.
