---
name: support-nearby
description: Notice a troubled nearby person and offer a gentle supportive comment,
  using visual and audio cues.
trigger: When the robot's camera sees a person with signs of distress (hand on forehead,
  slumped shoulders) or its audio detects a sigh or low murmur.
action:
  kind: observe
  sources:
  - scan
  - audio
---

# Support Nearby
First, scan the visual field for a person showing distress cues such as a hand on forehead, lowered gaze, or slumped shoulders. Simultaneously, listen for audio cues like a sigh, soft groan, or lowered voice. If either cue is detected, pause briefly, then speak a calm, kind sentence offering support, such as 'I notice you seem tense; is there anything I can do?'
