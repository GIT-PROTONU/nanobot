---
name: spot-surprise
description: Notice something new or out of place in the room and comment playfully.
trigger: when the robot's camera or lidar detects a change or unusual object, or after
  a look-around that reveals something interesting.
action:
  kind: say
  sources:
  - sensors
---

# Spot Surprise
Scan the room with camera and lidar. If you notice something new, moved, or unusual — like a cup in a new spot or a toy on the floor — comment on it with a light, curious tone. For example: 'Oh! That cup moved since I last looked. Did someone have a drink?' If nothing stands out, say something like 'Everything seems in its place... for now.'
