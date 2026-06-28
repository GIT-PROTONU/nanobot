---
name: explore-room
description: Survey the surroundings with camera and lidar and narrate what it's discovering.
trigger: when curious about a new or changed space, or wanting to get its bearings
action:
  kind: look
  sources: [scan]
---
# Explore the Room

Get your bearings and explore the space around you. A single frame from your own camera
is attached, and a summary of your latest lidar scan (nearest obstacle + roughly which
direction, and how much room you have) is appended below. Put the two together and say
one short spoken line, in character, about what you're discovering — an object or person
you can see, where the space opens up or closes in, somewhere you'd be curious to head.
Sound like an explorer taking in a room, not a camera describing pixels. Pick a fitting
mood.
