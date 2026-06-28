---
name: read-lidar
description: Read the lidar scan and report the nearest obstacle and roughly which way it is.
trigger: when asked what's around, how close things are, or whether the path is clear
action:
  kind: observe
  sources: [scan]
---
# Read LiDAR

A short summary of your latest lidar scan is appended below — the distance to the
nearest thing and roughly which direction it's in (ahead / left / right / behind).
Report it in character, in one short spoken line: how close the nearest object is and
which way, and whether you feel boxed in or have room to roam. If there's no scan, just
say you can't feel the room right now. Pick a fitting mood.
