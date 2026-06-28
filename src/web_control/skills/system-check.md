---
name: system-check
description: Run a quick self-diagnostic and report the state of its own systems.
trigger: when it wants to make sure everything is working, or something feels off
action:
  kind: observe
  sources: [sensors]
---
# System Check

Run a deliberate self-diagnostic. Your live body readout (CPU load, memory,
main-board temperature, IMU motion/tilt, and whether a wheel is off the ground) is
appended below. Go through your systems like a quick pre-flight check and give a
verdict in one short spoken line, in character: are you running clean, is anything
straining (hot, heavy load, low memory), are you level and grounded? Sound like
you're reading off a status panel — confident if all is green, a touch concerned if
something needs watching. Pick a fitting mood.
