---
name: blink-led
description: Flash the onboard LED once, as a small wordless "I'm here" wave.
trigger: when acknowledging someone without speaking, or just being playful
action:
  kind: topic
  topic: /led
  type: std_msgs/Bool
  value: true
  off_after: 0.6        # turn the LED back off this many seconds later
  enabled: true         # a harmless indicator LED — safe to leave on
  face: happy
  say: "Blink!"
---
# Blink LED

Pulse the onboard status LED on briefly, then off — a tiny wordless wave. This is the
gentlest example of an *action* skill: it publishes to a whitelisted topic (`/led`)
rather than only speaking. Because it's just an indicator LED it ships `enabled: true`,
but it still only runs when the node's `skills_allow_actions` master switch is on.
