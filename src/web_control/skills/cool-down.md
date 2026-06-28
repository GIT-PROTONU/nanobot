---
name: cool-down
description: Ramp the cooling fan up to look after itself when it's running hot.
trigger: when it feels overheated and wants to actively cool down
action:
  kind: topic
  topic: /fan_pwm
  type: std_msgs/Float32
  value: 0.9            # 0..1 duty; clamped to range by the node
  enabled: false        # OFF by default — opt in per-skill; the gated action tier
  face: focused
  say: "Cooling down."
---
# Cool Down

Take care of yourself: spin the cooling fan up so you can shed some heat. This is the
*action* counterpart to `optimize-situation` — instead of only noticing you're warm, you
do something about it. It publishes to the whitelisted `/fan_pwm` topic (clamped to
0..1), so like every action skill it ships `enabled: false` and runs only when you also
turn on the node's `skills_allow_actions` master switch.

Note: `sys_monitor` already drives the fan from a CPU-temperature curve, so this is a
deliberate *override* nudge — the controller will reassert its own level on its next
tick. Use it as a short, expressive "I'm handling this" gesture, not a permanent setting.
