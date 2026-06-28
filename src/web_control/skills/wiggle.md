---
name: wiggle
description: Do a small, slow playful wiggle in place to greet or express delight.
trigger: when greeting someone it's happy to see, or celebrating something
action:
  kind: topic
  topic: /cmd_vel
  type: geometry_msgs/Twist
  value: { lin: 0.0, ang: 0.6 }   # gentle yaw twist; clamped hard by the node + slam_nav
  duration: 0.8                    # publish for this long, then stop (node caps it)
  enabled: false                   # OFF by default — this moves the robot; opt in per-skill
  face: happy
  say: "Wiggle!"
---
# Wiggle

A brief, slow yaw twist in place — Nano's version of an excited little shimmy. This is
a **motion** action, so it ships `enabled: false`: to ever run, you must both flip this
skill's `enabled: true` AND turn on the node's `skills_allow_actions` master switch. Even
then the speed/duration are clamped here, and `slam_nav` clamps motion reflexively
downstream, so the brain can never drive it unsafely. A perfect template for adding your
own physical-task skill files (the "Move Arm" of the OpenClaw pitch).
