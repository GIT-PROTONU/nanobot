---
name: software-features-todo
description: "Software-only feature backlog (no new hardware), started 2026-07-13. APPROVED/TODO: named locations + go-to skill, IMU-fused odometry. Below that: the wider brainstorm (follow-me, voice, roam mode, sentry, odom autocal, push notifications, …) — ideas only, not approved"
metadata: 
  node_type: memory
  type: project
  originSessionId: 6d19487e-e4e3-4485-bc4a-663e32e9b88a
---

Post-GPU-vision backlog (that one is cleared — see [[gpu-vision-features-todo]]). Everything
here is buildable with the existing hardware: lidar, IMU, encoders, C270 cam+mic, speaker/TTS,
OLED, ESP32, LED, fan.

## TODO (user-approved 2026-07-13, not yet built)

**1. Named locations + "go to the kitchen".** slam_nav already has click-to-go (`goal_pose`)
and `go_home`/`save_map`. Add named waypoints: a `locations.json` in
`~/.local/state/nanobot/` (live-editable from the web map panel, same pattern as the Schedule
card / `schedule.json` — NOT ROS params, see [[rclpy-string-array-param-gotcha]]), plus a
`go-to.md` action skill (`kind: topic` → `goal_pose`, gated by `skills_allow_actions` like all
motion) that takes a location name. Compounds with what exists: scheduled routines ("patrol
the hallway at 22:00"), the LLM skill picker, and chat ("go check the door"). Save a location
= "remember this spot as X" (current pose) from the UI, and optionally a chat/skill path.

**2. IMU-fused odometry.** `wheel_odometry` integrates single-channel encoder ticks only —
differential heading from tick imbalance is its weakest output (ticks are signed by
*commanded* direction, no true quadrature). The BWT901CL publishes good yaw at 50 Hz on
`/imu/euler` and nothing fuses it. Add a complementary filter in `wheel_odometry` (encoder
translation + IMU yaw for rotation, ~30 lines; param-gated so it can be A/B'd against pure
encoder odom). Directly improves SLAM scan-match quality and the pickup/relocalize recovery.
Both nodes are already co-resident in sensor_hub, so no new cross-process traffic. Watch:
IMU yaw drifts slowly (no mag fusion guarantee) — fuse *rate/delta* yaw, not absolute yaw.

## Brainstorm (2026-07-13, NOT approved — ideas to pick from)

From the first round (see that session for detail): lidar **follow-me** (leg-cluster tracking
in `/scan`, hold distance, under the existing caution clamps — most "alive" per line of code);
**voice input** (zero-cost tier: browser `SpeechRecognition` → `/llm/chat`; on-robot tier:
RMS/band energy reflexes from the PCM — startle on loud noise, make the `listening` beat
event-driven like PIR did for `looking`); **roam/explore mode** (frontier-based wandering over
the existing occupancy grid, gated action skill); **no-go zones** (draw on the web map,
slam_nav treats as occupied — protects cables the lidar can't see); **history sparklines**
(ring-buffer of the vitals blob → CPU/RAM/temp graphs + fan-curve verification in the System
tab); **path record/replay** (teleop drive → named waypoint list → replayable skill, pairs
with the schedule); **conversation memory** (consolidate chat history into the reflection
self-narrative so it survives reboots); **audio emotes** (generated beeps/chirps via the
existing aplay path — faster/cheaper than TTS).

Second round:
- **Wheel-slip cross-check**: IMU gyro-z vs encoder-implied rotation rate disagree → slipping/
  dragged/stuck. Complements the optical bumper (which needs the camera); near-free once
  IMU-fused odometry exists. Consumer: caution fast-rule + a diagnostics alert.
- **Odometry auto-calibration routine**: a skill that spins 360° (gyro-integrated) and drives
  a short straight (lidar-verified), then solves ticks-per-metre + track width from the
  residuals — replaces hand-tuned `robot.yaml` odom constants. Same spirit as the ESP32
  straight-line trim autocal.
- **Sentry/guard mode**: park at a named location, watch the already-computed motion score;
  on a spike → snapshot to disk, decision-log entry, optional TTS challenge. Composes named
  locations + PIR + schedule ("guard the hallway at night") with almost no new machinery.
- **Map change detection**: diff the live scan/map against the saved map → "the chair moved"
  observations for beats/diary, and a relocalize-confidence hint.
- **Web push notifications**: SSE telemetry only works with the page open; a service-worker +
  Web Push (self-hosted VAPID) path could notify a phone on pickup/stall/sentry-alert. Bigger
  lift than the rest (HTTPS requirement is the main friction).
- **Coverage/exploration map**: visited-cells grid over the occupancy map → an "explored %"
  novelty signal feeding the curiosity trait and roam mode's frontier choice.
- **Daily odometer/activity stats**: metres driven, beats fired, skills run per day — folded
  into the reflection/diary prompts like the trait trajectory ("I drove 40 m today").
- **Mood-modulated TTS prosody**: map the chart's `drives`/mood to espeak `-p`/`-s` per
  utterance (excited = higher/faster, low energy = slower) — trivial, big expressiveness win.
- **Stuck-escape reflex**: on optical-bumper/wheel-slip stall, run a small gated
  reverse-and-wiggle routine instead of only alerting (action-tier, off by default).
- **Global relocalization**: today's lost-robot recovery is local-only (~0.5 m, see
  [[slam-autonomy-pickup-relocalize]]); a coarse global scan-match against the saved map
  would survive a true kidnap.

Explicitly out (hardware or excluded): wheel velocity PID (single-channel encoders), overhead
camera-geometry check (needs the robot), docking/cliff (user-excluded), on-device STT / face
recognition (too heavy for 1 GB H5).
