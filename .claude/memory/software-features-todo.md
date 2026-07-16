---
name: software-features-todo
description: "Software-only feature backlog (no new hardware), started 2026-07-13. TODO (approved): named locations + go-to skill, IMU-fused odometry, odom auto-calibration, wheel-slip cross-check. NICE-TO-HAVE (all user-endorsed, no priority): follow-me, voice, roam, sentry, no-go zones, history graphs, path replay, conversation memory, audio emotes, map-change detection, push notifications, coverage map, odometer stats, TTS prosody, stuck-escape, global relocalization, tap gestures, terrain-from-effort, WiFi fingerprint, chirp self-test, mirror detection, courier mode, alarm clock, rhythm learning, diary page, dream journal, games, expressive fan"
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

**3. Odometry auto-calibration routine.** A skill that spins 360° (gyro-integrated as truth)
and drives a short straight (lidar-verified), then solves ticks-per-metre + track width from
the residuals — replaces the hand-tuned `robot.yaml` odom constants. Same spirit as the ESP32
straight-line trim autocal. Natural to build AFTER IMU-fused odometry (item 2) since it
leans on trusting the gyro.

**4. Wheel-slip cross-check.** IMU gyro-z vs encoder-implied rotation rate disagree →
slipping/dragged/stuck. Complements the optical bumper (which needs the camera on);
near-free once item 2 exists. Consumers: caution fast-rule + a diagnostics alert; later the
stuck-escape reflex.

## Nice to have (user-endorsed 2026-07-13 — wanted, no deadline/priority; pick up when convenient)

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

## Nice to have — outside-the-box round (also user-endorsed 2026-07-13)

New *virtual sensors* from hardware already on board, plus household-intelligence and
play/character ideas:
- **Knock/tap gesture input (IMU as a touch sensor)**: the BWT901CL streams accel at 200 Hz;
  a knock on the chassis is a sharp, distinctive spike train. Detect single/double/triple
  taps in `imu_driver`'s reader thread → a `/tap` event → wake from quiet hours, ack a
  sentry alert, dismiss the alarm clock, "pet the robot" (playfulness nudge). A whole input
  channel with zero new hardware.
- **Floor-type from motor effort ("terrain from effort")**: commanded PWM duty vs achieved
  encoder speed = rolling resistance → classify carpet vs hardwood, annotate map regions,
  auto-adjust caution/speed per surface. Uses only signals already flowing.
- **WiFi RSSI localization prior**: log `/proc/net/wireless` RSSI against SLAM pose; the
  learned RSSI-per-area map is a coarse global-relocalization prior (helps true kidnap
  recovery, where scan-match alone is ambiguous) — classic WiFi fingerprinting, one number/s.
- **Chirp self-test + room acoustic fingerprint (speaker↔mic loop)**: play a short chirp,
  listen with its own mic — (a) verifies the whole audio path end-to-end (today a dead
  speaker is silent-invisible), (b) reverb/decay differs per room → coarse "which room am I
  in by ear" cross-check, (c) muffled = under furniture/covered.
- **Mirror detection via LED-blink correlation**: blink the ESP32 LED in a known pattern and
  correlate against camera luma/blob response — a match = "that's me" (a mirror or glass).
  Doubles as real safety: lidar reads mirrors as fake openings; flag those map cells.
- **Courier mode**: detect an object placed on the robot (z-accel transient + raised rolling
  resistance) → "take this to the kitchen" (named location) → announce arrival, wait for
  tap/pickup to confirm delivery. Composes items already on this list.
- **Physical alarm clock / come-get-you**: at a scheduled time, drive to a named location and
  speak/beep with rising insistence until the PIR motion score says a human moved (or a tap
  dismisses it). Schedule + go-to + TTS + PIR glued together.
- **Household rhythm learning**: aggregate motion/luma/noise by hour-of-day-and-weekday
  (same mechanism as the visual diary) → anticipate ("they usually come home around now" →
  wait near the door beat), and flag anomalies ("lights on at 3am", "no movement all
  Saturday"). The robot starts *knowing the house*, not just the room.
- **Daily diary web page ("robot blog")**: a `GET /diary` page composed at reflection time —
  LLM narrative + day's stats (odometer, beats, skills) + a couple of snapshots. Pure
  composition of existing pieces; very high delight-per-effort.
- **Dream journal**: during long/night reflection, have the smart model recombine the day's
  decision log into a short surreal "dream", spoken once next morning. Zero new plumbing —
  a reflection-mode prompt variant + one queued utterance.
- **Games**: hide-and-seek (human hides, robot roams toward motion/novelty); red-light-
  green-light (robot creeps while motion score is low, freezes theatrically when you move);
  both fit the gated action tier + existing signals.
- **Expressive fan**: the cooling fan is an audible actuator — a brief spin-up "sigh"/purr
  as an emote (bounded so thermals always win). Gimmick tier, but literally free.

Explicitly out (hardware or excluded): wheel velocity PID (single-channel encoders), overhead
camera-geometry check (needs the robot), docking/cliff (user-excluded), on-device STT / face
recognition (too heavy for 1 GB H5).

2026-07-16: a separate CODE-IMPROVEMENT list (review findings, not features) now lives
in-repo at `docs/TODO.md` — pending flashes/deploys, the tracking-latched-goal fix, the
brain_timeout code-default guard, ticks_per_rev verification. Check both lists.
