---
name: software-features-todo
description: "Software-only feature backlog (no new hardware), started 2026-07-13. DONE (all 4 approved items, as of aede005): named locations + go-to skill, IMU-fused odometry (via robot_localization EKF, NOT a wheel_odometry complementary filter), odom auto-calibration, wheel-slip cross-check. DONE 2026-07-16: IMU quality tools (6-axis toggle, interference self-test, mag-cal scatter, bandwidth filter) -- see imu-quality-tools-built. NICE-TO-HAVE (all user-endorsed, no priority): follow-me, voice input, roam, no-go zones, history sparklines, path replay, conversation memory, audio emotes, map-change detection, push notifications, coverage map, odometer stats, TTS prosody, stuck-escape, global relocalization, tap gestures, terrain-from-effort, WiFi fingerprint, chirp self-test, mirror detection, courier mode, alarm clock, rhythm learning, diary page, dream journal, games, expressive fan"
metadata: 
  node_type: memory
  type: project
  originSessionId: 6d19487e-e4e3-4485-bc4a-663e32e9b88a
---

Post-GPU-vision backlog (that one is cleared — see [[gpu-vision-features-todo]]). Everything
here is buildable with the existing hardware: lidar, IMU, encoders, C270 cam+mic, speaker/TTS,
OLED, ESP32, LED, fan.

## ALL FOUR APPROVED ITEMS DONE (aede005, 2026-08-08)

**1. Named locations + "go to the kitchen"** — DONE (aede005).
**2. IMU-fused odometry** — DONE, but implemented as a `robot_localization` **EKF**, NOT the
complementary-filter-in-`wheel_odometry` originally specced. Commits `b2b56d6` +
`07f07e4`: `ekf_node` (src/robot_bringup/config/ekf.yaml) fuses `/odom` (X/Y/Vx) +
`/imu/data` (yaw, gyro Z, accel) into `/odometry/filtered`, publishes `odom->base_link` TF;
slam_nav consumes `/odometry/filtered` (`odom_topic` in robot.yaml). `wheel_odometry`
deliberately stays raw — EKF excludes wheel yaw since single-channel ticks sign by commanded
direction and drift with slip (ekf.yaml odom0_config).
**3. Odometry auto-calibration** — DONE (aede005).
**4. Wheel-slip cross-check** — DONE (aede005).

## DONE (2026-07-16, same day) — IMU quality tools

User-approved 2026-07-16 ("add all to todo"), then built same session. All four
implemented + smoke/unit/manual-endpoint tested on the dev host; NOT yet
hardware-verified or deployed. See [[imu-quality-tools-built]] for the full writeup
(files touched, register values used, safety gating).

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

2026-07-16: a separate CODE-IMPROVEMENT list (review findings, not features) lives
in-repo at `docs/TODO.md`. All its code-side items were implemented same-day (tracking-
latched-goal fix in `_on_goal`/`_on_go_home`, `brain_timeout` code-default raised to
1800 + a runtime clamp to >=2x `purpose_period`, `_mk_goal` coordinate clamp, slam_nav
self-test constants promoted to live params, narrative-skill phrase-bank fallback in
`CognitionCore._invoke_skill`) -- 198 unit tests + `pixi run smoke` pass, not yet
deployed (board unreachable that session). What's left in `docs/TODO.md` is
hardware-only: flash the ESP32 stray-tick firmware, deploy the pending fixes,
hardware-verify IMU calibration/vision tracking/`ticks_per_rev`. Check both lists.

2026-08-08: all 4 approved items (named locations, IMU-fused odom = robot_localization EKF,
odom auto-calibration, wheel-slip cross-check) landed in aede005. No open feature items --
remainder is the nice-to-have pool above.
