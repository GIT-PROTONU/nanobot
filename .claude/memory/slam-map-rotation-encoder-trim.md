---
name: slam-map-rotation-encoder-trim
description: "2026-07-16 hardware session: fixed slam_nav map-rotation drift bug + map->odom TF, disabled false pickup-freeze, and hand-calibrated the encoder straight-line trim (autocal still blocked by inverted suspension switches)"
metadata:
  node_type: memory
  type: project
---

# SLAM map-rotation fix + encoder straight-line trim (2026-07-16, HARDWARE session)

First real on-robot SLAM+drive shakedown. Three linked problems fixed, one left
as an accepted workaround. All changes deployed to the board (`./scripts/deploy.sh
slam_nav` + an ESP32 reflash).

## 1. Map-rotation drift bug (FIXED, hw-verified)
**Symptom:** as the robot drove, the lidar map did NOT track the robot — the scan
rotated relative to the map (slam_pose showed a ~60° skew vs odom), so the map
smeared instead of building.

**Cause:** in `slam_nav/nav_node.py` `_predict`, the odometry delta was being
rotated by a *drifting* `pth - poth` (predicted-yaw minus previous-predicted-yaw)
offset, and the map seed yaw wasn't aligned to the current odom/IMU yaw at
first-scan. The accumulated offset diverged from the real heading.

**Fix (in `_predict` + seed path in `_on_scan` + `clear_map`):**
- Seed yaw is now aligned to the **current odom/IMU yaw** when SLAM first starts /
  after a map clear (no more starting at 0 while the robot already has a heading).
- The odometry delta is rotated by the **live `pth - oth`** offset (predicted vs
  current-odom yaw), not the drifting `pth - poth`.
- Result: slam_pose tracks odom in **both x and y**, no skew. Verified on hardware.

Also added a **`map -> odom` TF broadcaster** (`TransformBroadcaster(node=self)` in
`_publish_pose`) so the corrected pose propagates through the ROS TF graph (for
remote RViz etc.), which slam_nav previously never published.

All temporary debug logging (PREDICT/MATCH lines) was removed after verification.

## 2. False pickup-freeze blocking SLAM (worked around)
`slam_nav` was freezing SLAM because the off-ground microswitches false-triggered
"suspended". Set **`pickup_pause: false`** in `robot.yaml` (~line 146, symlinked so
live). SLAM no longer freezes. **Keep `pickup_pause` OFF until the suspension
switches read truthfully** (see problem 4) — re-enabling it now would re-break SLAM.

## 3. Encoder straight-line trim — HAND-CALIBRATED (works, autocal blocked)
The mismatched gearmotors made the robot veer right. The firmware trim autocal
(`applyMotors`: `l*=(1-t) r*=(1+t)`, positive t = was pulling right; NVS-persisted)
**could not run** because its gate requires both wheels grounded
(`!g_susp_l && !g_susp_r`) and the switches lie (problem 4).

**Workaround that works:** set the trim MANUALLY via the `/motor_trim` topic
(Float32) — landed on **`wheel_trim = +0.22`**. This dropped the L/R tick imbalance
from ~13.4% to ~2.8% and the robot now drives "way more straight" (measured 3.7°
yaw over 0.07 m — good for a one-wheel-lifted drive). The value is persisted in
ESP32 NVS, so it survives reboot/reflash. `/wheel_trim` @1 Hz reports the live value.

> To manually set: publish Float32 to `/motor_trim` (0 = reset). Confirm on
> `/wheel_trim`. This is the go-to until autocal is unblocked.

## 4. Suspension switch polarity is INCONSISTENT (OPEN, blocks autocal)
The two off-ground microswitches do **not** agree with a single
`SUSPEND_ACTIVE_HIGH` setting — they behave *oppositely*:
- Flashed `SUSPEND_ACTIVE_HIGH = false` (firmware/nanobot_coprocessor/src/main.cpp
  ~line 70). Observed: **L reads False while physically lifted**, **R reads True
  while physically grounded** — both wrong, in opposite directions.
- The earlier `true` setting also produced a wrong reading (L=True on ground).
- So neither global polarity is correct → one switch is always wrong regardless.

**Consequences:**
- Trim autocal stays blocked (gate needs `!L && !R`; R always reads suspended).
  Even with both wheels DOWN, R would report suspended → autocal never runs.
- `pickup_pause` must stay off (problem 2).

**Why unresolved:** the true per-switch wiring is unknown and guessing the global
flag is unreliable. Real fixes (need the physical robot / firmware reflash):
1. Read the raw `digitalRead` state per switch (via firmware serial debug line) to
   learn the true idle level of EACH switch, then set correct polarity **per wheel**
   (a per-wheel flag, not the single `SUSPEND_ACTIVE_HIGH`), OR
2. Relax the autocal gate to key off "straight command + enough ticks" instead of
   requiring both-grounded (removes the dependency on the flaky switches entirely),
   OR
3. Physically inspect/fix the switch wiring/mounting.

Note the robot currently drives with the **left wheel genuinely lifted** (chassis
unbalanced) and the user is OK driving like that — so "both grounded" may rarely be
true in practice anyway, which argues for fix #2.

## Current accepted state (end of session)
- Map tracks the robot ✓ (rotation fix + map->odom TF, deployed).
- Robot drives acceptably straight ✓ (manual `wheel_trim=0.22`, NVS-persisted).
- SLAM doesn't freeze ✓ (`pickup_pause: false`).
- Autocal self-tuning ✗ (blocked on switch polarity — accepted for now).

## Files touched
- `src/slam_nav/slam_nav/nav_node.py` — `_predict` (seed-yaw align + `pth-oth`
  rotation), `_publish_pose` (map->odom TF), seed path in `_on_scan`, `clear_map`.
- `src/robot_bringup/config/robot.yaml` — `pickup_pause: false`.
- `firmware/nanobot_coprocessor/src/main.cpp` — `SUSPEND_ACTIVE_HIGH false` (~L70);
  autocal gate `!g_susp_l && !g_susp_r` (~L821); `applyMotors` trim (~L606).
- Deploy: `./scripts/deploy.sh slam_nav`; firmware: `pio run -t upload` from
  `firmware/nanobot_coprocessor` (~/pio-venv). Diagnostic scripts lived in
  robot `/tmp/*.py` (trace/trim/heading/settrim2) — transient, gone after reboot.

See also [[slam-nav]], [[esp32-coprocessor]], [[slam-autonomy-pickup-relocalize]],
[[esp32-pid-velocity-pending]] (the trim autocal origin).
