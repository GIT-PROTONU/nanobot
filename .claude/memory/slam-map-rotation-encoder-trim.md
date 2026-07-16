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

> **SUPERSEDED by a later 2026-07-16 session — read this first.** Problems 3 & 4 below
> were revisited the SAME day in a follow-up session and the conclusions changed:
> - The "inconsistent per-wheel polarity" finding (problem 4) was wrong — the global
>   flag was simply **inverted**; `SUSPEND_ACTIVE_HIGH` flipped `false`→`true` and now
>   `suspended==true` correctly means the wheel is LIFTED. See [[esp32-coprocessor]].
> - The straight-line trim: the robot actually veers **LEFT** (not right), so the manual
>   `wheel_trim=+0.22` was wrong-way. Autocal was converging the wrong direction, so it's
>   now **DISABLED** and `TRIM_DEFAULT=-0.10` (boost left / cut right) is the new starting
>   point. Tune live via the web Coprocessor card slider or `POST /motor_trim`. The old
>   `pickup_pause: false` workaround may be safe to revert once the flipped switches are
>   verified on hardware.
> Keep the map-rotation fix (problem 1) — that one still holds.

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

## 3. Encoder straight-line trim — ROBOT VEERS LEFT (autocal disabled, fixed TRIM_DEFAULT)
The mismatched gearmotors make the robot veer **LEFT** (corrected from the earlier
"+0.22 right-veer" note — that diagnosis was backwards). In a later same-day session the
trim **autocal was DISABLED** (`TRIM_AUTOCAL 1→0`) because it was converging the WRONG
way: its encoder-signed imbalance pushed trim positive, which *increased* the left veer.

**Current fix:** a fixed starting offset **`TRIM_DEFAULT = -0.10`** (boost left / cut
right) is now the NVS fallback in `applyMotors` (`l*=(1-t) r*=(1+t)`, negative t = was
pulling left; NVS-persisted). Tune live via the web Coprocessor card **Wheel trim** slider
or `POST /motor_trim` (Float32, 0 = reset) — the new value persists to NVS once driven
straight. `/wheel_trim` @1 Hz reports the live value. If it still veers left after -0.10,
push the slider further negative (toward -0.30); if it now veers right, move toward 0/+.

> The earlier manual `wheel_trim=+0.22` is **superseded** — the robot veers left, not
> right, so a positive trim was wrong-way. NVS may still hold +0.22 from that session until
> a new `/motor_trim` value is written; set it via the slider/POST to overwrite.

## 4. Suspension switch polarity — FLIPPED to `true` (correct), was misdiagnosed
The earlier conclusion here ("switches read inconsistent polarity, no single flag can
be right") was WRONG. The global flag was simply **inverted**. On direct user instruction
("down should be up, up should be down") `SUSPEND_ACTIVE_HIGH` was flipped `false`→`true`
(firmware/nanobot_coprocessor/src/main.cpp ~L70, **built + flashed**).

- **`true` is now correct:** the switch reads HIGH (INPUT_PULLUP) while the wheel is
  LIFTED, LOW while on the ground. So `left/right_wheel_suspended == true` MEANS the
  wheel is **UP / lifted** (robot suspended) — the natural reading.
- This removes the autocal blocker premise (the old gate `!g_susp_l && !g_susp_r` is now
  meaningful). Autocal is still DISABLED for the separate reason in problem 3 (it was
  converging the wrong way).
- `pickup_pause` can be reconsidered: with truthful switches, re-enabling
  `pickup_pause: true` in robot.yaml should be safe — but **verify on hardware** before
  flipping it back (the false-trigger that forced it off is now expected to be gone).
- If a switch still misreads after this flip, THEN it's a genuine per-wheel wiring fault
  (fix options 1/3 above), but that was not the actual cause this time.

## Current accepted state (end of ORIGINAL session)
- Map tracks the robot ✓ (rotation fix + map->odom TF, deployed).
- Robot drives acceptably straight ✓ (manual `wheel_trim=0.22`, NVS-persisted) — *see
  the superseded note above; current trim is `TRIM_DEFAULT=-0.10` and robot veers left*.
- SLAM doesn't freeze ✓ (`pickup_pause: false`).
- Autocal self-tuning ✗ (blocked on switch polarity — accepted for now).

## UPDATED state (later same-day session — see top supersede note)
- Map tracks the robot ✓ (unchanged).
- Off-ground switches now truthful ✓ (`SUSPEND_ACTIVE_HIGH true` — `suspended==true`
  means wheel UP/lifted; built + flashed).
- Straight-line drive: autocal DISABLED, `TRIM_DEFAULT=-0.10` (boost left/cut right for
  the left veer); tune live via web Coprocessor card slider or `POST /motor_trim`.
- SLAM freeze + autocal: `pickup_pause` re-enable + autocal re-enable are now *plausible*
  again (switches truthful) but NOT yet done — verify on hardware before flipping.

## Files touched (original session)
- `src/slam_nav/slam_nav/nav_node.py` — `_predict` (seed-yaw align + `pth-oth`
  rotation), `_publish_pose` (map->odom TF), seed path in `_on_scan`, `clear_map`.
- `src/robot_bringup/config/robot.yaml` — `pickup_pause: false`.
- `firmware/nanobot_coprocessor/src/main.cpp` — `SUSPEND_ACTIVE_HIGH false` (~L70);
  autocal gate `!g_susp_l && !g_susp_r` (~L821); `applyMotors` trim (~L606).
- Deploy: `./scripts/deploy.sh slam_nav`; firmware: `pio run -t upload` from
  `firmware/nanobot_coprocessor` (~/pio-venv). Diagnostic scripts lived in
  robot `/tmp/*.py` (trace/trim/heading/settrim2) — transient, gone after reboot.

## Files touched (later session: suspend flip + trim autocal off + web trim slider)
- `firmware/nanobot_coprocessor/src/main.cpp` — `SUSPEND_ACTIVE_HIGH true`; `TRIM_AUTOCAL 0`
  + `TRIM_DEFAULT -0.10`; NVS load falls back to `TRIM_DEFAULT`.
- `src/web_control/web_control/telemetry.py` — `/motor_trim` added to `/publish` whitelist
  (`_mk_motor_trim`, clamps to `TRIM_MAX=0.30`); subscribes `/wheel_trim`, surfaces live
  value in the `esp` frame.
- `src/web_control/web/index.html` + `app.js` — Coprocessor card **Wheel trim** slider
  (±0.30) + live readout + reset button; slider re-seeds from the telemetry `wheel_trim`.
- Deploy: `./scripts/deploy.sh` (all pkgs) + `pio run -t upload` — both done this session.

See also [[slam-nav]], [[esp32-coprocessor]], [[slam-autonomy-pickup-relocalize]],
[[esp32-pid-velocity-pending]] (the trim autocal origin),
[[web-publish-topic-namespace-gotcha]] (the /motor_trim whitelist + web slider).
