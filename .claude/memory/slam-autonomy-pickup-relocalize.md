---
name: slam-autonomy-pickup-relocalize
description: "slam_nav autonomy: pick-up awareness (off-ground switches) + lost-robot self-recovery (wide scan-match relocalization)"
metadata:
  type: project
---

Two autonomy features added to `slam_nav` (`nav_node.py`), 2026-06-23, on `main`. Both
toggle live via params and are independent of motion (recovery spin only runs if
`enable_motion`). Web map panel shows a localization status (`loc`: ok/lost/relocalizing/
"picked up") appended to the map stats line.

**Pick-up awareness (#6):** subscribes the ESP's `/left_wheel_suspended` +
`/right_wheel_suspended` (Bool). BOTH off-ground = lifted → halt `/cmd_vel`, **freeze SLAM**
(in `_on_scan`: skip predict/match/integrate while carried so garbage scans don't smear the
map; still refresh the map file so the web shows "picked up"), and flip the OLED to a mood
(`pickup_face`, default "focused"; "" = leave OLED alone — avoids fighting the web face).
On set-down, arms relocalization. Param `pickup_pause` (default true).

**Lost-robot self-recovery (#3):** when the scan-to-map match score stays `< min_match_score`
for `recover_patience` scans *and* there are `>= recover_min_beams` in-range beams (so we
don't false-trigger in open space), enter `_recovering`: each scan runs a WIDE `grid.match`
(`recover_lin`/`recover_ang`/`recover_half`/`recover_refine`) around the prior, and `_control`
commands a slow in-place spin (`recover_spin`) to vary geometry. Exits when score `>=
recover_exit_score`, or gives up after `recover_timeout` (keeps best estimate). Does NOT
integrate into the map while recovering. Param `relocalize` (default true).

**KEY LIMITATION — local recovery only, not global kidnap.** The wide match searches only
±`recover_lin` (~0.5 m) around the prior; heading is rescued by the IMU-yaw delta applied in
`_predict` (so even a big carry-rotation recovers, since `/imu/euler` tracks it), but a
translation beyond ~0.5 m won't relocalize (no whole-map search — too costly on the H5). So
it handles bumps / slips / lift-and-replace-near-here, not "carried to another room."

`grid.match` already accepted `half`/`refine` args — no `occupancy.py` change needed. Params
live in `robot.yaml` slam_nav block; `relocalize` + `pickup_pause` are live-settable. NOT yet
hardware-verified. The `separate-sensor-nodes` fallback branch does NOT have these (main only).
See [[slam-nav]], [[esp32-coprocessor]].

**Calibration self-test (2026-06-23, same nav_node).** Trigger: publish `/selftest` Bool true
(web button "🔧 Self-test" in the map panel; **requires enable_motion**). Scripted drive
preempts nav in `_control`: still → forward `TEST_DIST` → back → in-place spin `TEST_TURNS`.
Checks: IMU at rest (|accel|≈9.81, gyro≈0, /imu/web alive); encoders both count + & balanced
forward (raw `/wheel_ticks`, dead/imbalance detection) and go negative on reverse; spin
cross-checks **IMU yaw vs wheel-odom yaw vs commanded** (wrapped accumulation) and suggests a
`wheel_separation` scale factor on mismatch. Report → log + `/selftest_result` String (shown
in the web `mapTestOut` panel) + OLED title. Lazy-subscribes `/wheel_ticks` + `/imu/web` only
during the run (destroyed after) to avoid steady CPU. Aborts on pick-up or motion-off. Tunables
= `TEST_*` module constants in nav_node.py. NB: web `/selftest` topics are UNprefixed (match the
node's relative names) — unlike the pre-existing `/slam_nav/go_home` buttons which look mismatched
against the node's relative `go_home` sub (latent, not touched).
