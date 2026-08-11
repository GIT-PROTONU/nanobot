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

**2026-08-11 — recovery hardened + kidnap relocalize (deployed, fix for the SLAM
"teleport" storm).** After a deploy that dragged the board through repeated halt/look
recovery, the robot's pose snapped across the map while driving — the local recovery was
trusting a single wide match too easily on a smeared map. Recovery retuned/rebuilt in
`nav_node.py` (see `recover_*` params in `robot.yaml`/`nav_node.py:~144-167`):

- **`recover_overlap: 0.30`** — recovery poses must ALSO pass an overlap-inlier gate
  (the `min_overlap_ratio` used for map building is far too permissive to trust;
  overlap ~0.9 with score ~-200 was the lost-storm signature).
- **`recover_exit_score: 20.0`** (raised from 4.0) — a scan-match score this strong is
  needed to end recovery.
- **`recover_confirm: 2`** — the SAME recovery candidate must reproduce across
  **2 consecutive scans** before it's trusted and integrated. A single global search run
  can score well at a WRONG-but-plausible spot (symmetric room / smeared map); the 2nd
  agreeing scan costs ~0.2 s and stops both the map-pose teleport and map-corrupting
  integration at a wrong pose. Confirmation state (`_recover_conf`, `_recover_conf_hits`,
  tolerance 0.30 m + 0.15 rad) resets on timeout give-up, map clear, and lost.
- **Kidnap-gated full-grid relocalize:** recovery now also runs a global full-grid
  search (`recover_global`/`recover_global_step` 4 cells=20 cm/`recover_global_period`
  2.0 s, via `grid.relocalize`), but it is gated to fire ONLY on `self._recover_kidnap
  or not self._ever_trusted` — never on a drive-time match loss. `_recover_kidnap` is set
  True on a pick-up set-down, False on lost-while-driving. This keeps the "carried to
  another room" case recoverable WITHOUT letting a drive-time storm fire an unfettered
  global search that teleports the map pose.

Some of this partially supersedes the "local recovery only" limitation below (the global
search now handles lift-and-carry). All new params are live-settable and whitelisted;
committed in `71f1462`, board verified (11 occupancy/recovery tests pass, deploy clean).

**KEY LIMITATION (pre-2026-08-11) — local recovery only, not global kidnap.** The wide match searches only
±`recover_lin` (~0.5 m) around the prior; heading is rescued by the IMU-yaw delta applied in
`_predict` (so even a big carry-rotation recovers, since `/imu/euler` tracks it), but a
translation beyond ~0.5 m won't relocalize (no whole-map search — too costly on the H5). So
it handles bumps / slips / lift-and-replace-near-here, not "carried to another room."
(Largely addressed by the 2026-08-11 kidnap-gated global search.)

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
