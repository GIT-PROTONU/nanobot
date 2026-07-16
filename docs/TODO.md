# Improvements TODO

Findings from the 2026-07-16 full-code review. Items here are *code/robustness
improvements spotted in review* — the separate feature backlog lives in
`.claude/memory/software-features-todo.md`, and open investigations in the other
memory files. Delete items as they land (or move them to git history).

All code-side items from the review pass have been implemented (2026-07-16). What's
left needs the physical robot and can't be closed from a dev host.

## Still needs a physical robot (can't be closed from a dev host)

- [ ] **Flash the ESP32 stray-tick firmware** (`/wheel_stray_ticks` + `/reset_ticks`,
      built 2026-07-15, not flashed).
- [ ] **Deploy to the board**: `goal_no_path_timeout` LDS-idle fix + the TTS
      shutdown-cutoff fix (`TtsEngine.wait`) — both committed 2026-07-15, not deployed.
- [ ] **Hardware-verify the IMU accel/mag calibration** (2026-07-16) — then re-run the
      self-test SPIN check to test the magnetometer-interference hypothesis
      (`selftest-spin-imu-mismatch` memory, still OPEN).
- [ ] **Hardware-verify vision target tracking** (2026-07-16, pan-only) — needs a
      calibrated colour target + `enable_motion` + `track_enable`.
- [ ] **wheel_odometry: verify `ticks_per_rev: 1440` against measured travel.** The
      comment already correctly says single-channel rising-edge (not quadrature); the
      true counts/rev can only be confirmed by driving a measured distance on the
      robot (ties into the odom-autocal backlog item).

## Done (2026-07-16)

- [x] **slam_nav: a goal set while tracking is active stays latched.** `_on_goal` /
      `_on_go_home` now reject goals while `track_enable` is on (log + drop) instead
      of silently latching a stale `_goal`.
- [x] **mood_node: `brain_timeout` code default was 90 s**, violating the documented
      invariant against the 600 s `purpose_period` default. Code default raised to
      1800 (matches robot.yaml) and `MoodNode.__init__` now clamps `brain_timeout` to
      >= 2x `purpose_period` at runtime with a warning, so a param-less launch can't
      reintroduce the personality-revert bug.
- [x] **telemetry `_mk_goal` didn't bound goal coordinates.** Clamped to
      +/-`GOAL_MAX_ABS_M` (12 m, half the default 24 m map) so an out-of-map goal
      fails fast instead of waiting on `goal_no_path_timeout`.
- [x] **slam_nav self-test constants (`TEST_*`) were module constants.** Promoted to
      live params (`test_lin`/`test_ang`/`test_dist`/`test_turns`/`test_settle`,
      declared in robot.yaml), whitelisted for `POST /param` like `track_*`.
- [x] **Narrative skills had no offline fallback.** `CognitionCore._invoke_skill` now
      tries the sensor-classified phrase bank before requiring the LLM for non-camera
      `say`/`observe`/`look` skills — see CLAUDE.md's skill-library section.
- [x] **Docs drift guard**: concrete model slugs live only in robot.yaml; prose uses
      config-key references (done in the 2026-07-16 doc pass).

## Deliberate, no change

- **LDS keeps spinning during vision tracking** — tracking rotations count as
  "moved", so the lidar never idle-parks while tracking, though tracking itself is
  camera-only. Deliberate (motion should keep the safety lidar hot); revisit if
  tracking sessions turn out to be long/stationary.
