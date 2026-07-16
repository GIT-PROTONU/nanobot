# Improvements TODO

Findings from the 2026-07-16 full-code review. Items here are *code/robustness
improvements spotted in review* — the separate feature backlog lives in
`.claude/memory/software-features-todo.md`, and open investigations in the other
memory files. Delete items as they land (or move them to git history).

## Pending deploy / verify (built, not yet on hardware)

- [ ] **Flash the ESP32 stray-tick firmware** (`/wheel_stray_ticks` + `/reset_ticks`,
      built 2026-07-15, not flashed).
- [ ] **Deploy to the board**: `goal_no_path_timeout` LDS-idle fix + the TTS
      shutdown-cutoff fix (`TtsEngine.wait`) — both committed 2026-07-15, not deployed.
- [ ] **Hardware-verify the IMU accel/mag calibration** (2026-07-16) — then re-run the
      self-test SPIN check to test the magnetometer-interference hypothesis
      (`selftest-spin-imu-mismatch` memory, still OPEN).
- [ ] **Hardware-verify vision target tracking** (2026-07-16, pan-only) — needs a
      calibrated colour target + `enable_motion` + `track_enable`.

## Correctness / robustness

- [ ] **slam_nav: a goal set while tracking is active stays latched.** `_control`
      returns before goal handling while `track_enable` is on, but `_on_goal` still
      latches `_goal` — so the robot silently drives off to a stale goal the moment
      tracking is disabled. Either reject goals while tracking (log + drop) or clear
      `_goal` when tracking turns off (mirror of what enabling already does).
- [ ] **mood_node: `brain_timeout` code default is 90 s** (`mood_node.py` param
      declaration) — the exact value that once violated the documented invariant
      (`brain_timeout` MUST stay ≫ `reflect_period`; robot.yaml correctly uses 1800).
      Raise the code default and/or add a runtime guard that clamps
      `brain_timeout` to ≥ 2×`reflect_period` with a warning, so a param-less launch
      (dev harness, future configs) can't reintroduce the personality-revert bug.
- [ ] **wheel_odometry: `ticks_per_rev: 1440` is annotated "x4 quadrature" history**,
      but the encoders are single-channel rising-edge — the true counts/rev may be off
      by 2–4×, which would scale all odometry distances. Verify against measured
      travel (ties into the odom-autocal backlog item).
- [ ] **telemetry `_mk_goal` doesn't bound goal coordinates** — a browser can POST a
      goal far outside the 24 m map. Harmless since `goal_no_path_timeout` now reaps
      it, but a cheap clamp to the map extent would fail fast instead of 20 s later.

## Nice-to-have / polish

- [ ] **slam_nav self-test constants (`TEST_*`) are module constants**, not params —
      fine today, but the SPIN-check investigation keeps retuning them by edit;
      params would allow live tuning like everything else.
- [ ] **Narrative skills have no offline fallback** (known gap, documented in
      CLAUDE.md): a named `say`/`observe`/`look` skill goes silent without the LLM,
      unlike the generic musing beat (phrase bank first). Could route through
      `bank_say` per situation.
- [ ] **LDS keeps spinning during vision tracking** — tracking rotations count as
      "moved", so the lidar never idle-parks while tracking, though tracking itself
      is camera-only. Deliberate for now (motion should keep the safety lidar hot);
      revisit if tracking sessions turn out to be long/stationary.
- [ ] **Docs drift guard**: the model-tier table in `docs/brain.md` went stale when
      robot.yaml's model slugs changed. Keep concrete model slugs ONLY in robot.yaml
      (done in the 2026-07-16 doc pass); prefer config-key references in prose.
