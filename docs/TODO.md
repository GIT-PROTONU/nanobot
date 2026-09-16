# TODO — Nano robot

Consolidated open work items. The TODO blocks that used to live in AGENTS.md /
CLAUDE.md were folded into this file on 2026-09-16 — those docs now point here.
Delete items as they land (or move them to git history). Named memory notes
(`selftest-spin-imu-mismatch`, `slam-map-rotation-encoder-trim`,
`software-features-todo`) are labels from earlier sessions — the note files are not
in this checkout.

> **NAV2 MIGRATION EXECUTED (2026-09-14)** — `docs/nav2-migration.md` was built and
> live-verified on the dev PC: the custom `slam_nav` stack, the robot_localization
> EKF and the map blob bridge are DELETED; navigation is now Nav2 Humble servers in
> one component container + slam_toolbox 2.6.10 as its own process. slam_nav/EKF-era
> items below are marked MOOT.

## Open — needs the physical robot

- [ ] **Map-vs-room alignment + wheel-odometry scale, validated on open floor**
      (from the AGENTS.md 2026-09-11 "map still skews" thread — the matcher-side
      causes were fixed there, and the slam_nav matcher itself is gone since the
      Nav2 migration; what survives is the physical/drive-chain validation):
      - Re-check `/odom` wheel scale against a measured rollout (ties into the
        `ticks_per_rev` item below); if real tyre scale/slip is a few %, consider
        `wheel_trim`/scale compensation — in manual driving the wheels are the only
        truth, and the map skews by the same %.
      - On open floor (the robot has only been exercised in a confined ~1×1.5 m
        area): clear a fresh map, drive a clean lap by hand, park, and confirm the
        walls line up with the room; a clean second lap must NOT paint a shifted
        mask (walls stay 1-2 cells, coverage grows monotonically).
      - Park → pause: the pose must sit back on the wheel-integrated odom position
        within a few cm — re-check only on a cleanly-built fresh map (the old 9 cm
        idle gap was the boot-into-saved-map frame mis-load, fixed 2026-09-12).
- [ ] **wheel_odometry: verify `ticks_per_rev: 1440` against measured travel.**
      The comment already correctly says single-channel rising-edge (not
      quadrature); the true counts/rev can only be confirmed by driving a measured
      distance on the robot (ties into the odom-autocal backlog item and the
      map-skew item above).
- [ ] **Flash the ESP32 stray-tick firmware** (`/wheel_stray_ticks` +
      `/reset_ticks`, built 2026-07-15, not flashed) — from the dev PC:
      `cd firmware/nanobot_coprocessor && pio run -t upload` (never build on the
      board).
- [ ] **Unblock the encoder trim autocal** (2026-07-16, memory note
      `slam-map-rotation-encoder-trim`). The off-ground microswitches read
      INCONSISTENT polarity (one wheel always reports "suspended"), so the autocal
      gate `!g_susp_l && !g_susp_r` never passes — straight-line trim is running on
      a MANUAL `wheel_trim=0.22` (NVS) instead. Fix options: (a) read raw
      per-switch `digitalRead` idle levels via firmware serial → set correct
      polarity PER WHEEL (not the single `SUSPEND_ACTIVE_HIGH`); or (b) relax the
      autocal gate to key off "straight command + enough ticks" and drop the switch
      dependency entirely; then re-flash and let autocal converge from a reset
      trim. Needs the physical robot.
- [ ] **Re-enable `pickup_pause: true`** (currently `false` in robot.yaml) only
      AFTER the suspension switches read truthfully — until then it false-freezes
      SLAM.
- [ ] **Hardware-verify the 2026-07-13 GPU-vision batch** (code-complete +
      unit/smoke/GL-tested on the dev PC only): named colour targets
      (`vision_targets.json` persist/select/delete), novelty score, camera-freeze
      diagnostic, vibration diagnostic, glare rejection (`vision_glare_derate`),
      OLED mask mirror (`/oled_mask`), vision→behaviour plumbing (anticipatory
      greeting, looming/clutter caution, ambient colour mood, novelty boost), and
      the visual diary (`vision_diary.json` + trend text in the reflect prompts).
- [ ] **Hardware-verify the IMU accel/mag calibration + the new 6-axis mode /
      bandwidth filter / interference self-test** (2026-07-16) — then re-run the
      self-test SPIN check to test the magnetometer-interference hypothesis
      (memory note `selftest-spin-imu-mismatch`, still OPEN). No protocol readback
      exists — verification is eyeballing |accel|≈9.8 + a smooth mag sweep via
      `/imu_calibrate` cmds + `/imu_calibrate_status`.
- [ ] **Tune the LDS spin-motor PID on hardware** (firmware/nanobot_coprocessor —
      the PID holding `/lds_target_rpm` is marked "tune on hardware").
- [ ] **Test cross-host zenoh discovery end-to-end** — `rviz_remote.sh --connect
      <ip>` (the `ZENOH_SESSION_CONFIG_URI` path) was written without a way to test
      it from the dev PC. If `ros2 topic list` on the dev PC doesn't show the
      robot's topics, check the installed `rmw_zenoh_cpp` version's docs for the
      current session-config env var/schema.

## Open — dev-PC / scripts

- [ ] **Create `ros2 topic echo /odom` drift script**: compare odom drift against
      ground truth.

## Deferred / excluded (not planned)

- Overhead-clearance camera-mount geometry check — needs the physical robot's
  mount.
- Docking / cliff items — explicitly excluded by the user.
- LDS keeps spinning during vision tracking — deliberate (tracking rotations count
  as "moved", so the safety lidar never idle-parks while tracking); revisit only if
  tracking sessions turn out long/stationary.

## Standing invariants (must never regress — full context in AGENTS.md)

- `brain_timeout` MUST stay well above `reflect_period` (shorter = the chart
  reverts accumulated trait drift during normal quiet; bit us once at 90 < 600).
- `behavior.quiet_start`/`quiet_end` and web_control `quiet_start`/`quiet_end`
  yaml windows must be kept in sync.
- `deploy.sh` does NOT push nanobot-brain — after brain-repo changes, rsync
  `brain/src` AND `brain/skills` to the board yourself.
- Any raw ROS field added to the telemetry frame must be JSON-safe after the
  rmw_zenoh round-trip (`DiagnosticStatus.level` arrives as `bytes`; normalize at
  ingest in `telemetry._on_diag`).
- ESP32 diff-drive limits stay synced to `robot.yaml`.
- `NAV_INFLATION_M` in `telemetry.py` must mirror `inflation_radius` in
  `config/nav2/nav2_params.yaml` (it is NOT read live).

## Resolved / retired (kept for the trail)

- [x] **Nav2 migration deployed to the board** (DONE 2026-09-14/15 — build +
      `sbc-setup.sh` unit set + live-verified; see `docs/nav2-migration.md`).
- [x] **Map view + click-to-goal + Locations rebuilt on Nav2** (2026-09-15,
      dev-verified — see AGENTS.md).
- [x] **TTS shutdown-cutoff fix deployed** — `TtsEngine.wait(timeout=)` landed
      with the 2026-07 deploys (the 2026-07-15 "not yet deployed" note in the
      pre-consolidation CLAUDE.md was stale).
- [x] **All code-side items from the 2026-07-16 review pass implemented**:
      slam_nav map-rotation drift fix (HW-verified), straight-line drive via
      manual `wheel_trim=0.22`, goal-latch-while-tracking, `brain_timeout` default
      1800 + runtime clamp, telemetry `_mk_goal` ±12 m clamp, slam_nav self-test
      constants promoted to live params, narrative-skill offline phrase-bank
      fallback, docs drift guard.
- [x] **slam_nav teleport/lost-storm recovery hardened** (2026-08-11, `71f1462`)
      and **web map Clear/Home/Save/Test/Stop buttons** rewired to the SSE gateway
      (2026-08-11).
- Retired with slam_nav (2026-09-14): `loop_alpha 0.1` → 0.3–0.5 + tighter
  `loop_apply_thresh` / more frequent `loop_probe_every`; the rotated-seed check
  (`rot_from`/`_seed_pth` vs odom yaw after a clean 360°); the
  `pos_tol_sparse`/`head_tol_sparse` retunes. Full historical trail: the "map
  skew" block in AGENTS.md.
- MOOT: sync dev-repo `ekf.yaml`/`robot.yaml` heading config — the EKF and
  `ekf.yaml` were deleted in the Nav2 migration; no `use_imu_yaw`/ekf references
  remain in `robot.yaml`.
- MOOT: EKF yaw process-noise tuning, slam_nav scan-matching logger,
  `/scan_bias`, `/scan_quality_metrics`, `/scan_matching_quality`, and the
  slam_nav vision pan-track loop (vision is expression/behaviour-level only).
