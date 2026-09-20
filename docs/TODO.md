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
      - **2026-09-19 SLAM diagnosis (live session)**: the old map was incoherent
        dust (852 scattered occ cells; live-scan-vs-map hit-rate 33% at ANY rigid
        offset — sensor itself was clean, 1-3 cm scan-to-scan parked). Root causes
        found: teleop/Nav2 rotation rates (34°/17° of sweep smear per scan —
        slam_toolbox has NO deskew; each scan covers a full 0.2 s revolution) +
        jerk-stall drive chain (above) + an oversized dust-fed pose graph. Fixed
        in config (deployed live 2026-09-19): `drive_max_ang 3.0 → 0.8`,
        `rotate_to_heading_angular_vel 1.5 → 0.5`, slam `minimum_travel_heading
        0.17 → 0.35` (fewer blurred nodes), `correlation_search_space_dimension
        0.5 → 0.8`. Fresh-map parked hit-rate after the fix: **96-98% at zero
        offset with a sharp peak** (was 15-33%); after a stall-contaminated
        out-and-back: 0.84-0.86 (one bad lock strip from the OUT jerk). Also:
        the board's no-RTC clock steps (~44 h at power-on, 2026-09-19 12:00:53)
        destroy live SLAM sessions — a bounded NTP wait now gates `unit_exec.sh`
        (deployed live; 20 s max, offline robots still boot).
      - RE-VALIDATE now that the firmware kick is flashed (2026-09-20): restart
        nano-slam (fresh map — no map_file_name is configured, so a restart IS a
        clear), drive a clean lap, confirm walls line up with the room, a second
        lap doesn't paint a shifted mask, and the out-and-back map-pose tracking
        has no bad locks (the BACK direction tracked perfectly both sessions;
        the stall-jerk FWD runs did not — recheck with working drive chain).
      - Re-check `/odom` wheel scale against a measured rollout (ties into the
        `ticks_per_rev` item below); if real tyre scale/slip is a few %, consider
        `wheel_trim`/scale compensation — in manual driving the wheels are the only
        truth, and the map skews by the same %.
      - Park → pause: the pose must sit back on the wheel-integrated odom position
        within a few cm — re-check only on a cleanly-built fresh map (the old 9 cm
        idle gap was the boot-into-saved-map frame mis-load, fixed 2026-09-12).
- [ ] **wheel_odometry: verify `ticks_per_rev: 1440` against measured travel.**
      The comment already correctly says single-channel rising-edge (not
      quadrature); the true counts/rev can only be confirmed by driving a measured
      distance on the robot (ties into the odom-autocal backlog item and the
      map-skew item above).
- [ ] **Flash the 2026-09-20 breakaway PUSH rewrite** (`MOTOR_PUSH_*` in main.cpp —
      IN REPO, UNFLASHED; compiled + size-checked on the dev PC, RAM 8.4 % / Flash
      35.6 %): `pio run -t upload` from `firmware/nanobot_coprocessor` with the ESP32
      USB tethered to the dev PC (as for the 2026-09-20 morning flash). Then
      re-validate on carpet with an instrumented drive (RELIABLE-QoS /wheel_ticks
      recorder, 15 Hz): **crawl-start lag target <0.5 s with no stall-kick-stall
      judder** (the flashed 80 ms-kick build measured 1-2 s of judder then a lurch —
      pulses don't sustain enough torque to break static friction; mid-run crawl at
      0.79 duty is already smooth and stops ramp cleanly), then spins + the SLAM
      re-validation item above. Watch the push-cap behavior against a deliberate
      wall/obstacle: the capped push should nudge-and-back-off (doubling to 1.6 s),
      not ram.
- [ ] **Post-flash ESP32 verification** (firmware FLASHED 2026-09-20 on the dev PC —
      `/wheel_stray_ticks` + `/reset_ticks` (built 2026-07-15), the 2026-09-17
      `TRIM_AUTOCAL 1` re-enable, and the 2026-09-19 low-duty breakaway kick;
      `upload_speed = 115200` added to platformio.ini since the default 460800
      handshake failed to verify): on the robot, reset trim to 0 (web Coprocessor
      card or `POST /motor_trim 0`), then drive straight a few seconds
      and let autocal converge — verify it settles on a NEGATIVE trim (the known left veer) and
      the robot tracks straight. Then re-validate crawl/spin maneuvers (Nav2 approach,
      in-place turns) and the SLAM re-validation item above.
      - **2026-09-19 drive-test evidence (SLAM diagnosis session)**: with the flashed
        deadband build (`MOTOR_MIN_DUTY 0.55`), wheels seized ~1.4-2.4 s into every
        command at crawl/mid duty — 0.12 m/s, 0.25 m/s AND 0.5 rad/s in-place spins
        (remapped duty ~0.60-0.83): moved, crawled, froze mid-command for the rest of
        the phase (my test ran full timeouts with the robot frozen). Fix (now flashed):
        `MOTOR_MIN_DUTY 0.55 → 0.70` + a breakaway kick (start-from-stop + 350 ms
        no-ticks re-kick, `MOTOR_KICK_*` in main.cpp).
      - **2026-09-20 carpet re-test (instrumented, RELIABLE /wheel_ticks @15 Hz)**: the
        kick build judders at crawl starts — ~1.2 s of 80 ms pulses before breakaway
        (sometimes 0.35 s — variance), then smooth continuous crawl (5.3 s at 0.79
        duty, zero frozen windows) and clean ramp-down stops. Motivated the PUSH
        rewrite above. Same session: a wheel pressed against an obstacle stays frozen
        through sustained full-duty pushing (expected physics). NOTE: at this deadband
        the slowest nonzero command ≈0.28 m/s physical (the [0.70..1] remap) — there is
        no true slow crawl; manual "slow" driving would need either a lower floor
        (stalls return) or the closed-loop WHEEL_PID path (OFF).
- [ ] **Diagnose the web gateway's intermittent 1-9 s POST stalls** (they chop the
      10 Hz /drive stream → dead-man cut mid-drive → stop → lurch on recovery —
      manual-driving feel depends on this as much as the firmware): 2026-09-20
      evidence — two `POST /drive` clients timed out on connect while TTS was
      speaking (espeak-ng running), and one processed 2.2 s late; board load
      reached 3.95/4 cores. A /proc-based stall trap (`/tmp/stall_trap.sh` on the
      board, 15 min windows, snapshots every app_hub thread's wchan/state on a
      >600 ms stall → `/tmp/stall_*.txt`) was deployed 2026-09-20 ~11:22 — review
      dumps; py-spy needs root (board sudo is passworded), so if /proc wchan is too
      coarse, add an in-process `faulthandler.register(SIGUSR1)` to app_hub (edit +
      restart, no ptrace needed) and SIGUSR1 on stall.
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
      - **2026-09-19: device attitude is GARBAGE right now** — parked robot reads
        roll ≈ +91° / pitch ≈ −134° (SSE `eul`), and device-fused yaw moved only
        ±2° across a session where the robot yawed 69°+ (drive test). Not in the
        SLAM chain (slam_toolbox consumes only /scan + TF) so mapping is unaffected,
        but the web IMU card / drift tool are junk until fixed. Suspect the device
        got into a bad mode (post mag-cal experiments?); try a USB replug /
        power-cycle of the BWT901CL first, then re-check |accel|≈9.8.
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
- LDS spin-motor PID tuning (firmware/nanobot_coprocessor — the PID holding
  `/lds_target_rpm`): seems fine as-is, marked done 2026-09-17; take a closer
  look only if the LDS doesn't turn on when it should.

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

- [x] **ESP32 firmware flashed** (2026-09-20, dev PC, `/dev/ttyUSB0` @115200 —
      stray-tick diagnostic, `TRIM_AUTOCAL 1`, and the low-duty breakaway kick all
      live on the coprocessor; follow-up verification is the open "Post-flash ESP32
      verification" item above).

- [x] **Nav2 migration deployed to the board** (DONE 2026-09-14/15 — build +
      `sbc-setup.sh` unit set + live-verified; see `docs/nav2-migration.md`).
- [x] **Map view + click-to-goal + Locations rebuilt on Nav2** (2026-09-15,
      dev-verified — see AGENTS.md).
- [x] **Feeds-health strip in the Map card** (2026-09-17 — five dots
      `ESP32 · LDS · Odom · TF · SLAM`, one per map-chain link; motivated by a
      "only one scan, map never grew" session where the broken feed had to be
      hunted over SSH. See the AGENTS.md Map block for the diagnosis table).
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
- [x] **Encoder trim autocal unblocked** (2026-09-17): the 2026-07-16
      "converged the wrong way" observation ran under the inverted-polarity
      switch gate (with `SUSPEND_ACTIVE_HIGH false` the `!g_susp_l && !g_susp_r`
      gate only passed while the robot was LIFTED — tuning on free-spinning
      wheels). With the polarity flip verified, the gate means "both wheels on
      the ground" and the loop math is sound negative feedback → `TRIM_AUTOCAL 1`
      re-set in firmware. Flash done 2026-09-20; the straight-drive converge
      check remains open (see "Post-flash ESP32 verification" above).
- MOOT: re-enable `pickup_pause: true` — the param was slam_nav's and died with
  the Nav2 migration (2026-09-14); no SLAM pickup-freeze exists anymore and the
  remaining pickup consumers (mood_node reflex, web snapshot) need no config
  flip. Residual gap to keep in mind: nothing pauses Nav2 while the robot is
  held off the ground (wheels spin → commanded-direction-signed encoder ticks
  corrupt `/odom` + SLAM). If that ever matters, add a pickup gate (ESP32-side
  duty/tick gate or web_control-side) as a new open item.
