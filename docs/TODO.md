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

- [x] **2026-09-21 (code DONE + FLASHED + A/B'd 2026-09-22): wheel-PID adaptive-filter
      N hysteresis.** Residual drive roughness ("better than this morning but still
      not smooth"). IMPLEMENTED (main.cpp `hystN()` + id 7, telemetry.py gate
      0..7/16-slice, AGENTS.md id list), flashed, A/B'd on the live robot:
      **inconclusive** — crawl 0.05 p2p 0.0216 OFF vs 0.0242 ON, boundary 0.055 0.0335
      vs 0.0285, 0.12 0.0218 vs 0.0290, no regression; worst legs both sides are
      DEADMAN input-delivery freezes (not filter flicker). Keep id 7 = 0.15 (NVS).
      Full detail in
      [`docs/wheel-pid-hysteresis-plan.md`](wheel-pid-hysteresis-plan.md),
      including the three REJECTED candidate fixes (#1 slew-lowering —
      already in place via the 50 Hz `slewTo`; #2 KI-lowering — contradicts
      the live sweep; #4 flip-reset deadband — costs sub-crawl reverses)
      held in reserve with their tradeoffs. Do not apply those blindly.
- [ ] **NEW 2026-09-21: LDS idle spin-down + jam guard — built + unit/smoke-tested,
      needs the robot.** The spin-down controller (`web_control/telemetry.py`
      `_lds_ctrl_tick`, owns `/lds_target_rpm` again after a week of nobody) and
      the firmware jam guard (`main.cpp` `ldsControl` — 6 s of "target set but
      can't spin" ⇒ latched motor park, `/lds_jam`) are code-complete; see the
      "LDS idle spin-down + jam guard (2026-09-21)" block in AGENTS.md. VERIFY on
      the robot: (1) deploy + `pio run -t upload`, then let it sit ≥90 s — `f.lds`
      state goes parked, spin 0 rpm, Lidar-card **last move** timer keeps counting,
      `/scan.bin` goes stale, Map feeds LDS dot amber (not red); (2) drive by
      hand (or send a goal) — lidar wakes to ~300 rpm within ~1 s of the first
      `/cmd_vel`, last-move resets; (3) block the rotor gently with a finger/
      string at Spin 300 → within ~6 s `f.lds.jam` true, state **JAM**, motor CUT
      (duty 0) — confirm no grind, and that setting Spin 0 clears the latch (a
      motion resumption retries at most once); (4) the IMU interference test
      still runs its lds phase cleanly (the controller is held for the whole
      run); (5) ESP32 power-cycle while parked → NVS-restored target 0 → no boot
      spin (first flash ever writes `ldstgt` on the first setpoint change).
      NVS note: `pio run -t upload` does NOT erase NVS — the new `ldstgt` key
      just appears on first save.
      **PERSISTENT (2026-09-21, dev-verified): the Lidar card's spin-down settings
      survive a restart/reboot** — `web_server._persist_lds_params`
      (`add_on_set_parameters_callback`) snapshots the whole cluster
      (enable/secs/manual_secs + the Spin slider's active rpm) to
      `~/.local/state/nanobot/lds.json` on any setter, boot re-applies it over
      robot.yaml before TelemetryHub, and `f.lds` carries `enable`+`secs` so a
      fresh page re-seeds its controls. Unit-tested (`test_lds_persist.py`) +
      smoke-covered (`POST /param lds_idle_secs=123` → lds.json).
- [x] **NEW 2026-09-21: intermittent full-duty lunges on corrupted `/cmd_vel` — firmware
      reject-gate + flip-state guard.** **2026-09-22: FIX BUILT + FLASHED + DEPLOYED +
      DRIVE-VERIFIED — CLOSED.** `cmd_cb` now (1) REJECTS any Twist that is non-finite
      or |linear.x|/|angular.z| > 2.0 (`CMD_REJECT_LIN/ANG` defines) BEFORE the
      ±maxlin/maxang clamp — keeps the previous targets, does NOT pet the cmd watchdog,
      counts + rate-limited-prints the reject on the USB debug serial (`[nano] cmd_vel
      REJECT … (N total)`, ≤1 line/s = the observable noise rate); (2) the PID tick
      re-seeds a wheel's PID state + ring when |measured vel| > max(1.5×live maxlin,
      `WHEEL_VEL_PHYS_MAX` 0.55 m/s — floored above the ~0.464 m/s physical ceiling).
      The flip-state part was already covered by the smoothness-pass-II ring zero.
      **ACCEPTANCE RUN 2026-09-22 17:5x (on battery): 10+ fwd→rev transitions @0.1 m/s
      (outback --reps 5 ×2) + a ~7 min soak across 0.05-0.15 m/s (crawl ×6, full
      ladder, outback 0.15 ×5, reverse rung) — ZERO lunge events** (no mean anywhere
      near the 0.37-0.45 m/s clamp band; max mean 0.147 @ 0.15 commanded), instant
      breakaways (0.00 s) on every transition, two benign 0.4 s deadman blips (the
      known delivery-stall family, self-recovered). maxlin RESTORED to 0.4 and stays.
      Open follow-ups (not part of this item): the noise SOURCE (fan/LDS/drive PWM vs
      the ttyS1 routing — routing/shielding/decoupling or a slower baud) and the
      2026-09-22 board power-cycles (see below). **2026-09-22
      session note: the BOARD itself power-cycled twice (14:36 — user switching ESP
      power; 16:32 — while idle, right after a sudo command, cause unknown). If the
      16:32 one was NOT a deliberate user power cycle, board-level power
      instability jumps to the top of the hardware-suspect list (same family as
      the esp32-hardware-fried-ground-fix).**
- [x] **Verify the 2026-09-21 wheel-PID smoothness pass on hardware** — **CLOSED
      2026-09-22.** NVS preflight: gains [5, 60, 0] + separation 0.102 + maxlin 0.4 /
      maxang 1.0 / slew 1.5 / dither 0 / vhyst 0.15 survived the reject-gate flash
      AND two more ESP reboots (verified live via `pid_tune.py state` three times).
      The two blocked sub-items ran 2026-09-22 17:5x on battery: **(2) fwd→rev ×10+
      — PASS** (regulates 0.085-0.099 of 0.100 within one frame, no frozen gap, no
      overshoot, breakaway 0.00 s every transition); **(3) crawl 0.05 ×6 — RUN**,
      aggregate p2p **0.014-0.030, means 82-96%** — rougher than the 0.007 pm-III
      baseline session: the first drive session ON BATTERY POWER (USB logic supply
      before) and a different carpet patch; run-to-run flip is the documented
      variance. Not a safety item (no lunge, no stall cluster — one STALL(LR) flag
      on 6 runs); if crawl roughness matters later, A/B the power config first
      (USB-logic + battery-drive vs all-battery) before touching gains. **(5) boot
      baseline seed: exercised across three ESP power-ups today (flash reboot,
      bounce watchdog reset, battery switch) — no first-drive jerk on any.**
      The 2026-09-21 first-pass results (web-gateway harness, dev PC):
      - (1) parked-at-zero bleed: **PASS** — fwd 2.5 s @0.10 → stop → 3 s watch:
        zero ticks after coast-down, /odom delta (0.0, 0.0); no rollback/nudge.
      - (4) `/reset_ticks` while parked: **PASS** — no lurch, ticks re-seed to
        fresh small counts, /odom delta (0.0, 0.0), stray [0,0] untouched.
      - hold-still-on-flat-floor with the page/keepalive open: **PASS** — 90 s +
        30 s soaks, ZERO tick movement (the bleed holds the flat floor).
      - (2) fwd→rev (direction-flip reset): **BLOCKED by the lunge item above** —
        clean runs regulate -0.10 m/s perfectly (flip within one frame, no frozen
        gap, no overshoot), but intermittent runs hit the instant-0.4 m/s
        full-duty event (now its own TODO item; the flip-reset's stale `wpd` is
        suspect). Re-run this check after the guard is flashed.
      - (3) crawl limit-cycling ×3 aggregates: **NOT YET RUN** (session ended on
        the lunge work). Run `pid_tune.py ladder --vlist 0.05 --repeat 3 --secs 6`
        and judge the aggregate vs the 0.009 m/s sweep baseline.
      - (5) boot baseline seed: exercised by today's post-flash driving (no
        first-drive jerk reported); a definitive check needs the next flash/power
        cycle.
      - gains note: live gains read **[5, 50, 0]** (the user's deliberate KI
        60→50 change) and **survived an in-session ESP32 self-reset reboot** —
        gains NVS persistence verified end-to-end. separation id2 **0.102** also
        survived (NVS OK). **pm session 2026-09-21: gains found drifted back to
        [6.8, 10.0, 0] (abandoned-session NVS writes — see the AGENTS.md gotcha)
        and maxlin/maxang at the lunge-guard 0.15/0.3; restored [5, 60, 0] +
        0.15/0.8 via pid_tune.py (verify survived the NEXT flash/reboot).**
- [ ] **Flash + verify the 2026-09-21 smoothness pass II** (**FLASHED 2026-09-21 pm**
      — adaptive velocity filter + stiction-aware I-term + DITHER; gains NVS
      survived the flash: 5/60/0 + params incl. dith id 6 = 0.05. The post-flash
      "robot drove nowhere" event was the ESP32 half-attached-session wedge (NOT
      the firmware) — see the AGENTS.md gotcha: healed by a target bounce
      (ping-watchdog ESP reboot), RX proven motion-free via a no-op
      /motor_params echo. **2026-09-21 pm VERIFICATION (on battery, in-place
      outback suite): crawl 0.05 p2p 0.007 — ~2x BETTER than the pre-flash
      baseline (0.011-0.013) with the DITHER OFF (dither 0.05 measured p2p
      0.016-0.018 = pure added ripple, zero spin benefit -> dith id 6 = 0 is
      the setting; it stays live-tunable); 0.12 band p2p 0.018-0.037 vs
      baseline 0.011-0.060 (worst outlier gone), fwd≈|rev| distances clean,
      NO lunge across 10+ fwd→rev transitions -> maxlin restored 0.4
      (lunge guard retired — the flip-stale-ring fix held). NO ESP drops
      during the whole suite (/esp32_reset stable).       REMAINING: spin-band
      SAG (mean 0.024-0.034 vs 0.041 at ±0.8 rad/s) = the rate-limited
      I-term recovering spin-band stick-slip slowly — WHEEL_I_WIND_RATE
      1.2 → 2.5 **FLASHED + VERIFIED 2026-09-21 pm III: 0.65 rad/s p2p
      0.027 → 0.015 (mean 85 → 88%), 0.8 best leg 95% of target; the band
      remains partly stiction-bound (single-channel + carpet) — a PERMANENT
      accepted limit (2026-09-21, user-decided: the encoders are and stay
      single-channel; no 2nd quadrature channel will ever be wired). THREAD CLOSED.**
      **2026-09-21 pm IV: turn ceiling 0.8 → 1.0 rad/s (all three clamps:
      firmware maxang id 4 live, robot.yaml drive_max_ang/move_ang_speed,
      MOVE_ANG_RANGE + slider) — 90° canned turn 4.09 → 3.52 s; smear trade
      11.5°/scan, watch map quality. The 0.4 m/s "stutter" measured = the
      saturation cliff (loaded full-duty ≈ 0.37 m/s; at 0.4 the loop has zero
      authority — reverse leg p2p 0.277 vs 0.028-0.085 at 0.3): the smooth
      cruise band is ≤0.15 m/s, 0.3 acceptable,       AVOID 0.35+ — a slider note
      or a web-side soft warning is an option if the user keeps hitting it.
      (2026-09-21 pm: the user hit it — their persisted slider was still 0.4
      and "still stuttering"; set to 0.15 via POST /move/config — 0.15
      measures p2p 0.024-0.056 / mean 91-98% vs 0.4's p2p 0.277. The user can
      still drag it up; the cliff is hardware.)
      **2026-09-21 pm: NAV made choppier by the same physics — Nav2's RPP
      cruised at desired_linear_vel 0.25 (the rough band) and its in-place
      rotate-to-heading (0.5 rad/s, translating nothing) tripped the progress
      checker (15 cm / 10 s) → "Failed to make progress" → the BT recovery
      (backup + 90° spin) = the "crazy spin" the user watched; a re-clicked
      goal succeeded. Config fix (nav2_params.yaml, restart-only): desired 0.18
      (smooth band), rotate_to_heading_angular_vel 0.5 → 0.8 (the accepted
      teleop ceiling; tracks 85-95% and halves rotation time), progress
      movement_time_allowance 10 → 15 s. Verified live via ros2 param get.****
      The braked-stop keepalive fix is deployed (firm stops). Spin-band
      map (2026-09-21 pm, dither 0, 5/60/0): 0.5 rad/s (wheels ±0.025)
      mean 92% of target p2p 0.031; 0.65 mean 85% p2p 0.027; 0.8 mean
      60-85% p2p 0.015-0.029 — stiction-bound at EVERY rate; kp 6.5 A/B'd
      no decisive spin gain (reverted to the swept 5) — the I-rate bump is
      the structural fix, not gains. Pre-flash
      baselines for reference (2026-09-21 pm, tuned 5/60/0, maxlin guard
      0.15 for the linear runs): crawl 0.05 p2p 0.011-0.013; spin 0.8 p2p
      0.014-0.032 SAG (mean 0.036-0.038 vs 0.041); straight 0.12 p2p 0.011-0.060.
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
        (deployed live; 20 s max, offline robots still boot). **2026-09-21 pm:
        the wait is NOT enough — a later NTP correction landed mid-session and
        wrecked NAV (future-dated TF stamps → failed map→base_link lookups →
        RPP "collision ahead" ×2 → the "crazy spin" + BT backup failure → goal
        aborted; heal = stack bounce + waking the parked lidar so a fresh slam
        gets scans). Candidate fix: a sys_monitor clock-step watcher
        (monotonic-vs-epoch drift between ticks; step > ~2 s = restart
        nano-slam + nano-nav automatically, logged to health.log). Also:
        arming the lidar rebuild window when /map goes stale while a fresh
        slam runs.**
        **2026-09-22: the clock-step watcher is BUILT, EXTENDED + LIVE-VERIFIED
        END-TO-END.** `sys_monitor`'s `clock_step` detector (epoch-minus-monotonic
        between ticks) restarts BOTH `nano-slam` AND `nano-nav` on a detected step
        (`CLOCK_STEP_RESTART_UNITS` in health_log.py; rate-limited by
        `CLOCK_STEP_RESTART_MIN` 300 s), unit-tested, deployed. Verified live on
        the board 2026-09-22 16:54 (with `systemd-timesyncd` STOPPED so a manual
        `date -s +5` step persists — with NTP RUNNING the daemon reverts a manual
        step within seconds and the 1 Hz detector correctly never sees it):
        detection within 1 s of the step (`drift ...758.80 -> ...763.81`), the
        exact two-unit sudoers rule accepted `sudo -n systemctl restart nano-slam
        nano-nav`, slam restarted in seconds, nav's ~90 s stop completed, both
        units active — no watchdog kill. TWO FIXES LANDED DURING VERIFY:
        (1) the restart is now a DETACHED `Popen(start_new_session=True)` — the
        old blocking `subprocess.run(timeout=45)` expired live at 16:22 (nav's
        stop alone takes ~90 s) AND blocked the executor against the 90 s
        watchdog pet; (2) the sudoers rule for the exact two-unit command is
        INSTALLED on the board (deploy/sudoers/nano-power). GOTCHAS FOUND ALONG
        THE WAY: the health.log contains NUL bytes from the board's abrupt
        power losses — **greps against it need `-a` or "binary file matches"
        HIDES real matches** (this false-negative cost an hour of debugging a
        watcher that was actually firing); `systemctl stop systemd-timesyncd`
        does NOT survive a reboot (re-run after any power cycle); a target
        bounce can hit nano-nav's 3 min start timeout (Result: timeout, the
        container KILLED) — retrying `sudo -n systemctl start
        nano-robot.target` once recovers it. Live lab-procedure note: the
        faithful step test is stop-timesyncd → `date -s` → watch → start
        timesyncd again (the daemon then re-corrects the offset; the watcher
        logs any follow-up step as rate-limited).
      - RE-VALIDATE now that the firmware kick is flashed (2026-09-20): restart
        nano-slam (fresh map — no map_file_name is configured, so a restart IS a
        clear), drive a clean lap, confirm walls line up with the room, a second
        lap doesn't paint a shifted mask, and the out-and-back map-pose tracking
        has no bad locks (the BACK direction tracked perfectly both sessions;
        the stall-jerk FWD runs did not — recheck with working drive chain).
        **2026-09-21: DONE on a fresh map (post-sep-fix)** — see the lap-validation
        bullet under the canned-moves item below: 97.4% scan-vs-map at zero
        translation, no shifted second mask from the repeat lap, residual ~4° pose
        yaw after ~540° of turning. What remains here is only the OPEN-FLOOR
        variant (the exercise area is ~1×1.5 m; a full room-scale lap with longer
        legs still wants more space).
      - ~~Re-check `/odom` wheel scale against a measured rollout~~ **DONE 2026-09-20 —
        the scale was 5.7× wrong, not a few %**: measured rollout (user-timed 5 s crawl)
        gave 2235 ticks / 1.86 m = **1202 ticks/m => `ticks_per_rev` 253, not 1440** (the
        1440 assumption was a quad-vs-single-channel counting error). Physical full-duty
        cruise is ~0.37 m/s loaded — the drive hardware was healthy all along; `/odom`,
        the PID's velocity feedback and `WHEEL_KFF`'s world were all mislabeled, which
        also means every pre-fix "crawl at 0.1 m/s" was really a saturated ~0.37 m/s
        lurch. Fixed in firmware + `robot.yaml` (both wheels' PIDs now regulate in true
        m/s). The SLAM map built on the 5.7×-scaled odom is garbage geometry — a
        `nano-slam` restart (fresh map) is mandatory, done with the 2026-09-20 deploy.
      - ~~`ticks_per_rev` verification~~ **DONE** (253, see above) — and the scale is
        now a LIVE parameter (firmware `/motor_params`, NVS-backed) so a future
        recalibration needs a POST, never a reflash.
      - Park → pause: the pose must sit back on the wheel-integrated odom position
        within a few cm — **2026-09-21 measured on a fresh map**: the map-vs-odom
        gap grew ~5.7 cm per scripted circuit (map→odom absorbing wheel yaw
        residual); the meaningful check is the scan-vs-map offset (97.4% at 0 cm —
        recorded in the lap-validation bullet). Revisit only if Nav2 shows
        pose-vs-room error in practice.
- [x] **Hardware-verify the web canned moves (`POST /move`, 2026-09-21)** — DONE
      same day on the robot: Drive 0.300 m → 362/1202 tpm ticks = **0.301 m
      (+1.3 mm), L/R within 1 tick**; Turn 90° → 152/150 ticks = **90.2°**;
      progress/error rode `f.move` as designed. Remaining eyeball item: the
      replace-while-running + joystick-takeover + browser-dead-man paths are
      logic-tested only (14 offline tests). NOTE: the morning "90.2°" tick check
      was CIRCULAR (divided by the wrong separation — see the 2026-09-21 lidar
      finding below); the turn's REAL physical rotation was ~1.58× over. After the
      separation fix: 90° → **89°** and 60° → **57°** physical by lidar
      self-correlation, so canned turns are genuinely true now.
      - **WHEEL SEPARATION was 1.58× wrong (found + fixed 2026-09-21)** — reported
        live by the user (115°→~180°, 60°→~90° while distance stayed exact), pinned
        by cross-correlating two `/scan.bin` range profiles across canned turns
        (146°/139°/144° physical for 90° requests) ⇒ true track **0.102 m**, not the
        0.16 chassis-width guess. Fixed in firmware (live `/motor_params` id 2,
        NVS-persisted while parked) + `robot.yaml` `wheel_odometry.wheel_separation`
        (the board's install config symlinks through to src — rsync + restart).
        Distance was never affected (scale is radius-side). VERIFY the 0.102
        survived the ESP32's next reboot (`/wheel_params` id 2).
      - **2026-09-21 fresh-map lap validation (post-sep-fix, scripted /move laps)**:
        fresh slam map + 2 laps of {out-and-back 0.3 m, 90° turn, out-and-back}:
        scan-vs-map hit-rate **97.4% at zero translation** (96-98% reference met —
        the map is coherent, no shifted second mask from the repeat lap; map grew
        557→711 occ cells as new headings were explored). Beam angles need the
        back-mount **+π** when projecting scans for this check (73% vs 9% flip test
        — same fact as the old heading_flip). Residual: the SLAM pose's yaw sits
        ~2-4° off scan-truth after ~540° of commanded rotation (~0.7%/turn — carpet
        slip in the odom prior; scans dominate for Nav2, so acceptable). Pose-chain
        gap (map vs odom while parked) grew ~5.7 cm/circuit — expected to be
        absorbed by map→odom; the meaningful metric is the scan-vs-map offset above.
- [x] **Retune the closed-loop wheel-PID in TRUE units (post-scale-fix)** — DONE
      2026-09-21 via `scripts/pid_tune.py` (new harness: serial-safe ladder +
      gains/params pokes over the web gateway, `--repeat N` for the high
      run-to-run variance). **Result: KP 5.0 / KI 60.0 / KD 0, NVS-persisted**
      (baseline [1.1, 46] hunted at every rung, crawl p2p up to 0.05 m/s; KI 60
      alone amplified the mid-speed limit cycle; KP is the damper — KP 5 + KI 60
      won the aggregate: crawl p2p avg 0.009 vs 0.017, no stalls anywhere, all
      rungs break away). Residual carpet-stiction limit-cycling is band- and
      run-dependent (0.08 can be spotless while 0.10 flags on the next repeat) —
      judge aggregates, not single runs. VERIFY the gains survived the next
      reboot (`/wheel_pid` readback should read [5,60,0] after a power cycle).
      NOTE 2026-09-21: the smoothness pass re-flashed the firmware (`pio run -t
      upload` does NOT erase NVS) — confirm the readback still shows [5,60,0].
      (The original five-check list for this item now lives merged with today's
      results in the "Verify the 2026-09-21 wheel-PID smoothness pass" item near
      the top of this section.)
- [ ] **OPEN BUG: /cmd_vel delivery to the ESP32 dies after a router/stack restart —
      Twist-specific, other topics keep flowing.** After `deploy.sh`/`stack.sh`
      restarts the zenoh router, the coprocessor's session re-attaches (heartbeat,
      /wheel_ticks, /lds_* all flow) and SOME subscriptions still deliver
      (/motor_pid write → /wheel_pid readback flips instantly), but /cmd_vel
      specifically goes deaf — observed twice (2026-09-20 pre- and post-flash), with
      web_control's keepalive, the board's `ros2 topic pub`, AND raw zenoh puts on
      the exact keyexpr. A full ESP32 reboot (flash = reboot) restores it until the
      next router restart; the 2026-09-20 fix-sequence was flash-then-deploy, which
      ends with the ESP32 deaf again. Workaround: power-cycle the coprocessor after
      any router restart. Diagnosis leads: the sub declare for the ONE multi-publisher
      topic (cmd_vel has 6 publishers across sessions; every delivered topic has ≤1)
      vs zenoh-pico's re-attach path; compare a router-restart vs ESP32-reboot
      declare table. The old "ESP32 wedged after stack restart" gotcha and this are
      likely the same root cause.
      - **2026-09-22: the HEAL is verified end-to-end (motion-free form)** — full
        `sudo -n systemctl restart nano-robot.target` → all units active, `esp32 UP
        1s after start`, and SBC→ESP RX proven on BOTH sides of the bounce via the
        motion-free /motor_params dither poke (id 6 readback flipped 0→0.02→0).
        Gains [5,60,0] survived the bounce's ping-watchdog ESP reboot too. The
        cmd_vel-SPECIFIC revival timing (does a router-only restart deafen it; how
        fast the bounce revives delivery) still needs the drive-session test below —
        motion is the only observable for cmd_vel delivery.
      - **2026-09-21 FIX FLASHED (deployed same day, robot live)**: the firmware
        **periodically UNDECLAREs + REDECLAREs every
        subscription** (`SUB_REDECLARE_MS 45000`, `SUBS` table + `subsRedeclare()` in
        main.cpp) — the fresh router instance's empty remote-sub table is refreshed
        in place, bounding the worst deaf window at 45 s, no ESP reboot needed. Note:
        zenoh-pico's Arduino build (`ZENOH_C_STANDARD=99` from its extra_script)
        compiles the `z_move` macro out — use the explicit `z_subscriber_move()`
        (same convention as the existing `z_config_move()`). Also confirmed live
        2026-09-21: a full stack restart did NOT always reproduce the bug (cmd_vel
        delivered after this one) — it's intermittent, consistent with a declare
        race, which the periodic re-declare covers. VERIFY: restart the router a few
        times and confirm /cmd_vel revives within ≤45 s if dropped — until then keep
        the power-cycle workaround in mind.
      - **2026-09-21 live-session observations (testing the smoothness pass)**: the
        ESP32 link dropped 5× in ~40 min and **self-recovered every time
        unattended** — the RX watchdog (`LINK_RX_TIMEOUT_MS 8000` → `esp_restart()`)
        re-handshakes on its own (heartbeat counter restarts, ticks resume ~15 Hz);
        NO physical power cycle was needed, unlike the old gotcha. BUT the drops
        today were **noise-driven, not router-restart-driven** (see the new lunge
        item: UART corruption bursts from fan/LDS/drive PWM, 9-s-apart COBS errors
        in the router journal even while parked) — so the controlled
        restart-router-and-time-the-revival test for the ≤45 s re-declare bound is
        STILL open. Also: the 10:05-10:08 router restarts that day were clean
        systemd deactivations (`NRestarts=0` — something REQUESTED them; not
        crashes, not the ExecStartPost probe), with a
        "clock not NTP-synced after 20s" stale-clock window in between; origin
        unidentified. And one CLI gotcha found the hard way: a bare `ros2 topic
        info/echo` from ssh runs under **fastrtps** (the CLI daemon's default) and
        sees NOTHING on the zenoh island — "Unknown topic" is the wrong-RMW
        symptom, not proof a topic is absent; the CLI needs
        `RMW_IMPLEMENTATION=rmw_zenoh_cpp` (and `ros2 daemon stop` after changing
        it).
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
        duty, zero frozen windows) and clean ramp-down stops. Same session: a wheel
        pressed against an obstacle stays frozen through sustained full-duty pushing
        (expected physics). NOTE: at this deadband the slowest nonzero command ≈0.28
        m/s physical (the [0.70..1] remap) — there is no true slow crawl; the
        closed-loop PID above replaces the remap and re-opens the slow range.
- [ ] **Post-flash ESP32 verification** (firmware FLASHED 2026-09-20 on the dev PC —
      `/wheel_stray_ticks` + `/reset_ticks` (built 2026-07-15), the 2026-09-17
      `TRIM_AUTOCAL 1` re-enable, and the 2026-09-19 low-duty breakaway kick;
      `upload_speed = 115200` added to platformio.ini since the default 460800
      handshake failed to verify): on the robot, reset trim to 0 (web Coprocessor
      card or `POST /motor_trim 0`), then drive straight a few seconds
      and let autocal converge — verify it settles on a NEGATIVE trim (the known left veer) and
      the robot tracks straight. Then re-validate crawl/spin maneuvers (Nav2 approach,
      in-place turns) and the SLAM re-validation item above.
      - NOTE 2026-09-20: the wheel-PID build (above) compiles `TRIM_AUTOCAL` OUT (a
        per-wheel velocity PID equalizes the wheels itself) — the autocal-convergence
        check here only applies while the legacy open-loop path is flashed. Under the
        PID, verify straight tracking directly (equal commanded wheel speeds → equal
        measured tick rates).
      - **2026-09-21: straight tracking VERIFIED under the PID** — fwd 0.10 m/s
        ×6 s: per-wheel cruise means L 0.104 / R 0.104 m/s (identical within
        quantization), L/R equal through the whole ramp; a second fwd→rev run
        showed L/R -0.097..-0.110 vs -0.089..-0.102 (≤±5%). Stray ticks [0,0],
        trim 0.0. Remaining sub-items (crawl/spin re-validation) ride on the
        smoothness-pass item above.
- [ ] **Diagnose the web gateway's intermittent 1-9 s POST stalls** (they chop the
      10 Hz /drive stream → dead-man cut mid-drive → stop → lurch on recovery —
      manual-driving feel depends on this as much as the firmware): 2026-09-20
      evidence — two `POST /drive` clients timed out on connect while TTS was
      speaking (espeak-ng running), and one processed 2.2 s late; board load
      reached 3.95/4 cores. A /proc-based stall trap (`/tmp/stall_trap.sh` on the
      board, 15 min windows, snapshots every app_hub thread's wchan/state on a >600 ms stall → `/tmp/stall_*.txt`) was deployed 2026-09-20 ~11:22 — review
      dumps; py-spy needs root (board sudo is passworded).
      - **2026-09-21: the in-process dump is DONE + live** — app_hub now
        `faulthandler.register(SIGUSR1, all_threads=True)` (`app_hub._install_stackdump`),
        so `kill -USR1 <pid>` dumps every thread's stack to journald without ptrace/root
        (verified live: the dump showed the `_man_loop` thread). The stall trap can now
        signal USR1 on a stall and read `journalctl -u nano-app`. Remaining open: WHY
        executor callbacks slip that far under load, and whether POST /drive handling
        itself (HTTP thread) still stalls under load.
      - **2026-09-21 stress reproduction: CLEAN** — 90 s all-core stress + a 16 s
        spoken line while timing `POST /drive {0,0}` at 2 Hz from the dev PC: 191/103
        POSTs, median 32 ms, p95 49-51 ms, max 79 ms, ZERO over 600 ms. The
        2026-09-20 1-9 s POST stalls did NOT reproduce under CPU+TTS load. Also
        caught in the act by the v2 trap (a 1.5 s D-state in an HTTP thread's
        `handle_one_request → readinto`): that one was a benign wifi-path socket
        read — the handler already bounds such reads (`_Handler.timeout = 30`), so
        half-open connections can't pin threads forever; the trap threshold was
        raised to 5 s D-state to cut false positives (trap lives at `/tmp/stall_trap.sh`
        on the board, re-run with nohup after a reboot). Remaining open: the
        executor-side slips under LLM/vision load (the 2026-09-20 wchan dumps), and
        any stall needing BOTH TTS AND heavy vision — next repro attempt should
        combine stress + an active camera/GPU-vision pass.
      - **2026-09-21: the IMU interference self-test was NEVER startable — deadlock
        found + fixed.** `IMUInterferenceTest.start()` held a plain `threading.Lock`
        across its checks + thread spawn and then returned `self.status()`, which
        re-acquires the same non-reentrant lock → self-deadlock (the start POST
        hangs forever, the run thread queues behind it, every status call piles
        up). Found live via the SIGUSR1 faulthandler dump (start handler parked in
        `status()` while holding the with-block). Fix: `RLock` (deployed +
        verified live — the full 5-phase test now runs to completion). Regression
        test: `test_imu_interference.py::test_start_returns_without_deadlock` (+ 3
        more).
      - **The /cmd_vel chop component is FIXED 2026-09-20**: the 10 Hz drive
        keepalive moved OFF app_hub's ROS executor into a dedicated thread
        (`web_server._drive_loop`, os.nice(-5) best-effort, daemon, joined in
        `destroy_node`) — executor slips under TTS/vision/LLM load can no longer
        starve the re-assert past the ESP32's 500 ms watchdog. Remaining open:
        WHY executor callbacks slip that far under load (the stall-trap dumps), and
        whether POST /drive handling itself (HTTP thread) still stalls under load —
        that part is ThreadingHTTPServer, not the executor.
      - **2026-09-22: stalls CONFIRMED LIVE during PID tuning, post scan-poll fix** —
        the outback suite caught real deadman events (both-wheels-freeze ≥0.4 s while
        commanded) on 2 of 4 legs pre-scan-fix and on ~2 of 4 legs in the run right
        after the 2026-09-22 deploy (the deploy's own post-restart settling made 4/4,
        clearing on re-run), so the /scan.bin poll fix reduced but did not eliminate
        them. NOTE the /proc stall trap (`/tmp/stall_trap.sh`) does NOT survive a
        reboot — it was gone when checked this day (re-run with nohup per the note
        above before the next repro attempt). Same session, the tuning instrument
        itself was fixed: see the AGENTS.md ESP32-PID gotcha — after a stall the
        backlogged SSE frames burst in and parse-time dt collapses, inflating
        pid_tune speeds ~6-10× (phantom HUNT / spin-overspeed verdicts); the frame
        now carries a build stamp `"t"` and pid_tune scores against it. One
        unexplained episode (~12:57): spin legs read sustained ~10× tick advance with
        correct 5/60/0 gains, no heartbeat reset, no recurrence in 12 recorded legs —
        re-probe with a passive frame recorder alongside the outback if it repeats.
      - **2026-09-22: COMBINED-LOAD REPRO CLEAN + the executor-slip ROOT CAUSE
        EVIDENCE CAPTURED.** Recreated the trap (5 s threshold, 30 min self-exit) and
        ran stress (90 s all-core) + live GPU-vision + 2 TTS lines while timing
        POST /drive {0,0} @2 Hz from the dev PC: **240 POSTs, median 29 ms, p95
        59 ms, max 290 ms, ZERO over 600 ms** — the HTTP path is clean under the
        full combined load (matches the 2026-09-21 stress+TTS-only clean; the
        dedicated keepalive thread + scan-poll fix hold). BUT the trap fired 7×
        during the run: **the app_hub MAIN thread (the SingleThreadedExecutor spin —
        web_server + oled_display + mood_node share it, hub.py) was caught in
        D-state with `wchan=mv64xxx_i2c_wait_for_completion`** (the Allwinner I2C
        driver's completion wait — the OLED bus, /dev/i2c-0 @400 kHz), several
        episodes ≥5 s. That is the executor-side slip mechanism the 2026-09-20
        wchan dumps were hunting: **an OLED I2C write blocks uninterruptibly on the
        shared executor thread and freezes every callback (telemetry tick, subs,
        params) for the duration** — below the 90 s watchdog, so no restart, just
        slipped callbacks (the 1-9 s POST-era symptom class; today's POSTs don't
        see it because /drive left the executor). ~~FIX CANDIDATES (not started)~~
        **FIXED + LIVE-VERIFIED 2026-09-22 17:4x: oled_display's panel I2C now runs
        on a dedicated worker thread** — a bounded (4) drop-oldest render queue
        (`DisplayNode._submit_draw`/`_draw_loop`); the executor-side timers and
        subscriptions submit closures, the worker executes ALL `canvas()` I2C
        (`_dashboard_render`/`_face_render`/`_draw_word_render`/`_mask_tick` + the
        /oled_system screens); `shutdown_sequence` sets `_draw_stop`, drains the
        queue, and renders the end-screen INLINE (the executor is already stopped
        and the daemon worker may die with the process). A wedged bus now costs a
        stale panel, NOT the shared executor. Unit-tested
        (`src/oled_display/test/test_draw_queue.py`: in-order, drop-oldest-keeps-
        newest, never-blocks, worker-survives-exceptions, stop-flag skip — 5 tests;
        194 total green) and re-verified under the exact repro load (90 s all-core
        stress + GPU vision + TTS + /drive timing): **ZERO i2c wchan in any trap
        dump — the main/executor thread no longer blocks on the OLED bus** (pre-fix:
        7 episodes of ≥5 s D-state in `mv64xxx_i2c_wait_for_completion`); POSTs
        median 40 / max 92 ms. Dumps: /tmp/stall_17*.txt on the board (trap expires
        ~17:4x + 30 min; re-run `nohup /tmp/stall_trap.sh` for more).
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
      - **2026-09-21: device attitude is HEALTHY again** — parked reads roll ~1.5°
        / pitch ~-2.6° (yesterday's garbage +91°/-134° gone after the power
        cycles); |a| = 9.80, |g| = 0.009 @22 Hz. The accel cal looks good.
      - **2026-09-21: the interference self-test now RUNS end-to-end (deadlock
        fixed — see its own bullet) and is CLEAN on the gyro axis**: 5 phases
        (baseline/LDS/fan/LED/motor wiggle) all read yaw wobble <=0.18° — no
        actuator disturbs the gyro. CAVEAT: the mag_noise column read 0.0 in every
        phase because **the mag vector is frozen** ([−487,380,−96] constant) —
        EXPECTED in **6-axis mode** (`imu_driver.axis6_mode` defaults TRUE: no
        magnetometer in yaw by design, the mag near motors/LDS was the old 9-axis
        disturbance source). So the old spin-interference hypothesis is moot in
        this mode; the mag-sweep calibration eyeball only applies if 9-axis is
        ever re-enabled. The test could flag "mag frozen (6-axis)" instead of a
        misleading 0.0 (small code tweak, low priority).
- [ ] **Test cross-host zenoh discovery end-to-end** — `rviz_remote.sh --connect
      <ip>` (the `ZENOH_SESSION_CONFIG_URI` path) was written without a way to test
      it from the dev PC.
      - **2026-09-21 first live test (robot online, dev PC)**: the config schema is
        CONFIRMED (`ZENOH_SESSION_CONFIG_URI` is the right env var for this
        rmw_zenoh_cpp build — grep the .so) and cross-host TCP connect + DATA
        routing WORK: a dev-PC session pointed at `tcp/<robot-ip>:7447` receives
        `/wheel_ticks` (ESP32 publisher) end-to-end. But DISCOVERY is broken
        one-sided: `ros2 topic list` from the dev PC shows ONLY the ESP32's topics
        (+system) — the robot's zenoh-rust ROS nodes (mode: peer, attached to the
        router via loopback) are invisible, so rviz_remote has no /scan, /odom,
        /tf, /map yet. Same blindness on the BOARD itself: a fresh `ros2 topic
        list` there saw 0-2 topics while the ESTABLISHED stack (telemetry's subs,
        made post-boot) flows fine — i.e. the router propagates the zenoh-pico
        CLIENT's declarations to later joiners but not the ROS PEERs'.
        Diagnosis leads: (a) zenoh peer-declaration propagation via the router
        (gossip locators are loopback-only — the "Unable to connect to any locator
        of scouted peer ... tcp/[::1]:..." warnings are the visible symptom); (b)
        the router's peer-table handling after re-attaches (the 2026-09-20
        /cmd_vel-dead family — a router journal line "Read error on Serial link:
        Unexpected Init flag in message" was caught 2026-09-21 = the ESP32
        re-handshaking into a still-established transport). Candidate fix to TEST:
        run the robot's units in CLIENT mode (export a session config with
        `mode: "client"` in unit_exec.sh) so every declaration routes through the
        router exactly like the ESP32's — client-mode sessions demonstrably
        propagate. Also: the ros2 CLI's persistent DAEMON caches a stale graph —
        always `ros2 daemon stop` before trusting a CLI graph view, and a bare
        ssh `ros2` runs under **fastrtps** (the CLI daemon's default RMW) which
        sees NOTHING on the zenoh island — "Unknown topic" is the wrong-RMW
        symptom, not proof of absence; export `RMW_IMPLEMENTATION=rmw_zenoh_cpp`
        first (hit live 2026-09-21 while hunting the /cmd_vel publisher list).

## Open — dev-PC / scripts

- [x] **`ros2 topic echo /odom` drift script** — DONE 2026-09-21:
      `scripts/odom_drift.py` (rclpy, runs on the board or any session that can see
      the graph) reports parked-drift rate (cm/min + degrees while /odom's twist
      says stationary) over streaks, with a summary. Board-run verified end-to-end.

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
- [x] **Web costmap overlay + robot-pose fix + Nav2 params-file repair**
      (2026-09-22, deployed + server-side verified; browser eyeball pending).
      The Map view's red robot dot/heading/inflation bubble NEVER rendered —
      `f.nav.pose` is an `[x, y, yaw]` array and `repaint()` read `p.x/p.y/p.th`
      (undefined → NaN → silent canvas no-op); the trail + pose readout worked,
      so the trail just ended in nothing. Fixed (array indexing). New **Costmap
      toggle** (local/global) on the Map card overlays Nav2's actual costmaps
      through `GET /local_costmap` + `/global_costmap` (telemetry lazy VOLATILE
      subs; `_on_costmap` re-projects the odom-frame local origin into the map
      frame via TF and carries `yaw` for the page's rotated draw; cells are
      Nav2 COSTS 0..255 → yellow→red heat ramp). Deploying exposed that BOTH
      costmaps had silently run on Nav2 DEFAULTS since the 2026-09-14
      migration (launch_ros inlines only per-component name sections into the
      load request, so the double-nested `local_costmap.local_costmap.*`
      sections never reached the child nodes) — fixed with a process-wide
      `--params-file "$NAV2_PARAMS"` on the container (`unit_exec.sh nav` +
      launch parity), which then exposed the int `width`/`height` gotcha
      (doubles abort the component constructors). Both documented in
      AGENTS.md. Live-verified: robot_radius 0.16 / inflation 0.25 now real,
      local costmap the true 40×40 rolling window, both routes fresh at ~1 Hz;
      `always_send_full_costmap: true` on the global costmap added (else the
      full-grid poll freezes on `_updates`). Also: `wheel_pid` had drifted to
      `[0,0,0]` again (NVS gotcha, 3rd occurrence) — restored the tuned
      KP 5 / KI 60 / KD 0 via POST, verified NVS-persisted across two restarts.
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
