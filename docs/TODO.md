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
- [ ] **NEW 2026-09-21: intermittent full-duty lunges on corrupted `/cmd_vel` — firmware
      reject-gate + flip-state guard (user deferred: "later TODO", robot manually
      speed-limited).** Symptom (user-confirmed): the robot lunges at ~0.4-0.45 m/s
      (the firmware's ±max_linear clamp regime) in bursts of ~1.5 s, 2-3× in a
      ~40 min session — including **from standstill with NO source** (web drive
      journal shows no POST; Nav2 idle; `/move` inactive; only app_hub's keepalive
      publishes `/cmd_vel`, process list clean). Mechanism: **UART line noise**
      (fan-MOSFET + LDS-motor + drive PWM switching near the 2.4 m ttyS1 link —
      9-s-apart COBS decode bursts in the router journal even while parked,
      10:40/10:57) corrupts an SBC→ESP32 `/cmd_vel` frame **in a still-decodable
      way**; zenoh's serial transport has no payload checksum, rmw_zenoh deserializes
      the garbage Twist, and the firmware's `clampf(±max_linear)` then EXECUTES the
      garbage as a full-speed command. The same noise explains today's repeated
      link drops + ESP32 RX-watchdog `esp_restart()` self-resets (5× today, each
      self-recovered unattended — no power cycle; the old "needs physical power
      cycle" gotcha is partially obsolete on this build). Chained: full-duty lunge
      → worst PWM noise → corruption burst → link drop → watchdog reset.
      **Planned fix (firmware, ~10 lines, deferred by the user):** (1) in `cmd_cb`
      reject any Twist that is non-finite or |linear.x|/|angular.z| beyond a sane
      envelope (e.g. > 2.0 m/s / > 2.0 rad/s — beyond every legit publisher incl.
      the 0.4 web clamp) BEFORE the maxlin/maxang clamp, keep the previous target,
      and count+println rejects on the USB debug serial (UART0, visible via
      `pio device monitor`) so the noise rate becomes observable; (2) in the PID
      tick, re-seed the wheel PID state when |measured vel| > 1.5×g_maxlin
      (physically impossible → catches the direction-flip lock too); (3) ALSO zero
      `wpd_l/wpd_r` in the 2026-09-21 direction-flip reset branches (the 2-window
      velocity history is left stale there — ~40 ms of wrong-sign velocity after
      every flip; at KP 5 + imax=1/ki it can wind to full duty in ~100 ms and lock
      via the commanded-direction tick signing until the wheel physically stops —
      the "instant 0.38 m/s" fwd→rev repro). VERIFY after flash: 5× fwd→rev
      transitions at 0.1 m/s + a ≥5 min drive soak with no lunge, and rejects
      visible on the debug serial when noise hits. **SAFETY STATE 2026-09-21 (pm
      session): maxlin back to 0.15 m/s (the lunge is a LINEAR full-duty event —
      this clamp is its guard) but maxang restored to 0.8 rad/s (the user's
      "turning is slow" was mostly the stale 0.3 NVS clamp + the 0.5 canned-turn
      default; spins/transitions in 8 outback legs fired no lunge with 0.8 live).
      Re-set via `pid_tune.py params --set 3=0.15,4=0.8`; the smoothness-pass-II
      flash (zeroes the flip-stale velocity RING) is the real fix — re-verify the
      5× fwd→rev transition check after flashing, then restore maxlin 0.4.** KFF
      recomputes on param change so the PID still regulates correctly at the lower
      clamp. Related hardware follow-up (not started): the noise SOURCE itself —
      fan/LDS/drive PWM vs the ttyS1 routing (same family as the earlier
      esp32-hardware-fried-ground-fix ground-bounce failure); options: routing/
      shielding/decoupling, or a slower baud — current link is 115200.
- [ ] **Verify the 2026-09-21 wheel-PID smoothness pass on hardware** (flashed +
      deployed 2026-09-21 — five structural fixes, NO gain changes; see the
      "2026-09-21 smoothness pass" block in AGENTS.md). **2026-09-21 first-pass
      results (web-gateway harness, dev PC):**
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
      remains partly stiction-bound (single-channel + carpet) — accepted
      until a 2nd quadrature channel. THREAD CLOSED.**
      **2026-09-21 pm IV: turn ceiling 0.8 → 1.0 rad/s (all three clamps:
      firmware maxang id 4 live, robot.yaml drive_max_ang/move_ang_speed,
      MOVE_ANG_RANGE + slider) — 90° canned turn 4.09 → 3.52 s; smear trade
      11.5°/scan, watch map quality. The 0.4 m/s "stutter" measured = the
      saturation cliff (loaded full-duty ≈ 0.37 m/s; at 0.4 the loop has zero
      authority — reverse leg p2p 0.277 vs 0.028-0.085 at 0.3): the smooth
      cruise band is ≤0.15 m/s, 0.3 acceptable, AVOID 0.35+ — a slider note
      or a web-side soft warning is an option if the user keeps hitting it.
      (2026-09-21 pm: the user hit it — their persisted slider was still 0.4
      and "still stuttering"; set to 0.15 via POST /move/config — 0.15
      measures p2p 0.024-0.056 / mean 91-98% vs 0.4's p2p 0.277. The user can
      still drag it up; the cliff is hardware.)**
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
        (deployed live; 20 s max, offline robots still boot).
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
