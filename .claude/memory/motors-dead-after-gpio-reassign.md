---
name: motors-dead-after-gpio-reassign
description: RESOLVED 2026-07-04, TWO stacked causes — crossed fwd pins (real harness LEFT=26/27 RIGHT=25/33) + ~60% PWM stiction deadband (fixed by MOTOR_MIN_DUTY remap in writeSide); tank turns confirmed working
metadata: 
  node_type: memory
  type: project
  originSessionId: 59a6f545-92fd-428c-bf44-72d88fcd903b
---

**FULLY RESOLVED 2026-07-04 (user-confirmed working, incl. tank turns). TWO stacked causes:**

1. **Crossed fwd pins** (below) — fixed, flashed, motors then moved but ONLY at the full
   0.4 m/s web-UI setting, and in-place turns did nothing.
2. **Stiction deadband**: duty is normalized to full-scale wheel speed
   `MAX_LIN + MAX_ANG*SEP/2 = 0.64 m/s`, so 0.4 m/s = 62% duty (barely moved) and a
   full-stick turn = ±0.24 m/s = 37% duty per wheel (below stiction → no turn). Fix:
   `writeSide()` remaps |duty| in (MOTOR_DEADZONE..1] → [MOTOR_MIN_DUTY..1]
   (0.55/0.02 defines next to INVERT_*). Trade-off: slowest crawl = whatever 55% duty
   gives — tune MOTOR_MIN_DUTY down toward the real stiction point; the proper slow-speed
   fix stays the blocked wheel PID ([[esp32-pid-velocity-pending]]).

Still unverified: why both suspend switches (pins 4/21) read "suspended", and the one
ESP32 reboot seen mid-test (possible brownout).

--- pin-crossing diagnosis (cause 1) ---

**Root cause (user-confirmed):** the e60e99e pin table was wrong. Actual harness:
**LEFT motor = GPIO 26+27, RIGHT motor = GPIO 25+33** (two DRV8871 boards, one per motor —
no STBY/enable pin, so the removed MOTOR_STBY code was irrelevant). Firmware had left=25/27
right=26/33, i.e. the two fwd lines crossed sides → every command PWM'd one input on each
driver → no drive. Fix in `main.cpp`: LEFT_IN_FWD 26 / LEFT_IN_REV 27 / RIGHT_IN_FWD 25 /
RIGHT_IN_REV 33 (fwd/rev within each pair is a guess — flip INVERT_LEFT/INVERT_RIGHT if a
wheel runs backward). Flashed + drive-tested 2026-07-04 — motors moved, but only at full
scale until cause 2 (deadband) was also fixed. CLAUDE.md + nanopi-neo-plus2-pinmap.md updated.

--- original diagnosis (superseded) ---

**Open issue (2026-07-04):** joystick/`/cmd_vel` doesn't drive the wheels, starting right
after commit `e60e99e` (reassign ESP32 GPIOs: motors→25/27/26/33, LDS PWM→18, right
suspend→21, right encoder→5).

**Motor drivers are TWO DRV8871 boards, one per motor** (user confirmed). DRV8871 has NO
standby/enable pin — the `MOTOR_STBY` (GPIO23) code removed in e60e99e was genuinely unused,
NOT the cause. DRV8871 facts that matter: inputs IN1/IN2 with internal pulldowns; **UVLO
cuts outputs below ~6.5 V VM** (low battery = silently dead motors); sleeps when both IN low.

Verified live on the board:
- ESP32 alive + **RX path works**: streaming `/lds_target_rpm 300` → lidar spun to 299.9 RPM
  → new firmware IS flashed and new LDS wiring (GPIO18) is correct.
- **All four motor channels dead**: forward (25+26), reverse (27+33), rotate +z (27+26),
  rotate −z (25+33) each streamed @10 Hz → `/wheel_ticks` stayed ~0, no motion. Crucially
  **GPIO25 (left fwd) kept the SAME assignment as before the rewire** → a per-wire mistake
  can't explain it; the cause is COMMON: VM power / ground to both DRV8871s, or battery
  under the 6.5 V UVLO, or the whole motor harness re-pinned differently than the firmware.
- One **unexplained ESP32 reboot** mid-testing (tick counters reset, heartbeat restarted) —
  possible supply dip/brownout; NOT reproducible on later bursts (hb stayed monotonic).
- Both `/[left|right]_wheel_suspended` read **true** the whole session — robot lifted, or
  switches not landed on new pins 4/21 (INPUT_PULLUP + ACTIVE_HIGH → unwired reads
  "suspended"). Doesn't gate motors (publish-only; slam_nav pickup halt is one-shot) but
  freezes autonomous nav.

**Next physical checks:** battery voltage (≥6.5 V at DRV8871 VM), VM+GND actually landed on
both driver boards, common ground ESP32↔drivers, IN wiring L=25/27 R=26/33, motor leads on
OUT terminals.

**Gotcha:** a single `ros2 topic pub --once` to the ESP32 (zenoh-pico serial) can be silently
lost — stream with `-r`/`-t` when testing. Also `/lds_target_rpm` had been zeroed since boot
(boot default 300; probably the web-UI slider) — left at 0, so lidar/map stay off until
re-spun. See [[esp32-zenoh-pico-integration]] and [[pin-bus-map]].
