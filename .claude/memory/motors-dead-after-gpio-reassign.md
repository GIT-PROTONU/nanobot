---
name: motors-dead-after-gpio-reassign
description: 2026-07-04 motors dead after e60e99e GPIO reassign; drivers are 2x DRV8871 (no STBY — that theory dead); ALL 4 PWM channels incl. unchanged GPIO25 produce no motion → suspect VM power/ground/low battery; one ESP32 reboot seen mid-test
metadata: 
  node_type: memory
  type: project
  originSessionId: 59a6f545-92fd-428c-bf44-72d88fcd903b
---

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
