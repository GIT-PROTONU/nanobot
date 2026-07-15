---
name: cooling-fan-control
description: SBC cooling fan = ESP32 PWM actuator on /fan_pwm, driven by sys_monitor's CPU-temp curve; web UI slider overrides via the fan_override param
metadata:
  node_type: memory
  type: project
  originSessionId: ce9593fa-fa1a-494e-a534-d9fded658468
---

Added (2026-06-24) a **SBC cooling fan** controlled by the ESP32 coprocessor and policed
by the SBC. Split of responsibility (matches the [[lds-scan-path-sbc-direct]] / LDS-PID
pattern): **ESP32 is a dumb PWM actuator, the SBC owns the policy.**

- **ESP32 firmware** ([[esp32-coprocessor]]): subscribes `/fan_pwm` (std_msgs/Float32, duty
  0..1) and drives LEDC **CH_FAN=5 on GPIO22** (free; default I2C SCL, unused here). Drive
  the fan through a **logic-level MOSFET** — the ESP can't source fan current. Subscriber
  needs NO liveliness token (only publishers do). `#define`s inline in `src/main.cpp`.
- **SBC** (`sys_monitor` / `monitor_node.py`, runs inside [[sbc-cpu-profile]]'s sensor_hub):
  publishes `/fan_pwm` once per `_tick` (1 Hz). **Temperature-driven, NOT CPU-load-driven**
  (the thermally-correct choice; flip to `cpu_pct` in `_publish_fan` if load-based is ever
  wanted). Fails safe to `fan_max_duty` if the thermal zone is unreadable (NaN). All params
  in `robot.yaml` under `sys_monitor:`, retunable live via `/param` (not persistent across a
  `sys_monitor` restart — `robot.yaml` is the durable source of truth).
- **Web UI override**: a "Cooling fan" panel (Auto checkbox + Fan starts at / Floor duty /
  Smoothing sliders + Override % + live duty readout). Auto on → sets param
  `fan_override = -1` (curve). Auto off → `fan_override = 0..1` forced. Sliders call
  `/sys_monitor/set_parameters` via `POST /param`; `/fan_pwm` rides the telemetry frame for
  the readout. `syncFan()` pushes the UI state on (re)connect.

## 2026-07-15 batch: fan wired up for real, curve tuned + firmware park fix

The physical fan (previously disconnected on purpose, see below) got wired up for real this
session. Three follow-on fixes, all in one sitting:

1. **Dead-band floor + off-below-threshold curve.** First pass added `fan_min_duty` (0.30)
   as a floor so the ramp doesn't spend its low end spinning the fan too weakly to move air
   (same issue the wheel motors had, `MOTOR_MIN_DUTY`) — but the first implementation
   applied that floor as the *idle* duty below `fan_temp_min` too, so the fan never actually
   turned off (user caught this: "now the fan never turns off because the minimum is 30
   percent"). Fixed `_publish_fan` in `monitor_node.py`: fully **0 duty below
   `fan_temp_min`**, jumps to `fan_min_duty` right at that threshold, ramps linearly to
   `fan_max_duty` by `fan_temp_max`. `fan_temp_min` moved 45→50°C per user request.
2. **EMA smoothing** (`fan_smooth_alpha`, default 0.15) on the auto-curve target duty — user
   reported the fan "changing speeds quite often," i.e. tick-to-tick CPU-temp noise made it
   audibly hunt. Manual override still applies instantly (deliberate user action bypasses
   the EMA); the EMA state is kept in sync during override so auto resumes smoothly.
   `fan_min_duty`/`fan_smooth_alpha` both added to the web `/param` whitelist +
   Cooling-fan-card sliders (Floor duty / Smoothing), same live-tune pattern as
   `fan_temp_min`.
3. **Firmware: fan now parks on SBC absence** (`firmware/nanobot_coprocessor/src/main.cpp`).
   User: "when the sbc is off but power still on fan still runs, should be off." Root cause:
   the fan intentionally had NO cmd watchdog (`g_fan_duty` just held its last commanded value
   forever on link loss) — useful for surviving a brief app-restart, but wrong for a genuine
   SBC shutdown, since the fan then ran forever off the last-known duty. Fixed to gate on
   `alive` (`linkAlive()`) exactly like the LDS spin-motor park: 0 duty whenever the link
   isn't alive (boot race, drop, or genuine SBC-off), resumes instantly once `sys_monitor`
   reconnects. `FAN_BOOT_DUTY` changed 0.4→0.0 (now moot in practice — the alive-gate zeroes
   it within one 100 Hz control tick of boot). Built + flashed via `pio run -t upload` over
   `/dev/ttyUSB0` (dev PC) — this is a firmware-only change, NOT part of `deploy.sh`.

Curve params (SBC side, `robot.yaml`): `fan_temp_min: 50.0`, `fan_temp_max: 70.0`,
`fan_min_duty: 0.30`, `fan_max_duty: 1.0`, `fan_smooth_alpha: 0.15`. All deployed to the
board (steps 1+2 via `deploy.sh`); step 3 flashed to the physical ESP32 directly.

**Superseded: the 2026-07-14 "fan disconnected on purpose" note below no longer applies —
the fan is wired up now.** Kept for history only:

> 2026-07-14 measurement (fan cmd 100%, CPU 84.9°C, kernel throttling) was NOT a bug to
> chase at the time: user confirmed the physical fan was disconnected on purpose. That's no
> longer the case as of 2026-07-15.

See [[esp32-coprocessor]] for the firmware-side alive/link-liveness machinery this now reuses.
