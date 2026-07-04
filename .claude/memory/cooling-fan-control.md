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
  the fan through a **logic-level MOSFET** — the ESP can't source fan current. **No cmd
  watchdog** on the fan (unlike motors, which stop on `/cmd_vel` timeout): cooling must
  persist. Boots at `FAN_BOOT_DUTY` (0.4) and holds last value on link loss. Subscriber
  needs NO liveliness token (only publishers do). `#define`s inline in `src/main.cpp`.
- **SBC** (`sys_monitor` / `monitor_node.py`, runs inside [[sbc-cpu-profile]]'s sensor_hub):
  publishes `/fan_pwm` once per `_tick` (1 Hz). Auto curve = **linear ramp on CPU
  temperature** (the `cpu-thermal` zone it already reads), params `fan_temp_min` 45 /
  `fan_temp_max` 70 °C → `fan_min_duty` 0 / `fan_max_duty` 1. **Temperature-driven, NOT
  CPU-load-driven** (the thermally-correct choice; flip to `cpu_pct` in `_publish_fan` if
  load-based is ever wanted). Fails safe to `fan_max_duty` if the thermal zone is unreadable
  (NaN). All params in `robot.yaml` under `sys_monitor:`, retunable live.
- **Web UI override**: a "Fan" panel (Auto checkbox + % slider + live duty readout). Auto
  on → sets param `fan_override = -1` (curve). Auto off → `fan_override = 0..1` forced.
  Slider calls `/sys_monitor/set_parameters` over rosbridge (same pattern as the rate
  sliders); `/fan_pwm` is bridged at 1 Hz just for the readout (cheap). `syncFan()` pushes
  the UI state on (re)connect.

**Firmware + SBC side ARE deployed** (updated 2026-07-05): the fan code has shipped in every
firmware flash since late June (the GPIO reassign kept CH_FAN=5 on GPIO22; motors were
confirmed working on this firmware 2026-07-04) and sys_monitor publishes `/fan_pwm` on the
board. Still unverified: the **physical fan/MOSFET wiring** — GPIO22 doesn't appear in
`nanopi-neo-plus2-pinmap.md` (the canonical wiring doc), so confirm a fan is actually hooked
up before trusting the duty readout to mean moving air.
