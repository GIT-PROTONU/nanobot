---
name: esp32-pid-velocity-pending
description: ESP32 PID WHEEL-velocity controller still pending (blocked on single-channel encoders = no direction). The separate LDS spin-speed PID is already built.
metadata: 
  node_type: memory
  type: project
  originSessionId: f0efed13-ad00-484a-a352-c5160d1be048
---

The user proposed (2026-06-18, as a suggestion) adding a **closed-loop PID wheel
velocity controller** to the ESP32 coprocessor: per-wheel parallel PID at 50–100 Hz
with anti-windup, feedback from encoder tick deltas over Δt, output = raw H-bridge PWM.

**NB — not the LDS PID.** A separate closed-loop PID *was* built (2026-06-19) for the
**spin-lidar motor speed** (`/lds_target_rpm` setpoint → PID on the serial RPM →
`/lds_duty`, with feedforward + anti-windup, in `loop()` at LDS_PID_HZ). That one was
clean because the lidar's RPM feedback is unambiguous. The **wheel** velocity PID
below is still UNBUILT — don't assume "PID done" means the wheels.

**Direction (update 2026-06-21): path (1) is now IMPLEMENTED in firmware.** The ISR
signs each tick by the last commanded wheel direction (`g_left_dir`/`g_right_dir`, set in
`cmd_cb`); `/wheel_ticks` is now signed Int64 (forward +, reverse −). Flashed 2026-06-21
on the `slam` branch so reverse odometry integrates correctly for SLAM. Still blind during
reverse-through-zero / stall / slip / being pushed (the known single-channel limitation).
So the *signed-velocity feedback* the wheel PID needs now exists; the PID itself is still
UNBUILT. Option (2) (wire the 2nd quadrature channel for true feedback) remains the only
fix for the blind cases.

**Also needed before implementing:** encoder CPR (counts per *wheel* rev, incl. gear
ratio) and wheel radius, to map `/cmd_vel` (m/s) ↔ tick-rate setpoints.

**Proposed plan (not yet built):** run PID inside the existing 100 Hz `control_cb`
(not a separate FreeRTOS task — the rclc executor is single-threaded), keep the
`/cmd_vel` Twist interface unchanged, anti-windup via integral clamp + conditional
integration + output clamp, reset integrator on cmd timeout / agent disconnect, add a
feedforward term and a `/wheel_vel` publisher for live tuning, gains in `config.h`.

**UPDATE (2026-06-22): the wheel PID is now IMPLEMENTED in firmware, gated OFF by default
(`#define WHEEL_PID_ENABLED 0`).** Per-wheel feedforward+PI(+D) with conditional integration
+ integral clamp, runs at `WHEEL_PID_HZ` (50) inside the Core-1 control loop, measures
velocity from `g_left/right_ticks` deltas over dt, output → `g_left/right_duty` (applyMotors
unchanged @100 Hz). `cmd_cb` stores per-wheel target m/s (`g_left/right_tgt`) when enabled,
else keeps the old open-loop duty path. Uses `WHEEL_RADIUS 0.0335` + `TICKS_PER_REV 1440`
(both copied from robot.yaml — the firmware doesn't read the yaml). `WHEEL_KFF = 1/full-scale`
so KP=KI=KD=0 reproduces today's open-loop behavior (safe baseline). No `/wheel_vel` topic
(would mean touching the proven publisher wire-identity table) — instead measured vel is on
the **UART0 debug console** via STATUS_PRINT for tuning. **To use: set WHEEL_PID_ENABLED 1,
flash, then tune KP/KI on hardware.** Still blind on reverse-through-zero/stall/slip (option
(2), 2nd quadrature channel, remains the only fix). Not yet flashed/tuned as of this note.

**Why this matters:** the scaffolding is in; the remaining work is on-hardware tuning (and
deciding whether the single-channel blind spots are acceptable). See [[esp32-zenoh-pico-integration]].
