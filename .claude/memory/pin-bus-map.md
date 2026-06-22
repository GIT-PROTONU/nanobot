---
name: pin-bus-map
description: Current SBC bus/port + ESP32 GPIO pin assignments for the Nano robot
metadata: 
  node_type: memory
  type: reference
  originSessionId: 9c559fa2-8d06-4b5a-aaf1-f9c937da8fbf
---

Canonical pin map: repo **`nanopi-neo-plus2-pinmap.md`** ("Current project usage"
section), kept in sync with `src/robot_bringup/config/robot.yaml` + firmware
`firmware/nanobot_coprocessor/src/main.cpp`. Key assignments (audited/corrected 2026-06-22):

- **SBC i2c-0** (PA11/PA12): SSD1306 **OLED** @0x3c, bus raised to **400 kHz** (user overlay `i2c0-400k`).
- **SBC i2c-1** (PA18/PA19): PCA9685 @0x40 — **retired/unused** (ESP32 owns motors).
- **SBC `/dev/ttyS1`** (UART1/PG6-PG7): **ESP32 zenoh-pico link** (on-board Bluetooth disabled).
- **SBC `/dev/ttyS2`** (UART2/PA0-PA1): **LDS02RR scan** @115200 → `lds_driver_py` `/scan`.
- **USB**: BWT901CL IMU `/dev/imu` (CH340), Logitech C270 cam+mic `/dev/camera`.
- **ESP32 GPIO**: enc L19/R26, off-ground switches 18/27, motor STBY 23, H-bridge IN L25/4 R32/33,
  LED 2, UART2 link TX17/RX16, LDS data RX=GPIO14 (TX13 unused), LDS spin PWM 21.

⚠️ The older committed memory [[deployment-state]] said the LDS was on `ttyS1` — **stale**
(it's `ttyS2` now; `ttyS1` became the ESP32 link). A correction note was added there. See
[[oled-display-perf]] for the 400 kHz OLED-bus rationale.
