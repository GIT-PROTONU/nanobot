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
- **ESP32 GPIO** (re-verified 2026-07-09 against `firmware/nanobot_coprocessor/src/main.cpp`,
  current since the 2026-07-04 GPIO reassignment — the previous version of this memory was
  stale on this section): enc L=19/R=5, off-ground switches L=4/R=21, H-bridge (DRV8871, one
  per motor, **no STBY/enable pin**) LEFT fwd/rev=26/27, RIGHT fwd/rev=25/33, onboard LED=2,
  UART2 zenoh link TX=17/RX=16, LDS data UART1 RX=14 (TX=13 genuinely free — the LDS only
  streams), LDS spin-motor PWM=18, cooling-fan PWM=22. Board is `esp32dev` (generic 38-pin
  WROOM-32 devkit, all GPIOs broken out).
- **Free ESP32 GPIOs (14 used, board = 38-pin devkit)**: **13, 15, 23, 32** (full I/O, no
  caveats — 15 is a boot-strap pin but harmless after boot), **34/35/36/39** (input-only,
  ADC1, no pull-up/down — sensing only, can't drive a motor/LED/PWM), and **12** (full I/O but
  a boot-strap pin — keep low/floating at boot or it can force the wrong flash voltage mode).
  Reserved/never-usable: 0/1/3 (boot button + USB debug console UART0), 6–11 (internal SPI
  flash, not broken out). So **8 pins are safe to use freely, +1 (GPIO12) usable with care.**

⚠️ The older committed memory [[deployment-state]] said the LDS was on `ttyS1` — **stale**
(it's `ttyS2` now; `ttyS1` became the ESP32 link). A correction note was added there. See
[[oled-display-perf]] for the 400 kHz OLED-bus rationale.
