---
name: project-overview
description: "What the Nano robot project is, its hardware, and the key stack decisions (refreshed 2026-07-06)"
metadata: 
  node_type: memory
  type: project
  originSessionId: 0ebdfb94-1ddc-4321-994d-4ecc12775e00
---

Nano = a mobile robot built on a **NanoPi NEO Plus2 (Allwinner H5, aarch64, 1 GB RAM)**
running **Armbian**, with a personality (statechart + LLM) layered on top of the ROS stack.

Hardware: Roborock **LDS02RR** lidar (scan → SBC UART2 `/dev/ttyS2`; RPM → ESP32),
an **ESP32-WROOM coprocessor** (motors via DRV8871, single-channel encoders, off-ground
switches, LDS spin PID, cooling fan — native zenoh-pico over UART1 `/dev/ttyS1`, see
[[esp32-coprocessor]]), **SSD1306** OLED (i2c-0 @400 kHz), **BWT901CL** IMU (USB `/dev/imu`),
**Logitech C270** webcam + mic (USB), speaker via the H5's analog codec (espeak-ng TTS).
The **PCA9685** is retired (ESP32 owns motors).

Stack decisions (current truth, 2026-07-06):
- **pixi + RoboStack** (ROS 2 **Humble** as conda packages, channel `robostack-staging`), no apt.
- Middleware **`rmw_zenoh`** (RAM) — router = a custom **serial-capable zenohd** so the ESP32
  joins the graph directly ([[robostack-zenoh-no-serial]]).
- **All Python (rclpy).** The Rust r2r LDS driver is abandoned (doesn't build here; its
  ~1.6 GB toolchain must NEVER return to pixi.toml).
- **No rosbridge.** The browser talks only to `web_control`'s HTTP server: SSE `/telemetry`
  + whitelisted `POST /publish|/param|/drive`, heavy data via `/dev/shm` blobs
  ([[architecture-two-planes-three-hubs]]).
- On the board the nodes run as **three single-process hubs** matching fault domains —
  `sensor_hub` (imu+sys+odom+lds), `slam_nav`, `app_hub` (web+oled+behavior) — supervised by
  **per-unit systemd** (`nano-robot.target`, Restart=on-failure + watchdog + MemoryMax).
- Personality/brain: Sismic presence statechart + OpenRouter LLM cognition + skill library +
  purpose/A-B layer + time awareness/quiet hours — human overview in `docs/brain.md`.

Central config: `src/robot_bringup/config/robot.yaml`. Bus/pin reference:
`nanopi-neo-plus2-pinmap.md`. (Historical note: the user first said "split like ROS2 but
DON'T use ROS2", then reversed — they ARE using ROS 2.)
