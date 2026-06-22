---
name: slam-map-empty-lidar-spin
description: "slam_nav map panel empty? the lidar isn't spinning — check that first"
metadata: 
  node_type: memory
  type: project
  originSessionId: f5423a71-8286-4b09-8f88-0a003aabeb2a
---

`slam_nav` only writes `/dev/shm/nano_map.bin` (served at `/map`, rendered by the web
"show map" panel) when a `/scan` arrives. No scan = no map file = `/map` 503 = empty panel.
`/scan` requires the LDS02RR to be **physically spinning**.

Spin chain: ESP32 PID drives the spin motor toward `/lds_target_rpm`. The firmware
**already defaults to 300 rpm on boot** (`LDS_TARGET_RPM 300.0f` → `g_lds_target`,
`LDS_ENABLED 1` in `firmware/nanobot_coprocessor/src/main.cpp`), so the lidar auto-spins
whenever the ESP32 is powered. The web "LDS target" slider publishes `/lds_target_rpm`
(Float32) to **override** it live — but only on drag (its shown "300" is display-only, not
published on page load). `/lds_rpm` and `/lds_hz` = 0 means the lidar is producing no
frames → almost always the **lidar is unpowered/disconnected** (e.g. robot on the bench),
not a software bug. See [[esp32-zenoh-pico-integration]].
