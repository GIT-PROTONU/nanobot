---
name: esp32-build-flash-on-dev-pc
description: "ESP32 firmware is both built AND flashed on the dev PC, never on the SBC"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: f73399f0-03b6-4f1c-b9d1-3baa795b9c72
---

ESP32 firmware development and flashing both stay on the dev PC (`pio run -t upload`). Do not propose building or flashing from the SBC.

**Why:** Building on the board is already off-limits (CLAUDE.md: 7 GB rootfs, ~2 GB toolchain+micro-ROS tree, 1 GB RAM OOM risk, uncertain aarch64 toolchain). We also weighed flash-only on the SBC (esptool, ~382 KB, ~30 MB RAM, seconds) — user explicitly chose to keep everything on the dev PC anyway.

**How to apply:** For firmware changes, edit on the dev PC and flash over USB with `pio run -t upload`. Don't suggest SBC flashing scripts. See [[micro-ros-agent-source-build]].
