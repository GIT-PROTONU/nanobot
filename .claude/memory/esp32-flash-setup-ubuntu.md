---
name: esp32-flash-setup-ubuntu
description: "How the ESP32 coprocessor firmware is built/flashed on the Ubuntu dev box (PlatformIO venv, penv symlink, port)"
metadata: 
  node_type: memory
  type: project
  originSessionId: e378609d-bfa0-4599-9931-d35cfe36c672
---

Dev machine is native Ubuntu 24.04 (no WSL). PlatformIO Core lives in a venv at
`~/pio-venv` (pip-installed; `pio` symlinked into `~/.local/bin`). esptool/cmake/ninja
were pip-installed into that same venv.

Two non-obvious gotchas for building `firmware/esp32_coprocessor` (micro-ROS):
- micro_ros_platformio's `extra_script.py` sources `$PROJECT_CORE_DIR/penv/bin/activate`
  (`~/.platformio/penv`). A pip/venv install has no penv, so symlink it:
  `ln -sfn ~/pio-venv ~/.platformio/penv`.
- Its host build needs `cmake`+`ninja` on PATH (gcc/g++/make/git already present);
  `~/pio-venv/bin/pip install cmake ninja` puts them in the activated penv.

The **ESP32 is the CP2102 adapter → `/dev/ttyUSB0`** (chip ESP32-D0WD-V3, MAC
c0:49:ef:cf:38:b0; stable id `usb-Silicon_Labs_CP2102_..._0001-if00-port0`). The CH340/
CH341 (1a86) adapter is the UART2 zenoh-link/IMU/other — do NOT flash that. Identify with
`udevadm info -q property -n <p> | grep ID_MODEL` (CP2102 vs USB_Serial) or
`~/pio-venv/bin/esptool.py --port <p> chip_id` (ESP32 syncs; CH341 gives "no serial data").

**GOTCHA: `pio run -t upload` auto-detects `/dev/ttyUSB1` (the CH341 link adapter) and
fails with "Failed to connect to ESP32: No serial data received".** Always pass
`--upload-port /dev/ttyUSB0` explicitly. The debug console (`pio device monitor`) is also
on ttyUSB0.

NOTE (2026-06): the firmware dir is now `firmware/nanobot_coprocessor` (native zenoh-pico,
NOT micro-ROS — the penv/cmake/ninja notes above were for the retired micro-ROS build and
no longer apply). Build/flash that survived a stale zenoh-pico CMakeCache from the old
`zenoh_pico_spike/` path: `rm -rf .pio/libdeps/esp32dev/zenoh-pico/build` then rebuild.

User `ib` was added to `dialout`; until next login use `sg dialout -c '…'`.
Reflash: `cd firmware/nanobot_coprocessor && source ~/pio-venv/bin/activate && pio run -t upload --upload-port /dev/ttyUSB0`.
See [[esp32-zenoh-pico-integration]] and CLAUDE.md for the topic contract.
