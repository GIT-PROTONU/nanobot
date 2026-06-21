---
name: esp32-tooling-setup
description: RESUME-AFTER-REBOOT — Windows/WSL toolchain setup to flash+test the ESP32 coprocessor firmware (in progress 2026-06-18)
metadata: 
  node_type: memory
  type: project
  originSessionId: f2810fa6-5edc-4735-8cb9-f3174ccc7eb3
---

Setting up tooling on the Windows dev PC to flash + test `firmware/esp32_coprocessor`
(the micro-ROS coprocessor — see [[esp32-coprocessor]]). **Paused for a reboot.**

**Already installed on Windows (non-elevated, via scoop):** Python 3.14.6
(`~\scoop\apps\python\current\python.exe`) + PlatformIO Core 6.1.19. Pre-existing:
winget, scoop, git, VS Code, COM ports COM1/COM9/**COM10** (ESP32 on COM10, USB-serial
driver already present).

**KEY FINDING:** native-Windows `pio run` FAILS — micro_ros_platformio's library build
runs a POSIX `. ./...` (source) step cmd.exe can't run (`'.' is not recognized...`).
micro-ROS firmware cannot be built on native Windows. So the whole toolchain (build +
flash + agent) moves to **WSL2**. The native Windows PlatformIO is left installed but
unused. (Build log: `firmware/esp32_coprocessor/build.log`; `.pio/` dir was created.)

**What the user is running now (elevated PowerShell), then REBOOT + create Ubuntu user:**
```
wsl --install                                   # WSL2 + Ubuntu (was NOT installed at all)
winget install --exact --id dorssel.usbipd-win  # forward ESP32 USB -> WSL
```

**NEXT STEPS after reboot (Claude drives via `wsl`, no admin except one usbipd bind):**
1. Verify `wsl -l -v`. Repo is reachable in WSL at `/mnt/c/Users/ib_st/Desktop/Nano`.
2. In Ubuntu: install PlatformIO (`pipx install platformio` or pip) + build deps
   (git, cmake, python3-venv); `cd firmware/esp32_coprocessor && pio run` (micro-ROS
   lib builds fine on Linux).
3. Forward the board: `usbipd list`; `usbipd bind --busid <x-y>` (ONE-TIME, elevated —
   flag to user); `usbipd attach --wsl --busid <x-y>` → appears as `/dev/ttyUSB0`.
4. Flash: `pio run -t upload --upload-port /dev/ttyUSB0`.
5. Agent + test: run `micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200` (via this
   repo's pixi in WSL — pixi.toml has ros-humble-micro-ros-agent; or docker image).
   Verify `ros2 topic echo /wheel_ticks` ticks and `ros2 topic pub /cmd_vel ...` drives
   the H-bridge. Offered (not yet made) a host-side cmd_vel-ramp/echo test helper.

**Open question:** is the ESP32's USB-serial chip CP2102 (10c4:ea60) or CH340
(1a86:7523)? Needed for the board-side udev rule; CH340 collides with the IMU's ID.
Current `deploy/udev/95-nano-usb.rules` assumes CP2102.
