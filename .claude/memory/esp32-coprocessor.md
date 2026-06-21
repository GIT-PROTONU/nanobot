---
name: esp32-coprocessor
description: "ESP32-WROOM coprocessor — native zenoh-pico (NOT micro-ROS) motor/encoder/LDS, wire contract"
metadata: 
  node_type: memory
  type: project
  originSessionId: 1e16332e-edf3-4369-89c0-03cd717126fc
---

ESP32-WROOM coprocessor offloading real-time motor/encoder/LDS-spin work from the H5 SBC.
Now in `firmware/nanobot_coprocessor/` (the old `firmware/esp32_coprocessor/` micro-ROS build
was deleted upstream — its leftover local `.pio/` cache is untracked, safe to `rm -rf`).

**Architecture (rewritten 2026-06-21): native zenoh-pico over UART, NO micro-ROS / agent /
Fast-DDS.** Joins the SBC's rmw_zenoh graph directly in rmw_zenoh's exact wire format +
liveliness tokens. Needs a **serial-capable zenohd** (conda libzenohc lacks transport_serial)
built via `tools/build_zenohd_serial.sh aarch64`; stack.sh runs it. Link = ESP32 UART2
(TX17/RX16) ↔ SBC /dev/ttyS1. Tunables are inline `#define`s at the top of `src/main.cpp`
(no config.h). Build/flash from dev PC: `cd firmware/nanobot_coprocessor && pio run -t upload`
(pio in ~/pio-venv; do NOT build on the board).

Wire contract: sub `cmd_vel`(Twist), `led`(Bool), `lds_target_rpm`(Float32); pub `wheel_ticks`
(Int64MultiArray [L,R]), `left/right_wheel_suspended`(Bool), `esp32_temp`/`esp32_hall`,
`lds_rpm`/`lds_hz`/`lds_duty`, `esp32_heartbeat`. Also closed-loop PID-controls the LDS02RR
spin motor (reads its RPM off UART1 RX=GPIO14).

**Encoders are SINGLE-CHANNEL** (no quadrature/PCNT) → no hardware direction. Fix applied
2026-06-21 (branch `slam`): ISR signs each tick by the last commanded wheel direction via an
**int8 dir flag** (`g_left_dir/g_right_dir`, set in cmd_cb) — never read a float in the ISR
(ESP32 FPU unsafe). Counts are now signed; `wheel_odometry` integrates them unchanged. This
matters for [[slam-nav]] (unsigned counts made /odom read reverse as forward). Related:
[[deployment-state]], [[project-overview]].
