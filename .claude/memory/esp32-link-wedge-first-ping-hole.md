---
name: esp32-link-wedge-first-ping-hole
description: "ESP32 zenoh link can wedge permanently (ready=true, no ping ever seen) — ping watchdog never arms; recovery = hard reset; diagnose with /proc/interrupts ttyS1"
metadata: 
  node_type: memory
  type: project
  originSessionId: b99056bb-4367-4e5a-b5dc-13cff3253bb9
---

2026-07-04: "ESP32 not connecting to SBC" incident. Symptom: after an SBC power-cycle,
`/proc/interrupts` on the board showed **ttyS1 = 0 interrupts since boot** (ESP32 totally
silent on the wire) while zenohd-serial held the port fine. ESP32 itself was alive on
USB (`/dev/ttyUSB0`, dev PC): LDS PID running, tick prints fine — but no zenoh TX at all.
A hard reset (RTS pulse over USB) fixed it instantly: fresh boot → clean InitSyn →
"zenoh CONNECTED" → heartbeat back on the graph.

Root cause (firmware watchdog hole in `firmware/nanobot_coprocessor/src/main.cpp`):
- `LINK_CONNECT_DEADLINE_MS` (40 s) only fires while `!ready`.
- `LINK_RX_TIMEOUT_MS` (8 s ping-loss reboot) only arms **after the first `/esp32_ping`
  is seen** (`g_ping_seen`), and is re-armed false on every (re)connect.
- Hole: if `z_open()` succeeds (ready=true) against a router that dies before the first
  ping arrives (e.g. race during an SBC power-cycle), NO watchdog can ever fire → the
  ESP32 sits wedged forever, zero bytes on the UART, until a manual power-cycle/reset.

FIXED + FLASHED 2026-07-04: `LINK_FIRST_PING_DEADLINE_MS` (90 s) — `ready && !g_ping_seen`
past the deadline → `esp_restart()`, capped at `LINK_FIRST_PING_MAX_REBOOTS` (5)
consecutive SW reboots via an `RTC_NOINIT_ATTR` counter (cleared on any ping and on
non-SW resets), preserving the original fail-safe (no boot loop if pings are
legitimately absent). Verified: reconnects + heartbeat on the graph after flash.

Related gotcha found while verifying: the web page re-publishes its lidar slider on
every rosbridge (re)connect (`syncLdsTgt`, web/index.html) — an open tab with the
slider at 0 silently commands `/lds_target_rpm=0` to a freshly-rebooted ESP32, so
"lidar won't spin after ESP reboot" can just be a forgotten browser tab, not firmware.

Diagnosis recipe (fast): `grep ttyS1 /proc/interrupts` twice on the board — climbing =
traffic, 0/frozen = ESP32 silent. ESP32 console via dev-PC USB: reset with RTS pulse
(pyserial: dtr=False, rts=True→False) and watch for "zenoh CONNECTED" vs "z_open failed".
Note: non-interactive ssh has no `pixi` on PATH — use `~/.pixi/bin/pixi`; stack lives at
`~/Nano` on the board. Related: [[esp32-zenoh-pico-integration]],
[[motors-dead-after-gpio-reassign]] (separate issue, still open).
