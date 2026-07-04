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

Proposed fix (not yet implemented): a first-ping deadline — `ready && !g_ping_seen`
for ~90 s → `esp_restart()`, optionally capped by an RTC_NOINIT reboot counter to keep
the original fail-safe (no boot loop if pings are legitimately absent).

Diagnosis recipe (fast): `grep ttyS1 /proc/interrupts` twice on the board — climbing =
traffic, 0/frozen = ESP32 silent. ESP32 console via dev-PC USB: reset with RTS pulse
(pyserial: dtr=False, rts=True→False) and watch for "zenoh CONNECTED" vs "z_open failed".
Note: non-interactive ssh has no `pixi` on PATH — use `~/.pixi/bin/pixi`; stack lives at
`~/Nano` on the board. Related: [[esp32-zenoh-pico-integration]],
[[motors-dead-after-gpio-reassign]] (separate issue, still open).
