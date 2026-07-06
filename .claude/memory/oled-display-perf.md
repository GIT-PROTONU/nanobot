---
name: oled-display-perf
description: OLED SSD1306 is I2C-bus-bound; perf characterization + the animated-eyes face mode
metadata: 
  node_type: memory
  type: reference
  originSessionId: 9c559fa2-8d06-4b5a-aaf1-f9c937da8fbf
---

The SSD1306 OLED is on **i2c-0 = `/soc/i2c@1c2ac00`** @0x3c. Its cost is **entirely the
I2C flush**, not Python/drawing.

**Bus speed raised 100kHz → 400kHz** (2026-06-22) via a *user* DT overlay
(`/boot/overlay-user/i2c0-400k.dtbo`, `user_overlays=i2c0-400k` in armbianEnv;
**codified idempotently in `deploy/sbc-setup.sh`**, needs a reboot). Full-frame flush:
**103ms → 38ms** (fps ceiling ~9.7 → ~26). OLED is the only device on bus 0, so safe.
Could push ~1MHz for ~10ms flushes if ever needed.

**Per full-frame flush @400kHz = ~38ms wall = ~8ms CPU + ~30ms I2C wait.** The 79% wait
is the thread *asleep* (hardware sun6i-i2c controller, interrupt-driven — not CPU). The
~8ms CPU is luma packing 8192px→1024B + ~32 smbus chunk syscalls.

**KEY: CPU scales with flush COUNT, not bus speed** (~1% of one core per flush/sec).
Raising bus speed only shrinks the *wait* → buys smoothness/latency, **NOT CPU**. To cut
CPU you cut flushes (dirty-check / lower fps) or bytes-per-flush (partial-region updates;
complex, little gain for full-screen). Measured aggregate (4-core) CPU: dashboard ~1.2%,
normal face mood ~2.5%, every-frame "stress" ~5.5%. RAM flat ~65MB regardless. See
[[sbc-cpu-profile]] for the per-core-vs-aggregate gotcha.

**`oled_display` has two modes** (`display_node.py`): a status DASHBOARD (clock, IP, SBC+
ESP temps, ESP/IMU/LDS online dots + data rates) and a **FACE / animated-eyes** mode.
Moods (happy/angry/focused + a "stress" max-load test) are selected from the web UI via a
**`/oled_face` std_msgs/String** topic ("" = dashboard). Cheapness levers baked in: the
face timer is **cancelled when not in face mode** (zero idle wakeups) and a per-frame
**dirty-check** skips the 38ms flush when the picture is unchanged (`anim_fps`=20, under
the 26fps ceiling). Title/brand is "NANOBOT", overridable via `/oled_text`.

**2026-07-06 updates:** luma's per-pixel frame-pack replaced with `np.packbits`
(`_patch_fast_display`, ~10 ms → ~0.6 ms/frame); the node's five telemetry subscriptions
(esp hb/temp, imu/web, imu/euler, lds_hz) are GONE — the dashboard reads sys_monitor's
**vitals blob** (`/dev/shm/nano_vitals.json`, read only while the dashboard is pinned,
local /proc fallback when stale), so face mode costs zero cross-process deserialize; the
node now runs inside **app_hub** (SIGTERM end-screen preserved in the hub main). The
`/oled_system` end-screen trigger is now published by the SERVER on POST /system/* (the
page no longer publishes topics directly).

**Shutdown/restart end-screens** (so the panel doesn't freeze on its last frame): 
**`/oled_system` ("restart"|"shutdown")** is published the instant the button is clicked, so
the node switches screens immediately (not after teardown). "shutdown" → "Shutting down"
screen then SSD1306 **display-off** (`hide()`, panel dark) done in the SIGTERM-graceful main
loop; "restart" → a "Restarting" screen left up for the relaunched node to redraw over (no
power-off → no race). `web_server.py` also drops a `/dev/shm/nano_oled_action` hint file as a
CLI fallback. The node's `main()` uses `spin_once` + a `threading.Event` SIGTERM handler so
the final draw/power-off runs on the main thread (no re-entrant I2C from a signal handler).
