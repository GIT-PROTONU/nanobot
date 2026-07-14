---
name: esp32-zenoh-pico-integration
description: "ESP32 runs zenoh-pico over direct UART to the SBC (no micro-ROS agent, no DDS) — full pipeline working; all the hard-won gotchas"
metadata: 
  node_type: memory
  type: project
  originSessionId: f73399f0-03b6-4f1c-b9d1-3baa795b9c72
---

The ESP32 coprocessor was ported from micro-ROS to **native zenoh-pico** (firmware/zenoh_pico_spike/), talking straight to the SBC's `rmw_zenoh` graph over a **direct hardware UART** — ESP32 UART2 (TX=GPIO17, RX=GPIO16) ↔ SBC UART1 (/dev/ttyS1). No agent, no DDS. A serial-capable `zenohd` (built with `--features transport_serial`; conda's lacks it — see [[robostack-zenoh-no-serial]]) runs on the SBC as the router (listens TCP + the serial UART). Build it: `firmware/zenoh_pico_spike/tools/build_zenohd_serial.sh aarch64`, installed at `~/Nano/bin/zenohd-serial`. stack.sh updated (router replaces conda rmw_zenohd + drops micro_ros_agent; lds_driver_py disabled since ttyS1 is now the ESP32 link).

**Pin reassignments** (GPIO16/17 are now the zenoh UART2 link, so the micro-ROS pins moved): MOTOR_STBY 17→23, LDS_RX 16→35 (UART1, RX-only; only matters if LDS re-enabled). Everything else unchanged from the old config.h.

**Hard-won firmware essentials** (all in the spike main.cpp / config):
- Locator MUST be device form `serial/UART_2` (NOT pin form `serial/17.16` — `_z_open_serial_from_pins` skips the z-serial link handshake; zenoh-pico bug). UART_2 maps to pins 16/17.
- Patch zenoh-pico `begin(baudrate)`→`begin(baudrate,SERIAL_8N1,rxpin,txpin)` via pre-build `extra_scripts` (default pins didn't drive GPIO17).
- Feature flags via `-DZENOH_GENERIC` + project `include/zenoh_generic_config.h` (shipped config.h hard-#defines; build flags ignored). Z_FEATURE_LINK_SERIAL=1.
- MULTI_THREAD=1 + Z_FEATURE_BATCH_TX_MUTEX=1: the serial RX read BLOCKS until a full frame, so a dedicated read task must own RX; our publishes + lease keepalives are TX-mutex-serialized. Single-thread gates publishing at the router keepalive rate (~0.4/s).
- CDR aligns from the BODY start (after the 4-byte encap header): Int64MultiArray needs 4 pad bytes before the int64 data (36B); Twist linear.x@buf+4, angular.z@buf+44.
- Router MUST use rmw_zenoh's ROUTER config (not zenohd defaults) + exit_on_failure:false; stack.sh generates it. Default routing lets ROS peers gossip into a direct mesh.
- LDS (June 2026): now ENABLED (`#define LDS_ENABLED 1`) and moved to **UART1 RX=GPIO14** (was 35; 25/4 were rejected — they're the left-motor PWM). Sleek read: UART1 is drained **once per PID tick (50 Hz), not every loop** — every frame carries the RPM, so we only need the latest speed to close the spin PID (`setRxBufferSize(1024)` so a burst survives between ticks). esptool needs `--connect-attempts 4` to flash. Observed on bench: LDS reports ~370 rpm steady even with PID duty=0 → the GPIO21 spin-motor PWM may not actually drive/brake this unit (free-spinning or externally powered); PID has no speed authority as wired — revisit LDS_MOTOR_PIN wiring if closed-loop spin is wanted.
- LDS is now **dual-read**: its single TX line fans out to BOTH the ESP32 (UART1/GPIO14, RPM→spin PID) AND the **SBC's UART2 = /dev/ttyS2** (full scan → `lds_driver_py` → `/scan`). ttyS1 is the ESP32 zenoh link, so the SBC LDS reader moved ttyS1→ttyS2 (matches the old abandoned Rust node's ttyS2 default). SBC needs the **`uart2` device-tree overlay** (added to `deploy/sbc-setup.sh`) + reboot — without it `/dev/ttyS2` exists but gives I/O error. Config: `robot.yaml` lds_driver.port=/dev/ttyS2; `scripts/stack.sh` re-enables the lds launch. Verified on hardware (June 2026): live `fa`-headed LDS frames on ttyS2 + `/scan` and ESP32 RPM both show in the web UI. Committed d7c7ad6.
- **ZENOH PRIORITY — do NOT pin zenoh-pico's tasks to Core 0.** Tried patching system.c `xTaskCreate`→`xTaskCreatePinnedToCore(...,0)` to give the SBC link a dedicated core; it REGRESSED the link: the prio-12 read/lease tasks starved the prio-5 zenohTask so `z_open` succeeded but publisher **declares never ran** → board connects but announces no topics → web UI gets no ESP32 data (no "zenoh CONNECTED" print). REVERTED. zenoh already has priority over the LDS without pinning: read/lease tasks run at prio 12 vs the Arduino loopTask (LDS+control) at prio 1, AND the UART2 (zenoh) RX ISR is on Core 0 while UART1 (LDS) RX ISR is on Core 1 (each `begin()` installs its ISR on the calling core). Verified stable 30s+ under live LDS streaming. See [[single-webui-from-sbc]].

**WORKS:** all 9 pubs (LDS off: 6) decode as correct ROS types, all subs (cmd_vel/led) received; session stable (1/s heartbeat steady 30s+); data reaches **raw zenoh clients** reliably.

**RESOLVED — the fix was rmw_zenoh LIVELINESS TOKENS.** Without them the ESP32 wasn't a graph participant and rmw_zenoh subscribers (rosbridge/web) only received its data intermittently (raw `**` zenoh clients always did). Fix in firmware: set a FIXED session zid (`Z_CONFIG_SESSION_ZID_KEY`, use a palindromic all-nonzero hex like all-`e5` to dodge byte-order/leading-zero ambiguity) and declare one `z_liveliness_declare_token` per publisher with keyexpr
`@ros2_lv/0/<zid>/0/<eid>/MP/%/%/<node>/%<topic>/<type>/TypeHashNotSupported/:1:,1:,:,:,,`
(MP=publisher, eid unique per entity, `%`=empty enclave/namespace, `%<topic>` mangles the leading `/`). Capture the exact format from a real publisher via `RUST_LOG=zenoh::net::routing::hat::router::token=debug rmw_zenohd` + `ros2 topic pub`. After this: `ros2 topic list` shows the ESP32 topics, rclpy + rosbridge reliably receive all of them, web UI works. Verified full path ESP32→UART→zenohd-serial→rmw_zenoh→rosbridge→web.

Link-connect watchdog (June 2026): if the ESP boots before the SBC's serial zenohd, its repeated failed handshakes leave the serial link in a state an in-process `z_open()` retry won't re-sync — the only cure was a manual ESP power-cycle (a fresh boot sends a clean InitSyn the now-listening router accepts). Fixed in firmware by a Core-1 watchdog: if `ready` isn't reached within `LINK_CONNECT_DEADLINE_MS` (default 40 s) of boot, `esp_restart()` itself (== the manual power-cycle; also rescues a z_open() wedged on Core 0). `ready` made volatile, `g_boot_ms` seeded in setup(). Shorter deadline = faster auto-recovery once the SBC is up, but more wasted reboots while it's still booting — tune on hardware. **Not yet flashed/verified on hardware as of this note.**

Runtime reconnect (2026-06-22): the *router restart after a good connect* gap is now handled by an **SBC→ESP liveness ping**. The always-on `web_control` node publishes `/esp32_ping` (Int32) at 1 Hz (independent of any browser); the firmware subscribes and, if `ready` but no ping for `LINK_RX_TIMEOUT_MS` (8 s), `esp_restart()`s to re-handshake. **Fails safe:** the timer arms only after the first ping is seen (`g_ping_seen`), so a topic mismatch / feature-off never causes a reboot. Subscriber needs no liveliness token (only publishers do). `std_msgs` added to web_control package.xml. Not yet flashed/verified as of this note. Both watchdogs share the same `esp_restart()` remedy as the connect-deadline one above.

**GOTCHA (found 2026-07-14, bench-verified): `ready` alone is NOT proof the SBC is present.**
`z_open()` for the serial transport succeeds as soon as it opens the local UART peripheral —
that needs no peer to respond, so `ready` goes true even with the UART2 link cable completely
unplugged (confirmed on the bench: board printed "zenoh CONNECTED" with nothing wired to
GPIO16/17). The only reliable evidence of a real SBC is `g_ping_seen` (set true only inside
`ping_cb`, i.e. an actual `/esp32_ping` message was received; reset false on every (re)connect
attempt). Added a `linkAlive()` helper (`return ready && g_ping_seen`) in firmware and switched
the new LDS-park / CPU-low-power gating (below, and see [[esp32-coprocessor]]) to use it instead
of bare `ready` — anything that needs to know "is the SBC actually there" should use
`linkAlive()`, not `ready`.
