---
name: robostack-zenoh-no-serial
description: "RoboStack conda libzenohc lacks the serial transport feature; zenohd can't do serial"
metadata: 
  node_type: memory
  type: reference
  originSessionId: f73399f0-03b6-4f1c-b9d1-3baa795b9c72
---

The RoboStack conda `libzenohc` 1.9.0 (used by `ros-humble-rmw-zenoh-cpp` 0.1.8 → `rmw_zenohd`) was **built without the `transport_serial` cargo feature**. Both `listen` and `connect` on a `serial//dev/ttyXXX#baudrate=...` endpoint fail at runtime with `Unicast not supported for serial protocol` (zenoh-link/src/lib.rs:183). The `serial` keyword IS accepted in the JSON5 config schema (it's a known protocol enum), so it fails only at link-open time, not config parse.

**Consequence:** an ESP32 running zenoh-pico over USB serial cannot connect to the installed zenohd. The "everything on zenoh, no micro-ROS agent, ESP32 over serial" design is blocked at the router. Unblock needs: a zenohd built with `--features transport_serial` (heavy Rust build), or ESP32→zenoh over WiFi/TCP (TCP works in the conda build), or keeping the micro-ROS agent. See [[micro-ros-agent-source-build]], [[esp32-build-flash-on-dev-pc]].

**What IS proven:** the zenoh→rmw_zenoh wire format on Humble — keyexpr `0/<topic>/<dds_type>/TypeHashNotSupported`, CDR-LE payload, attachment `seq(i64)+ts(i64)+0x10+gid[16]`. A plain-zenoh publisher emitting that was received by `ros2 topic echo` under rmw_zenoh. The zenoh-pico ESP32 firmware also builds (firmware/zenoh_pico_spike).

**Serial DOESN'T WORK end-to-end (exhaustively tested 2026-06-19):** even with a serial-capable zenohd, the zenoh-pico ESP32 client could NOT establish a serial session. The ESP32 sends a valid zenoh InitSyn (raw bytes physically captured on the host, e.g. `e0 01 02 18...`), but **zenohd's serial listener never ingests/parses it** — its trace stops at "I'm cleaning the buffers" and no transport is established. Reproducible across BOTH CP2102 (ESP32 onboard USB/UART0) and two CH340 adapters (UART2), so it is NOT adapter-specific — it's a zenoh-pico 1.9.0 ↔ zenoh-c 1.9.0 serial-transport/framing incompatibility (cf. zenoh-pico #357, zenoh roadmap "Struggling ESP32 serial to linux" #120). A one-off "peer connected" earlier was a stale TCP client misattributed. **Conclusion: serial is a dead end with this stack; use WiFi/TCP (mature) or keep the micro-ROS agent.**

**Serial-capable zenohd (built):** `zenohd v1.9.0` built in docker (`rust:1-bookworm`, `cargo build -p zenohd --features zenoh/transport_serial`) DOES support serial — listener `-l serial//dev/ttyUSB0#baudrate=115200` opens fine and routes to rmw_zenoh over TCP. Binary was at /tmp/zbuild/zenohd-x86_64 (x86 only; rebuild as needed). zenoh-pico serial gotcha: shipped `config.h` hard-#defines features so `-DZ_FEATURE_LINK_SERIAL` build flags are ignored — must use `-DZENOH_GENERIC` + a project `include/zenoh_generic_config.h` (config.h `#ifdef ZENOH_GENERIC` includes it). With serial enabled, ESP32 `z_open(serial/1.3#baudrate=115200)` attempts the handshake and a peer connected ONCE, but **reliable data flow over the shared USB-UART0 was NOT achieved** — hits zenoh-pico serial issue #357 (transport closes after 1 connect, no client retry) + UART0 being shared with the ESP32 boot ROM log / DTR-RTS auto-reset / console (no debug visibility). Likely fix: dedicated UART (GPIO16/17) + a separate USB-FTDI adapter (the community-standard ESP32 zenoh-serial setup), leaving UART0 for boot/console. Otherwise WiFi/TCP works today with the (serial-less) conda router.
