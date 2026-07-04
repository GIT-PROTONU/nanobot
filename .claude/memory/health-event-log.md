---
name: health-event-log
description: Durable ESP32/LDS outage log at ~/.local/state/nanobot/health.log (sys_monitor/health_log.py) — first place to look for intermittent failures
metadata: 
  node_type: memory
  type: project
  originSessionId: fc2698c0-a1cc-4bcc-a2df-8ef79d9728ef
---

**The robot keeps a durable, timestamped health-event log** (added 2026-07-05) for
diagnosing the intermittent ESP32/LDS outages:
`~/.local/state/nanobot/health.log` (param `sys_monitor.health_log_path`; rotates once
to `.1` at 512 KB; survives reboots/stack restarts; lines are mirrored as `[health]`
warnings into `.run/sensors.log`).

- Logic is ROS-free in `src/sys_monitor/sys_monitor/health_log.py` (`HealthWatch`),
  wired into `sys_monitor`'s existing 1 Hz tick — offline tests in
  `src/sys_monitor/test/test_health_log.py` (run with
  `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pixi run python -m pytest src/sys_monitor/test`;
  without that env var the RoboStack `launch_testing` pytest plugin crashes collection).
- Watches `/esp32_heartbeat` (>5 s silent = esp32 DOWN), `/lds_rpm`+`/lds_hz`, and the
  `/dev/shm/nano_scan.bin` header. Logs **transitions only**: boot marker, first-UP
  (its absence = the [[esp32-link-wedge-first-ping-hole]] wedge), DOWN with a
  **classified cause** using the [[lds-scan-path-sbc-direct]] discriminators
  (rpm≈0 → power/spin-motor · rpm ok + frames 0 → ESP32 RX branch · blob mtime old →
  driver dead · port-open failed), a still-down counter snapshot every 60 s, and UP
  with outage duration + rx/err deltas.
- **The SBC-branch frozen-vs-garbled verdict is deliberately a SECOND line ~5 s after
  the DOWN edge** ("lds outage counters: … -> PA1 branch dead / degraded/garbled"):
  at the edge the counter window still holds pre-failure bytes, so the call is made
  only from samples taken during the outage — and only when upstream (rpm/frames) is
  fine, since a frozen rx is expected when the lidar isn't spinning.
- Verified live 2026-07-05 by a reversible drill: `ros2 topic pub --times 3 -w 0
  /lds_target_rpm std_msgs/msg/Float32 '{data: 0.0}'` (NB `-w 0` — plain `pub -1`
  waits forever for a "matching subscription" it never sees for the ESP32's
  zenoh-pico sub), log showed the correct DOWN classification + UP after restore.
