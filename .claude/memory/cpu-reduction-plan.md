---
name: cpu-reduction-plan
description: "Actionable idle-CPU reduction plan for the Nano stack (3 tiers, all keep sensor rates+functionality); user to pick scope"
metadata: 
  node_type: memory
  type: project
  originSessionId: 0b7a90af-3716-490a-8ac2-008ec20c8677
---

Plan from the 2026-06-23 profiling session (measurements in [[sbc-cpu-profile]]). Goal the
user set: **reduce CPU without reducing any sensor data rate or functionality.** Idle
baseline ≈ 83% of one core. **Nothing implemented yet** — user said "I'll decide and
continue later." When resuming, RE-PROFILE first (board state may differ) before editing.

Three tiers (escalating impact + risk). All preserve every sensor rate + feature:

**1. IMU device auto-rate (sensor_hub / `imu_node.py`).** Nothing subscribes to /imu/data,
yet the BWT901CL streams 100 Hz → ~400 frames/s parsed for 0 consumers. Fix: when
`pub_imu.get_subscription_count()==0`, program the device RRATE to `max(euler_rate,
web_rate)≈25 Hz`; bump back to `publish_rate` (100) the instant a subscriber appears. The
machinery already exists (`_configure_device`, `output_rate_hz`/`_dev_rate_for`, the
`_need_reconfig` Event) — just key the target on the live sub count (poll it on a slow timer
or re-check each reconnect). /imu/data stays 100 Hz whenever actually used. Est. ~6% off
(reader thread 8.8%→~2.5%). Low risk.

**2. SLAM skip-when-stationary (`nav_node.py` `_on_scan`).** Biggest idle win (~20% off
*when parked*). If odom translation <~5 mm AND IMU-yaw delta <~0.3° since the last PROCESSED
scan, skip `grid.match` + `grid.integrate` (a stationary robot's pose+map can't change) —
just keep `_last_scan` fresh (front-stop layer) and refresh the map-file telemetry at
`_write_period`. Full SLAM resumes instantly on motion. Use the same odom/IMU deltas
`_predict` already computes. Caveat: don't skip while `_recovering`/seeding the first scan.
Low–moderate risk (pick thresholds carefully; must still fire on pure rotation).

**3. OLED de-chatter (~7% off).** Its 10.4% is cross-process deserialization, NOT drawing.
Two parts: (a) swap luma's pure-Python frame-pack for `np.packbits` in the SSD1306 page
layout (17× faster; ~1%, easy, no behavior change) — see the page/bit math in the
session's lumabench. (b) The real win: kill the 15 Hz /imu/web + /lds_hz cross-process subs
the OLED only samples at 1 Hz. Cleanest = **merge oled_display into the sensor_hub process**
so it reads IMU rate / LDS hz / sys CPU%+temp directly in-process (no zenoh deserialize);
keep ESP topics (external, 1 Hz, cheap). MUST run the OLED render+I2C flush (~20 ms, mostly
wait) on its OWN thread inside sensor_hub so it doesn't stall the sensor executor. Trade-off:
OLED loses independent restart (same deal sensor_hub already made). More invasive.

Minor/optional: odom 15→10 Hz, map_write 2→1 Hz — marginal, probably skip.

Realistic outcome: idle ~83% → ~77% (tier1) / ~60% (tier1+2) / ~50% (all). Note: this is the
IDLE baseline (web UI closed); UI-open cost is dominated by rosbridge (separate, in
[[sbc-cpu-profile]]). NOT a blank-sheet rewrite — the code is already tight (IMU node
especially); these are targeted restructurings. Related: [[oled-display-perf]],
[[single-webui-from-sbc]].

**Also uncommitted/contextual:** the OLED now shows SBC CPU **busy %** (delta /proc/stat,
matches web `cpu_percent`) + RAM used % in a right-hand vitals column (display_node.py,
committed 2433128 was the load-avg version; the %-swap was deployed live but its commit is
bundled with this session's work).