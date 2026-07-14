---
name: cpu-reduction-plan
description: "CPU reduction TODO — 2026-07-14 re-profile done (50% idle / 60% UI-open): three NEW approved-for-todo levers = (1) executor-wakeup cut (IMU topic rates / in-process handoff, odom 15→10), (2) gpu_vision fps drop, (3) LDS serial read batching; old tier plan below is history"
metadata: 
  node_type: memory
  type: project
  originSessionId: 0b7a90af-3716-490a-8ac2-008ec20c8677
---

**TODO (2026-07-14, user asked to queue these — from the post-overhaul re-profile in
[[sbc-cpu-profile]]; baselines: ~50% of 4 cores idle, ~60% with the web UI open on the
Camera view; ~1.0 core of that is rclpy executor wait-set churn):**

1. **Cut executor wakeups (~1.0 core at stake, biggest lever).** sys_monitor is the ONLY
   subscriber of /imu/euler (25 Hz) + /imu/web (15 Hz) and is co-resident with imu_driver
   in sensor_hub — either hand samples over in-process (queue/direct call, keep the topics
   publishing at a low debug rate) or just drop both rates to ~5 Hz in robot.yaml. Also
   odom publish_rate 15→10 Hz (nav's control loop is already 10 Hz; odom wakes sensor_hub's
   timer + nav + app_hub subs). NOTE the web-slider live-tuning contract on these rates —
   don't break the "raise live for tuning" path, and mind that /imu/euler feeds the web
   angle display via telemetry (served from the vitals blob, so a low topic rate is fine).
2. **Lower gpu_vision fps (~0.38 core idle, mostly the synchronous glReadPixels stall —
   scales ~linearly with fps).** Halving fps ≈ 0.15–0.2 core back, same alerts with slower
   reaction. gpu_duty already runs >100% under the full pass set, so the loop never hits
   its configured rate anyway.
3. **Batch the LDS serial read (~0.15 core).** `lds_node.py:173` does
   `ser.read(in_waiting or 1)` → thousands of tiny wakeups/s at 115200 baud; read with a
   minimum chunk (e.g. 128–256 B) + timeout instead so the reader wakes ~40×/s.

Realistic combined outcome: idle ~50% → ~20–25%. User approved queueing all three
(2026-07-14); implementation order suggestion was 2+3 first (small, low-risk), then 1.

---

Historical plan from the 2026-06-23 profiling session (measurements in
[[sbc-cpu-profile]]). Goal the user set: **reduce CPU without reducing any sensor data
rate or functionality.** Idle baseline then ≈ 83% of one core.

**STATUS (2026-07-05, user asked to "optimise for low cpu and ram"):**
- **Tier 1 (IMU auto-rate): OBSOLETE, do not build.** robot.yaml now ships
  `publish_rate: 1.0` (commit 7f346c3) and the device stream follows it, so the reader
  already parses ~5 frames/s. A subscriber-count-keyed rate would FIGHT the web slider's
  "raise live for tuning" contract (it was started and deliberately reverted this session).
- **Tier 2 (SLAM skip-when-stationary): IMPLEMENTED** in `nav_node._on_scan` —
  `still_skip`/`still_lin` (5 mm)/`still_ang` (~0.3°) params in robot.yaml; skips
  match+integrate when odom+IMU deltas since the last PROCESSED scan are under threshold
  (prev-trackers deliberately not updated so drift accumulates and eventually processes);
  never skips while seeding/recovering/self-testing; pose+map telemetry re-published at
  the map-write cadence. NOT yet re-profiled on the board.
- **Tier 3a (OLED np.packbits): IMPLEMENTED** — `display_node._patch_fast_display`
  overrides luma's per-pixel pack (~10 ms) with np.packbits (verified byte-identical
  vs luma's offsets/mask on random frames). Tier 3b (merge oled into sensor_hub, kills
  the cross-process sub tax) still open.
- **2026-07-06 (architecture overhaul, see [[architecture-two-planes-three-hubs]]):**
  rosbridge is DELETED (the dominant UI-open cost, ~a full core — replaced by the SSE
  /telemetry gateway with lazy subs, ~zero idle cost), and oled_display was merged into
  the new **app_hub** (with web_control + mood_node) — that's tier 3b's RAM half.
  CAVEAT: the cross-process /imu/web + /lds_hz sub tax the plan wanted to kill by
  merging OLED into *sensor_hub* is NOT eliminated (rclpy has no intra-process
  shortcut; those topics still cross processes into app_hub). If a future profile
  shows it still matters, the fix is display_node reading a sensor_hub-written
  /dev/shm vitals blob (or moving those two subs' data into sensor-side shm), not
  more process merging.
- When next on the board, RE-PROFILE (per-process /proc jiffies or /tmp/py-spy) to
  confirm the tier-2 win when parked AND the new idle/UI-open baselines post-overhaul.

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