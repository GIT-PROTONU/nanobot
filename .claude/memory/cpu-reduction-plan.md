---
name: cpu-reduction-plan
description: "CPU reduction TODO — items 1/2/3/5/6/7 ALL BUILT+deployed+hardware-verified 2026-07-14 (commit 2f582f9): param services, QoS-event patch, LDS batching, odom/IMU rate drops, gpu_vision batched-draws+stats-atlas restructure, ESP32 wheel_ticks 30→15Hz flashed. Only fan (item 4, left alone on purpose — disconnected) and gpu_vision option (c) remain open. Old tier plan below is history"
metadata: 
  node_type: memory
  type: project
  originSessionId: 0b7a90af-3716-490a-8ac2-008ec20c8677
---

**STATUS (2026-07-14, same-day follow-up session — everything queued is now shipped
except the fan):** all of items 1/2/3/5/6/7 below are DONE, committed
(`2f582f9`), deployed to the board via `scripts/deploy.sh`, and hardware-verified
(units active, `gpu_vision: GL context up, renderer=Mali450`, no errors in 30s of
post-restart logs; smoke test + a synthetic-frame old-vs-new comparison harness both
green). Specifically:
- Item 1 (executor-wakeup cut): done via the **plain rate-drop path**, not the
  in-process-handoff alternative — `wheel_odometry.publish_rate` 15→10Hz,
  `imu_driver.euler_rate`/`web_rate` 25/15→5Hz (moot today since `publish_rate=1Hz`
  already caps them lower still, but bounds future publish_rate-raised debugging).
  Plus the QoS-event patch (item 6) and param-service opt-out (item 7), see below.
- Item 2 (gpu_vision): **both (a) and (b) built**, NOT just the plain fps-drop
  fallback. (a) restructured `_loop` into submit-all-draws → read-all-pixels →
  all-CPU-scoring phases (was N interleaved draw→read stalls/frame). (b) went
  further: the 7 small per-signal stat reductions (motion/blob/luma/color-cast/
  edge/overhead/highlight) now render into ONE shared "stats atlas" FBO via
  viewport-offset slots (`draw_fullscreen` gained `x0`/`y0`; `build_downsample_chain`
  gained `drop_last=True` to skip each chain's own final stage) — **1 glReadPixels
  total instead of 7**, each slot cheaply extracted back into its own contiguous
  buffer (`extract_atlas_slot`, `ctypes.memmove` per row) so every downstream CPU
  scoring function is byte-for-byte unchanged. Verified via a synthetic-frame
  comparison harness (loads the pristine pre-session module + the edited one side by
  side, feeds both identical fake-camera frames through a lockstep queue-based
  camera, diffs every public readout) across 4 cases (target set/unset × viewers
  attached/not) — 0 mismatches each time, both after (a) alone and after (a)+(b).
  **Not built: option (c)** (double-buffer + EGL-fence-gated read of the PREVIOUS
  frame's atlas — true zero-stall, costs ~1 frame/66ms of alert latency) — only
  worth it if a re-profile after (a)+(b) still shows gpu_vision hot.
- Item 3 (LDS batching), 5 (ESP32 `/wheel_ticks` 30→15Hz, flashed via
  `pio run -t upload` on `/dev/ttyUSB0`), 6, 7: all done as originally scoped.
- Item 4 (fan): **intentionally left alone** — user confirmed the fan is
  disconnected on purpose, not a bug to chase.
- **Not yet done: a fresh on-board re-profile** to measure the actual combined win
  (the plan's 50%→20-25% idle estimate was never re-measured after this batch).
- Minor architectural leftover, still open, low priority: OLED's `/imu/web`+
  `/lds_hz` subs still cross a process boundary into `app_hub` (noted in the
  2026-07-06 overhaul, unaffected by this batch) — revisit only if a future profile
  shows it still matters.

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
2. **gpu_vision readback restructure (preferred over a plain fps drop; ~0.38 core idle,
   mostly glReadPixels stalls).** The loop does 7–9 INTERLEAVED draw→readback cycles per
   frame (gpu_vision.py `_loop`), each read draining the pipeline the pass before it just
   filled. Board probe (2026-07-14, Mesa 25.0.7 lima): `GL_NV_pixel_buffer_object` +
   `GL_EXT_map_buffer_range` + `GL_OES_mapbuffer` + `EGL_KHR_fence_sync`/`GL_OES_EGL_sync`
   are ALL exposed, and desktop-GL bindAPI works. Design, in order of effort: (a) reorder
   to submit ALL passes then do ALL readbacks — N stalls → 1; (b) pack the small stat
   outputs into one tiny "stats atlas" FBO (viewport regions) — 1 readback total; (c)
   double-buffer results and read back the PREVIOUS frame's atlas, optionally gated by an
   EGL fence poll — zero stall (Mesa only waits on the producing job; a frame-old job is
   done), costs 1 frame (~66 ms) of signal latency, fine for the alerts. PBO path exists
   as a bonus but (a)–(c) work with plain glReadPixels. A cam_fps drop stays the trivial
   fallback lever.
3. **Batch the LDS serial read (~0.15 core).** `lds_node.py:173` does
   `ser.read(in_waiting or 1)` → thousands of tiny wakeups/s at 115200 baud; read with a
   minimum chunk (e.g. 128–256 B) + timeout instead so the reader wakes ~40×/s.

**Added 2026-07-14 (second test pass on the live board):**

4. **THERMAL — fix the fan FIRST (hardware).** Live: fan commanded 100% duty yet CPU
   84.9°C and climbing, past the 75°C and 80°C passive trip points → the kernel was
   actively throttling (measured 1008 MHz vs the 1104 MHz cap; trips at
   75/80/85/90/95, critical 105). So ALL CPU% measurements this session are inflated
   by a reduced clock, and compute is being lost to heat. [[cooling-fan-control]]
   always said physical fan wiring was unconfirmed — this measurement says it is NOT
   effective (unwired, not spinning, or insufficient). Verify GPIO22→MOSFET→fan
   hardware. Only after cooling works: optionally raise `scaling_max_freq`
   1104000→1368000 (the OPP exists in the DT; +24% clock = same work reads ~20%
   lower) — do it with a stress-test + temp watch.
5. **/wheel_ticks 30→15 Hz (firmware one-liner).** `main.cpp` ~line 480 publishes
   every 33 ms — the fastest topic in the graph, waking sensor_hub's executor (and
   app_hub's when the UI is open) 30×/s to feed a 15 Hz odom. 33→66 ms costs nothing
   (odom integrates cumulative counts).
6. **Disable default QoS-event handlers (part of the executor tax).** rclpy Humble
   attaches one default incompatible-QoS `QoSEventHandler` waitable to EVERY pub+sub
   (`qos_event.py` `elif self.use_default_callbacks:`) — profile showed
   `qos_event.add_to_wait_set/get_num_entities` hot, and the stack has ~200 pubs+subs.
   Fix: pass `PublisherEventCallbacks(use_default_callbacks=False)` /
   `SubscriptionEventCallbacks(use_default_callbacks=False)` — cleanest as a small
   monkeypatch of `create_event_handlers`→`[]` in the three hub mains (one shared
   helper), losing only the incompatible-QoS warning log (irrelevant: single vendor,
   fixed QoS). Verify on dev-PC sim first.
7. **Drop parameter services on non-tunable nodes.** Every node carries 6 param
   services = 6 wait-set entities; telemetry's PARAM_WHITELIST only tunes
   imu_driver/lds_driver/wheel_odometry/slam_nav/sys_monitor/web_control. `behavior`,
   `oled_display`, `map_bridge` can construct with
   `start_parameter_services=False` (params still load from robot.yaml overrides) —
   18 fewer entities iterated per spin, mostly in app_hub.

Measured but left alone: zenohd-serial ~8% (no strace on board; small fish),
uvcvideo kworkers ~5% (inherent at 15 fps capture), journal quiet (no log spam).
Entity counts measured live (`ros2 node info`): app_hub ≈103 subs+pubs+services
(web_control 57 + behavior 30 + oled 16) + timers + qos-event waitables; sensor_hub
≈50; slam_nav ≈25 — the per-spin rebuild iterates all of them, which is why the
executor tax dwarfs the callbacks.

Realistic combined outcome: idle ~50% → ~20–25%. User approved queueing all three
(2026-07-14); implementation order suggestion was 2+3 first (small, low-risk), then 1.

Related finding (2026-07-14, live-verified — NOT a bug, no fix needed): the web UI's
master camera switch (`POST /vision/camera_enable`) DOES fully stop the gpu_vision
thread + release V4L2 (confirmed via API toggle + top -H: thread gone, fresh thread on
re-enable). The user-reported "no effect on CPU in the webui" is because it only
removes ~20–30% of ONE core ≈ 5–8 points on the all-cores CPU% readout, which jitters
about that much anyway — while app_hub's main-thread executor churn (untouched by the
switch) dominates. Once TODO items 1+2 land, the switch's effect should become visible.

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