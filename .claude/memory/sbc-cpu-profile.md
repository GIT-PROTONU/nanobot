---
name: sbc-cpu-profile
description: "Where the NanoPi H5's CPU goes; rosbridge w/ open web UI is the dominant cost; remaining lever is the unthrottled /imu/data sub"
metadata: 
  node_type: memory
  type: project
  originSessionId: f0a4bf50-cc12-4361-a366-bfe320e5ba22
---

Profiled the SBC at ~50% CPU (2026-06-20). It's not one runaway — it's the whole
rclpy/rosbridge stack on the weak quad H5. Steady-state breakdown (of 400% = 4 cores):

- **rosbridge_websocket ~100% (a full core)** — DOMINANT, but only while a browser
  has the web UI open. It serializes ~14 subscribed topics to JSON over the websocket.
- encoder_node (wheel_odometry), oled, lds_node, imu_node, zenohd each ~10-25%.

Key insight: under rmw_zenoh **every subscriber deserializes each message in-process**
before its callback. So subscribing to a heavy topic just to read one field is costly
on this board. Fixed two cases: oled_display now reads `/lds_hz` (Float32) instead of
deserializing `/scan` (LaserScan, ~360 floats) 10x/sec; odom default dropped 30->15 Hz
(now live-retunable via web slider). Took overall ~51% -> ~43% busy.

rosbridge levers — **measured, with surprises** (A/B on the live board via a stdlib
ws probe, comparing rosbridge CPU delta per added subscriber):
- A single extra `/imu/data` (50 Hz) subscriber adds only **~5-6%** to rosbridge, so a
  topic's per-client serialize is small; rosbridge's ~80% is mostly the *shared* ROS-side
  receive (rclpy building a Python msg per incoming sample) across all bridged topics.
- **CBOR (`compression:"cbor"`) cuts bandwidth, NOT CPU** — verified ~equal CPU, smaller
  bytes. Don't reach for it to save CPU. (Also: a 2nd subscriber's compression is ignored
  if another client already subscribed to that topic uncompressed.)
- **`throttle_rate` does NOT reduce rosbridge's ROS-side receive** — it only caps the
  outgoing websocket rate; rclpy still builds every incoming sample. So throttling a
  high-rate topic barely helps SBC CPU.
- The only real rate-preserving win: stop *bridging* a heavy high-rate topic. DONE for
  IMU (commit ccd7139): imu_node publishes a tiny `/imu/web` Vector3Stamped (|accel|,
  |gyro|, measured /imu/data Hz) at web_rate=15 Hz; the web reads that + `/imu/euler` and
  no longer subscribes to `/imu/data`, which still publishes 50 Hz for ROS.

Other facts: closing the browser tab drops ~a full core (true idle ~25%); viewing
`/stream.mjpg` adds a big V4L2/JPEG jump. See [[single-webui-from-sbc]].

How this was profiled (reusable): SSH to the board (host in deploy.sh), then
`top -b -o %CPU` / read `/proc/<pid>/stat` utime+stime jiffies over a fixed window for
a clean per-process CPU%. To A/B a rosbridge change without the real browser, a pure-
stdlib websocket client (HTTP upgrade + one masked `subscribe` frame, then recv+discard)
subscribes to a topic and you measure rosbridge's CPU delta — no extra libs on the 1 GB
board. Confounder: rosbridge serves a topic per its FIRST subscriber's settings, so to
test compression/rate cleanly use a topic the browser isn't already subscribed to.

Status (2026-06-20): oled, odom (30->15), and the /imu/web decoupling are deployed +
committed (oled/odom f93e51a, imu ccd7139). The full rosbridge drop from the IMU change
needs the browser reloaded onto the new page (so nothing holds /imu/data open) — that
final confirmation was still pending at end of session.

**Update (2026-06-22) — two more rate-preserving wins (not yet deployed/verified):**
- **/scan moved off rosbridge** (the heaviest bridged msg, 360 floats). `lds_driver_py`
  now writes a compact blob to `/dev/shm/nano_scan.bin` (JSON header + raw float32 ranges);
  `web_control` serves `/scan.bin`; the page polls it (~12.5 Hz, skips unchanged seq) and
  draws — same lidar view + point-count + scan-Hz readouts, zero rosbridge LaserScan
  builds. Same pattern as `/map`. /scan still publishes for slam_nav.
- **Process merge for RAM:** new `sensor_hub` package runs imu+sys+odom+lds in ONE process
  (SingleThreadedExecutor); saves ~100+ MB vs four interpreters. Node names/topics/params/
  services unchanged (per-name param loading from one --params-file works). Trade-off: no
  independent crash/restart; serial drivers self-heal on their own threads. Web shows IMU
  connectivity via /imu/web staleness (red "lost"), so a device drop is still visible.
  `stack.sh` launches one `sensors` entry; `do_down` keeps the old per-node patterns to
  sweep pre-merge stragglers. See [[single-webui-from-sbc]].

**Update (2026-06-23) — IDLE baseline profiled (web UI CLOSED), per-process AND per-thread.**
Prior profiling focused on the UI-open rosbridge cost; this is the always-on 24/7 cost that
runs regardless of the browser. Idle total ≈ **83% of one core** (of 400% = 4 cores):
- **sensor_hub ~40%** → per-thread: main executor **28%** (odom 15 Hz + TF + /wheel_ticks
  ingest + sys_monitor), **IMU serial-parse thread 8.8%**, zenoh rx/tx ~3%. (LDS reader
  thread was ~0 — lidar low/idle data that sample.)
- **nav (slam) ~26%** → ~all main-thread per-scan match+integrate. The planner was idle
  (no goal), so this is pure SLAM, not navigation.
- **OLED ~11%** → **10.4% on the MAIN thread, and it is NOT rendering.** Benchmarked on the
  board: full dashboard PIL render = **8 ms**, luma monochrome frame-pack = **9.7 ms** (a
  pure-Python triple loop; np.packbits does the same in 0.56 ms = 17×), I2C flush is mostly
  *wait*. Render+pack+flush ≈ only ~2.3% at 1 Hz. The other ~8% is **deserializing
  /imu/web (15 Hz) + /lds_hz + esp topics over zenoh just to repaint once a second** — same
  "every subscriber deserializes in-process" tax as above, [[esp32-zenoh-pico-integration]].
- zenohd ~5% (router, inherent).

Decisive new fact: **nothing in the stack subscribes to /imu/data** (grep-confirmed: nav
uses /imu/euler, web uses /imu/web). Yet the BWT901CL device is pinned to stream 100 Hz, so
the parser chews ~400 frames/s (accel+gyro+angle+mag) for a topic with **zero consumers**.
Profiler scripts (per-process + per-thread /proc utime+stime samplers, and the luma/render
micro-benchmarks) were one-off in /tmp on the board; re-derive from this note if needed.
The actionable reduction plan from this session is in [[cpu-reduction-plan]] (user will
decide scope later — nothing implemented yet).

**Update (2026-07-04) — "80% CPU" investigated; new dominant waster = map_bridge.**
Load avg 5.9 on 4 cores, web UI open. Sustained per-process (of 400%): rosbridge ~85-93,
**map_bridge_node ~75-100 (NEW regression)**, sensor_hub ~70, nav ~42, oled ~35, web ~27,
zenohd ~8. Findings:
- **map_bridge burns ~a full core doing nothing useful**: slam_nav rewrites
  `/dev/shm/nano_map.bin` every ~0.5 s (`map_write_rate` 2.0) even when SLAM is paused
  ("picked up"), and the bytes are **md5-identical** between writes — but map_bridge's
  only change-check is **mtime**, so it republishes the full 480×480 = 230k-cell
  OccupancyGrid at 2 Hz forever, **with zero /map subscribers** (no RViz attached; the web
  UI polls the blob over HTTP). Measured on-board: `tolist()` 12 ms + int8-range validate
  137 ms per publish, plus CDR+zenoh. Fixes (not yet applied): hash the body instead of
  mtime, and/or skip when `pub.get_subscription_count()==0`, and/or have slam_nav skip
  rewriting an unchanged map.
- rosbridge ~0.9 core = the known UI-open cost, now with ~25 subscribed topics totalling
  ~110 msg/s (probe-measured: /wheel_ticks 30 Hz, /imu/euler 21, /odom 15, /imu/web 13,
  lds×3 @5, rest ≤1 Hz — all as designed, no flooding/bounce). py-spy confirms time goes
  to per-msg extract_values + JSON + websocket send.
- /imu/data still streams 100 Hz with zero consumers (unchanged from 2026-06-23).
- Profiling tool now on the board: **`/tmp/py-spy`** (aarch64 binary from the v0.4.0 wheel;
  needs sudo, ptrace_scope=1; `--nonblocking` panics on this platform — use normal mode).

**Same day, FIXED + deployed (user chose to fix all except the IMU rate):**
- `map_bridge_node._tick` now (a) returns immediately when `/map` has no subscribers
  (TRANSIENT_LOCAL cache serves late joiners) and (b) dedupes on an md5 of the grid *body*
  (mtime alone is useless — slam_nav's header telemetry churns every write). 75-100% → **0.6%**.
- `GridMap` gained a `rev` counter (bumped in `integrate`/`load`); `nav_node._write_map`
  caches `occupancy_int8().tobytes()` + `coverage()` and only recomputes when `rev` moves —
  the full-grid `np.exp` at 2 Hz was ~half of nav's CPU while paused. py-spy after: zero
  occupancy.py samples; nav's remainder is rclpy executor/deserialize tax (~35%).
- mood_node's `self._clock = SimulatedClock()` collision (see [[stack-autoheal]]) fixed →
  behavior node now survives boot.
- Result: idle-with-no-browser total ≈ **42% of one core** (was ~95% of four with the UI
  open). rosbridge's ~0.9-core UI-open cost and the 100 Hz IMU stream remain untouched (the
  IMU-rate lever was explicitly declined for now).
