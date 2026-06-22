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

NB: this note is on the **`separate-sensor-nodes`** branch (sensor nodes = 4 processes).
`main` additionally merges imu+sys+odom+lds into one `sensor_hub` process (~100+ MB RAM
saved); revert to this branch to undo only that merge. See [[single-webui-from-sbc]].
