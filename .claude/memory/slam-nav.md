---
name: slam-nav
description: "slam_nav package — super-light custom 2D SLAM + planned navigation, staged build on branch slam"
metadata: 
  node_type: memory
  type: project
  originSessionId: 1e16332e-edf3-4369-89c0-03cd717126fc
---

Custom lightweight 2D SLAM/nav for Nano (NOT Nav2/slam_toolbox — too heavy for the 1 GB H5),
in the new `src/slam_nav` package. Goal: robot-vacuum-style click-a-point-on-the-map → drive
there avoiding obstacles. Built on branch **`slam`** (pushed 2026-06-21), 3 stages:

- **Stage 1 — DONE in repo (not yet hardware-verified):** `occupancy.py` (numpy log-odds grid +
  inverse-sensor-model scan integration + correlative scan-to-map matcher) and `nav_node.py`
  (pose = /odom translation + /imu/euler yaw, refined by scan-match each /scan; writes map+pose
  to `/dev/shm/nano_map.bin`). `web_control` serves it at `GET /map`; index.html "Map" panel
  polls it (pure HTTP, no rosbridge). No motion. Config block `slam_nav:` in robot.yaml; wired
  into stack.sh. **Verify on board: deploy, tick "show map", drive slowly, watch the map form.**
- **Stage 2 — DONE in repo:** `occupancy.plan()` plans on a downsampled grid (ds=4 → 120x120),
  inflates by robot radius, vectorized wavefront from goal, descends + simplifies. nav_node subs
  /goal_pose, pubs /plan (Path). UI: click map → /goal_pose, green path + yellow goal overlay.
- **Stage 3 — DONE in repo:** nav_node control timer = periodic replan + pure-pursuit → /cmd_vel
  + reactive front-cone stop/replan from live /scan. Gated behind `enable_motion` (default false,
  live-togglable via /slam_nav/set_parameters). UI: "enable motion" toggle + Stop button.
  All three stages pushed to branch `slam` 2026-06-21; NONE hardware-verified yet.

**Extras added 2026-06-22 (uncommitted on branch slam, not hardware-verified):** all cheap,
each independently toggleable, behaviour-changers default OFF for safe testing. (1) **Map
persistence** — `GridMap.save/load` (np.savez_compressed, atomic; geometry-checked); param
`map_store` (""=off) auto-loads on boot + `/slam_nav/save_map` Bool topic; `autosave_period`
(0=off). (2) **Auto-explore** — `GridMap.frontiers()` (free cell bordering unknown, nearest-
first on the coarse grid; refactored shared `_coarse()` helper out of plan()); param
`auto_explore` (off), drives to nearest reachable frontier when idle (still needs enable_motion).
(3) **Return-to-home** — `/slam_nav/go_home` Bool → goal=map origin. (4) **Breadcrumb trail** —
`trail_max` ring buffer in /dev/shm header. (5) **Coverage+health telemetry** in map header
(seen %, free m², scan-match score, mode). (6) **Stuck watchdog** — `stuck_timeout` (0=off).
Web UI: auto-explore/go-home/save-map controls, trail(cyan)/home(magenta) overlays, stats line.
**WiFi telemetry** added to sys_monitor (separate from slam): `wifi_iface/ssid/signal_dbm/
quality_pct` from /proc/net/wireless (pure read) + SSID via cached `iwgetid`/`iw` subprocess
(5 s); web System panel shows "SSID -56dBm (78%)" colour-coded. occupancy.py extras unit-tested
on the dev host (save/load round-trip, coverage, frontiers, plan post-refactor).

**Perf (2026-06-22):** `integrate()` free-space raycast vectorized — per-beam Python loop
replaced by a repeat/cumsum ragged-index pass; bit-identical output, ~1.5x faster on x86
(~2-3x expected on the A53). This was the per-scan CPU hot spot. Considered the Roborock-style
tricks (submaps, 2/4-bit cell packing, pose-graph pruning, static/dynamic split) and chose NOT
to: our whole grid is ~0.9 MB (480² f32) on a ~970 MB board, so per-node Python overhead — not
the map array — dominates RAM; bit-packing would *slow* numpy. We already avoid the gmapping
 particle-filter RAM blowup (single-pose correlative scan-to-map matcher). Real remaining gaps if
ever needed: bounded/sparse grid (only if map grows much larger). TRUE LOOP CLOSURE DONE 2026-07-16
(see [[slam-loop-closure]]) — the correlative match is no longer only a local stand-in.

**Full static review done 2026-06-21 (HEAD `e7cca93`):** no bugs found. Verified package
wiring (setup.py console_script `nav_node` ↔ stack.sh launch/down/status; robot.yaml keys ↔
declare_parameters ↔ web UI service calls), occupancy.py match() broadcast indexing + plan()
row/col bookkeeping, signed-tick chain (ISR `+= g_left_dir` → int64 publish → /odom reverse),
`/imu/euler` published unconditionally so nav can rely on it, and the enable_motion safety
gating. All changed Python byte-compiles. Watch-list (not bugs): plan() `max_iter=1000` can
under-converge on a long snaking corridor; integrate() free-space raycast is a per-beam Python
loop = heaviest CPU spot. Still needs the on-board run (deploy → show map → drive → click-to-go).

Chosen knobs (user picked): scan-matching tier, whole-floor coverage, plan+pursuit+replan.
SLAM quality depends on the signed-tick firmware fix (now applied) — see [[esp32-coprocessor]].
**2026-07-16 (first real HW SLAM+drive run):** the map-rotation drift bug in `_predict`
(map didn't track the robot, ~60° skew) is FIXED + a `map->odom` TF was added; `pickup_pause`
turned OFF (switches false-trigger); straight-line drive hand-fixed via manual `wheel_trim=0.22`.
Full writeup + the still-OPEN suspension-polarity/autocal blocker → [[slam-map-rotation-encoder-trim]].
I cannot build/flash from the Windows dev host (colcon runs in the board's pixi env; pio in
~/pio-venv). (The "deferred big RAM wins" noted here — node consolidation and replacing
rosbridge — were BOTH done since: sensor_hub/app_hub + the SSE gateway, see
[[architecture-two-planes-three-hubs]].

**2026-07-16 (later same day): TRUE LOOP CLOSURE / drift correction added** — see
[[slam-loop-closure]]. The accumulated map IS the keyframe store (no separate pose graph,
kept RAM at ~48 B steady-state). Every `loop_probe_every` (50) scans nav_node re-matches the
current scan against the map centered on the odometry-predicted (drifted) pose with a WIDE
window; a strong match landing >`loop_min_shift` (0.3 m) from the corrected pose = a far
re-visit = accumulated global drift, which is SMOOTHED into a persistent `_loop_off` (and the
grid warped only once the per-event step exceeds `loop_apply_thresh`). On by default. Deployed
+ verified live (params confirmed via `ros2 param get`); HW drift-removal not yet driven.)
