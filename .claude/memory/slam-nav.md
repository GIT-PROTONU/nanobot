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
I cannot build/flash from the Windows dev host (colcon runs in the board's pixi env; pio in
~/pio-venv). Deferred big RAM wins NOT done: consolidating the 6 rclpy nodes into one process,
and replacing rosbridge — both high-risk refactors needing on-board verification.
