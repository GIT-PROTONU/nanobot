---
name: lds-idle-spindown-ui-fix
description: "LDS idle spin-down never observed: 2026-07-14 fixed a web-UI reconnect-resync bug, 2026-07-15 fixed a 2nd cause (unreachable goal latches forever, not yet deployed)"
metadata: 
  node_type: memory
  type: project
  originSessionId: 832e8fcf-fb80-43d2-a47a-4dc17a08306f
---

**Consolidated LDS UI** (commit b7f36d5, deployed+hardware-verified 2026-07-14): all
lidar-related web controls — scan rate/points/lost/rx-errors, publish-rate slider, ESP32
spin rpm/frames/duty, spin-target slider, idle spin-down toggle+timeout — now live in ONE
"Lidar (LDS)" card in the Sensors tab, instead of split across that card + the
"Coprocessor (ESP32)" card + the "Navigation (slam_nav)" card.

**Root cause of "I've never seen the LDS power down automatically"**: `slam_nav` owns
`/lds_target_rpm` (toggles it between `lds_active_rpm`/`lds_idle_rpm` based on
motion/goal state, `_update_lds_idle` in `nav_node.py`) and by design only republishes on
a genuine idle↔active *transition* — so a manual override wouldn't get fought every tick.
But `web/app.js`'s `syncLdsTgt()` unconditionally re-published the slider's raw value
straight to `/lds_target_rpm` on **every browser (re)connect**, including initial page
load. So the act of opening the web UI to check on the lidar force-woke it back to active
every time — the one way to observe the parked state prevented it from ever being
observed.

**Fix**: the "Spin target" slider now calls `setParam("slam_nav","lds_active_rpm",…)`
instead of publishing the topic directly. `slam_nav` remains the sole owner of
`/lds_target_rpm`; its `lds_active_rpm` param-change handler applies the new setpoint
immediately if currently spinning (added a republish there), or at the next wake if
currently parked. `syncLdsTgt()` on reconnect is now harmless — it only nudges a param
slam_nav reads, never overrides slam_nav's own idle/active decision.

**Generalizable gotcha**: this repo's "on (re)connect, re-assert the page's authoritative
state" pattern (`syncLdsTgt`/`syncFan`/`publishPickupOv` in `app.js`, run from the SSE
connect handler) is safe for topics/params nothing else owns state-machine-style, but
fights ANY node that owns a topic and only republishes on change. If a future control's
"live" behavior looks like it's never engaging, check whether a reconnect-resync is
silently overriding it. See [[architecture-two-planes-three-hubs]] for the two-plane
web_control design this pattern lives in.

Verified live on hardware: `POST /param {slam_nav, lds_active_rpm, 200}` moved the
measured spin rpm from ~297→193 immediately; restored to 300 after.

**2026-07-15: a SECOND, different root cause found — an unreachable goal latches forever
and silently blocks idle-park too (not yet deployed).** User reported "LDS is not
spinning down when the robot is not moving" on live hardware. Traced via
`journalctl -u nano-nav`: a `goal_pose` click during earlier testing (`goal set: (0.78,
-0.09)` at 20:21:18) hit `no path to goal` at 20:24:26/33 and was never cleared — no
mechanism in `nav_node.py` ever gives up on a goal with zero path. 25+ minutes later the
goal was still latched, live telemetry's `plan` field was still the same stale 2-point
path, and `_update_lds_idle`'s `needs_scans` counts `self._goal is not None` as a reason
to stay active — so the LDS was correctly never parking, even though the robot was
completely stationary the whole time. `stuck_timeout` (existing watchdog) doesn't cover
this case: it only fires when *commanded but not moving*, and an unreachable goal with an
empty path never commands motion at all.

**Fix**: new `goal_no_path_timeout` param (20.0s default, 0=off, live-tunable via
`/param`) in `nav_node.py` — if replanning finds zero path for longer than this, the goal
is abandoned (`"goal abandoned: no path for Ns"` log, goal/path cleared, published empty).
Reset on every fresh goal (`_on_goal`/`_on_go_home`) so a new goal always gets its own
full grace period. `_explore_step` frontiers are unaffected (only adopted if already
confirmed reachable). Whitelisted for `/param` (`PARAM_WHITELIST["slam_nav"]`) and
documented in `robot.yaml` + the web UI's Navigation card hint. Smoke test + full colcon
build both pass; **not yet deployed to the board**.
