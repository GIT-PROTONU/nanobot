---
name: lds-idle-spindown-ui-fix
description: "2026-07-14: consolidated all LDS UI into one card + fixed why idle spin-down was never observed (web UI reconnect resync fought slam_nav's owned topic)"
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
