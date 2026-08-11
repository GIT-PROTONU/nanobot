---
name: web-map-clear-buttons-nogo
description: "2026-08-11: web map Clear/Home/Save/Test buttons + the /dev/shm nano_nogo.bin blob — clear now drops goal/path + resets the no-go mask; the blob's transient 'vanishing' was the deploy-restart teardown, not a code path"
metadata:
  node_type: memory
  type: project
---

# Web map buttons + the no-go (nogo) blob (verified live 2026-08-11)

The map buttons were a pile-up of two bug classes, both fixed in `71f2462`-era work and
verified end-to-end on the board.

## The button issue
`index.html`'s map handlers (~`3313-3329`) were still `map.js`-era stubs: `mapHome`/
`mapSave`/`mapTest`/`mapStop` referenced dead ROSLIB topic globals (`homeTopic`/
`saveTopic`/`testTopic`/`cmdTopic`), and `mapClear` had **no handler at all** — so the
"Clear map" button silently did nothing. They were rewired to the SSE gateways:

- `mapHome` → `pub("/slam_nav/go_home", true)`
- `mapSave` → `pub("/slam_nav/save_map", true)`
- `mapClear` → confirm → `pub("/slam_nav/clear_map", true)` AND the page clears its
  `mapGoal` marker + `mapPlan` path locally (telemetry only emits `f.plan` while
  non-empty, so the stale lines would have lingered) — see index.html `~3317`
- `mapTest` → `pub("/selftest", true)`
- `mapStop` → `sendDrive(0, 0)`

All four topics are whitelisted in `telemetry.py` `_pubs` with bare ROS topic names
(`go_home`/`save_map`/`clear_map` — the `web-publish-topic-namespace` gotcha, not
`/slam_nav/...`).

## `_on_clear_map` semantics (nav_node.py ~1309)
A clear wipes the grid and additionally:
- `self._nogo_dirty = True` — so `/dev/shm/nano_nogo.bin` is **rewritten to a
  `count: 0` mask** (230472 B = header + 480×480 tiles). Without this, the old no-go
  overlay kept showing cleared zones.
- Drops `self._goal` / `self._goal_is_frontier` / `self._path` and calls
  `_publish_path()` to flush the browser's green plan line — else the robot keeps
  driving toward a point that no longer exists on the fresh map.

Live-verified: clear → header `{"w":480,"h":480,"res":0.05,"ox":-12,"oy":-12,
"count":0}`; `/map/nogo` serves HTTP 200; `save_map` → "map saved to
/home/ibster/nanobot/maps/nano_map.npz"; `go_home` → "go home: heading to map origin";
`selftest` → correctly gated ("self-test: enable motion first" while motion off).

## The nogo blob "vanishing" — NOT a code bug
During the deploy, `/dev/shm/nano_nogo.bin` appeared right after a clear, then
disappeared ~18-66 s later; even a hand-written dummy file vanished. Hunted for a
deleter (grepped both repos, tmpfiles.d, crons, timers, `os.remove`/`os.replace`
sites, `map_bridge_node`, web_server `_serve_shm`) — there is none. Root cause: **all
vanishings were inside the `stack.sh restart` teardown window right after `deploy.sh`**
(old processes + the `user@1000` session being torn down); once the stack settled, the
blob persisted across clears and idle time indefinitely (20+ min checkpoint). Moral: a
missing `/dev/shm` blob right around a restart is teardown, not a logic bug — re-check
after the stack is quiet before chasing a deleter.

See also [[slam-nav]], [[slam-autonomy-pickup-relocalize]],
[[single-webui-from-sbc]], [[web-publish-topic-namespace-gotcha]].