---
name: web-publish-topic-namespace-gotcha
description: "2026-07-16: go_home/save_map (and the new clear_map) web buttons silently never worked — web_control published to /slam_nav/X while nav_node subscribed to bare /X (no ROS namespace anywhere); fixed, and now a known class of bug to check for"
metadata: 
  node_type: memory
  type: project
  originSessionId: c5961d6a-8b34-4313-9d6d-8ead3f62d4e5
---

Found while investigating "clear map button does nothing" — turned out to be a
**pre-existing bug that also broke `go_home` and `save_map`**, not something new. Both
had presumably never worked over the web UI.

**The bug**: `telemetry.py`'s `/publish` whitelist created ROS publishers with topic
strings like `pub(Bool, "slam_nav/go_home", 5)` — a relative topic that resolves
(since `web_control`'s node has no namespace) to the absolute topic `/slam_nav/go_home`.
But `nav_node.py` subscribes to the bare relative topic `"go_home"`, which resolves to
`/go_home` — because **neither the systemd path (`scripts/unit_exec.sh`) nor
`bringup.launch.py` ever sets a ROS namespace for slam_nav.** Two completely
disconnected topics. The `/publish` endpoint still returned `{"status":"ok"}` every
time, because publishing succeeds regardless of whether anyone is subscribed —
nothing in the request/response path could ever reveal the mismatch.

**How it was actually confirmed** (worth repeating for any future "button does
nothing" report): `ros2 topic list` on the board showed BOTH `/go_home` and
`/slam_nav/go_home` as separate topics with zero overlap — the direct tell. Then
`curl -X POST localhost:8080/publish -d '{"topic":"/slam_nav/go_home","value":true}'`
followed by `journalctl -u nano-nav` (watching for the expected log line, e.g.
`_on_go_home`'s log or the new `_on_clear_map`'s "map cleared by user request") is a
fast way to verify a whitelisted button's effect end-to-end on real hardware, bypassing
the browser entirely. **`pixi run smoke` cannot catch this class of bug** — it only
validates that the whitelist/HTTP plumbing accepts or rejects a topic name; it never
checks that a real subscriber on the other end actually receives the message.

**The fix**: dropped the `"slam_nav/"` prefix from the ROS topic string passed to
`pub(...)` for `go_home`/`save_map`/`clear_map`, so it matches nav_node's bare
subscription — kept the external whitelist KEY (`"/slam_nav/go_home"`, what the browser
POSTs as `topic`) unchanged so `map.js` didn't need to change.

**Takeaway for future whitelist entries**: a `"/slam_nav/..."`-shaped whitelist key in
`telemetry.py`'s `_pubs` dict does NOT imply nav_node is namespaced — check the actual
subscribed topic name in the target node before assuming a prefix will resolve. If
slam_nav (or any node) is ever given a real ROS namespace later, this whole class of
`pub(Bool, "slam_nav/x", ...)` call would need re-auditing in the other direction.

**2026-07-16 (later session): `/motor_trim` added to the whitelist.** New entry
`"/motor_trim": (pub(Float32, "motor_trim", 5), self._mk_motor_trim)` — the ROS topic
is bare `"motor_trim"` (no `/slam_nav/` prefix; the ESP32 subscribes to the bare name,
same namespace situation as the go_home fix). `_mk_motor_trim` rejects values outside
`±TRIM_MAX` (0.30) rather than clamping, so an out-of-range POST is silently ignored
instead of silently saturating — a deliberate deviation from the clamp-everywhere pattern
of the other helpers, because a saturated trim would hide a tuning mistake. The live
value echoes back on the `esp.wheel_trim` field of the `/telemetry` frame (telemetry.py
subscribes `/wheel_trim`) and is shown on the web Coprocessor card's **Wheel trim** slider.
Backed by the ESP32 straight-line trim (see [[esp32-coprocessor]],
[[slam-map-rotation-encoder-trim]]).
