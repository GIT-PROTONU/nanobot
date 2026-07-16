---
name: vision-target-tracking
description: "2026-07-16 pan-only visual servoing added to slam_nav — turns to keep the calibrated colour-blob target centered; code-complete, not hardware-verified"
metadata: 
  node_type: memory
  type: project
  originSessionId: 14e4c608-da7d-428f-98eb-761ede6c133e
---

Added a "track and center" feature (2026-07-16, code-complete + smoke-tested on the
dev PC, NOT yet hardware-verified/deployed): when enabled, the robot turns in place to
keep the GPU-vision calibrated colour-blob target horizontally centered in the camera
frame.

**Design decisions (user-confirmed):**
- **Rotate only** — no forward/back approach/standoff-distance control. Simpler, no new
  forward-collision risk.
- **Mutually exclusive with goal nav, track wins** — enabling `track_enable` overrides
  any active click-to-go goal / auto-explore; turning it on is treated as explicit
  current intent.

**Where it lives:** `slam_nav`/`nav_node.py`, NOT `web_control`/`mood_node` — slam_nav
already owns continuous `/cmd_vel`, pick-up freeze, `enable_motion` master switch, and
the `trait_motion` caution clamp, so tracking reuses all of that instead of duplicating
safety logic. `mood_node`/`brain.py` are hard-contracted expression-only (never publish
`/cmd_vel`) and `web_control` is a request-response HTTP model, both wrong fits for a
continuous driver.

**Data path:** `gpu_vision.py`'s existing `target` property `(x, y, confidence)`
(0..1 normalized image coords, top-left origin, already mirror-corrected; confidence
doubles as blob-size/closeness) is now included in the existing `/vision/state` JSON
topic (`web_server.py:_vision_state_tick`, 2 Hz, only published while GPU vision is
live) as a new `"target"` field. `slam_nav` subscribes to `/vision/state` and runs a
P-controller: `err = x - 0.5`, `w = clamp(-track_kp * err, ±track_max_ang)`, deadband +
min-confidence + staleness (`track_timeout`) gating, all through the existing `_send()`
(so it's automatically gated by `enable_motion`).

**New params (`slam_nav`, robot.yaml + telemetry.PARAM_WHITELIST, all live-tunable):**
`track_enable` (bool, default false), `track_kp` (2.0), `track_max_ang` (0.8 rad/s),
`track_deadband` (0.04), `track_conf_min` (0.02), `track_timeout` (0.6s).

**Web UI:** Camera card — "🎯 Track target" toggle + a "▸ Tracking tuning" expandable
section (turn gain / max turn rate / deadband / min confidence), same pattern as the
existing blob-tuning/vision-alerts sections. Hover hints added.

**Not yet done:** hardware verification (turn direction sign, gain tuning, deadband
feel) — the `x=0 left / x=1 right` → `w=-kp*err` sign convention was derived from
gpu_vision's documented mirror-correction comment, not confirmed on the real robot.
See [[gpu-vision-implemented]] for the underlying blob-tracking pipeline, [[slam-nav]]
for the node it extends.
