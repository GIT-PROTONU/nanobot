---
name: slam-loop-closure
description: "2026-07-16: true loop closure / drift correction added to slam_nav — lowest-CPU/RAM variant (no pose graph; map is the keyframe store), on by default, deployed"
metadata:
  node_type: memory
  type: project
---

# True loop closure / drift correction in slam_nav (2026-07-16)

The `map->odom` TF (2026-07-16 earlier session) *closed the localization loop* but did NOT
correct accumulated drift. The remaining gap ("true loop closure (drift)", noted in
[[slam-nav]]) is now implemented — the correlative scan-to-map match is no longer only a local
stand-in.

## Design (chosen for LOWEST CPU/RAM impact)
No keyframe pose graph, no stored scans. **The accumulated `log`/`seen` grid IS the keyframe
store.** Loop closure = periodically re-matching the current scan against the map centered on
the *odometry-predicted* (drifted) pose with a WIDE window; a strong match that lands far from
the corrected pose reveals the accumulated global offset, which is bled off smoothly.

- `occupancy.GridMap.transform(dx,dy,dth)` — rigid rotate+translate of the grid (pure numpy,
  nearest-cell resample). Called ONLY on loop events. Lossy on thin walls, but each loop event
  immediately re-integrates the current scan (re-fortifying walls), so net loss stays bounded.
- `nav_node._predict(px,py,pth, off=None)` — folds `_loop_off` into the prior; `off=(0,0,0)`
  lets the probe measure the raw drifted chain without feedback.
- `nav_node._maybe_loop_close(va, vr)` — every `loop_probe_every` scans: wide `match()` against
  the offset-free prior; if `score >= loop_score` AND it disagrees with the corrected pose by
  >`loop_min_shift`, nudge `_loop_off` toward the correction by `loop_alpha` (smoothing), and
  warp the grid by the small step once it exceeds `loop_apply_thresh`. Reset on `clear_map`.

## Params (declared in nav_node.__init__, live-tunable, whitelisted in telemetry.PARAM_WHITELIST)
`loop_closure` (true), `loop_probe_every` (50), `loop_lin` (0.5 m), `loop_ang` (1.0 rad),
`loop_score` (4.0), `loop_min_shift` (0.3 m), `loop_alpha` (0.1), `loop_apply_thresh` (0.05 m).
robot.yaml has them under `slam_nav:`; telemetry.py whitelists all 8 for POST /param.

## Cost (measured/estimated)
- RAM: ~48 B steady state; transient ≤~1.1 MB only during a rare grid warp (freed after).
- CPU: +1 `match()` every ~50 scans (a few ms every ~10 s at 300 rpm) + a few float ops
  otherwise → <0.1% of one core averaged.

## Verification
- `src/slam_nav/test/test_occupancy.py` (new, pure-numpy): transform shift correctness, small-
  warp consistency, synthetic drift-loop recovery (score≥4.0, drift>0.3 m detected).
- End-to-end math sim: `_loop_off` converges to cancel a (0.4,-0.3,0.3) drift in ~60 detections.
- `pixi run build` (14 pkgs) + `pixi run smoke` (gateway/whitelists/vitals/shutdown all PASS).
- Deployed to board via `scripts/deploy.sh slam_nav`; `nano-nav` UP; `ros2 param get` confirmed
  loop_closure=True, loop_probe_every=50, loop_alpha=0.1, loop_min_shift=0.3.
- NOT yet done: a real loop drive on hardware to confirm the map stops skewing on return.

## NOTE — pytest run gotcha
The dev host's `pixi run python -m pytest` is BROKEN (launch_testing plugin conflict) — see
[[pytest-run-gotcha]]. Ran the occupancy tests as a standalone module import instead.

See also [[slam-nav]], [[slam-map-rotation-encoder-trim]] (the map-rotation fix that came first).
