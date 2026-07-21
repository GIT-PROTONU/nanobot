---
name: vision-target-tracking
description: "2026-07-16 pan-only visual servoing in slam_nav; refined 2026-07-21 with smooth deadband, coast-on-loss, integral, target-velocity feedforward, confidence-scaled authority — code-complete, not hardware-verified"
metadata: 
  node_type: memory
  type: project
  originSessionId: 14e4c608-da7d-428f-98eb-761ede6c133e
---

Added a "track and center" feature (2026-07-16, code-complete + smoke-tested on the
dev PC, NOT yet hardware-verified/deployed; refined 2026-07-21 for smoother + more
accurate tracking): when enabled, the robot turns in place to keep the GPU-vision
calibrated colour-blob target horizontally centered in the camera frame.

**Design decisions (user-confirmed):**
- **Rotate only** — no forward/back approach/standoff-distance control. Simpler, no new
  forward-collision risk.
- **Mutually exclusive with goal nav, track wins** — enabling `track_enable` overrides
  any active click-to-go goal / auto-explore; turning it on is treated as explicit
  current intent.

**Where it lives:** `slam_nav`/`nav_node.py::_track_step`, NOT `web_control`/`mood_node`
— slam_nav already owns continuous `/cmd_vel`, pick-up freeze, `enable_motion` master
switch, and the `trait_motion` caution clamp, so tracking reuses all of that instead of
duplicating safety logic. `mood_node`/`brain.py` are hard-contracted expression-only
(never publish `/cmd_vel`) and `web_control` is a request-response HTTP model, both
wrong fits for a continuous driver.

**Data path:** `gpu_vision.py`'s existing `target` property `(x, y, confidence)`
(0..1 normalized image coords, top-left origin, already mirror-corrected; confidence
doubles as blob-size/closeness) is included in the `/vision/state` JSON topic
(`web_server.py:_vision_state_tick`, 10 Hz — was 2 Hz, bumped 2026-07-16 to match the
control loop 1:1, only published while GPU vision is live) as a `"target"` field.
`slam_nav` subscribes to `/vision/state` and runs the controller, all through the
existing `_send()` (so it's automatically gated by `enable_motion`).

**Controller evolution** (the `_track_step` docstring records all five findings):

1. **2026-07-16 (1st):** originally a pure P-controller:
   `err = x - 0.5`, `w = clamp(-track_kp * err, ±track_max_ang)`, deadband +
   min-confidence + staleness (`track_timeout`) gating. A vision-mirroring flip
   (`err = 0.5 - x`) was briefly tried and **reverted** — a hardware test confirmed
   gpu_vision's x is NOT mirrored.
2. **2026-07-16 (2nd):** added an **age-taper** `fresh = max(0, 1 - age/track_timeout)`
   on the final `w` — at 2 Hz /vision/state the loop was coasting on stale data up to
   0.5 s per update, which no P-gain reduction could fix. Then bumped `/vision/state`
   to 10 Hz to match the control loop.
3. **2026-07-16 (3rd):** hardware STILL overshot → root cause was the robot's own
   turning momentum, not staleness. Added a **D damping term** (`track_kd`) on the
   robot's OWN measured yaw rate (numeric derivative of `/imu/euler`) that brakes
   against its current spin — a PD-on-measurement design.
4. **2026-07-16 (4th):** still overshooting near the setpoint → root cause was the
   ESP32's stiction compensation (`MOTOR_MIN_DUTY`/`MOTOR_DEADZONE`): any |w| below
   ~0.16 rad/s produces zero motion, anything above jumps to ~55% duty — the actuator
   is close to a relay, not smoothly variable. Added a **stiction dither**
   (`track_min_eff_ang`): below the floor, TIME-dither instead — a delta-sigma/PWM-style
   modulator firing full-floor pulses only often enough to average out to the desired
   slow rate. Also raised `track_max_ang` (0.8 → 2.0 rad/s) since duty barely rises
   across the whole range.

5. **2026-07-21 refinement (smoother + more accurate tracking, additive on top of the
   PD + age-taper + stiction-dither above):**
   - **Smooth deadband** (`track_deadband_soft`): `eff_err` linearly ramps from 0 at
     `|err|=deadband` to `err` at `|err|=deadband+soft`, replacing the hard bang-in/out
     at the deadband edge (a classic limit cycle: enter → stop → drift out → snap a
     turn → re-enter) with a smooth blend. 0 = original hard edge.
   - **Coast on transient loss** (`track_coast`): a None frame or sub-conf dip used to
     fire a hard `send(0,0)` and jerk the robot stop-start every time. Now the last
     commanded `w` is held with exponential decay for `track_coast` seconds before
     stopping — a brief occlusion costs a gentle coast, not a stutter.
   - **Integral term** (`track_ki`, OPT-IN/default 0): anti-windup (conditional +
     clamped, max contribution `track_max_ang/2`) cancels steady-state offset from
     camera misalignment or stiction bias. Bled (×0.7) on target loss, (×0.5) on
     deadband. Off by default so existing tuned behaviour is unchanged until opted in.
   - **Target-velocity feedforward** (`track_kff`): EMA-smoothed `dx/dt` of the
     target's normalized x gives predictive lead — the robot starts turning BEFORE the
     visual error grows, so a moving target is followed smoothly instead of always
     lagging by one P-reactive step.
   - **Confidence-scaled authority** (`track_conf_scale`): output is scaled by
     `min(1, conf/track_conf_scale)` so a weak/noisy lock can't yank the robot at full
     P-gain; full authority is restored once `conf >= track_conf_scale`. 0 = disable.

   Final control law (per fresh valid sample, after the smooth-deadband taper):
   ```
   w_raw = -track_kp*eff_err - track_ki*int - track_kd*yaw_rate - track_kff*x_rate
   w     = conf_scale * fresh * clamp(w_raw, ±track_max_ang)   # then stiction-dither
   ```

**Params (`slam_nav`, robot.yaml + telemetry.PARAM_WHITELIST, all live-tunable):**
`track_enable` (bool, default false), `track_kp` (2.0), `track_kd` (0.5),
`track_max_ang` (2.0 rad/s), `track_min_eff_ang` (0.2 rad/s), `track_deadband` (0.04),
`track_deadband_soft` (0.04), `track_conf_min` (0.02), `track_conf_scale` (0.10),
`track_timeout` (0.6 s), `track_coast` (0.2 s), `track_ki` (0.0 = off),
`track_kff` (0.5). The web "▸ Tracking tuning" expandable has sliders for all of them.

**Web UI:** Camera card — "🎯 Track target" toggle + the "▸ Tracking tuning" expandable
section (turn gain / braking / max + min effective turn rate / center deadband + soft
edge / min + full-authority confidence / coast / integral / feedforward), same pattern
as the existing blob-tuning/vision-alerts sections. Hover hints added.

**Not yet done:** hardware verification (turn direction sign, gain/damping tuning,
deadband feel, and now the five 2026-07-21 refinements — does the smooth deadband kill
the limit cycle? does coast-on-loss feel right vs. a hard stop? does `track_kff` help
follow a moving target without overshoot? does `track_ki` cancel the steady offset
without windup?) — the `x=0 left / x=1 right` → `w=-kp*err` sign convention was derived
from gpu_vision's documented mirror-correction comment, not confirmed on the real robot.
See [[gpu-vision-implemented]] for the underlying blob-tracking pipeline, [[slam-nav]]
for the node it extends.
