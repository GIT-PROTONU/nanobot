---
name: imu-mount-rotation-fixed
description: "2026-07-16: found + fixed the real IMU mounting problem (roll/pitch swapped ~90deg) behind the open selftest-spin-imu-mismatch investigation; full 3-axis mount_{roll,pitch,yaw}_deg correction built, persisted, deployed, hardware-confirmed working"
metadata: 
  node_type: memory
  type: project
  originSessionId: c5961d6a-8b34-4313-9d6d-8ead3f62d4e5
---

Resolves [[selftest-spin-imu-mismatch]]. The IMU's own onboard-fused roll/pitch/yaw
(BWT901CL) were coming out with **roll and pitch swapped** relative to the chassis —
confirmed via a clean 3-pose static test (rest flat, prop up only the front, prop up
only one side; whichever reported angle moved for a given physical tilt revealed the
axis mapping) rather than by guessing. That swap is exactly what a **pure 90° rotation
about the shared vertical (yaw) axis** produces — verified this is an *exact* algebraic
identity, not an approximation, by round-tripping real numbers through the actual
deployed code. On this robot: `mount_yaw_deg=-90` alone fixed it (no roll/pitch mount
component needed); a small residual `mount_roll_deg=13` was also set (likely a genuine
few-degree physical tilt in the mount, separate from the axis-swap). Real values now
live on the board: `mount_yaw_deg=-90, mount_roll_deg=13, mount_pitch_deg=0`, plus
`offset_x_mm=-70, offset_y_mm=-45, offset_z_mm=100` (the translational lever-arm
offset, a separate concept — see below).

**The correction is a full 3-axis rotation, not a shortcut.** `imu_driver/imu_node.py`:
`mount_matrix(roll_deg, pitch_deg, yaw_deg)` builds a fixed 3x3 rotation (pure Python,
no numpy — a handful of multiplies at ~100Hz doesn't need it) representing how the
sensor is twisted/tipped on the chassis. `rotate_mount()` applies it to the RAW
accel/gyro/mag as full 3-vectors every cycle. `correct_orientation()` reconstructs
robot-frame roll/pitch/yaw by converting the sensor's own reported angles to a matrix,
composing with the mount matrix, and re-extracting Euler angles — **not** an algebraic
shift of the three scalars. That shortcut only happens to be exact for a pure-yaw-only
mount (proven: an extra fixed rotation about the shared Z axis just adds to the leading
yaw term of a ZYX decomposition) — a mount with any roll/pitch component genuinely
couples all three angles together, which is exactly the "twisting it changes the
reported pitch" symptom a yaw-only shortcut can't fix. All of this was numerically
verified (built a standalone round-trip check reproducing the exact deployed functions
before shipping) rather than trusted from hand-derived algebra — worth repeating that
discipline if this code is ever touched again, it's easy to get a sign/composition-order
wrong here.

**Persistence had two separate gotchas, both fixed:**
1. **Loaded-but-not-reflected-in-ROS-params.** The persisted file
   (`~/.local/state/nanobot/imu_mount.json`, same convention as `tts.py`'s settings
   file) was being loaded into internal driver state correctly (the actual correction
   math used it fine), but the ROS parameters themselves were never updated to match —
   so `ros2 param get` (and anything else reading params back) still showed the
   robot.yaml default of 0. Fixed by calling `self.set_parameters(...)` right after
   loading, before `add_on_set_parameters_callback` is registered (so it can't recurse
   into the save-on-change handler).
2. **UI showed 0 even when persistence WAS working.** The web UI's offset/mount-rotation
   fields are plain `<input>`s with a static "0" in the HTML — nothing ever told the
   browser what the driver actually loaded. Fixed with a latched status topic
   (`imu_mount_settings`, mirroring the existing `imu_calibrate_status` pattern),
   published on startup and on every change, forwarded into the SSE frame by
   `telemetry.py`, and applied to the six input fields by `app.js` (deduped so it can't
   fight with an in-progress edit — a saved value only republishes after the SAME
   onchange that set it).

**Recurring gotcha, bit us directly this session: a new imu_driver param exposed via
the web UI must ALSO be added to `telemetry.py`'s `PARAM_WHITELIST`, or `POST /param`
silently returns "not whitelisted" and the value never reaches the node** — the web UI
looked broken for a while purely because `mount_yaw_deg` was declared as a ROS param
and wired into the HTML/JS but forgotten in the whitelist dict.

See also [[web-publish-topic-namespace-gotcha]], found while debugging a related "why
doesn't my new button work" report in the same session.
