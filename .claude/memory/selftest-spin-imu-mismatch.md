---
name: selftest-spin-imu-mismatch
description: "2026-07-15: robot's first live drive — self-test SPIN check fails erratically (IMU yaw vs odom vs commanded all inconsistent run-to-run); open question, not yet resolved"
metadata: 
  node_type: memory
  type: project
  originSessionId: 45be2e8a-ac2f-4486-96cb-ab8b2dfd0218
---

**2026-07-15: robot drove under its own power for the first time** (after the
`INVERT_RIGHT` harness-pin fix, see [[esp32-coprocessor]]). Live testing surfaced two
problems: self-calibration (`nav_node.py`'s `_start_selftest`) not passing, and the SLAM
map coming out wrong.

**Self-test SPIN leg is inconsistent run-to-run, not just off by a fixed scale**:
- Run 1 (20:21): `cmd +360deg, IMU +522, odom +93` → IMU over-reports by 45%, odom
  under-reports to ~1/4 turn.
- Run 2 (20:25, ~3 min later, same commanded motion): `cmd +360deg, IMU +70, odom +55` →
  both now way under — this time trips the harder `IMU YAW NOT TRACKING` fail branch.
- FWD leg both times: right wheel ~15-20% slower than left (`R/L=0.84`, `0.81`).
- REV leg symmetry swung `0.92` → `2.05` between runs.

Ruled out as the cause: the self-test code itself (checked — `/imu/euler` feed is 5Hz,
well under the wrap-aliasing threshold for a 0.6 rad/s spin, so `_accum_rotation`'s
per-sample wrapping can't fabricate this swing); `publish_rate: 1.0` on `imu_driver` is a
red herring — `/imu/euler` is fed by the actual device stream, not throttled to the
CPU-reduction 1Hz publish rate.

**Two live hypotheses, unresolved — need eyes-on-hardware to distinguish**:
1. **Wheel slip during in-place spin** (e.g. on carpet) — would produce exactly this
   run-to-run-inconsistent over/under pattern and isn't a software bug; wheel_separation
   tuning wouldn't fix it.
2. **Magnetometer interference** — the BWT901CL's yaw fusion uses the onboard mag; if it's
   mounted close to a motor or a lot of ferrous chassis, motor current/proximity is a
   classic cause of a wandering compass heading that looks just like this.

Asked the user to check the test surface and IMU mounting; answer not yet received.

**Also explains the bad map**: `slam_nav`'s `use_imu_yaw: true` trusts this same IMU yaw
delta for every scan-match rotation prior, not just the self-test — and the session's
`journalctl -u nano-nav` log was near-continuous `localization lost` / `relocalize timed
out` the entire ~8+ minutes observed, consistent with an unreliable rotation prior
corrupting the map on every turn. This same relocalize churn also fed into a second,
independently-found bug — see [[lds-idle-spindown-ui-fix]] (an unreachable goal from this
same test session left the LDS idle spin-down permanently blocked).
