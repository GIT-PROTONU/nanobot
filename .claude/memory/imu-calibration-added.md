---
name: imu-calibration-added
description: "2026-07-16: BWT901CL WitMotion accel+mag calibration wired into imu_driver + web UI (5-byte hex protocol); code-complete, NOT hardware-verified"
metadata: 
  node_type: memory
  type: project
  originSessionId: 567bcedb-2fbf-428c-9301-7b555331b19b
---

Built 2026-07-16, motivated by the open [[selftest-spin-imu-mismatch]] investigation
(magnetometer interference is one of the two live hypotheses for the erratic self-test
SPIN yaw). **Committed + smoke-tested on the dev PC, NOT yet deployed/run against the
real BWT901CL hardware** — the actual calibration quality/behavior is unverified.

**Protocol**: WitMotion's documented 5-byte hex commands over the same UART the driver
already uses (`imu_driver/imu_node.py` already sent the unlock frame for RSW/RRATE
config, so this reuses that exact wire pattern): unlock (`FF AA 69 88 B5`) → start accel
cal (`FF AA 01 01 00`, ~3s, robot must be still+level) or start/stop mag cal (`FF AA 01
02 00` / `FF AA 01 00 00`, robot physically rotated through all orientations while
active) → save to flash (`FF AA 00 00 00`, otherwise resets on power-cycle).

**Wiring**: `imu_node.py` — new `imu_calibrate` (String) sub sets a `threading.Event`
(`_cal_pending`) + pending command, serviced in the reader thread (same thread-ownership
rule as the existing `_need_reconfig` pattern — `self.ser` must only be touched there);
`_do_calibrate` does the actual write, `imu_calibrate_status` (latched String) reports
back. `telemetry.py` whitelists `/imu_calibrate` (validated to the 4 known commands,
`_mk_calibrate`) and adds a lazy `/imu/mag` subscription + the latched status to the SSE
frame (`imuMag`, `imuCalStatus`). Web UI: IMU card in `index.html` gets 4 buttons
(Calibrate accel / Start+Stop mag cal / Save) and a `mag xyz` live readout.

**"Check calibration" is a live-readout eyeball check, not a readback command** — the
WitMotion protocol has no documented "read calibration status" register, only
write/action ones. Practical check: after accel cal, `|accel|` (already in the IMU card)
should settle near ~9.8 m/s² when stationary; after mag cal, `mag xyz` should sweep
smoothly through a symmetric range when the robot is rotated, not clip/stick at one
value. Both ride existing/new telemetry fields, no new hardware readback needed.

**Next step before this is useful**: run it on the actual robot and see whether it
measurably tightens the self-test SPIN drift documented in
[[selftest-spin-imu-mismatch]] — if the drift persists after a clean mag cal, that
points harder at wheel slip instead.

**2026-07-16 follow-up (same day, later session): SLAM error margins in the IMU card.**
User asked "what error values are within margin for good SLAM" — added live
green/amber/red grading (client-side only: `app.js` `slamGrade()` + HINTS entries +
`index.html` hint text; no server change, smoke-tested). Margins are DERIVED from
slam_nav's real tuning, not guesses: scan matcher corrects heading up to `match_ang`
0.12 rad = **±6.9°/scan**, parked still-skip re-matches after `still_ang` ~0.3°, so slow
drift is continuously absorbed — the map-loser is a sudden mag *jump* > ~7° between
scans. Graded readouts (stationary-gated via the drift check's `still_s`): yaw drift
rate ≤1°/min green / <6 amber / ≥6 red; yaw total vs half-window 3.5°; roll/pitch drift
0.3°/1°; |accel| 9.81±0.3/±1; |gyro| ≤1/≤3 °/s; mag noise now also shown as **% of
field** (~0.6° heading wobble per 1%), ≤2% green / ≤6% amber. NOT yet deployed to the
board.
