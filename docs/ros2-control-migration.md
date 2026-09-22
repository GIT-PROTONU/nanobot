# ros2_control Migration Plan — standardize the motor-control layer, delete custom SBC drive code

Status: **PLANNED** (2026-09-22). Not started. User decisions baked in: full
`ros2_control` migration; **ESP32 per-wheel PID STAYS on the ESP32** (cascaded —
the inner 50 Hz loop is immune to SBC load and is NOT part of this migration);
canned moves route through **Nav2 `/goal_pose`**; **minimize custom code** —
every standard component that can replace custom code gets used.

Goal: replace the custom SBC motor-control plumbing (`wheel_odometry` node, the
web drive keepalive thread, the web drive clamps, the canned-move maneuver
thread, the firmware body-frame kinematics) with the canonical ROS 2
diff-drive stack: `controller_manager` + `diff_drive_controller` +
`joint_state_broadcaster` + a minimal C++ `SystemInterface` plugin + `twist_mux`,
and delete ~3× the custom code it adds.

## Verified facts that pin down the design

- **Packages available on robostack-staging** (Humble, checked 2026-09-22):
  `ros-humble-ros2-control 2.54.0`, `ros-humble-diff-drive-controller 2.53.1`,
  `ros-humble-controller-manager 2.54.0`, `ros-humble-twist-mux 4.3.0` (+ the
  `joint_state_broadcaster`/`velocity_controllers` metapackages). None are
  installed today — `pixi list` has zero ros2_control surface, and
  `grep -rn "ros2_control|controller_manager|diff_drive_controller|..."` across
  `src/ scripts/ firmware/ docs/` returns NOTHING. The current stack is 100%
  custom on the motor path.
- **diff_drive_controller (Humble) contract** (verified `control.ros.org/humble`
  userdoc): subscribes `~/cmd_vel` (TwistStamped, `use_stamped_vel=true`) or
  `~/cmd_vel_unstamped` (Twist, `use_stamped_vel=false`); publishes `~/odom`
  + `/tf` (`enable_odom_tf`); params `left_wheel_names`/`right_wheel_names`,
  `wheel_separation`, `wheel_radius`, `cmd_vel_timeout` (default 0.5 s — the
  dead-man), `publish_rate`, `position_feedback` (true = integrate joint
  POSITION state), `open_loop`, `odom_frame_id`/`base_frame_id`, and
  task-space limits `linear.x.has_velocity_limits`/`max_velocity`,
  `linear.x.has_acceleration_limits`/`max_acceleration`, `angular.z.*`.
  Feedback = wheel position (or velocity) state interfaces; output = wheel
  VELOCITY command interfaces. **It is open-loop in the body frame** — it does
  NOT do closed-loop heading hold during translation; straightness comes from
  the inner per-wheel velocity loops (the ESP32 PID).
- **Nav2 Humble publishes plain `geometry_msgs/Twist` on `/cmd_vel`** — no
  `use_stamped_vel`/`cmd_vel_stamped` exists in Humble's controller_server
  (TwistStamped support landed in Iron). So `diff_drive_controller` gets
  `use_stamped_vel: false` (Twist mode, `~/cmd_vel_unstamped` remapped to
  `/cmd_vel`). All four current `/cmd_vel` publishers are Twist too.
- **`/cmd_vel` publisher census** (grep-verified): ① web_control teleop
  keepalive `web_server.py:575` (the "SOLE publisher" by design),
  ② web_control skill-action tier `web_server.py:699` (gated by
  `skills_allow_actions`), ③ `imu_interference.py:57` (self-test),
  ④ Nav2 `controller_server` (Humble RPP, default topic). Plus the sim-only
  `gz_bridge.yaml:13-17` ROS→GZ bridge. Four real publishers, no arbitration →
  `twist_mux` becomes the standard replacement for the keepalive's implicit
  priority.
- **`/odom` subscriber census**: `telemetry.py:663` (lazy, reads pose only —
  no twist/covariance), `sys_monitor/monitor_node.py:143` (staleness stamp
  only), Nav2 `controller_server` + `bt_navigator` (`nav2_params.yaml:98,177`),
  `scripts/odom_drift.py:66`, RViz `nano.rviz:44`. All read the standard
  `nav_msgs/Odometry` layout with `odom`/`base_link` frames — diff_drive_controller
  is a transparent swap with `odom_frame_id: odom`, `base_frame_id: base_link`.
  Covariance: wheel_odometry publishes zeros; diff_drive_controller defaults to
  zeros; nothing consumes nonzero covariance.
- **`/joint_states`**: published lazily by wheel_odometry (`encoder_node.py:86`,
  only when subscribed); the ONLY consumer is `robot_state_publisher`
  (`bringup.launch.py:69-71`, `visualize.launch.py:34-35` — default topic, no
  remap). `joint_state_broadcaster` replaces it 1:1.
- **`/wheel_encoders`** (`robot_msgs/WheelEncoders`): published lazily by
  wheel_odometry (`encoder_node.py:87,213`), **zero subscribers anywhere**.
  `robot_msgs` is a single-message package used ONLY by wheel_odometry; the
  `robot_bringup/package.xml:17` exec_depend is already stale. Both deletable.
- **`/reset_ticks`**: web button `index.html:1450` → gateway whitelist
  `telemetry.py:232` → firmware `main.cpp:814` (zeros raw+stray counters).
  wheel_odometry ALSO subscribes it (`encoder_node.py:110,158`) to re-seed its
  prev-tick baseline so /odom doesn't see a phantom jump. After migration the
  re-seed moves into the hardware interface (a `/reset_ticks` sub that drops its
  own baseline). Web + firmware sides unchanged.
- **`sensor_hub` decoupling**: `hub.py:33` (import) + `hub.py:37`
  (`NODE_CLASSES` tuple) are the ENTIRE coupling — "No inter-node dependencies,
  so construction order is irrelevant" (`hub.py:36`). Removing EncoderNode is a
  2-line change; per-node init is already try/except-wrapped (`hub.py:87-90`).
- **`sim_bridge_node` is NOT fully deletable**: five jobs — (a) wheel-tick
  synthesis from gz joint states (`sim_bridge_node.py:78-80,119-130`) — THIS
  part dies with `gz_ros2_control`; (b) gz IMU → `/imu/euler`+`/imu/web`
  (`:83-85,133-150`), (c) gz `/scan` → the `nano_scan.bin` shm blob
  (`:89,153-155`), (d) synthetic ESP32 board telemetry (`:92-101,158-166`),
  (e) no-op sinks (`:104-106`). (b)-(e) survive: `imu_driver`/`lds_driver_py`
  are NOT launched in sim (`bringup.launch.py:92-97` `UnlessCondition(sim)`), so
  sim_bridge IS the IMU/scan source there.
- **Serial budget is the hard constraint**: `/cmd_vel` crosses the 115200-baud
  UART2 (zenoh-pico) to the ESP32; SUSTAINED 10 Hz flow decays the link
  (zenoh-pico's tiny UART RX FIFO, measured 2026-09-20 — 3 Hz cruises clean).
  This is why the keepalive is 3.3 Hz and Nav2 `controller_frequency` is 4.0.
  Under ros2_control the serial flow becomes the hardware interface's
  `/wheel_cmd` publishes — its cadence must be throttled BELOW update_rate
  (re-assert on change OR every ~200 ms, whichever first), which also feeds the
  firmware's 500 ms `CMD_TIMEOUT_MS` dead-man.
- **ESP32 firmware facts** (`firmware/nanobot_coprocessor/src/main.cpp`):
  `cmd_cb` at `:641-665` parses raw CDR `hdr(4)+6×f64`, reads linear.x at
  body+0 and angular.z at body+44, clamps to `g_maxlin`/`g_maxang`, computes
  the kinematics `vl = fv - fw*g_wsep*0.5f; vr = fv + fw*g_wsep*0.5f` (`:653`)
  → PID setpoints `g_left_tgt`/`g_right_tgt` (`:655`). The PID loop ticks at
  `WHEEL_PID_HZ 50` in `loop()` (`:1278-1400`), `wheelPid` core at
  `:1069-1097`. SUBS table at `:808-828` (`cmd_vel` entry `:810`). Pubs table
  at `:583-600` — `/wheel_ticks` ~15 Hz (`:914-922`). `/motor_pid`/`/motor_params`
  (`:725-760`, `set_param` `:442-456`) + `/wheel_pid`/`/wheel_params` readbacks
  (`:942-962`) are the live-tuning surface — ALL STAY (the PID is firmware-side).
  Geometry: `ticks_per_rev 253` (NVS), `wheel_radius 0.0335`,
  `wheel_separation 0.102` — currently ALSO in `robot.yaml wheel_odometry.*`
  (lines 38-60) and STALE in the URDF xacro (`nano.urdf.xacro:21` says 0.16).
- **robot.yaml drive blocks** (verbatim in `web_control:ros__parameters`,
  lines 317-349): `drive_max_lin: 0.4`, `drive_max_ang: 1.0`, `drive_timeout:
  0.6`, `move_lin_speed: 0.12`, `move_ang_speed: 1.0` (+ `move_max_dist/deg`,
  `move_turn_kp`, `move_timeout` declared at `web_server.py:599-605`, persisted
  to `~/.local/state/nanobot/move.json` — the "persisted UI wins" pattern).
  All become controller params or die with the maneuver thread.

## Target architecture

```
web teleop /drive  ─▶ /cmd_vel_teleop  (prio 10) ┐
Nav2 RPP           ─▶ /cmd_vel_nav     (prio 5)  ├▶ twist_mux ─▶ /cmd_vel ─▶ diff_drive_controller
imu self-test      ─▶ /cmd_vel_test    (prio 1)  ┘                │ body(v,w)→wheel(vl,vr), odom, TF,
                                                                  │ cmd_vel_timeout, task-space limits
                                                                  ▼
                                              controller_manager (C++, update_rate ~20 Hz)
                                               ├ diff_drive_controller   → /odom + odom→base_link TF
                                               ├ joint_state_broadcaster → /joint_states
                                               └ nano_hardware_interface (the ONE custom C++ plugin)
                                                     │ write(): /wheel_cmd [vl,vr] m/s   ──UART2/zenoh──▶ ESP32
                                                     │ read():  /wheel_ticks [L,R]       ◀──UART2/zenoh── ESP32
                                                     ▼                                          │
                                              (firmware: NO kinematics, NO Twist)       per-wheel PID @50 Hz
                                                                                              │
                                                                                     /odom consumers unchanged:
                                                                                     telemetry, sys_monitor, Nav2,
                                                                                     slam_toolbox (TF), RViz
```

Canned moves: `POST /move` → compute goal = current `map→base_link` TF pose
translated `dist` m / rotated `deg` → publish `/goal_pose` (frame `map`) →
Nav2 RPP drives with closed-loop heading (straight by construction) → progress
from the EXISTING `f.nav.status` (`/navigate_to_pose/_action/status`); cancel =
`POST /nav/cancel` (exists).

## Net custom-code change (~3:1 deletion)

**Deleted (~950 lines):**

| Code | Where | Replaced by |
|---|---|---|
| `wheel_odometry` package (node, setup, tests) | `src/wheel_odometry/` | diff_drive_controller (/odom+TF) + joint_state_broadcaster (/joint_states) |
| `odom_math.py` + `test_odom_math.py` (midpoint integration, quat) | `src/wheel_odometry/` | the controller's odometry |
| `robot_msgs` package (WheelEncoders.msg only, zero consumers) | `src/robot_msgs/` | deleted outright (+ stale `robot_bringup/package.xml:17` dep) |
| drive keepalive thread (`_drive_loop`, `_publish_drive`, `_drive_thread`, `_drive_lock`, `_drive_v/_w`, `BRAKE_GRACE`) | `web_server.py:575-592,875-919,142` | `cmd_vel_timeout` |
| drive clamps + `drive_max_lin/ang` params | `web_server.py:838-873,570-572` | `linear.x.max_velocity` / `angular.z.max_velocity` |
| canned-move machinery (`_maneuver_step`, `_man_loop`, `_run_maneuver`, `_clamp_move_cfg`, `_wrap_angle`, `MOVE_*` constants `:127-152`, `move_*` params + move.json persistence `:599-614,1059-1084`, `f.move` frame) | `web_server.py` | ~30-line Nav2 goal publisher |
| `test_maneuver.py` | `src/web_control/test/` | gone (canned moves use Nav2) |
| EncoderNode in sensor_hub | `hub.py:33,37` (+docstring `:1-3`) | nothing (2-line removal) |
| telemetry whitelist `wheel_odometry.publish_rate` | `telemetry.py:115` | controller params |
| sim_bridge wheel-tick synthesis | `sim_bridge_node.py:57-80,119-130` | gz_ros2_control |
| firmware body-frame kinematics `vl=v-w·sep/2` + Twist parse | `main.cpp:641-665` | diff_drive_controller |

**Added (~290 lines, of which only ~150 is logic):**

| Piece | Size | Note |
|---|---|---|
| `src/nano_hardware_interface/` C++ SystemInterface plugin | ~150 | THE one irreducible custom piece — ros2_control HW interfaces are C++-only in Humble; no generic zenoh-serial plugin exists |
| `config/controllers.yaml` | ~40 | standard config |
| URDF `<ros2_control>` block | ~15 | standard |
| `twist_mux.yaml` | ~20 | standard node |
| canned-move goal publisher | ~30 | the feature itself |
| firmware `/wheel_cmd` Float64MultiArray sub | ~20 changed | mirrors `motor_pid_cb`'s MultiArray CDR parse (`main.cpp:725-737`) but f64 |
| `nano-control.service` + `unit_exec.sh control` case | ~15 | standard systemd wiring |

## Phases

### Phase 1 — deps + URDF
- `pixi.toml` top-level `[dependencies]` (BOTH platforms — the board runs
  controller_manager): `ros-humble-ros2-control`,
  `ros-humble-controller-manager`, `ros-humble-diff-drive-controller`,
  `ros-humble-joint-state-broadcaster`, `ros-humble-velocity-controllers`,
  `ros-humble-twist-mux`.
- `nano.urdf.xacro`: fix the stale `wheel_separation` 0.16 → 0.102 (line 21);
  add a `<ros2_control name="nano" type="system">` block: both wheel joints,
  `<command_interface name="velocity"/>`, `<state_interface name="position"/>`
  + `<state_interface name="velocity"/>`, `plugin` = `nano_hardware_interface/NanoSystem`.
  Joints stay `type="continuous"`, axis `0 1 0`.

### Phase 2 — `nano_hardware_interface` (new C++ package)
- `SystemInterface` plugin, 2 joints. `on_init` reads params
  (`ticks_per_rev`, `wheel_radius`, `invert_left/right`) from
  `controllers.yaml` (ONE source of truth now).
- `read()`: `/wheel_ticks` sub on a helper thread → double-buffered joint
  position (rad = ticks·2π/ticks_per_rev) copied into state interfaces;
  velocity state derived from Δticks/dt. `/reset_ticks` sub → drop the
  baseline (replaces `encoder_node.py:158`).
- `write()`: per-wheel velocity cmd (m/s = rad/s·wheel_radius) → `/wheel_cmd`
  Float64MultiArray `[vl,vr]`, **re-asserted on change OR every ~200 ms**
  (whichever first) — decouples serial traffic from `update_rate`, keeps the
  link in the tested-safe regime, feeds the ESP dead-man.
- Async pattern: ROS pub/sub on a non-realtime thread; lock-free copies into
  the realtime cycle (the standard ros2_control async HW interface pattern —
  NEVER ros calls inline in read/write).
- `plugin.xml` + `package.xml` export `ros2_control::SystemInterface`;
  ament_cmake. Verify `scripts/build.sh` builds a repo C++ package (it
  currently builds conda C++ + Python only; colcon should just work, but the
  CMake Python hints there are for the Python pkgs).

### Phase 3 — controller + mux configs
- `config/controllers.yaml`:
  `controller_manager.update_rate: 20` (odometry quality; serial throttled in
  the HW iface); `diff_drive_controller`: `left_wheel_names: [left_wheel_joint]`,
  `right_wheel_names: [right_wheel_joint]`, `wheel_separation: 0.102`,
  `wheel_radius: 0.0335`, `open_loop: false`, `position_feedback: true`,
  `enable_odom_tf: true`, `odom_frame_id: odom`, `base_frame_id: base_link`,
  `cmd_vel_timeout: 0.6`, `publish_rate: 15.0`, `use_stamped_vel: false`,
  limits `linear.x.has_velocity_limits: true, max_velocity: 0.4` (= old
  `drive_max_lin`), `angular.z...: 1.0` (= old `drive_max_ang`), acceleration
  limits ≈ the firmware `WHEEL_TGT_SLEW 1.5` m/s² equivalent (tuned live).
  Remap `~/cmd_vel_unstamped → /cmd_vel`.
- `config/twist_mux.yaml`: teleop `/cmd_vel_teleop` prio 10 (lock timeout),
  nav `/cmd_vel_nav` prio 5, test `/cmd_vel_test` prio 1 → `/cmd_vel`.
  Nav2 `controller_server` gets a `cmd_vel`→`/cmd_vel_nav` remap in
  `nav2.launch.py`; `imu_interference.py:57` switches to `/cmd_vel_test`;
  skill actions (`web_server.py:699`) + web `/drive` publish `/cmd_vel_teleop`.

### Phase 4 — firmware (flash required)
- Replace the `cmd_vel` SUBS entry (`main.cpp:810`) with `wheel_cmd`
  (Float64MultiArray `[vl,vr]` m/s); parse mirrors `motor_pid_cb` (`:725-737`)
  but f64. `cmd_cb`'s kinematics line `:653` and the body clamps go away —
  store `g_left_tgt`/`g_right_tgt` directly. Keep a per-wheel speed clamp as
  backstop.
- KEEP: the whole PID block (`:1278-1400`, `wheelPid :1069-1097`), gains, NVS,
  `/motor_pid`/`/motor_params` live tuning (+ web sliders), trim, the 500 ms
  dead-man, `/wheel_ticks` pub. `/motor_params` ids 3/4 (maxlin/maxang) become
  advisory (the controller enforces body limits).
- After ANY flash expect the router serial desync → `sudo -n systemctl restart
  nano-robot.target` (the documented post-flash heal).

### Phase 5 — delete the custom SBC code (same deploy as 4)
- `rm -rf src/wheel_odometry src/robot_msgs`; drop `robot_bringup/package.xml:17`
  stale dep. Remember the colcon stale-glob gotcha: deleting files under a
  package needs `rm -rf build/<pkg> install/<pkg>` on dev AND board.
- `hub.py`: remove EncoderNode import + tuple entry; docstring line.
- `web_server.py`: delete the keepalive thread + `BRAKE_GRACE`, the clamps,
  ALL the maneuver machinery (constants block, `_maneuver_step`,
  `_man_loop`/`_run_maneuver`, `_clamp_move_cfg`, `_wrap_angle`, `move_*`
  params + move.json load/save, `f.move` frame build). `drive()` now just
  publishes `/cmd_vel_teleop` (clamped ONCE against the controller's
  advertised limits for UI display only — enforcement is the controller's).
- `telemetry.py`: drop the `wheel_odometry` whitelist entry (`:115`); `/odom`
  sub (`:663`) and `/wheel_ticks` sub (`:665`) stay (ESP32 still publishes
  ticks for the Coprocessor card).
- Delete `test_maneuver.py`; `test_odom_math.py` goes with the package.

### Phase 6 — canned moves → Nav2 (~30 lines)
- `_run_maneuver` rewrite: TF lookup `map→base_link`, build the goal
  (`dist` m along the current heading, then `deg` rotation about the goal) as
  PoseStamped frame `map`, publish `/goal_pose`. Report progress from the
  existing `f.nav.status`; `POST /move {cancel:true}` → `POST /nav/cancel`.
- **Accepted behavior change**: canned moves now REQUIRE SLAM + Nav2 (need
  `map→odom` TF). No fallback (a fallback = custom code). Clear error to the
  page when Nav2 is down.
- Turn-only moves = goal at current position, rotated heading (RPP
  `rotate_to_heading` handles it).

### Phase 7 — systemd + launch
- `nano-control.service` (Type=simple, like nav): `unit_exec.sh control` →
  `controller_manager --ros-args --params-file controllers.yaml` (+ a loader
  step or `spawner` for diff_drive_controller/joint_state_broadcaster/twist_mux).
  `After=nano-router nano-sensors` (needs `/wheel_ticks`); add to
  `nano-robot.target`; `MemoryMax` + CPUAffinity per the nav-unit pattern.
  Update `stack.sh:35` unit list.
- `bringup.launch.py` (dev path): add controller_manager + spawners +
  twist_mux.

### Phase 8 — Gazebo sim parity
- Replace `gz-sim-diff-drive-system` (`nano.urdf.xacro:110-123`) + the
  sim_bridge wheel-tick portion with `gz_ros2_control` loading the same
  controllers.yaml — the sim then runs the REAL diff_drive_controller against
  gz wheel joints. `sim_bridge_node` keeps IMU/scan-blob/ESP32-telemetry/no-op
  jobs (b)-(e); delete its tick synthesis (`:57-80,119-130`) + tick params.
  NOTE `linux-64`-only target section discipline (no board bloat).

### Phase 9 — tests + live verification
- `pixi run smoke`: update for the new /odom + /joint_states publishers and
  the /cmd_vel path (teleop → mux → controller).
- New: HW-interface integration test (mock `/wheel_ticks` → assert `/wheel_cmd`
  out + joint states in).
- **Live board tests (the risk items)**: ① serial — 60 s drive with zero
  delivery loss at the 200 ms re-assert cadence; ② PID parity — KP 5/KI 60
  still tracks (setpoints are the same numbers the firmware used to compute
  internally — should be a no-op); ③ odom parity lap vs the old wheel_odometry
  (lidar self-correlation ruler, per the 2026-09-21 separation fix); ④ dead-man
  — kill web_control mid-drive → `cmd_vel_timeout` → firmware dead-man → stop;
  ⑤ canned straightness — 1 m move, measure lateral drift (now bounded by RPP
  heading control, not just wheel matching); ⑥ post-flash router desync heal
  (target restart), then the ESP-deaf triage via `/esp32_reset`.

## Risks / open items
1. **Serial decay** — the 115200 zenoh-pico FIFO (10 Hz measured failure).
   Mitigation: on-change + 200 ms floor in write(). Fallback: raise UART baud
   both ends (router config + firmware UART2 — already a TODO item).
2. **First repo C++ package** — verify `scripts/build.sh` handles ament_cmake
   sources; watch board RAM/disk during the colcon build.
3. **Canned moves require Nav2/SLAM** — accepted (least code). If regretted
   later, an odom-frame heading-hold fallback is the documented re-add.
4. **`/motor_trim`** stays firmware-side (already tuned); the controller's
   `wheel_separation_multiplier`/radius multipliers are the standard
   equivalent — revisit only if trim misbehaves.
5. **URDF geometry change** (0.16→0.102) affects RViz/Gazebo visuals — cosmetic;
   verify the model still renders sensibly.

## Sequencing
1 → 2 → 3 → 4 (flash) → 5 → 7 as ONE deploy; live-verify (9 items ①-⑥); then
6 (canned moves) can land independently; 8 (sim parity) last.
