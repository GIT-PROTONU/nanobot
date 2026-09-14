# Nav2 Migration Plan — replace custom `slam_nav` with Nav2 + slam_toolbox

Status: **EXECUTED 2026-09-14** (see "Execution notes" at the bottom for the four
deviations found while running it live). The checked-in facts below were verified
against upstream Nav2 Humble 1.1.20 + the slam_toolbox `ros2` source and held,
except where corrected in the notes.
Goal: single-process Nav2 stack (planner_server + controller_server + bt_navigator +
behavior_server + lifecycle_manager + slam_toolbox in ONE ComponentContainer), lean RAM/CPU.

## Verified upstream facts that pin down the design

- Humble servers are registered rclcpp components (`rclcpp_components_register_nodes`):
  `nav2_planner::PlannerServer`, `nav2_controller::ControllerServer`,
  `nav2_bt_navigator::BtNavigator`, `behavior_server::BehaviorServer`,
  `nav2_lifecycle_manager::LifecycleManager`, and slam_toolbox
  `RCLCPP_COMPONENTS_REGISTER_NODE(slam_toolbox::AsynchronousSlamToolbox)` — all can be
  composed into ONE `ComposableNodeContainer` (`launch_ros.actions.ComposableNodeContainer`
  spawns `rclcpp_components/component_container` AND issues `LoadComposableNodes`).
- `LoadComposableNodes._load_node` blocks in `wait_for_service(timeout=1s)` until the
  container's `_container/load_node` service appears (launch_ros Humble) — so systemd can
  exec `component_container` first, then a transient `nav2.launch.py load_only:=true`
  attaches and loads the six components, then exits.
- `RecoveryNode number_of_retries="1"` (verified `recovery_node.cpp`): run primary → on
  FAILURE run recovery child → retry primary once → FAILURE/abort on second failure.
  = exactly "Path failure → Retry once → Clear costmaps → Back up → Spin → Retry path → Abort".
- Humble BT node/service names: `<ClearEntireCostmap service_name=
  "local_costmap/clear_entirely_local_costmap" | "global_costmap/clear_entirely_global_costmap">`,
  `<BackUp backup_dist backup_speed>`, `<Spin spin_dist>`.
- Costmap `width`/`height` are METERS (2.0×2.0 m → 40×40 cells @0.05; 24×24 m → 480×480);
  update/publish frequencies are Hz.
- RPP (Humble) has no `max_angular_vel`; `rotate_to_heading_angular_vel` is the angular cap.
  Speed scaling = `use_regulated_linear_velocity_scaling` (curvature) +
  `use_cost_regulated_linear_velocity_scaling` (costmap proximity), gated by
  `inflation_cost_scaling_factor > 0`.
- slam_toolbox publishes its OccupancyGrid on `/map` (transient-local; gated on having a
  subscriber) — costmap `static_layer` subscription triggers it; NO map_server needed.
- slam_toolbox resolves the laser via the scan's `header.frame_id` (`laser`) against the TF
  tree — a static `base_link → laser` transform is REQUIRED (URDF has `laser_link`, not
  `laser`). Yaw π on that TF = this unit's `heading_flip: true` (reversed sensor head).
- Nav2 pose chain: `map→odom` (slam_toolbox) ∘ `odom→base_link` (wheel_odometry
  `publish_tf: true` once the EKF is dropped).
- `web_control` already publishes `/goal_pose` (PoseStamped, frame `map`) = bt_navigator's
  goal topic; controller publishes `/cmd_vel` directly (ESP32 contract preserved).
- `nav2-clear-costmap-service` is NOT a conda package — the clear plugin lives in
  `nav2_behavior_tree`.
- **Lifecycle-manager bond gotcha** (verified `nav2_lifecycle_manager.cpp`): on ACTIVATE the
  manager creates a `bond::Bond(...).waitUntilFormed()` for EVERY name in `node_names_`.
  Only `nav2_util::LifecycleNode` (planner/controller/bt/behavior_server) creates the
  reverse bond. slam_toolbox's `SlamToolbox` is a RAW `rclcpp_lifecycle::LifecycleNode`
  (`slam_toolbox_common.hpp:68`) — it never bonds, so listing it in `node_names_` makes
  `waitUntilFormed` time out → `changeStateForAllNodes()` returns false → **activation
  aborts**. slam_toolbox must NOT be in `node_names_`; its lifecycle is driven by its own
  explicit configure→activate launch events instead.
- **Lifecycle-manager ordering** (verified `lifecycle_manager.cpp`): CONFIGURE/ACTIVATE
  iterate `node_names_` FORWARD, so `bt_navigator` must be the LAST entry (it depends on
  planner/controller being up). Reverse order is used for teardown.
- Humble BT param key is **`default_nav_to_pose_bt_xml`** (verified
  `navigate_to_pose.cpp getDefaultBTFilepath()`). `default_bt_xml_filename` is the older
  deprecated name — silently ignored in Humble (falls back to the heavy default tree).
- **Ceres thread count is NOT configurable** (verified `solvers/ceres_solver.cpp:166`):
  `options_.num_threads = 50;` is hardcoded, and no `ceres_num_threads` param is read
  anywhere in slam_toolbox; the only registered solver is `solver_plugins::CeresSolver`
  (`solver_plugins.xml`). CPU/RAM protection on the 1 GB H5 comes from pose-graph gating
  (`minimum_travel_distance`/`minimum_travel_heading`) + systemd `CPUAffinity`/`Nice` +
  `MALLOC_ARENA_MAX=2`, NOT a YAML thread knob.

## Architecture

ONE process (`nav2_container`) hosting:
planner_server · controller_server · bt_navigator · behavior_server · lifecycle_manager ·
slam_toolbox (async).

Lifecycle split within the single process:
- `nav2_lifecycle_manager` manages ONLY the four Nav2 servers:
  `node_names: ['planner_server', 'controller_server', 'behavior_server', 'bt_navigator']`
  — **bt_navigator LAST** (forward activation order), and slam_toolbox excluded (raw
  `rclcpp_lifecycle::LifecycleNode`, no bond — see verified facts above).
- slam_toolbox gets its own explicit autostart events (`OnStateTransition` →
  `ChangeState(CONFIGURE)` → `(ACTIVATE)`) in the launch, the async node's standard pattern.

Dropped from the stack: `slam_nav` (deleted), `nano-ekf` (robot_localization EKF; slam
replaces the fusion role), `nano-map` (map_bridge reads the deleted blob).

## Deliverable 1 — `src/robot_bringup/config/nav2/nav2_params.yaml`

| Node | Key contents |
|---|---|
| **slam_toolbox** | `mode: mapping` (async), `resolution: 0.05`, `min_laser_range: 0.12`, `max_laser_range: 6.0` (match LDS driver), `odom_frame: odom`, `map_frame: map`, `base_frame: base_link` (= Nav2 `robot_base_frame`, avoids TF timeouts), `scan_topic: /scan`, `solver_plugin: solver_plugins::CeresSolver`, `ceres_linear_solver: SPARSE_NORMAL_CHOLESKY` (NO `ceres_num_threads` — see facts), `minimum_travel_distance: 0.1`, `minimum_travel_heading: 0.17` (pose-graph growth guard → caps Ceres memory), `minimum_time_interval: 0.5`, `throttle_scans: 1`, `map_update_interval: 5.0`, `transform_publish_period: 0.1`, `enable_interactive_mode: false`, `use_map_saver: false`, `debug_logging: false`, online-async matcher subset |
| **controller_server** | `controller_frequency: 10.0`, `odom_topic: /odom`, progress/goal checkers (xy 0.15 / yaw 0.25), `FollowPath` = `RegulatedPurePursuitController`: `desired_linear_vel: 0.25`, `lookahead_dist: 0.6` (min 0.3 / max 0.9 / time 1.5), `rotate_to_heading_angular_vel: 1.5`, `use_velocity_scaled_lookahead_dist: true`, both velocity scalings on, `inflation_cost_scaling_factor: 3.0`, `allow_reversing: false` |
| **planner_server** | `GridBased` = `nav2_navfn_planner/NavfnPlanner`, `use_astar: true`, `allow_unknown: true`, `max_planning_time: 1.0` |
| **bt_navigator** | `global_frame: map`, `robot_base_frame: base_link`, `odom_topic: /odom`, `bt_loop_duration: 100` (10 Hz), minimal `plugin_lib_names` (7: compute_path_to_pose, follow_path, clear_costmap_service, spin_action, back_up_action, recovery_node, pipeline_sequence), `default_nav_to_pose_bt_xml` injected at launch |
| **behavior_server** | `behavior_plugins: [spin, backup]` only; per-plugin C++ types REQUIRED (Humble crashes without them): `spin.plugin: "nav2_behaviors/Spin"`, `backup.plugin: "nav2_behaviors/BackUp"`; `costmap_topic: local_costmap/costmap_raw`, `footprint_topic: local_costmap/published_footprint`, cycle 10 Hz, `global_frame: map`, `robot_base_frame: base_link` |
| **local_costmap** | `rolling_window: true`, `width: 2.0`, `height: 2.0`, `resolution: 0.05`, `update_frequency: 2.0`, `publish_frequency: 1.0`, `always_send_full_costmap: true` (footprint for behavior_server), `plugins: [static_layer, inflation_layer]` (`inflation_radius: 0.25`, `cost_scaling_factor: 3.0`), `global_frame: odom`, `robot_radius: 0.16` |
| **global_costmap** | `width: 24.0`, `height: 24.0`, `resolution: 0.05`, `update_frequency: 1.0`, `plugins: [static_layer, inflation_layer]` (same radii), `global_frame: map`, `robot_radius: 0.16` |

Explicitly ABSENT everywhere: AMCL, velocity_smoother, collision_monitor, map_server,
waypoint_follower, smoother_server, voxel/3D/range/keepout costmap layers.

## Deliverable 2 — `src/robot_bringup/config/nav2/recovery_bt.xml`

Single `MainTree` — exact syntax (service_name names must match the Nav2 server namespaces):
```xml
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="1" name="NavigateRecovery">
      <PipelineSequence name="Navigate">
        <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
        <FollowPath path="{path}" controller_id="FollowPath"/>
      </PipelineSequence>
      <Sequence name="Recovery">
        <ClearEntireCostmap name="ClearLocal" service_name="local_costmap/clear_entirely_local_costmap"/>
        <ClearEntireCostmap name="ClearGlobal" service_name="global_costmap/clear_entirely_global_costmap"/>
        <BackUp backup_dist="0.15" backup_speed="0.1"/>
        <Spin spin_dist="1.57"/>
      </Sequence>
    </RecoveryNode>
  </BehaviorTree>
</root>
```
Semantics from `number_of_retries="1"` (verified `recovery_node.cpp`): fail → clear local +
global costmaps → back up 0.15 m → spin 90° → retry path once → abort if still failing.

## Deliverable 3 — `src/robot_bringup/launch/nav2.launch.py`

- `ComposableNodeContainer` (`nav2_container`, `rclcpp_components/component_container`)
  with the six components; **LifecycleManager `autostart` on the FOUR Nav2 servers only**
  (`planner_server`, `controller_server`, `behavior_server`, `bt_navigator`).
- slam_toolbox in the same container but driven by its OWN autostart events
  (`OnStateTransition` → `ChangeState(CONFIGURE)` → `(ACTIVATE)`), not the lifecycle
  manager.
- Loads `nav2_params.yaml`; injects absolute `recovery_bt.xml` path via
  **`default_nav_to_pose_bt_xml`** (Humble key; `default_bt_xml_filename` is deprecated and
  silently ignored) into the bt_navigator param block; hands each `ComposableNode` the full
  file (launch_ros selects each node's section by name).
- `static_transform_publisher` `base_link → laser`, yaw π when `heading_flip:=true`
  (default true = this unit's reversed head; the TF-world replacement for
  `slam_nav`'s `heading_flip: true`).
- `load_only:=true` arg → emit ONLY `LoadComposableNodes` (no container spawn) for the
  systemd direct-exec pairing.

## Wiring & cleanup

- `scripts/unit_exec.sh`: `nav` unit execs `rclcpp_components/component_container` directly
  (the single heavy process); a `nano-nav-loader` (After=, Requisite=) runs a bounded poll
  loop first — up to ~30 s at 0.5 s intervals, `ros2 node list --no-daemon` grep for
  `nav2_container` (or `ros2 service list` grep `nav2_container/_container/load_node`) —
  exiting non-zero with a log line if the container never appears, THEN runs
  `nav2.launch.py load_only:=true` once, then exits. Fallback documented:
  `ros2 launch robot_bringup nav2.launch.py`.
- nav systemd unit gets **`CPUAffinity=` + `Nice=`** (the `MALLOC_ARENA_MAX=2` is already in
  `unit_exec.sh`) — the real H5 guard against Ceres's hardcoded 50-thread spike +
  pose-graph growth (see facts: no `ceres_num_threads` param exists).
- `stack.sh`: units become `router app sensors nav nav-loader` — drop `ekf`, `map`.
- `bringup.launch.py`: remove `slam_nav`, `map_bridge`, EKF nodes; include `nav2.launch.py`;
  keep wheel_odometry/lds/imu+frontend nodes. `wheel_odometry.publish_tf: true` (robot.yaml).
- Delete `src/slam_nav/` entirely (code, tests, occupancy.py, nav_node.py).
- `pixi.toml` (both platforms): add individual nav2 sub-packages + slam_toolbox —
  `nav2-msgs, nav2-util, nav2-core, nav2-costmap-2d, nav2-behavior-tree, nav2-planner,
  nav2-controller, nav2-bt-navigator, nav2-behaviors, nav2-lifecycle-manager,
  nav2-regulated-pure-pursuit-controller, nav2-navfn-planner, slam-toolbox`.
  (NOT the `navigation2` meta — pulls AMCL.) Verify each resolves on linux-aarch64;
  fallback to the meta only if a piece is missing there.
- `package.xml` / `setup.py`: install `config/nav2/*.yaml` + `config/nav2/*.xml`; add exec deps.

## Scrap-now / reimplement-later (web UI)

Remove from active stack (record TODO in AGENTS.md): web Map panel (blob), wall guard, map
buttons, `/slam_pose`, `/plan`, `nano_map.bin`/`nano_nogo.bin` consumers, and telemetry.py's
`/odometry/filtered` + `/slam_pose` consumers (EKF + nav_node gone). Goal-sending already
works via `/goal_pose`.

## Verification

- YAML/XML parse + `py_compile` of launches; confirm `LoadComposableNodes` attaches to a
  separately-started container (verified upstream: it retries `wait_for_service`);
  confirm pixi packages on linux-aarch64; `pixi run build`.
- Board bring-up: measure RSS/CPU — container < 150 MB RSS; idle CPU ≈ 0; BT tick ≤10 Hz
  only while navigating. Run `scripts/stack.sh` clean down→verify→up.
## Execution notes (2026-09-14 — what actually got built, and why it differs)

Deliverables 1-3 landed as `config/nav2/nav2_params.yaml`, `config/nav2/recovery_bt.xml`,
`launch/nav2.launch.py`; the wiring/cleanup section was executed in full (systemd units,
`unit_exec.sh`, `stack.sh`, `sbc-setup.sh`, bringup.launch.py, robot.yaml, slam_nav deleted,
pixi/package.xml deps, web-UI scrap, AGENTS.md TODO). Live verification on the dev PC
(fresh zenoh router + separately-started container + `load_only:=true` attach + fake
odom/TF/scan sources) confirmed: 5/5 components load, all four Nav2 servers CONFIGURE +
ACTIVATE with bonds ("Managed nodes are active"), slam_toolbox publishes `/map` +
`map→odom` TF, and a `/goal_pose` streams `/cmd_vel` at the RPP cap through the minimal
recovery BT. Four deviations from the plan as written:

1. **Container variant: `component_container_isolated`, not plain `component_container`.**
   Upstream nav2 Humble uses the isolated variant for a reason: on the plain (single-
   threaded) container, one component's construction/long callback blocks the ONE
   executor thread that serves every other component's services — the lifecycle
   manager's 2 s service deadlines lose that race and bringup aborts (live-verified
   failure, then verified fixed by switching).
2. **Managers load LAST.** A composed `nav2_lifecycle_manager` self-starts via an
   internal ~0 s timer (verified in nav2 1.1.x lifecycle_manager.cpp); loading it before
   the nodes it manages races the next component's construction. Order is now
   planner → controller → behavior → bt_navigator → lifecycle_manager.
3. **slam_toolbox is a SEPARATE process, not composed.** The plan's lifecycle design
   (second manager, bond gotcha, OnStateTransition autostart) describes slam_toolbox
   ≥2.7. The only robostack build for either platform is **2.6.10**, where SlamToolbox is
   a PLAIN `rclcpp::Node` whose executable main calls `configure()` itself — no lifecycle
   services, and the registered component is inert when composed (nothing calls
   configure; verified against the installed binaries). So it runs as `nano-slam.service`
   (`unit_exec.sh slam` → `async_slam_toolbox_node` + nav2_params.yaml). RAM cost is one
   extra rclcpp process; revisit composition if robostack ever ships ≥2.7.
4. **Two extra BT/behavior bits the default through-poses tree forced**: bt_navigator
   also loads and VALIDATES the upstream default `navigate_through_poses` tree at
   activation, so the plugin list gained `rate_controller`/`goal_updated`/`wait`, and
   behavior_server hosts the `wait` plugin — otherwise activation fails with
   "'wait' action server not available" even though we never publish
   `/navigate_through_poses`.

Env notes: all 13 conda packages resolve on linux-aarch64 (robostack-staging, verified
via the anaconda API). The dev-PC env was re-resolved for the new deps, which wiped a
manual `nanobot-brain` editable install — `pip` was added to pixi.toml and the brain
reinstalled editable; the brain repo was fast-forwarded 2 commits to restore
`strip_em_dash` (the glue↔brain lockstep gotcha in AGENTS.md, exactly as documented).
