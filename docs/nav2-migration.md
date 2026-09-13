# Nav2 Migration Plan — replace custom `slam_nav` with Nav2 + slam_toolbox

Status: PLANNED (checked against upstream Nav2 Humble 1.1.20 + slam_toolbox `ros2` source).
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

## Architecture

ONE process (`nav2_container`) hosting:
planner_server · controller_server · bt_navigator · behavior_server · lifecycle_manager ·
slam_toolbox (async).

Dropped from the stack: `slam_nav` (deleted), `nano-ekf` (robot_localization EKF; slam
replaces the fusion role), `nano-map` (map_bridge reads the deleted blob).

## Deliverable 1 — `src/robot_bringup/config/nav2/nav2_params.yaml`

| Node | Key contents |
|---|---|
| **slam_toolbox** | `mode: mapping` (async), `resolution: 0.05`, `min_laser_range: 0.12`, `max_laser_range: 6.0` (match LDS driver), `odom_frame: odom`, `map_frame: map`, `base_frame: base_footprint`, `scan_topic: /scan`, `minimum_time_interval: 0.5`, `throttle_scans: 1`, `map_update_interval: 5.0`, `transform_publish_period: 0.1`, `enable_interactive_mode: false`, `use_map_saver: false`, `debug_logging: false`, online-async matcher subset |
| **controller_server** | `controller_frequency: 10.0`, `odom_topic: /odom`, progress/goal checkers (xy 0.15 / yaw 0.25), `FollowPath` = `RegulatedPurePursuitController`: `desired_linear_vel: 0.25`, `lookahead_dist: 0.6` (min 0.3 / max 0.9 / time 1.5), `rotate_to_heading_angular_vel: 1.5`, `use_velocity_scaled_lookahead_dist: true`, both velocity scalings on, `inflation_cost_scaling_factor: 3.0`, `allow_reversing: false` |
| **planner_server** | `GridBased` = `nav2_navfn_planner/NavfnPlanner`, `use_astar: true`, `allow_unknown: true`, `max_planning_time: 1.0` |
| **bt_navigator** | `global_frame: map`, `robot_base_frame: base_link`, `odom_topic: /odom`, `bt_loop_duration: 100` (10 Hz), minimal `plugin_lib_names` (7: compute_path_to_pose, follow_path, clear_costmap_service, spin_action, back_up_action, recovery_node, pipeline_sequence), `default_nav_to_pose_bt_xml` injected at launch |
| **behavior_server** | `behavior_plugins: [spin, backup]` only; `costmap_topic: local_costmap/costmap_raw`, `footprint_topic: local_costmap/published_footprint`, cycle 10 Hz, `global_frame: map`, `robot_base_frame: base_link` |
| **local_costmap** | `rolling_window: true`, `width: 2.0`, `height: 2.0`, `resolution: 0.05`, `update_frequency: 2.0`, `publish_frequency: 1.0`, `always_send_full_costmap: true` (footprint for behavior_server), `plugins: [static_layer, inflation_layer]` (`inflation_radius: 0.25`, `cost_scaling_factor: 3.0`), `global_frame: odom`, `robot_radius: 0.16` |
| **global_costmap** | `width: 24.0`, `height: 24.0`, `resolution: 0.05`, `update_frequency: 1.0`, `plugins: [static_layer, inflation_layer]` (same radii), `global_frame: map`, `robot_radius: 0.16` |

Explicitly ABSENT everywhere: AMCL, velocity_smoother, collision_monitor, map_server,
waypoint_follower, smoother_server, voxel/3D/range/keepout costmap layers.

## Deliverable 2 — `src/robot_bringup/config/nav2/recovery_bt.xml`

Single `MainTree`:
```
RecoveryNode(number_of_retries="1")
 ├─ PipelineSequence [ ComputePathToPose → FollowPath ]        (primary)
 └─ Sequence [ ClearEntireCostmap local
             → ClearEntireCostmap global
             → BackUp backup_dist="0.15" backup_speed="0.1"
             → Spin spin_dist="1.57" ]                          (recovery)
```
Semantics from `number_of_retries="1"`: fail → clear → back up 0.15 m → spin 90° →
retry path once → abort if still failing.

## Deliverable 3 — `src/robot_bringup/launch/nav2.launch.py`

- `ComposableNodeContainer` (`nav2_container`, `rclcpp_components/component_container`)
  with the six components; LifecycleManager `autostart` on the five servers.
- Loads `nav2_params.yaml`; injects absolute `recovery_bt.xml` path into the bt_navigator
  param block; hands each `ComposableNode` the full file (launch_ros selects each node's
  section by name).
- `static_transform_publisher` `base_link → laser`, yaw π when `heading_flip:=true`
  (default true = this unit's reversed head; the TF-world replacement for
  `slam_nav`'s `heading_flip: true`).
- `load_only:=true` arg → emit ONLY `LoadComposableNodes` (no container spawn) for the
  systemd direct-exec pairing.

## Wiring & cleanup

- `scripts/unit_exec.sh`: `nav` unit execs `rclcpp_components/component_container` directly
  (the single heavy process); a `nano-nav-loader` (After=, Requisite=) runs
  `nav2.launch.py load_only:=true` once, then exits. Fallback documented:
  `ros2 launch robot_bringup nav2.launch.py`.
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