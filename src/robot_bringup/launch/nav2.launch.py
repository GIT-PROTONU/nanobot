"""Nav2 + slam_toolbox stack (replaces slam_nav).

The Nav2 Humble servers (planner_server · controller_server · bt_navigator ·
behavior_server · lifecycle_manager) run composed in ONE container
(`nav2_container`, rclcpp_components/component_container_isolated — upstream
nav2 Humble's own choice: per-component callback groups, so a service call to
one component is still served while another's constructor/long callback runs;
on a plain single-threaded container the lifecycle manager's 2 s service
deadlines can lose that race, live-verified). Lean RAM/CPU on the 1 GB board —
the Humble servers are all registered rclcpp components, so composition is free.

slam_toolbox runs as a SEPARATE process: the robostack Humble package is
slam_toolbox 2.6.10, where SlamToolbox is a PLAIN rclcpp::Node whose
`configure()` is called by its own executable main (async_slam_toolbox_node) —
there are no lifecycle services to manage and the registered component is
inert when composed (nothing calls configure; verified against the installed
2.6.10 binaries; robostack has no newer build for either platform). The plan's
"drive slam_toolbox's lifecycle" design (second lifecycle manager + bond gotcha)
applies to slam_toolbox >= 2.7 only. See docs/nav2-migration.md + AGENTS.md.

Lifecycle split (verified against nav2 1.1.x):
  * `lifecycle_manager` (autostart, bonded) manages ONLY the four Nav2 servers —
    bt_navigator LAST (CONFIGURE/ACTIVATE iterate node_names forward and the
    navigator depends on the others). A composed manager self-starts through an
    internal ~0 s timer when autostart is on; both the managers and the nodes
    they manage are loaded in an order that puts the managers LAST — a manager
    constructed earlier races the next component's construction and its bringup
    can time out (live-verified).
  * A static base_link → laser TF replaces slam_nav's heading_flip: this unit's
    sensor head is mounted facing BACK, so the transform carries a yaw of π
    (heading_flip:=true, default) to align the lidar's 0° beam with the
    physical drive direction.
  * default_nav_to_pose_bt_xml is the Humble param key (the older
    default_bt_xml_filename is silently ignored) and gets the ABSOLUTE path of
    config/nav2/recovery_bt.xml. The no-recovery variant
    (config/nav2/no_recovery_bt.xml) is NOT injected here: web_control sends
    it per goal via the NavigateToPose action's behavior_tree field when the
    web Recovery switch is off (Humble reads default_nav_to_pose_bt_xml once
    at configure, so the goal field is the only live lever).
  * velocity_smoother (2026-09-23) caps Nav2's speed + linear/angular
    accel/decel between the controller and /cmd_vel — the LINEAR accel limit
    RPP Humble lacks. Its params are dynamically reconfigurable, so
    web_control's POST /nav/config tunes them live (no restart).

Systemd pairing: the board runs the container directly (nano-nav.service via
scripts/unit_exec.sh nav), attaches the components with a nano-nav-loader
oneshot (`ros2 launch robot_bringup nav2.launch.py load_only:=true` — emits
ONLY the LoadComposableNodes action; launch_ros retries the container's
load-node service every 1 s), and runs slam_toolbox as nano-slam.service.
Dev fallback: `ros2 launch robot_bringup nav2.launch.py` spawns container +
components + slam_toolbox together.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode

PARAMS = os.path.join(
    get_package_share_directory("robot_bringup"), "config", "nav2", "nav2_params.yaml")
RECOVERY_BT = os.path.join(
    get_package_share_directory("robot_bringup"), "config", "nav2", "recovery_bt.xml")

# The six composed nodes (slam_toolbox is NOT one — see module docstring). Each
# gets the FULL params file; launch_ros selects the section matching the node's
# NAME. Managers load LAST (after every node they manage) — see docstring.
# Remap chain (2026-09-23): controller publishes /cmd_vel_nav, velocity_smoother
# caps speed+accel and publishes the final /cmd_vel (web teleop keepalive
# publishes /cmd_vel directly — teleop bypasses the smoother; the firmware's
# per-wheel slew handles its accel). Topic names inside the smoother are
# hardcoded (subscribes cmd_vel, publishes cmd_vel_smoothed) so routing is
# remaps-only.
COMPONENTS = [
    ComposableNode(
        package="nav2_planner", plugin="nav2_planner::PlannerServer",
        name="planner_server", parameters=[PARAMS]),
    ComposableNode(
        package="nav2_controller", plugin="nav2_controller::ControllerServer",
        name="controller_server", parameters=[PARAMS],
        remappings=[("cmd_vel", "/cmd_vel_nav")]),
    ComposableNode(
        package="nav2_velocity_smoother",
        plugin="nav2_velocity_smoother::VelocitySmoother",
        name="velocity_smoother", parameters=[PARAMS],
        remappings=[("cmd_vel", "/cmd_vel_nav"),
                    ("cmd_vel_smoothed", "/cmd_vel")]),
    ComposableNode(
        package="nav2_behaviors", plugin="behavior_server::BehaviorServer",
        name="behavior_server", parameters=[PARAMS]),
    ComposableNode(
        package="nav2_bt_navigator", plugin="nav2_bt_navigator::BtNavigator",
        name="bt_navigator", parameters=[
            # Humble key; the BT path must be ABSOLUTE (injected from the share dir).
            {"default_nav_to_pose_bt_xml": RECOVERY_BT}]),
    ComposableNode(
        package="nav2_lifecycle_manager",
        plugin="nav2_lifecycle_manager::LifecycleManager",
        name="lifecycle_manager",
        parameters=[{
            "autostart": True,
            # FIVE servers, bt_navigator LAST so the forward-order activation
            # finds its dependencies up (velocity_smoother after its input,
            # controller_server). slam_toolbox is not listed: it is a
            # plain node in 2.6.10 with no lifecycle at all (docstring).
            "node_names": ["planner_server", "controller_server",
                           "velocity_smoother", "behavior_server",
                           "bt_navigator"]}]),
]


def generate_launch_description():
    container_name = LaunchConfiguration("container_name")
    load_only = LaunchConfiguration("load_only")
    heading_flip = LaunchConfiguration("heading_flip")

    # The one heavy Nav2 process. Also spawned by scripts/unit_exec.sh nav
    # (systemd nano-nav.service); load_only:=true suppresses it for that pairing.
    # The one heavy Nav2 process. Also spawned by scripts/unit_exec.sh nav
    # (systemd nano-nav.service); load_only:=true suppresses it for that pairing.
    # The container gets the FULL params file process-wide: launch_ros inlines
    # only the per-component name sections into the load requests, so the
    # double-nested local_costmap/global_costmap sections would never reach the
    # child costmap nodes the servers create at runtime (they silently ran on
    # Nav2 defaults — found 2026-09-22 when the web costmap overlay shipped).
    container = ComposableNodeContainer(
        package="rclcpp_components", executable="component_container_isolated",
        name=container_name, namespace="", output="screen",
        parameters=[PARAMS],
        condition=UnlessCondition(load_only))

    # Attach the six components; retries the container's load_node service
    # every 1 s (launch_ros Humble), so a separately-started container is fine.
    # Fully-qualified target (upstream style) so the client resolves regardless
    # of any namespace the including launch applies.
    load_nodes = LoadComposableNodes(
        target_container=["/", container_name],
        composable_node_descriptions=COMPONENTS)

    # slam_toolbox 2.6.10: plain node, self-configuring executable. On the
    # board the nano-slam systemd unit runs it directly; here it is spawned
    # only on the ros2-launch dev path (not with load_only:=true).
    slam_toolbox = Node(
        package="slam_toolbox", executable="async_slam_toolbox_node",
        name="slam_toolbox", output="screen",
        parameters=[PARAMS],
        condition=UnlessCondition(load_only))

    # Static base_link -> laser: the TF-world replacement for slam_nav's
    # heading_flip (this unit's sensor head faces BACK → yaw π). Offset matches
    # the URDF's laser_joint (top centre of the base, +2 cm head height). Owned
    # by the nano-tf systemd unit on the board (never emitted with load_only —
    # a long-running static_transform_publisher would hang the loader oneshot),
    # spawned only on the ros2-launch dev path.
    static_tf_args = ["--x", "0", "--y", "0", "--z", "0.065",
                      "--frame-id", "base_link", "--child-frame-id", "laser"]
    static_tf = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="base_to_laser",
        arguments=static_tf_args + ["--yaw", PythonExpression(
            ['3.14159265 if "', heading_flip, '" == "true" else "0"'])],
        output="screen",
        condition=UnlessCondition(load_only))

    return LaunchDescription([
        DeclareLaunchArgument("container_name", default_value="nav2_container"),
        DeclareLaunchArgument(
            "load_only", default_value="false",
            description="true = emit ONLY LoadComposableNodes (no container, no "
                        "slam_toolbox, no static TF) for the systemd pairing where "
                        "nano-nav / nano-slam run the processes directly."),
        DeclareLaunchArgument(
            "heading_flip", default_value="true",
            description="Sensor head faces BACK on this unit: put yaw pi on the "
                        "base_link -> laser static TF (slam_nav heading_flip: true)."),
        container,
        load_nodes,
        slam_toolbox,
        static_tf,
    ])
