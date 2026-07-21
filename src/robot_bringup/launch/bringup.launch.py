"""Top-level bringup: the one node graph shared by the real robot and the Gazebo dev-sim.

Nodes that run identically either way (real robot hardware or Gazebo): web_control
(the browser's telemetry/control gateway — no rosbridge), oled_display, behavior
(mood_node), sys_monitor, wheel_odometry, slam_nav,
robot_state_publisher. Only the lowest hardware-transducer layer is swapped by `sim`:

    sim:=false (default) -- the real LDS02RR (lds_driver_py) + BWT901CL IMU (imu_driver).
        The ESP32 coprocessor (motors/encoders/board telemetry) is NOT started here: it
        talks native zenoh-pico over a direct serial link, joined to the graph by a
        serial-capable zenohd -- that's out-of-band setup `stack.sh` handles, not
        something `ros2 launch` starts. In production the board runs `stack.sh` directly
        (installed executables, not `ros2 launch`, to save RAM on the 1 GB board) --
        this `sim:=false` path is a `ros2 launch`-based debug alternative that mirrors
        stack.sh's node set, not a replacement for it.
    sim:=true -- Gazebo Sim (ros_gz_sim) + ros_gz_bridge + sim_hardware stand in for the
        lidar/IMU/encoders/ESP32 telemetry, publishing the exact same topic contracts (see
        src/sim_hardware/sim_hardware/sim_bridge_node.py) so every node above is none the
        wiser. Dev-PC only -- see pixi.toml's `sim` task / scripts/sim_run.sh.

`rviz:=true` launches RViz2 with the checked-in config (RobotModel/TF/LaserScan/Map).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = get_package_share_directory("robot_bringup")
    params = os.path.join(bringup_share, "config", "robot.yaml")
    ekf_params = os.path.join(bringup_share, "config", "ekf.yaml")
    xacro_path = os.path.join(bringup_share, "urdf", "nano.urdf.xacro")
    world_path = os.path.join(bringup_share, "worlds", "nano_room.sdf")
    bridge_config = os.path.join(bringup_share, "config", "gz_bridge.yaml")
    rviz_config = os.path.join(bringup_share, "rviz", "nano.rviz")

    sim = LaunchConfiguration("sim")
    rviz = LaunchConfiguration("rviz")
    web = LaunchConfiguration("web")
    ekf = LaunchConfiguration("ekf")

    robot_description = ParameterValue(Command(["xacro ", xacro_path]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument("sim", default_value="false",
                              description="true = Gazebo Sim hardware stand-in (dev PC); "
                                          "false = the real LDS/IMU hardware nodes."),
        DeclareLaunchArgument("rviz", default_value="false",
                              description="Also start RViz2 with the checked-in config."),
        DeclareLaunchArgument("web", default_value="true",
                              description="Also start the web control page/gateway."),
        DeclareLaunchArgument("ekf", default_value="true",
                              description="true = robot_localization EKF fuses wheel "
                                          "odometry + IMU; false = raw wheel odometry only."),

        # ---- always: the shared node graph -----------------------------------------
        Node(package="wheel_odometry", executable="encoder_node",
             name="wheel_odometry", parameters=[params], output="screen"),
        Node(package="oled_display", executable="display_node",
             name="oled_display", parameters=[params], output="screen"),
        Node(package="sys_monitor", executable="monitor_node",
             name="sys_monitor", parameters=[params], output="screen"),
        Node(package="behavior", executable="mood_node",
             name="behavior", parameters=[params], output="screen"),
        Node(package="slam_nav", executable="nav_node",
             name="slam_nav", parameters=[params], output="screen"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             name="robot_state_publisher",
             parameters=[{"robot_description": robot_description}], output="screen"),
        # Republishes slam_nav's /dev/shm/nano_map.bin as nav_msgs/OccupancyGrid for RViz
        # (the web UI keeps reading the blob directly) -- useful on the real robot too.
        Node(package="sim_hardware", executable="map_bridge_node",
             name="map_bridge", output="screen"),

        # ---- robot_localization EKF: sensor fusion between wheel odometry and IMU ----
        # Fuses /odom (wheel encoders) + /imu/data (IMU orientation/gyro/accel) into a
        # single filtered state on /odometry/filtered. The EKF also publishes the
        # odom -> base_link TF, replacing wheel_odometry's own TF broadcast
        # (wheel_odometry.publish_tf is set to false in robot.yaml when EKF is active).
        # Standalone config in config/ekf.yaml.
        Node(package="robot_localization", executable="ekf_node",
             name="ekf_node",
             parameters=[ekf_params],
             output="screen",
             condition=IfCondition(LaunchConfiguration("ekf"))),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("web_control"), "launch", "web.launch.py"])),
            launch_arguments={"params": params}.items(),
            condition=IfCondition(web),
        ),

        # ---- sim:=false: the real hardware nodes -------------------------------------
        Node(package="lds_driver_py", executable="lds_node",
             name="lds_driver", parameters=[params], output="screen",
             condition=UnlessCondition(sim)),
        Node(package="imu_driver", executable="imu_node",
             name="imu_driver", parameters=[params], output="screen",
             condition=UnlessCondition(sim)),

        # ---- sim:=true: Gazebo Sim + the ros_gz bridge + the sim_hardware stand-in ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])),
            launch_arguments={"gz_args": f"-r {world_path}"}.items(),
            condition=IfCondition(sim),
        ),
        # A few seconds' head start for the Gazebo server to come up before asking it to
        # spawn the model (untested timing -- a race here just needs `sim_run.sh` re-run;
        # bump the delay if the dev PC is slow to start Gazebo).
        TimerAction(period=4.0, actions=[
            Node(package="ros_gz_sim", executable="create",
                 name="spawn_nano",
                 arguments=["-topic", "robot_description", "-name", "nano",
                           "-x", "0", "-y", "0", "-z", "0.05"],
                 output="screen"),
        ], condition=IfCondition(sim)),
        Node(package="ros_gz_bridge", executable="parameter_bridge",
             name="ros_gz_bridge",
             arguments=["--ros-args", "-p", f"config_file:={bridge_config}"],
             condition=IfCondition(sim), output="screen"),
        Node(package="sim_hardware", executable="sim_bridge_node",
             name="sim_bridge", parameters=[params], output="screen",
             condition=IfCondition(sim)),

        # ---- optional RViz2 -----------------------------------------------------------
        Node(package="rviz2", executable="rviz2", name="rviz2",
             arguments=["-d", rviz_config],
             condition=IfCondition(rviz), output="screen"),
    ])
