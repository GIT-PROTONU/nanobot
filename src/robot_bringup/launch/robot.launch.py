"""Top-level bringup: starts every hardware node + the web stack.

Each subsystem is its own node (LDS / odometry / motors / display / web), wired
together only through ROS topics — the same separation-of-concerns ROS 2 itself
encourages, so you can launch/restart/debug any one in isolation:

    ros2 run motor_control motor_node --ros-args --params-file <robot.yaml>
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = get_package_share_directory("robot_bringup")
    params = os.path.join(bringup_share, "config", "robot.yaml")

    use_web = LaunchConfiguration("web")
    # The Rust LDS node is a plain cargo binary (built via `pixi run build-lds`),
    # not an installed ament target, so point at it explicitly. Override if needed.
    lds_bin = LaunchConfiguration("lds_bin")

    return LaunchDescription([
        DeclareLaunchArgument("web", default_value="true",
                              description="Also start rosbridge + the web control page."),
        DeclareLaunchArgument(
            "lds_bin",
            # `pixi run bringup` runs from the workspace root; allow $LDS_BIN override.
            default_value=os.environ.get(
                "LDS_BIN",
                os.path.join(os.getcwd(), "src", "lds_driver", "target",
                             "release", "lds_driver")),
            description="Path to the compiled Rust LDS binary."),

        # --- LDS (Rust / r2r) ------------------------------------------------
        ExecuteProcess(
            cmd=[lds_bin, "--ros-args", "--params-file", params],
            name="lds_driver",
            output="screen",
        ),

        # --- Wheel encoders -> odometry --------------------------------------
        Node(package="wheel_odometry", executable="encoder_node",
             name="wheel_odometry", parameters=[params], output="screen"),

        # --- cmd_vel -> PCA9685 motors ---------------------------------------
        Node(package="motor_control", executable="motor_node",
             name="motor_control", parameters=[params], output="screen"),

        # --- Status OLED ------------------------------------------------------
        Node(package="oled_display", executable="display_node",
             name="oled_display", parameters=[params], output="screen"),

        # --- BWT901CL IMU -> /imu/data, /imu/mag, /imu/euler -----------------
        Node(package="imu_driver", executable="imu_node",
             name="imu_driver", parameters=[params], output="screen"),

        # --- Board health -> /diagnostics ------------------------------------
        Node(package="sys_monitor", executable="monitor_node",
             name="sys_monitor", parameters=[params], output="screen"),

        # --- Web control (rosbridge + static page) ---------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("web_control"), "launch", "web.launch.py"])),
            condition=IfCondition(use_web),
        ),
    ])
