"""Web stack: the static control page server (which is also the browser's telemetry +
control gateway — SSE /telemetry, POST /publish|/param|/drive; no rosbridge).

Reusable standalone (`pixi run web`) or included by robot_bringup.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    web_port = LaunchConfiguration("web_port")
    params = LaunchConfiguration("params")

    return LaunchDescription([
        DeclareLaunchArgument("web_port", default_value="8080"),
        DeclareLaunchArgument("params", default_value="",
                              description="Path to a ROS params YAML file (robot.yaml)."),

        # Serves index.html + the SSE telemetry stream + the whitelisted control POSTs.
        Node(
            package="web_control", executable="web_server",
            name="web_control", output="screen",
            parameters=[{"web_port": web_port}, params],
        ),
    ])
