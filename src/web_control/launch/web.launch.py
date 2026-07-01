"""Web stack: rosbridge websocket + rosapi + the static control page server.

Reusable standalone (`pixi run web`) or included by robot_bringup.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    web_port = LaunchConfiguration("web_port")
    rosbridge_port = LaunchConfiguration("rosbridge_port")
    params = LaunchConfiguration("params")

    return LaunchDescription([
        DeclareLaunchArgument("web_port", default_value="8080"),
        DeclareLaunchArgument("rosbridge_port", default_value="9090"),
        DeclareLaunchArgument("params", default_value="",
                              description="Path to a ROS params YAML file (robot.yaml)."),

        # ROS <-> websocket bridge the browser connects to.
        Node(
            package="rosbridge_server", executable="rosbridge_websocket",
            name="rosbridge_websocket", output="screen",
            parameters=[{"port": rosbridge_port}],
        ),
        # Lets the page enumerate topics/params (optional but handy).
        Node(
            package="rosapi", executable="rosapi_node",
            name="rosapi", output="screen",
        ),
        # Serves index.html.
        Node(
            package="web_control", executable="web_server",
            name="web_control", output="screen",
            parameters=[{"web_port": web_port, "rosbridge_port": rosbridge_port}, params],
        ),
    ])
