"""Remote RViz for the REAL robot -- run this on the dev PC while the robot runs its own
`stack.sh` as normal. This is deliberately NOT the Gazebo dev-sim path (bringup.launch.py
sim:=true): Gazebo simulates a robot you don't have running; this instead just watches the
real one live over the shared rmw_zenoh graph.

Starts ONLY `robot_state_publisher` (for the URDF mesh + static TF, e.g. base_link ->
laser_link/imu_link) and `rviz2`. It deliberately does NOT relaunch wheel_odometry,
slam_nav, sensor_hub, etc. -- the robot is already publishing all of that; a second copy
on the dev PC would just be a second, redundant publisher of the same topics.

Prerequisites (see CLAUDE.md "Remote RViz"):
  - the robot is up (`stack.sh up`, which now also runs `sim_hardware.map_bridge_node`
    so /map exists as a real topic -- /dev/shm is per-machine RAM, so that node MUST run
    on the board, not here)
  - the dev PC's rmw_zenoh session can reach the robot's zenohd-serial router (same
    ROS_DOMAIN_ID -- already guaranteed by this repo's shared pixi.toml activation env --
    and either the same LAN with multicast scouting working, or an explicit
    ZENOH_SESSION_CONFIG_URI connect endpoint -- see scripts/rviz_remote.sh)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("robot_bringup")
    xacro_path = os.path.join(bringup_share, "urdf", "nano.urdf.xacro")
    rviz_config = os.path.join(bringup_share, "rviz", "nano.rviz")
    robot_description = Command(["xacro ", xacro_path])

    return LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             name="robot_state_publisher",
             parameters=[{"robot_description": robot_description}], output="screen"),
        Node(package="rviz2", executable="rviz2", name="rviz2",
             arguments=["-d", rviz_config], output="screen"),
    ])
