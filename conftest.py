"""Offline (ROS-free) test path setup: put each source package's parent on sys.path
so the pure helper modules import without a colcon build. Only stdlib-only modules
are importable this way (rclpy-dependent ones import but can't be instantiated)."""
import os
import sys

_ROOT = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.join(_ROOT, "src")

for _pkg in ("web_control", "imu_driver", "lds_driver_py", "wheel_odometry"):
    _dir = os.path.join(_SRC, _pkg)
    if os.path.isdir(_dir) and _dir not in sys.path:
        sys.path.insert(0, _dir)
