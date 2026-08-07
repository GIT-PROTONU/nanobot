"""Offline tests for the odometry auto-calibration solver (slam_nav.calibrate).

Pure math, no ROS — run with:  pixi run python -m pytest src/slam_nav/test
(or just:  python src/slam_nav/test/test_calibrate.py)

The test context may not have slam_nav installed (no colcon build), so we add the
package source dir to sys.path ourselves.
"""
import math
import os
import sys

_SRC = os.path.join(os.path.dirname(__file__), "..", "..", "slam_nav")
if os.path.isdir(_SRC):
    sys.path.insert(0, os.path.abspath(_SRC))

from slam_nav import calibrate


def test_scale_from_forward_linear():
    mpt = 0.0001
    # odometry reported exactly the commanded distance -> no change
    assert calibrate.scale_from_forward(1.0, 1.0, mpt) == mpt
    # odometry under-reported by 2x (commanded 1.0 m but odom says 0.5 m)
    # -> m_per_tick must double so the same ticks accumulate the full distance
    assert calibrate.scale_from_forward(1.0, 0.5, mpt) == mpt * 2.0
    # odometry over-reported (odom 2.0 m for 1.0 commanded) -> halve m_per_tick
    assert calibrate.scale_from_forward(1.0, 2.0, mpt) == mpt * 0.5


def test_scale_from_forward_guards():
    mpt = 0.0001
    assert calibrate.scale_from_forward(1.0, 0.0, mpt) == mpt      # no odom motion
    assert calibrate.scale_from_forward(0.0, 1.0, mpt) == mpt      # no commanded dist
    assert calibrate.scale_from_forward(1.0, -1.0, mpt) == mpt     # reversed (bad leg)
    assert calibrate.scale_from_forward(1.0, 1.0, 0.0) == 0.0      # degenerate baseline
    assert calibrate.scale_from_forward(float("nan"), 1.0, mpt) == mpt
    assert calibrate.scale_from_forward(1.0, float("inf"), mpt) == mpt


def test_radius_from_scale():
    # m_per_tick = 2*pi*r / ticks_per_rev  =>  r = mpt*tpr/(2*pi)
    tpr = 1440
    r = 0.0335
    mpt = 2.0 * math.pi * r / tpr
    assert calibrate.radius_from_scale(mpt, tpr) == r
    # double the m_per_tick -> radius doubles at fixed ticks_per_rev
    assert math.isclose(calibrate.radius_from_scale(mpt * 2.0, tpr), r * 2.0)
    assert calibrate.radius_from_scale(0.0, tpr) == 0.0
    assert calibrate.radius_from_scale(mpt, 0) == 0.0
    assert calibrate.radius_from_scale(float("nan"), tpr) == 0.0


def test_separation_from_spin():
    sep = 0.16
    # IMU and odom agree -> no change
    assert calibrate.separation_from_spin(1.0, 1.0, sep) == sep
    # odometry under-rotated (0.8 rad) vs IMU (1.0 rad): the wheelbase was too wide,
    # so it must shrink by odom/imu to let the same tick difference produce more
    # heading change. Corrected math (NOT the old inverted imu/odom hint).
    assert math.isclose(calibrate.separation_from_spin(0.8, 1.0, sep), sep * 0.8)
    # odometry over-rotated -> wheelbase grows
    assert math.isclose(calibrate.separation_from_spin(1.25, 1.0, sep), sep * 1.25)


def test_separation_from_spin_guards():
    sep = 0.16
    assert calibrate.separation_from_spin(1.0, 0.0, sep) == sep    # no IMU truth
    assert calibrate.separation_from_spin(0.0, 1.0, sep) == sep    # no odom rotation
    assert calibrate.separation_from_spin(1.0, -1.0, sep) == sep   # reversed sign
    assert calibrate.separation_from_spin(1.0, 1.0, 0.0) == 0.0    # degenerate baseline
    assert calibrate.separation_from_spin(float("nan"), 1.0, sep) == sep
    assert calibrate.separation_from_spin(1.0, float("inf"), sep) == sep


def test_end_to_end_sign_is_not_inverted():
    """Regression: the OLD self-test hint recommended separation * (imu/odom), which
    was inverted. A robot that over-rotates in odometry (odom 1.25 vs IMU 1.0) needs
    a WIDER wheelbase (sep*1.25). The old formula would have shrunk it to sep*0.8 —
    the wrong-way convergence seen on hardware. Assert the corrected direction."""
    sep = calibrate.separation_from_spin(1.25, 1.0, 0.16)
    assert sep > 0.16
    assert math.isclose(sep, 0.16 * 1.25)


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        fn()
        print(f"PASS {fn.__name__}")
    print(f"{len(fns)} tests passed")
