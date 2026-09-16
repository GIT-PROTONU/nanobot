"""Offline tests (ROS-free) for wheel_odometry.odom_math — the pure differential-drive
integration behind /odom and the odom->base_link TF (the pose chain slam_toolbox rides on).

    pixi run test
"""
import math

import pytest

from wheel_odometry.odom_math import integrate_pose, meters_per_tick, yaw_to_quat


# ---- meters_per_tick ----------------------------------------------------------
def test_meters_per_tick_matches_circumference_formula():
    # real robot values: r=0.0335 m, 1440 ticks/rev
    mpt = meters_per_tick(0.0335, 1440)
    assert mpt == pytest.approx(2 * math.pi * 0.0335 / 1440)
    assert mpt == pytest.approx(1.4617e-4, rel=1e-3)


def test_meters_per_tick_scales_with_radius_and_resolution():
    assert meters_per_tick(0.1, 1000) > meters_per_tick(0.05, 1000)   # bigger wheel
    assert meters_per_tick(0.05, 500) == pytest.approx(2 * meters_per_tick(0.05, 1000))


# ---- integrate_pose -----------------------------------------------------------
def test_straight_drive_moves_along_heading_without_turning():
    # equal wheel travel -> ds only, no rotation; drives due east from th=0
    x, y, th, ds, dth = integrate_pose(0, 0, 0.0, 0.10, 0.10, 0.16)
    assert (ds, dth) == pytest.approx((0.10, 0.0))
    assert (x, y, th) == pytest.approx((0.10, 0.0, 0.0))


def test_pure_spin_in_place_keeps_position():
    # wheels counter-rotate equally -> rotation only, no translation
    sep = 0.16
    dl, dr = -0.05, 0.05
    x, y, th, ds, dth = integrate_pose(1.0, 2.0, 0.3, dl, dr, sep)
    assert ds == pytest.approx(0.0)
    assert dth == pytest.approx((dr - dl) / sep)
    assert (x, y) == pytest.approx((1.0, 2.0))
    assert th == pytest.approx(0.3 + dth)


def test_midpoint_uses_half_step_heading():
    # turning arc: the translation must use th + dth/2 (midpoint), not th or th+dth.
    # Drive a quarter-circle-ish step: th=0, dth=pi/2.
    sep = 0.16
    dl, dr = 0.0, math.pi * sep / 2                  # dth = (dr-dl)/sep = pi/2, ds = dr/2
    ds_expected = dr / 2
    x, y, th, ds, dth = integrate_pose(0, 0, 0.0, dl, dr, sep)
    assert dth == pytest.approx(math.pi / 2)
    # midpoint heading = pi/4 -> move at 45 deg
    assert (x, y) == pytest.approx((ds_expected * math.cos(math.pi / 4),
                                    ds_expected * math.sin(math.pi / 4)))
    assert th == pytest.approx(math.pi / 2)


def test_reverse_motion_is_negative_displacement():
    x, y, th, ds, dth = integrate_pose(0, 0, 0.0, -0.05, -0.05, 0.16)
    assert (x, ds) == pytest.approx((-0.05, -0.05))
    assert dth == 0.0


def test_heading_wraps_into_pi_range():
    # a step that pushes heading just past +pi wraps to ~-pi
    sep = 0.16
    dth_target = math.pi + 0.1
    dr = dth_target * sep                            # dl = 0
    x, y, th, ds, dth = integrate_pose(0, 0, 0.0, 0.0, dr, sep)
    assert th == pytest.approx(math.atan2(math.sin(dth_target), math.cos(dth_target)))
    assert -math.pi <= th <= math.pi


def test_no_tick_change_is_a_noop():
    x, y, th, ds, dth = integrate_pose(0.5, -0.5, 1.0, 0.0, 0.0, 0.16)
    assert (x, y, th, ds, dth) == pytest.approx((0.5, -0.5, 1.0, 0.0, 0.0))


def test_curved_path_accumulates_consistently():
    # constant forward + turn follows the circle around the ICC (radius
    # R = sep*(dl+dr) / (2*(dr-dl))). Midpoint integration is a chord
    # approximation of the arc, so position converges to the exact arc
    # endpoint only to O(step^2) — assert within that discretization error,
    # while the heading accumulates EXACTLY (it doesn't depend on the step).
    sep, step = 0.16, 0.02
    dl, dr = step, step * 2                          # constant forward + turn
    dth_step = (dr - dl) / sep
    R = sep * (dl + dr) / (2 * (dr - dl))            # arc radius of the ICC
    n = 2
    x = y = th = 0.0
    for _ in range(n):
        x, y, th, _, _ = integrate_pose(x, y, th, dl, dr, sep)
    # heading: exact, no discretization error
    assert th == pytest.approx(n * dth_step)
    # position: exact arc endpoint, ICC on the left at start (th=0)
    dth_total = n * dth_step
    cx, cy = 0.0, R
    ang0 = math.atan2(0.0 - cy, 0.0 - cx)            # robot starts at angle -90deg on the circle
    ex = cx + R * math.cos(ang0 + dth_total)
    ey = cy + R * math.sin(ang0 + dth_total)
    assert (x, y) == pytest.approx((ex, ey), rel=2e-3)


def test_finer_steps_converge_to_the_exact_arc():
    # the midpoint integrator is consistent: the same TOTAL motion (4 steps of
    # (dl, dr)) split into N sub-steps each approaches the exact arc endpoint
    # as N grows (catches a regression to e.g. forward-Euler, which converges
    # to a DIFFERENT curve).
    sep, dl, dr = 0.16, 0.02, 0.04
    dth_step = (dr - dl) / sep
    R = sep * (dl + dr) / (2 * (dr - dl))
    dth_total = 4 * dth_step
    ex = R * math.sin(dth_total)                     # ICC on the left at start (th=0)
    ey = R * (1.0 - math.cos(dth_total))

    def endpoint(n):                                 # same motion, n sub-steps per step
        x = y = th = 0.0
        for _ in range(4 * n):
            x, y, th, _, _ = integrate_pose(x, y, th, dl / n, dr / n, sep)
        return x, y

    coarse = math.dist(endpoint(2), (ex, ey))
    fine = math.dist(endpoint(2000), (ex, ey))
    assert fine < coarse / 10.0                      # convergence
    assert fine == pytest.approx(0.0, abs=1e-6)


# ---- yaw_to_quat ---------------------------------------------------------------
def test_yaw_to_quat_identity():
    x, y, z, w = yaw_to_quat(0.0)
    assert (x, y) == (0.0, 0.0)
    assert z == pytest.approx(0.0)
    assert w == pytest.approx(1.0)


def test_yaw_to_quat_quarter_turn():
    z, w = yaw_to_quat(math.pi / 2)[2:]
    assert (z, w) == pytest.approx((math.sqrt(0.5), math.sqrt(0.5)))


def test_yaw_to_quat_half_turn_is_pure_z():
    x, y, z, w = yaw_to_quat(math.pi)
    assert (x, y) == (0.0, 0.0)
    assert z == pytest.approx(1.0, abs=1e-12)
    assert w == pytest.approx(0.0, abs=1e-12)


def test_yaw_to_quat_is_unit_for_arbitrary_yaw():
    for yaw in (0.3, -2.7, 6.1, -6.1):
        q = yaw_to_quat(yaw)
        assert math.sumprod(q, q) == pytest.approx(1.0, abs=1e-12)
