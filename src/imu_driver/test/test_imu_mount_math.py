"""Offline tests (ROS-free) for imu_driver's pure mount/lever-arm math.

These functions build the robot-frame `/imu/data` that the whole navigation chain
consumes (SLAM heading, EKF fusion, drift tool) — a sign or coupling error here
silently poisons the map (the 2026-08-10 lost-storm was exactly such a scale bug).
The tests pin the deployed conventions:

    * matrices are row-major 3-tuples-of-3-tuples, ZYX (yaw-pitch-roll) Euler
    * MOUNT_M maps a vector's SENSOR-frame components into ROBOT-frame components
    * correct_orientation composes the sensor attitude with the mount as a full
      matrix product (the per-angle shortcut is only exact for a pure-yaw mount
      with no roll/pitch coupling — the bug class this rewrite fixed)
    * lever_arm_correction SUBTRACTS the lever-arm acceleration an IMU mounted
      off-centre picks up (centripetal omega x (omega x r) + tangential alpha x r)

    pixi run test
"""
import math

import pytest

from imu_driver.imu_node import (
    _cross,
    _matmul3,
    _matvec3,
    _transpose3,
    correct_orientation,
    euler_to_matrix,
    euler_to_quat,
    lever_arm_correction,
    matrix_to_euler,
    mount_matrix,
    rotate_mount,
)

IDENT = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def _close_mat(a, b, tol=1e-9):
    for ra, rb in zip(a, b):
        for va, vb in zip(ra, rb):
            assert va == pytest.approx(vb, abs=tol)


# ---- small matrix helpers ------------------------------------------------------
def test_cross_right_hand_rule():
    assert _cross((1, 0, 0), (0, 1, 0)) == pytest.approx((0, 0, 1))
    assert _cross((1, 0, 0), (1, 0, 0)) == pytest.approx((0, 0, 0))   # parallel


def test_matvec_and_transpose():
    m = ((0, -1, 0), (1, 0, 0), (0, 0, 1))            # Rz(+90)
    assert _matvec3(m, (1, 0, 0)) == pytest.approx((0, 1, 0))
    _close_mat(_transpose3(m), ((0, 1, 0), (-1, 0, 0), (0, 0, 1)))
    _close_mat(_matmul3(IDENT, m), m)


# ---- euler <-> matrix ----------------------------------------------------------
def test_euler_to_matrix_zero_is_identity():
    _close_mat(euler_to_matrix(0.0, 0.0, 0.0), IDENT)


def test_euler_matrix_roundtrip():
    for r, p, y in ((0.1, -0.2, 2.5), (-1.2, 0.4, -3.0), (0.0, 0.3, 0.0)):
        rr, pp, yy = matrix_to_euler(euler_to_matrix(r, p, y))
        assert (rr, pp, yy) == pytest.approx((r, p, y), abs=1e-9)


def test_euler_to_matrix_pure_yaw_is_rz():
    _close_mat(euler_to_matrix(0.0, 0.0, math.pi / 2), ((0, -1, 0), (1, 0, 0), (0, 0, 1)))


# ---- mount matrix / rotate_mount ------------------------------------------------
def test_mount_matrix_identity_for_zero_angles():
    _close_mat(mount_matrix(0, 0, 0), IDENT)


def test_mount_matrix_yaw90_sends_sensor_x_to_robot_y():
    # a sensor board glued 90deg CCW (viewed from above): its "forward" (x) points
    # along the robot's LEFT (+y); its "left" (y) points along the robot's BACK (-x).
    m = mount_matrix(0, 0, 90)
    assert rotate_mount((1, 0, 0), m) == pytest.approx((0, 1, 0))
    assert rotate_mount((0, 1, 0), m) == pytest.approx((-1, 0, 0))
    assert rotate_mount((0, 0, 1), m) == pytest.approx((0, 0, 1))     # z is up in both frames


def test_rotate_mount_inverts_with_transpose():
    # rotating into the robot frame and back with the transpose restores the input
    for angles in ((0, 0, 37), (12, -8, 90), (-45, 30, 0)):
        m = mount_matrix(*angles)
        v = (0.3, -1.2, 0.7)
        back = _matvec3(_transpose3(m), rotate_mount(v, m))
        assert back == pytest.approx(v, abs=1e-12)


def test_rotate_mount_is_length_preserving():
    m = mount_matrix(23, -14, 66)
    v = (1.0, 2.0, 3.0)
    assert math.sumprod(rotate_mount(v, m), rotate_mount(v, m)) == pytest.approx(14.0)


# ---- correct_orientation --------------------------------------------------------
def test_correct_orientation_identity_mount_is_passthrough():
    assert correct_orientation(5, -7, 33, IDENT, IDENT) == pytest.approx((5, -7, 33))


def test_correct_orientation_yaw_only_mount_shifts_heading():
    # sensor twisted +90 in yaw: whatever the sensor calls "0" the robot sees at
    # -90 (the sensor's forward aims 90deg left of the robot's forward).
    m, mt = mount_matrix(0, 0, 90), _transpose3(mount_matrix(0, 0, 90))
    assert correct_orientation(0, 0, 0, m, mt) == pytest.approx((0, 0, -90), abs=1e-6)
    assert correct_orientation(0, 0, 90, m, mt) == pytest.approx((0, 0, 0), abs=1e-6)
    assert correct_orientation(0, 0, -30, m, mt) == pytest.approx((0, 0, -120), abs=1e-6)


def test_correct_orientation_pitch_and_roll_swap_on_yaw90_mount():
    # with the sensor yawed 90 its x lies along robot +y and its y along robot -x,
    # so sensor roll -> robot PITCH and sensor pitch -> robot ROLL (sign-flipped):
    # exactly the coupling a per-angle shortcut (copy roll/pitch, add mount yaw)
    # cannot represent.
    m, mt = mount_matrix(0, 0, 90), _transpose3(mount_matrix(0, 0, 90))
    assert correct_orientation(5, 0, 0, m, mt) == pytest.approx((0, 5, -90), abs=1e-6)
    assert correct_orientation(0, 5, 0, m, mt) == pytest.approx((-5, 0, -90), abs=1e-6)


def test_correct_orientation_sensor_roll_becomes_robot_pitch_on_yaw90_mount():
    # THE coupling the per-angle shortcut gets wrong: with the sensor yawed 90, its
    # x-axis lies along robot +y, so a roll about the SENSOR x is a robot PITCH.
    # A shortcut (copy roll, add mount yaw to sensor yaw) would report roll 10.
    m, mt = mount_matrix(0, 0, 90), _transpose3(mount_matrix(0, 0, 90))
    roll, pitch, yaw = correct_orientation(10, 0, 0, m, mt)
    assert roll == pytest.approx(0, abs=1e-6)
    assert pitch == pytest.approx(10, abs=1e-6)
    assert yaw == pytest.approx(-90, abs=1e-6)


# ---- lever_arm_correction ---------------------------------------------------------
def test_lever_arm_noop_at_centre():
    acc = (1.0, 2.0, 3.0)
    assert lever_arm_correction(acc, (0, 0, 1), (0, 0, 0), (0, 0, 0)) is acc


def test_lever_arm_subtracts_centripetal():
    # IMU 0.1 m forward of the axle, spinning 2 rad/s about z (CCW): the sensor
    # picks up 0.4 m/s^2 pointing INWARD (-x); subtracting it recovers the axle's
    # own acceleration (omega^2 * r = 4 * 0.1).
    out = lever_arm_correction((0.0, 0.0, 9.8), (0, 0, 2.0), (0, 0, 0), (0.1, 0, 0))
    assert out == pytest.approx((0.4, 0.0, 9.8))


def test_lever_arm_subtracts_tangential():
    # angular accel alpha=1 rad/s^2 about z, sensor at +0.1 m x: tangential accel
    # is +y at alpha*r = 0.1; the corrected reading loses it.
    out = lever_arm_correction((0.0, 0.0, 9.8), (0, 0, 0), (0, 0, 1.0), (0.1, 0, 0))
    assert out == pytest.approx((0.0, -0.1, 9.8))


def test_lever_arm_zero_gyro_and_alpha_changes_nothing():
    acc = (0.5, -0.5, 9.8)
    out = lever_arm_correction(acc, (0, 0, 0), (0, 0, 0), (0.05, 0.02, -0.01))
    assert out == pytest.approx(acc)


# ---- euler_to_quat -----------------------------------------------------------------
def test_euler_to_quat_identity():
    assert euler_to_quat(0, 0, 0) == pytest.approx((0, 0, 0, 1))


def test_euler_to_quat_known_rotations():
    z, w = euler_to_quat(0, 0, math.pi / 2)[2:]
    assert (z, w) == pytest.approx((math.sqrt(0.5), math.sqrt(0.5)))
    z, w = euler_to_quat(0, 0, math.pi)[2:]
    assert (z, w) == pytest.approx((1.0, 0.0), abs=1e-12)
    x, w = euler_to_quat(math.pi / 2, 0, 0)[0], euler_to_quat(math.pi / 2, 0, 0)[3]
    assert (x, w) == pytest.approx((math.sqrt(0.5), math.sqrt(0.5)))


def test_euler_to_quat_is_unit_and_matches_matrix():
    for r, p, y in ((0.2, -0.5, 1.1), (-2.0, 0.3, -0.4)):
        q = euler_to_quat(r, p, y)
        assert math.sumprod(q, q) == pytest.approx(1.0, abs=1e-12)
        # cross-check against the matrix convention: rotate the x basis vector
        m = euler_to_matrix(r, p, y)
        rotated = _matvec3(m, (1, 0, 0))
        qx, qy, qz, qw = q
        q_rot = _matvec3(((1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)),
                          (2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)),
                          (2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy))),
                         (1, 0, 0))
        assert q_rot == pytest.approx(rotated, abs=1e-9)
