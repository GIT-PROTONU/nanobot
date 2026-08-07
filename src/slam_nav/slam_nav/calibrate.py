"""Pure odometry auto-calibration (no ROS).

Solves the two odometry constants that the hand-tuned robot.yaml values can only
approximate, given two scripted motions measured during the slam_nav self-test:

  * a forward straight leg  -> linear scale  (m per encoder tick, or wheel radius)
  * an in-place spin leg    -> track width   (wheel separation / base length b)

Differential-drive odometry:

  forward distance over a leg    D = 0.5*(dL + dR) * m_per_tick
  heading change over a spin     dth = (dR - dL) / wheel_separation

where dL/dR are signed per-wheel *metres* (tick deltas * m_per_tick) and the IMU is
treated as rotation truth (the single-channel encoders sign ticks by commanded
direction, so their integrated heading drifts with slip — see config/ekf.yaml).

Corrections so odometry matches truth after the drive:

    new_m_per_tick   = m_per_tick        * (commanded / reported)      [linear]
    new_separation   = separation        * (odom_theta / imu_theta)   [rotation]

The rotation ratio multiplies whatever value was TOO SMALL. dth = (dR-dL)/b, so to
fit the IMU truth b must be rescaled by (dR-dL)/imu/dth-truth -> b_new/b_old =
odom_theta / imu_theta. The old self-test hint (nav_node) had this BACKWARDS
(separation * imu/odom) — exactly the sign inversion that kept the ESP32 autocal
"converging the wrong way" on hardware.

All functions are pure float math, unit-testable without ROS.
"""

import math


def scale_from_forward(commanded_m: float, odom_m: float, m_per_tick: float) -> float:
    """Correct m_per_tick from a straight leg. Distance accumulation is linear in
    m_per_tick, so ratio = commanded/reported applies directly.

    Returns the corrected m_per_tick, or the input if the leg is unusable.
    """
    if (not math.isfinite(commanded_m) or not math.isfinite(odom_m)
            or not math.isfinite(m_per_tick)):
        return m_per_tick
    if abs(odom_m) < 1e-9 or m_per_tick <= 0.0:
        return m_per_tick
    ratio = commanded_m / odom_m
    if not math.isfinite(ratio) or ratio <= 0.0:
        return m_per_tick
    return m_per_tick * ratio


def radius_from_scale(new_m_per_tick: float, ticks_per_rev: float) -> float:
    """Wheel radius that yields new_m_per_tick while ticks_per_rev stays fixed
    (m_per_tick = 2*pi*r / ticks_per_rev  =>  r = m_per_tick*ticks_per_rev/(2*pi))."""
    if not math.isfinite(new_m_per_tick) or new_m_per_tick <= 0.0 or ticks_per_rev <= 0:
        return 0.0
    return new_m_per_tick * ticks_per_rev / (2.0 * math.pi)


def separation_from_spin(odom_theta: float, imu_theta: float, separation: float) -> float:
    """Correct the wheel separation (m) so odometry heading matches the IMU on a
    pure in-place spin: new = separation * (odom_theta / imu_theta). Returns the
    corrected value, or the input if a leg wasn't usable."""
    if (not math.isfinite(odom_theta) or not math.isfinite(imu_theta)
            or not math.isfinite(separation)):
        return separation
    if abs(imu_theta) < 1e-6 or separation <= 0.0:
        return separation
    ratio = odom_theta / imu_theta
    if not math.isfinite(ratio) or ratio <= 0.0:
        return separation
    return separation * ratio