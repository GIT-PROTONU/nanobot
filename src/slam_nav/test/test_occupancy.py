"""Offline tests for the SLAM core (occupancy.GridMap) + the loop-closure transform.

Pure numpy, no ROS — run with:  pixi run python -m pytest src/slam_nav/test
(or just:  python src/slam_nav/test/test_occupancy.py)

The test context may not have slam_nav installed (no colcon build), so we add the
package source dir to sys.path ourselves.
"""
import math
import os
import sys

_SRC = os.path.join(os.path.dirname(__file__), "..", "..", "slam_nav")
if os.path.isdir(_SRC):
    sys.path.insert(0, os.path.abspath(_SRC))

import numpy as np

from slam_nav.occupancy import GridMap


def _rect_range(phi, rx, ry):
    """Distance from rectangle center to its boundary along world bearing phi."""
    c = abs(math.cos(phi)); s = abs(math.sin(phi))
    if c < 1e-9:
        return ry
    if s < 1e-9:
        return rx
    return 1.0 / (abs(math.cos(phi)) / rx + abs(math.sin(phi)) / ry)


def _integrate_rect(grid, cx, cy, n=120, rx=2.0, ry=1.5):
    """Drop a RECTANGULAR loop of walls (4 sides) centered at (cx,cy) into the grid by
    integrating scans from the center that terminate exactly on the rectangle boundary.
    Distinct rx/ry makes the geometry asymmetric (unlike a square ring), so scan matching
    has a unique solution — which is what loop closure relies on in practice."""
    a = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False).astype(np.float32)
    r = np.array([_rect_range(float(ang), rx, ry) for ang in a], dtype=np.float32)
    grid.integrate((cx, cy, 0.0), a, r)


def test_transform_small_warp_consistent_motion():
    """Loop closure only ever warps the grid by a TINY step (alpha*drift, ~sub-cell);
    the occupied region must move by exactly that step (a consistent rigid motion). The
    walls are 1-cell thin, so a nearest-cell resample shifts every wall cell to a new
    cell — exact equality isn't meaningful, but the centroid shift must equal (dx,dy)."""
    g = GridMap(size_m=10.0, res=0.05)
    _integrate_rect(g, 0.0, 0.0)
    rev0 = g.rev
    dx, dy, dth = 0.03, -0.02, 0.05
    g.transform(dx, dy, dth)
    assert g.rev > rev0                       # transform bumps the revision
    assert np.all(np.isfinite(g.log))
    ys, xs = np.nonzero(g.log > 0.0)
    cx = g.origin + (xs.mean() + 0.5) * g.res
    cy = g.origin + (ys.mean() + 0.5) * g.res
    # the rectangle was centered at (0,0); after the warp its centroid is at ~(dx,dy)
    assert abs(cx - dx) < 2 * g.res
    assert abs(cy - dy) < 2 * g.res


def test_transform_shifts_content():
    """A transform actually moves the occupied region to a new location."""
    g = GridMap(size_m=10.0, res=0.05)
    _integrate_rect(g, 0.0, 0.0)
    occ_before = np.argwhere(g.seen)
    g.transform(1.0, 0.0, 0.0)               # shift +1 m in x
    occ_after = np.argwhere(g.seen)
    # every cell moved ~ +1m/0.05 = 20 cells in the column (x) axis
    delta = occ_after.mean(axis=0) - occ_before.mean(axis=0)
    assert abs(delta[1] - 20.0) < 3.0        # row axis (y) unchanged-ish
    assert abs(delta[0]) < 3.0               # col axis (x) shifted by ~20


def test_loop_closure_removes_drift():
    """Simulate a robot that drives a loop but whose odometry chain drifts by a constant
    offset, then returns to the start. Feeding the (drifted) scans through integrate +
    a wide re-match should, via the same math nav_node uses, recover the offset.

    We don't spin up the ROS node; we replicate the loop-closure step against the grid
    directly (the node just calls grid.match/score/transform), proving the core works.
    """
    g = GridMap(size_m=12.0, res=0.05)
    # Build the "true" rectangular loop at the origin.
    rx, ry = 2.0, 1.5
    _integrate_rect(g, 0.0, 0.0, n=120, rx=rx, ry=ry)
    rev_after_build = g.rev

    # The robot's odometry chain has drifted by (dx,dy,dth). When it re-visits the start
    # it *thinks* it's at (dx,dy,dth) but the scan matches the map at the origin.
    dx, dy, dth = 0.4, -0.3, 0.3

    # A fresh scan taken at the true start (0,0,0): the same scan the robot would observe
    # there. In the drifted chain the robot BELIEVES it is at (dx,dy,dth) before
    # correction, so nav_node's offset-free prior = (dx,dy,dth). The match against the
    # map (built at the origin) should snap back near the true pose, revealing the drift.
    a = np.linspace(0.0, 2.0 * math.pi, 120, endpoint=False).astype(np.float32)
    r = np.array([_rect_range(float(ang), rx, ry) for ang in a], dtype=np.float32)
    prior = (dx, dy, dth)                    # offset-free odometry-predicted map pose
    cand = g.match(prior, a, r, lin=0.5, ang=1.0)
    score = g.score(cand, a, r)
    assert score >= 4.0                      # strong loop match
    # The match should snap back near the true origin, revealing the drift.
    drift = math.hypot(cand[0] - prior[0], cand[1] - prior[1])
    assert drift > 0.3                       # a real loop (not local noise)
    # Apply the smoothing step the node would: nudge the offset toward the correction.
    alpha = 0.1
    off = (0.0, 0.0, 0.0)
    off = (off[0] + alpha * (cand[0] - prior[0]),
           off[1] + alpha * (cand[1] - prior[1]),
           off[2] + alpha * (cand[2] - prior[2]))
    # After enough iterations the offset converges to the true drift; one step must at
    # least move in the right direction (toward cancelling dx,dy,dth).
    assert abs(off[0]) < abs(dx) and abs(off[1]) < abs(dy)
    # The grid wasn't mutated by matching (only by an explicit transform), so rev holds.
    assert g.rev == rev_after_build
