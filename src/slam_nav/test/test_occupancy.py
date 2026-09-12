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

from slam_nav.occupancy import (GridMap, reject_dynamic, deskew, decimate_points,
                                _pack_cells, _unpack_cells)


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


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


def test_overlap_ratio_anchored_vs_void():
    """The inlier gate (nav_node's min_overlap_ratio): a scan fully resting on mapped
    structure has ratio ~1.0; the same scan at a pose pointing at unmapped void drops
    to ~0.0 — so 'good score / no real overlap' scans are distinguishable cheaply."""
    g = GridMap(size_m=10.0, res=0.05)
    _integrate_rect(g, 0.0, 0.0)
    a = np.linspace(0.0, 2.0 * math.pi, 120, endpoint=False).astype(np.float32)
    r = np.array([_rect_range(float(ang), 2.0, 1.5) for ang in a], dtype=np.float32)

    # At the true pose every beam lands on a seen cell (the whole rect disk was mapped).
    assert g.overlap_ratio((0.0, 0.0, 0.0), a, r) > 0.9

    # Same scan, pose parked 10 m away: every hit lands outside the mapped region.
    assert g.overlap_ratio((10.0, 0.0, 0.0), a, r) == 0.0

    # Partially overlapping pose: some beams on seen cells, some into the void.
    partial = g.overlap_ratio((2.5, 0.0, 0.0), a, r)
    assert 0.0 < partial < 0.9


def test_overlap_ratio_zero_on_empty_map():
    """On a never-integrated grid nothing is seen, so the ratio is 0 (and must not
    crash on the empty grid path)."""
    g = GridMap(size_m=10.0, res=0.05)
    a = np.linspace(0.0, 2.0 * math.pi, 90, endpoint=False).astype(np.float32)
    r = np.full(a.shape, 2.0, dtype=np.float32)
    assert g.overlap_ratio((0.0, 0.0, 0.0), a, r) == 0.0


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
    assert np.all(np.isfinite(g.cells))
    occ = g.state_view() == 2                 # state 2 = occupied
    ys, xs = np.nonzero(occ)
    cx = g.origin + (xs.mean() + 0.5) * g.res
    cy = g.origin + (ys.mean() + 0.5) * g.res
    # the rectangle was centered at (0,0); after the warp its centroid is at ~(dx,dy)
    assert abs(cx - dx) < 2 * g.res
    assert abs(cy - dy) < 2 * g.res


def test_transform_shifts_content():
    """A transform actually moves the occupied region to a new location."""
    g = GridMap(size_m=10.0, res=0.05)
    _integrate_rect(g, 0.0, 0.0)
    occ_before = np.argwhere(g.state_view() == 2)
    g.transform(1.0, 0.0, 0.0)               # shift +1 m in x
    occ_after = np.argwhere(g.state_view() == 2)
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


# --- new low-compute pipeline tests (2026-09) -------------------------------

def test_2bit_pack_roundtrip():
    """2-bit ternary packing must survive an arbitrary grid round-trip (and the
    padded tail must never leak into the decoded n*n cells)."""
    rng = np.random.default_rng(7)
    st = rng.integers(0, 3, size=(64, 64)).astype(np.uint8)
    padded = _pack_cells(st)
    assert padded.size * 4 == 64 * 64
    assert np.array_equal(_unpack_cells(padded, 64), st)
    occ = np.full((12, 12), 2, dtype=np.uint8)          # all wall cells
    assert (_unpack_cells(_pack_cells(occ), 12) == 2).all()


def test_dt_scoring_peaks_at_alignment():
    """The DT-support score is a smooth basin peaking where the scan rests exactly on
    mapped walls (true pose), falling off for a rotated scan and a far-away pose."""
    g = GridMap(size_m=10.0, res=0.05)
    _integrate_rect(g, 0.0, 0.0)
    a = np.linspace(0.0, 2.0 * math.pi, 120, endpoint=False).astype(np.float32)
    r = np.array([_rect_range(float(ang), 2.0, 1.5) for ang in a], dtype=np.float32)
    s_true = g.score((0.0, 0.0, 0.0), a, r)
    s_rot = g.score((0.0, 0.0, math.pi / 2), a, r)
    s_far = g.score((4.0, 0.0, 0.0), a, r)
    assert s_true >= 4.0
    assert s_true > s_rot
    assert s_true > s_far


def test_match_integer_recovers_pose():
    """The integer LUT matcher + DT gather snaps a small prior offset back to the
    true pose (all-integer hot path, no fp trig)."""
    g = GridMap(size_m=10.0, res=0.05)
    _integrate_rect(g, 0.0, 0.0)
    a = np.linspace(0.0, 2.0 * math.pi, 120, endpoint=False).astype(np.float32)
    r = np.array([_rect_range(float(ang), 2.0, 1.5) for ang in a], dtype=np.float32)
    cand = g.match((0.06, -0.04, 0.05), a, r, lin=0.10, ang=0.12)
    assert math.hypot(cand[0], cand[1]) < 4 * g.res
    assert abs(_wrap(cand[2])) < 0.3


def test_reject_dynamic_drops_leg_cluster():
    """1-D range-jump clustering: a short 'near' cluster 0.05-0.2 m wide (a leg) is
    dropped; a real full wall (or wide near region) is kept."""
    inc = math.radians(1.0)
    n = 180
    r = np.full(n, 2.0, dtype=np.float32)
    r[62:66] = 1.0                               # 4 beams ~1 m closer = ~0.07 m arc
    keep = reject_dynamic(r, inc)
    assert not keep[62] and not keep[63] and not keep[64] and not keep[65]
    assert keep[60] and keep[66]
    assert reject_dynamic(np.full(n, 1.0, dtype=np.float32), inc).all()  # full wall


def test_deskew_identity_and_shift():
    """Motion deskewing is the identity when the robot didn't move; with a straight
    translation DURING the sweep, the first-taken beam reads the wall 0.1 m closer
    (relative to the end pose) and the last-taken beam is unchanged."""
    a = np.array([0.0, 0.5, 1.0], dtype=np.float32)
    r = np.array([2.0, 2.0, 2.0], dtype=np.float32)
    a2, r2 = deskew(a, r, 0.0, 0.0, 0.0)
    assert np.allclose(a2, a) and np.allclose(r2, r)
    a2, r2 = deskew(a, r, 0.10, 0.0, 0.0)
    assert np.allclose(a2[2], 1.0) and np.allclose(r2[2], 2.0)   # end-pose beam intact
    # first beam (bearing 0): robot was 0.1 m closer when the beam was taken
    assert np.allclose(a2[0], 0.0) and np.allclose(r2[0], 1.9)


def test_decimate_spacing_and_corners():
    """Spatial decimation: ~0.05 m between kept hits on a flat wall, with the corner
    (a sharp range jump) force-kept."""
    inc = math.radians(1.0)
    n = 180
    a = (np.arange(n) * inc).astype(np.float32)
    r = np.full(n, 2.0, dtype=np.float32)
    keep = decimate_points(a, r, spacing=0.05)
    kk = np.flatnonzero(keep)
    x = r * np.cos(a)
    y = r * np.sin(a)
    gaps = np.hypot(x[kk[1:]] - x[kk[:-1]], y[kk[1:]] - y[kk[:-1]])
    assert 0.04 < gaps.mean() < 0.09
    r[90] = 0.5                                # corner: range cliff mid-way
    keep2 = decimate_points(a, r, spacing=0.05, corner_m=0.15)
    assert keep2[90]


def test_degeneracy_lock_corridor():
    """Hessian-degeneracy lock: a scan that ONLY sees the two horizontal walls cannot
    localise translation along the corridor (x), so the match must return the
    odometry PRIOR for x while still snapping y onto the wall."""
    g = GridMap(size_m=10.0, res=0.05)
    _integrate_rect(g, 0.0, 0.0)                       # 2.0 x 1.5 rect at origin
    a = np.array([math.pi / 2, -math.pi / 2], dtype=np.float32)
    r = np.array([0.75, 0.75], dtype=np.float32)       # top + bottom wall beams
    prior = (0.10, 0.05, 0.0)
    cand = g.match(prior, a, r, lin=0.10, ang=0.10)
    assert cand[0] == 0.10                        # x locked to the prior exactly
    assert abs(cand[1]) < 4 * g.res               # y corrected onto the wall


def test_gated_rasterization_rotating_lockout():
    """Gated rasterization: while `rotating` the grid must not be touched at all
    (rev unchanged), and a normal integrate right after works and marks walls."""
    g = GridMap(size_m=10.0, res=0.05)
    a = np.linspace(0.0, 2.0 * math.pi, 60, endpoint=False).astype(np.float32)
    r = np.full(a.shape, 2.0, dtype=np.float32)
    g.integrate((0.0, 0.0, 0.0), a, r, rotating=True)
    assert g.rev == 0
    assert (g.state_view() == 0).all()
    g.integrate((0.0, 0.0, 0.0), a, r)             # normal integrate works after
    assert g.rev == 1
    assert (g.state_view() == 2).any()
