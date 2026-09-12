"""Tiny 2D occupancy-grid SLAM core — pure numpy, no ROS deps (stays cheap + testable).

Low-compute scan-matching architecture (2026-09):

* The PERMANENT occupancy map is a 2-bit packed ternary grid (00=Unknown, 01=Free,
  10=Occupied) — one uint8 holds four cells, so a 1200x1200 @ 2 cm whole-floor map
  persists/serves in ~360 KB instead of ~5.8 MB of float32 log-odds. Ephemeral
  working caches (decoded state, an int8 distance transform, a small bleach counter)
  live alongside it but are never persisted.

* Scoring no longer ray-casts log-odds. A pre-computed integer DISTANCE TRANSFORM of
  the occupied mask is rebuilt (cached per `rev`) with a pure-numpy int8 chamfer that
  is exact out to the support radius; `score()` is a rapid array gather of the scan's
  endpoint cells into that DT.

* The matcher's inner loop is integer-only: beam angles and candidate headings are
  quantised to a 4096-bin table (0.088°/bin) and cos/sin come from pre-tabulated
  fixed-point LUTs (2^16 scale), so a per-candidate heading is just a shift of the
  angle index plus a multiply-and-shift — no fp trig, no float ops in the hot path.

* A Hessian-degeneracy check measures the score-surface variance on each axis of the
  final coarse-to-fine pass and LOCKS any flat axis back to the odometry prior (a
  corridor or symmetric pocket can't pull the pose along a direction the map doesn't
  constrain). The wheels own every unobservable axis.

* The scan PREPARATION pipeline (module-level functions) runs before matching:
  `reject_dynamic` (1-D range-jump clustering drops moving-leg clusters 0.05-0.2 m
  wide), `deskew` (per-beam odometry interpolation across the mirror sweep), and
  `decimate_points` (keeps hits ~0.05 m apart in wall space, preserving corners).
  `GridMap.conflict_mask` additionally drops beams whose endpoint lands in CONFIRMED
  free space — the second dynamic-obstacle stage.

* `integrate()` honours GATED RASTERIZATION: while the robot is actively rotating the
  map is locked out entirely (a spinning mirror paints arcs of beams and would smear
  walls); pose refinement keeps running, only the grid freezes.

Memory at 24 m / 2 cm (1200x1200): packed cells 360 KB + state 1.44 MB + bleach
1.44 MB + DT 1.44 MB (int8) + no-go 1.44 MB ≈ 6.2 MB (was ~9 MB for float32+bool).
"""
import math
import os

import numpy as np

# --- fixed-point trigonometry -------------------------------------------------
# LUT size is a power of two so the wrap (angle mod 2pi) is a cheap bitmask. All
# angles are converted to bin indices NQ*(angle/2pi); cos/sin for a bin are stored in
# fixed point scaled by COS_SCALE=2^16, so `distance_cells * cos_lut >> 16` is the
# axis projection in grid cells, rounded half-up, with zero float arithmetic.
ANG_NQ = 4096                        # LUT bins per full turn (0.0879 deg/bin)
ANG2Q = ANG_NQ / (2.0 * math.pi)     # radians -> bin index
ANG_STEP = 2.0 * math.pi / ANG_NQ     # bin index -> radians
COS_SCALE = 1 << 16                   # fixed-point scale for the cos/sin tables
COS_HALF = 1 << 15                    # half = round-half-up constant for the shift
COS_SHIFT = 16

# --- occupancy states (2 bits per cell) --------------------------------------
STATE_UNKNOWN = 0                    # 00: never swept by a beam
STATE_FREE = 1                       # 01: beams have confirmed empty here
STATE_OCC = 2                        # 10: a beam endpoint (wall/obstacle)

SUPPORT_RADIUS_M = 0.15              # m: scoring kernel radius (DT support basin)
EXACT_B = 2                          # extra points for a beam endpoint EXACTLY on an
                                     # occupied cell: the old log-odds 'wall spike' on
                                     # top of the DT basin, so the peak stays sharp
                                     # (without it a 1-2 cell plateau lets a scan park
                                     # 10-15 cm off and match just as well)
BLEACH_N = 16                        # beam passes through an OCC cell before it's
                                     # freed again (moved chair / opened door)


def _pack_cells(state):
    """2-bit pack an (n,n) uint8 state array (0/1/2) into ceil(n*n/4) uint8 bytes.
    4 cells per byte, LSB-first: cell index k -> bits [2k, 2k+1] of byte k//4."""
    flat = np.ascontiguousarray(state, dtype=np.uint8).ravel()
    pad = (-flat.size) & 3                       # pad to a whole number of bytes
    if pad:
        flat = np.concatenate([flat, np.zeros(pad, dtype=np.uint8)])
    f = flat.reshape(-1, 4)
    return (f[:, 0] | (f[:, 1] << 2) | (f[:, 2] << 4) | (f[:, 3] << 6)).astype(np.uint8)


def _unpack_cells(packed, n):
    """Inverse of _pack_cells -> (n,n) uint8 states. Each byte splits back into its
    4 cells by bit-slicing (offset 0/2/4/6 bits), then the padded tail is dropped."""
    p = np.ascontiguousarray(packed, dtype=np.uint8).astype(np.uint16)
    flat = np.empty(p.size * 4, dtype=np.uint8)
    flat[::4] = (p & 0x03).astype(np.uint8)          # cells 0,4,8,..  (bits 0-1)
    flat[1::4] = ((p >> 2) & 0x03).astype(np.uint8)  # cells 1,5,9,..  (bits 2-3)
    flat[2::4] = ((p >> 4) & 0x03).astype(np.uint8)  # cells 2,6,10,.. (bits 4-5)
    flat[3::4] = ((p >> 6) & 0x03).astype(np.uint8)  # cells 3,7,11,.. (bits 6-7)
    return flat[: n * n].reshape(n, n)


# --- scan preparation (module-level; caller runs these before match/integrate) --

def reject_dynamic(ranges, angle_inc, rmin_w=0.05, rmax_w=0.20,
                   rmin_range=0.35, jump=0.25):
    """Return a bool KEEP mask over beams for DYNAMIC-OBJECT REJECTION.

    1-D range-jump clustering along the beam sweep: a moving occluder (a person's
    legs) appears as a short 'near' cluster — the range drops a step below the
    background wall, holds a few beams, then jumps back up. We detect every such
    cluster, measure its wall-arc width `mean_range * beams * angle_inc`, and drop
    clusters whose width falls in [rmin_w, rmax_w] m (a leg is 0.05-0.2 m wide). A
    real wall makes a wide (or full-sweep) near region and is kept. `rmin_range`
    guards the robot's own nearby mount/floor from being mistaken for a leg. The
    sweep is treated circularly (a cluster straddling the angle-0 seam is resolved by
    scanning a tripled copy of the range array)."""
    n = ranges.size
    keep = np.ones(n, dtype=bool)
    if n < 6 or not (angle_inc > 0.0):
        return keep
    rr = np.asarray(ranges, dtype=np.float64)
    r3 = np.concatenate([rr, rr, rr])                 # tripled: circular cluster scan
    d = np.diff(r3)                                    # along-sweep range derivative
    starts = np.flatnonzero(d[:-1] < -jump) + 1        # beam after a drop (near begins)
    ends = np.flatnonzero(d[:-1] > jump) + 1           # first beam back at the far range
    for s0 in starts:
        s = int(s0)
        if s >= n:                                     # clusters must begin in [0, n)
            continue
        k = int(np.searchsorted(ends, s, side="right"))
        if k >= ends.size:
            continue
        e = int(ends[k])                               # first end strictly after s
        if e - s >= n:                                 # spans >half the sweep: a wall
            continue
        span = e - s
        idx = np.arange(s, s + span) % n
        rmid = float(rr[idx].mean())
        arc = rmid * span * angle_inc                  # perpendicular wall-arc (m)
        if rmin_w <= arc <= rmax_w and rmid >= rmin_range:
            keep[idx] = False
    return keep


def deskew(angles, ranges, dx, dy, dth):
    """MOTION DESKEW: interpolate the odometry across the mirror sweep.

    The LDS spins continuously, so beam i was acquired at sweep fraction f = i/(n-1)
    while the robot was still moving. The acquisition pose is `end + (f-1)*(dx,dy,dth)`
    where (dx,dy,dth) is the map-frame motion over the WHOLE sweep (end - start). Each
    beam's endpoint is pushed by that interpolated pose and re-expressed as a fresh
    (angle, range) relative to the END pose, so everything downstream keeps a single
    robot pose. Returns (a2, r2) float32. Exact identity when d = 0."""
    n = ranges.size
    if n == 0:
        return angles, ranges
    f = np.arange(n, dtype=np.float32) / max(1, n - 1)
    ox = (f - 1.0) * dx                               # pose offset vs END pose (map)
    oy = (f - 1.0) * dy
    brg = np.asarray(angles, dtype=np.float32) + (f - 1.0) * dth
    ex = ox + np.asarray(ranges, dtype=np.float32) * np.cos(brg)
    ey = oy + np.asarray(ranges, dtype=np.float32) * np.sin(brg)
    a2 = np.arctan2(ey, ex)
    r2 = np.maximum(np.hypot(ex, ey), 0.0)
    return a2.astype(np.float32), r2.astype(np.float32)


def decimate_points(angles, ranges, spacing=0.05, corner_m=0.15):
    """SPATIAL DECIMATION for matching: keep beams so consecutive kept HIT POINTS are
    ~`spacing` metres apart in wall space (dropping redundant flat-wall beams) while
    FORCING corners — a sharp range jump between neighbours — to survive. Returns a
    bool KEEP mask."""
    n = ranges.size
    if n < 3:
        return np.ones(n, dtype=bool)
    a = np.asarray(angles, dtype=np.float64)
    r = np.asarray(ranges, dtype=np.float64)
    x = r * np.cos(a)
    y = r * np.sin(a)
    step = np.empty(n)
    step[1:] = np.hypot(x[1:] - x[:-1], y[1:] - y[:-1])
    step[0] = step[1] if n > 1 else 0.0
    dr = np.empty(n)
    dr[1:] = np.abs(r[1:] - r[:-1])
    dr[0] = 0.0
    keep = np.zeros(n, dtype=bool)
    keep[0] = True
    acc = 0.0
    for i in range(1, n - 1):
        acc += step[i]
        # keep when the 0.05 m tick is reached, OR a corner (range cliff) lands here
        if acc >= spacing or dr[i] > corner_m or dr[i + 1] > corner_m:
            keep[i] = True
            acc = 0.0
    keep[-1] = True
    return keep


class GridMap:
    def __init__(self, size_m=24.0, res=0.05, rmin=0.12, rmax=6.0):
        self.res = float(res)
        self.n = int(round(size_m / self.res))          # square n x n grid
        self.rmin, self.rmax = float(rmin), float(rmax)
        # World coordinate of cell [0,0] (lower-left). The robot starts at the centre,
        # so the map can grow outward in every direction from the origin.
        self.origin = -0.5 * self.n * self.res
        # --- PERMANENT 2-bit packed occupancy grid (persisted + served) ----------
        self.cells = _pack_cells(np.zeros((self.n, self.n), dtype=np.uint8))
        # --- ephemeral working caches, rebuilt when `rev` moves -----------------
        self._state = None            # (n,n) uint8 decoded 0/1/2
        self._state_rev = -1
        self._dt = None               # (n,n) int8 distance-to-occupied, in cells
        self._dt_rev = -1
        # The old log-odds map's "bleach" effect (a wall fades after beams sweep
        # through it) in integer form: an OCCUPIED cell counts free-space passes and
        # flips to FREE after BLEACH_N of them. Ephemeral, never persisted.
        self._bleach = np.zeros((self.n, self.n), dtype=np.uint8)
        self.rev = 0
        # DT-support kernel radius in cells + the free-space 'confirmed' margin for
        # the map-conflict filter (a wall return needs to be this far inside a free
        # area before it counts as transient).
        self.SUPPORT = max(3, int(round(SUPPORT_RADIUS_M / self.res)))
        self.CONFIRM = max(2, self.SUPPORT // 2)
        # Max score value one beam can contribute = SUPPORT + EXACT_B; scores are
        # NORMALISED by this so '1.0' means one perfectly-aligned beam regardless of
        # map resolution (keeps the nav absolute thresholds meaningful).
        self.SCORE_MAX = self.SUPPORT + EXACT_B
        # No-go mask: cells the PLANNER must never route through (human-marked
        # restricted zones). Untouched by scan integration; persisted + web overlay.
        self.forbidden = np.zeros((self.n, self.n), dtype=bool)
        self.forb_rev = 0
        # Fixed yaw offset between the map frame and the odom frame (radians), set
        # once at seed time by the nav node. PERSISTED with the grid so that a later
        # boot that loads the saved map re-adopts the same map-frame-vs-odom rotation
        # instead of treating the loaded cell layout as if it were drawn in this run's
        # odom frame (which silently rotates/shifts every wall -> the "map shifted and
        # rotated over each other" mismatch, 2026-09-12). nav_node owns the live value.
        self.rot_from = 0.0
        # Seed tuple persisted alongside rot_from (see nav_node): the odom pose
        # (x, y, yaw) at the instant the map frame was anchored, plus the map-frame
        # position/heading that pose mapped to. With these two the nav node can rebuild
        # the exact map<->odom rigid frame on a reload (R(rot_from) about the OPENED
        # seed), instead of guessing the frame from this run's odom origin.
        self.seed_odom_x = self.seed_odom_y = self.seed_odom_t = 0.0
        self.seed_dx = self.seed_dy = self.seed_pth = 0.0
        self._coarse_cache = None   # (key, (blocked, seen_c, m, res_c)) memo
        # --- integer trigonometry LUTs (built once; fixed at 4096 bins each) -----
        qq = np.arange(ANG_NQ, dtype=np.float64)
        self.cosq = np.rint(np.cos(qq * ANG_STEP) * COS_SCALE).astype(np.int32)
        self.sinq = np.rint(np.sin(qq * ANG_STEP) * COS_SCALE).astype(np.int32)

    # --- world <-> grid ------------------------------------------------------
    def w2g(self, x, y):
        """World metres -> (col, row) integer cell indices (no bounds check)."""
        c = np.floor((np.asarray(x) - self.origin) / self.res).astype(np.int32)
        r = np.floor((np.asarray(y) - self.origin) / self.res).astype(np.int32)
        return c, r

    def _inb(self, c, r):
        return (c >= 0) & (c < self.n) & (r >= 0) & (r < self.n)

    @staticmethod
    def _valid(ranges, rmin, rmax):
        return np.isfinite(ranges) & (ranges >= rmin) & (ranges <= rmax)

    # --- integer fixed-point helpers (the only trig the matcher ever does) -----
    @staticmethod
    def _quant(angles):
        """Radians -> (int32) LUT bin index. ANG_NQ is a power of two, so the wrap is
        a `& (NQ-1)` bitmask instead of a modulo; rint does round-half-even, plenty."""
        return (np.rint(np.asarray(angles, dtype=np.float64) * ANG2Q).astype(np.int64)
                & (ANG_NQ - 1)).astype(np.int32)

    def _rcell(self, ranges):
        """Metres -> integer cell distance from the robot (banker's rounding)."""
        return np.rint(np.asarray(ranges, dtype=np.float64) / self.res).astype(np.int32)

    @staticmethod
    def _off(rc, lut):
        """Fixed-point axis projection: `rc * lut >> 16` with round-half-up.
        rc is a cell distance (int32), lut a 2^16-scaled cos/sin table entry; the
        int64 intermediate keeps the product exact, the shift returns cells. This one
        helper is the ENTIRE trigonometry the hot loops need."""
        return ((rc.astype(np.int64) * lut.astype(np.int64) + COS_HALF)
                >> COS_SHIFT).astype(np.int32)

    @staticmethod
    def _kval(d, support, bonus):
        """Map DT distances -> per-beam score values. Graded as the old log-odds did:
        the DT basin (linear support) makes the surface climb smoothly toward walls,
        and an EXACT wall hit (dt == 0) earns an extra `bonus` — the sharp "wall
        spike" that keeps the peak one cell wide instead of a plateau. All integer."""
        dd = d.astype(np.int32)
        return np.where(dd == 0, support + bonus, np.maximum(0, support - dd))

    def _endpoints(self, xc0, yc0, q, rc):
        """(col, row) endpoint cells for a scan at integer pose cell (xc0, yc0)."""
        co = self.cosq[q]
        si = self.sinq[q]
        return xc0 + self._off(rc, co), yc0 + self._off(rc, si)

    def state_view(self):
        """Decoded (n,n) uint8 0/1/2 working copy, cached per rev (kept in sync by
        integrate/transform/load so the cache is what every consumer reads)."""
        if self._state is None or self._state_rev != self.rev:
            self._state = _unpack_cells(self.cells, self.n)
            self._state_rev = self.rev
        return self._state

    # --- distance transform (pre-computed compute saver) ----------------------
    def _dt_map(self):
        """Integer chamfer DT of the occupied mask, in CELLS, cached per rev. Pure
        numpy int8: each sweep rolls the field along the 4 axes and takes the minimum
        (city-block distance +1 per move), so after SUPPORT+2 sweeps every cell within
        the support radius carries its EXACT L1 distance and the rest saturate at
        SUPPORT. The scoring kernel below is then a pure array gather."""
        if self._dt is not None and self._dt_rev == self.rev:
            return self._dt
        st = self.state_view()
        n = self.n
        cap = self.SUPPORT
        big = cap + 4                                     # unreachable filler
        dt = np.full((n, n), big, dtype=np.int8)
        dt[st == STATE_OCC] = 0
        for _ in range(cap + 2):
            u = np.roll(dt, -1, axis=0); u[-1, :] = big
            d = np.roll(dt, 1, axis=0); d[0, :] = big
            l = np.roll(dt, -1, axis=1); l[:, -1] = big
            r = np.roll(dt, 1, axis=1); r[:, 0] = big
            dt = np.minimum(dt, np.minimum(np.minimum(u + 1, d + 1),
                                            np.minimum(l + 1, r + 1)))
        self._dt = np.minimum(dt, cap)
        self._dt_rev = self.rev
        return self._dt

    # --- scan-to-map matching -------------------------------------------------
    def score(self, pose, angles, ranges):
        """DT-support score: sum of max(0, SUPPORT - dt_cells) over in-bounds scan
        endpoints, NORMALISED by SUPPORT. The value is ~'how many beams rest on (or
        within 0.15 m of) structure', so it is res-independent and the existing
        nav thresholds (min_match_score=1.., recover_exit_score=20.) keep their
        meaning. Hot path is integer; the single divide happens at the end."""
        if ranges.size == 0:
            return 0.0
        px, py, pth = pose
        q = self._quant(np.asarray(angles, dtype=np.float64) + pth)
        rc = self._rcell(ranges)
        xc0 = int(round((px - self.origin) / self.res))
        yc0 = int(round((py - self.origin) / self.res))
        cxp, cyp = self._endpoints(xc0, yc0, q, rc)
        n = self.n
        ok = (cxp >= 0) & (cxp < n) & (cyp >= 0) & (cyp < n)
        if not ok.any():
            return 0.0
        dt = self._dt_map()
        cs = np.clip(cxp, 0, n - 1)
        rs = np.clip(cyp, 0, n - 1)
        v = dt[rs, cs]
        vals = self._kval(v, self.SUPPORT, EXACT_B)
        total = int(np.where(ok, vals, 0).sum())
        return total / float(self.SCORE_MAX)

    def overlap_ratio(self, pose, angles, ranges):
        """Fraction of in-bounds beams that land on a cell the map has SEEN (free or
        occupied). The inlier complement of score(): says how much of the scan rests
        on previously-mapped area, so a 'good score / no real overlap' scan gets
        caught. 1.0 = every beam on a seen cell; 0.0 = nothing overlaps."""
        if ranges.size == 0:
            return 0.0
        px, py, pth = pose
        q = self._quant(np.asarray(angles, dtype=np.float64) + pth)
        rc = self._rcell(ranges)
        xc0 = int(round((px - self.origin) / self.res))
        yc0 = int(round((py - self.origin) / self.res))
        cxp, cyp = self._endpoints(xc0, yc0, q, rc)
        n = self.n
        ok = (cxp >= 0) & (cxp < n) & (cyp >= 0) & (cyp < n)
        if not ok.any():
            return 0.0
        st = self.state_view()
        cs = np.clip(cxp, 0, n - 1)
        rs = np.clip(cyp, 0, n - 1)
        seen = st[rs, cs] != STATE_UNKNOWN
        return float(seen[ok].mean())

    def match(self, prior, angles, ranges, lin=0.10, ang=0.12, half=4, refine=2,
              lock_degenerate=True):
        """Correlative scan-to-map match: coarse-to-fine search around `prior` for the
        (x, y, theta) that maximises the DT-support score. Every per-candidate op is
        integer (LUT trig + fixed-point offsets + DT gather); floats appear only in
        window setup and the returned pose. `lock_degenerate` measures the score
        surface's variance along each axis of the final pass and, wherever it is flat,
        locks that axis entirely to the odometry prior (Hessian-degeneracy lock)."""
        bx, by, bth = prior
        if ranges.size == 0:
            return bx, by, bth
        anq = self._quant(angles)
        rc = self._rcell(ranges)
        n = self.n
        dt = self._dt_map()
        best_s, best_pose = -1, (bx, by, bth)
        for it in range(refine):
            sc = 0.35 ** it                               # shrink the window each pass
            xs = bx + np.linspace(-lin * sc, lin * sc, 2 * half + 1)
            ys = by + np.linspace(-lin * sc, lin * sc, 2 * half + 1)
            ths = bth + np.linspace(-ang * sc, ang * sc, 2 * half + 1)
            xc = np.rint((xs - self.origin) / self.res).astype(np.int32)
            yc = np.rint((ys - self.origin) / self.res).astype(np.int32)
            tq = self._quant(ths)
            final_pass = it == refine - 1
            mats = [] if final_pass else None
            for k, t in enumerate(tq):
                # one heading: shift the beam angle LUT indices by the heading bin, so
                # cos/sin for EVERY beam is a table gather + one fixed-point multiply.
                q = (anq + t) & (ANG_NQ - 1)
                co = self.cosq[q]
                si = self.sinq[q]
                hxc = self._off(rc, co)                   # per-beam offset (cells)
                hyc = self._off(rc, si)
                cx = xc[:, None] + hxc[None, :]           # (Nx, P) candidate cols
                cy = yc[:, None] + hyc[None, :]           # (Ny, P) candidate rows
                okx = (cx >= 0) & (cx < n)
                oky = (cy >= 0) & (cy < n)
                cxc = np.clip(cx, 0, n - 1)
                cyc = np.clip(cy, 0, n - 1)
                # gather DT over the (Nx, Ny, P) lattice, convert to the score kernel,
                # mask out-of-bounds, sum per (x,y) candidate: all ints, no floats.
                vals = self._kval(dt[cyc[None, :, :], cxc[:, None, :]],
                                  self.SUPPORT, EXACT_B)
                s = np.where(okx[:, None, :] & oky[None, :, :], vals, 0).sum(axis=2)
                i, j = np.unravel_index(int(np.argmax(s)), s.shape)
                sv = int(s[i, j])
                if sv > best_s:
                    best_s, best_pose = sv, (float(xs[i]), float(ys[j]), float(ths[k]))
                if final_pass:
                    mats.append(s)
            if final_pass and mats:
                # --- Hessian-degeneracy lock ----------------------------------
                # Var(scores) along the x/y/heading strips of the final surface: a
                # flat strip means the map doesn't constrain that axis here (long
                # corridor, symmetric pocket) -> let the ODOMETRY PRIOR own it.
                S = np.stack(mats)                        # (Nt, Nx, Ny) int
                ii = int(np.argmax(S))
                kth, i1, j1 = np.unravel_index(ii, S.shape)
                vx = float(S[kth, :, j1].var())
                vy = float(S[kth, i1, :].var())
                vt = float(S[:, i1, j1].var())
                denom = max(vx + vy, 1e-9)
                eps = 0.03
                xo, yo, to = best_pose
                if vx < eps * denom:
                    xo = bx                                # x unobservable -> lock
                if vy < eps * denom:
                    yo = by                                # y unobservable -> lock
                if vt < eps * denom:
                    to = bth                               # heading unobservable
                best_pose = (xo, yo, to)
        return best_pose

    def relocalize(self, angles, ranges, step=4, n_headings=16, npts=90, keep=6):
        """Global scan-to-map search (kidnap recovery): scores a decimated scan at
        EVERY grid cell on a `step`-cell lattice across `n_headings` yaw steps, then
        coarse-to-fine refines the top `keep` candidates. Returns
        (x, y, theta, score) or None when nothing above the free-space floor scored.
        Same integer DT machinery as match()."""
        if ranges.size == 0:
            return None
        if len(ranges) > npts:
            idx = np.linspace(0, len(ranges) - 1, npts).astype(int)
            angles, ranges = angles[idx], ranges[idx]
        anq = self._quant(angles)
        rc = self._rcell(ranges)
        n = self.n
        dt = self._dt_map()
        xc = np.arange(0, n, max(1, int(step))).astype(np.int32)
        yc = xc
        ths = np.linspace(-np.pi, np.pi, n_headings, endpoint=False)
        tq = self._quant(ths)
        cands = []
        for ki, t in enumerate(tq):
            q = (anq + t) & (ANG_NQ - 1)
            co = self.cosq[q]
            si = self.sinq[q]
            hxc = self._off(rc, co)
            hyc = self._off(rc, si)
            cx = xc[:, None] + hxc[None, :]
            cy = yc[:, None] + hyc[None, :]
            okx = (cx >= 0) & (cx < n)
            oky = (cy >= 0) & (cy < n)
            cxc = np.clip(cx, 0, n - 1)
            cyc = np.clip(cy, 0, n - 1)
            vals = self._kval(dt[cyc[None, :, :], cxc[:, None, :]], self.SUPPORT, EXACT_B)
            s = np.where(okx[:, None, :] & oky[None, :, :], vals, 0).sum(axis=2)
            k2 = min(2, int(s.size))
            if k2 >= 1:
                th0 = float(ths[ki])
                for fi in np.argpartition(s.ravel(), -k2)[-k2:]:
                    i, j = np.unravel_index(int(fi), s.shape)
                    cands.append((int(s[i, j]), th0,
                                  self.origin + (xc[i] + 0.5) * self.res,
                                  self.origin + (yc[j] + 0.5) * self.res))
        cands.sort(key=lambda c: -c[0])
        best = None
        for _, th, x, y in cands[:keep]:
            cand = self.match((x, y, th), angles, ranges,
                              lin=2 * step * self.res, ang=0.35, half=4, refine=3)
            sc = self.score(cand, angles, ranges)
            if best is None or sc > best[3]:
                best = (cand[0], cand[1], cand[2], sc)
        if best is None or best[3] <= 0.0:
            return None
        return best

    def conflict_mask(self, pose, angles, ranges, confirm=None):
        """Map-conflict filtering (2nd dynamic-obstacle stage): return a bool KEEP mask
        over beams. A 'wall' beam whose endpoint lands on a cell the map has CONFIRMED
        as free space (state FREE and at least `confirm` cells from any occupied cell)
        is a transient return — a moving obstacle or a ghost — not a structure change,
        so it is dropped before matching and integration to stop phantom walls from
        appearing inside an open floor."""
        n = ranges.size
        keep = np.ones(n, dtype=bool)
        if n == 0:
            return keep
        px, py, pth = pose
        q = self._quant(np.asarray(angles, dtype=np.float64) + pth)
        rc = self._rcell(ranges)
        xc0 = int(round((px - self.origin) / self.res))
        yc0 = int(round((py - self.origin) / self.res))
        cxp, cyp = self._endpoints(xc0, yc0, q, rc)
        n2 = self.n
        ok = (cxp >= 0) & (cxp < n2) & (cyp >= 0) & (cyp < n2)
        if not ok.any():
            return keep
        st = self.state_view()
        dt = self._dt_map()
        conf = self.CONFIRM if confirm is None else int(confirm)
        cs = np.clip(cxp, 0, n2 - 1)
        rs = np.clip(cyp, 0, n2 - 1)
        confirmed_free = (st[rs, cs] == STATE_FREE) & (dt[rs, cs] >= conf)
        keep &= ~(confirmed_free & ok)
        return keep

    # --- map update ----------------------------------------------------------
    def integrate(self, pose, angles, ranges, rotating=False):
        """Rasterize a scan into the ternary grid (endpoint -> Occupied, ray cells ->
        Free, occupied cells bleached after BLEACH_N passes). GATED RASTERIZATION:
        when `rotating` is True (the robot is spinning), the map is locked out
        completely — a rotating mirror paints each beam as an arc and would smear the
        walls — so we return WITHOUT mutating anything (pose refinement via match()
        may keep running). Returns True if content changed."""
        if rotating:
            return False
        px, py, pth = pose
        v = self._valid(ranges, self.rmin, self.rmax)
        a = np.asarray(angles, dtype=np.float64)[v] + pth
        rr = np.asarray(ranges, dtype=np.float64)[v]
        if rr.size == 0:
            return False
        st = self.state_view()
        n = self.n
        q = self._quant(a)
        co = self.cosq[q]
        si = self.sinq[q]
        rc = self._rcell(rr)
        xc0 = int(round((px - self.origin) / self.res))
        yc0 = int(round((py - self.origin) / self.res))

        # occupied endpoints: mark occupied (reset any pending bleach).
        ec, er = self._endpoints(xc0, yc0, q, rc)
        m = (ec >= 0) & (ec < n) & (er >= 0) & (er < n)
        if m.any():
            ecm = ec[m]
            erm = er[m]
            st[erm, ecm] = STATE_OCC
            self._bleach[erm, ecm] = 0

        # free space: sample every ray at the grid pitch from the robot up to one cell
        # shy of the hit. Per sample the cell offset is `k * dir >> 16` in cells —
        # integer fixed point, no float ops. Cells that are OCCUPIED only accumulate a
        # bleach count instead of flipping; free/unknown become FREE immediately.
        per = np.maximum(0, rc - 1)                       # samples per beam
        total = int(per.sum())
        if total:
            bi = np.repeat(np.arange(rr.size, dtype=np.int64), per)
            k = np.arange(total, dtype=np.int64) - np.repeat(np.cumsum(per) - per, per)
            sox = self._off(k, co[bi])
            soy = self._off(k, si[bi])
            fc = xc0 + sox
            fr = yc0 + soy
            mf = (fc >= 0) & (fc < n) & (fr >= 0) & (fr < n)
            if mf.any():
                fcm = fc[mf].astype(np.int64)
                frm = fr[mf].astype(np.int64)
                flat = frm * n + fcm
                uniq, cnt = np.unique(flat, return_counts=True)
                ur, uc = np.divmod(uniq, n)
                occ = st[ur, uc] == STATE_OCC
                nb = np.minimum(self._bleach[ur, uc].astype(np.int32) + cnt, 255)
                self._bleach[ur, uc] = nb.astype(np.uint8)
                flip = occ & (nb >= BLEACH_N)
                st[ur, uc] = np.where(occ & ~flip, STATE_OCC, STATE_FREE).astype(np.uint8)

        # write the mutated state back into the permanent 2-bit store.
        self.cells = _pack_cells(st)
        self.rev += 1
        self._state_rev = self.rev       # _state already matches cells
        self._dt_rev = -1                # occupancy changed -> DT must rebuild
        return True

    # --- loop closure: rigid map transform ----------------------------------
    def transform(self, dx, dy, dth):
        """Rigidly shift/rotate the whole grid by (dx, dy, dth) in world metres/rad.
        Used by loop closure to bleed off accumulated global drift. Rare (loop events
        only), so the resample cost is irrelevant. Bleach counters reset (they
        re-learn in a few scans)."""
        c, s = math.cos(dth), math.sin(dth)
        st = self.state_view()
        ys, xs = np.mgrid[0:self.n, 0:self.n].astype(np.float64)
        wy = self.origin + (ys + 0.5) * self.res
        wx = self.origin + (xs + 0.5) * self.res
        sx = c * (wx - dx) + s * (wy - dy)
        sy = -s * (wx - dx) + c * (wy - dy)
        sc, sr = self.w2g(sx, sy)
        m = (sc >= 0) & (sc < self.n) & (sr >= 0) & (sr < self.n)
        sic = sc[m].astype(np.int64)
        sir = sr[m].astype(np.int64)
        tic = xs[m].astype(np.int64)
        tir = ys[m].astype(np.int64)
        new = st.copy()
        new[tir, tic] = st[sir, sic]
        self.cells = _pack_cells(new)
        self._state = new
        self._bleach[:] = 0
        self.rev += 1
        self._state_rev = self.rev
        self._dt_rev = -1

    # --- export --------------------------------------------------------------
    def occupancy_int8(self):
        """ROS-style occupancy: -1 unknown, 0 = FREE, 100 = OCCUPIED. Row 0 =
        origin_y (bottom). Free MUST export as 0 (or free-space dust renders as
        'walls'); 100 = solid black under the web 'sharp walls' posterize (>=70)."""
        st = self.state_view()
        out = np.full(st.shape, -1, dtype=np.int8)
        out[st == STATE_FREE] = 0
        out[st == STATE_OCC] = 100
        return out

    def coverage(self):
        """(seen_fraction, free_m2, occ_m2) — cheap mapping telemetry (two boolean
        sums over the grid)."""
        st = self.state_view()
        seen = st != STATE_UNKNOWN
        cell_a = self.res * self.res
        return (float(seen.sum()) / float(self.n * self.n),
                float((st == STATE_FREE).sum()) * cell_a,
                float((st == STATE_OCC).sum()) * cell_a)

    def occ_count(self):
        """Number of occupied (wall/obstacle) cells — a LOCAL structure measure that
        stays meaningful on a small-room map whose GLOBAL coverage fraction is tiny
        (recover_min_seen compares against the whole 24 m grid and a 1.5 m² room can
        never approach it). Zero structure = nothing to localize against."""
        return int((self.state_view() == STATE_OCC).sum())

    # --- no-go zones (human edits) -------------------------------------------
    def nogo_count(self):
        """Number of marked no-go cells (for map telemetry)."""
        return int(self.forbidden.sum())

    def _brush(self, c, r, radius, val):
        y0, y1 = max(0, r - radius), min(self.n - 1, r + radius)
        x0, x1 = max(0, c - radius), min(self.n - 1, c + radius)
        yy, xx = np.mgrid[y0:y1 + 1, x0:x1 + 1]
        disk = (xx - c) ** 2 + (yy - r) ** 2 <= radius * radius
        self.forbidden[yy[disk], xx[disk]] = val

    def apply_stroke(self, x0, y0, x1, y1, brush_cells, erase=False):
        c0, r0 = self.w2g(x0, y0)
        c1, r1 = self.w2g(x1, y1)
        dc, dr = float(c1 - c0), float(r1 - r0)
        steps = int(math.hypot(dc, dr) / 2) + 1
        val = False if erase else True
        for i in range(steps + 1):
            t = i / max(1, steps)
            c, r = int(round(c0 + dc * t)), int(round(r0 + dr * t))
            self._brush(c, r, brush_cells, val)
        self.forb_rev += 1
        return int(self.forbidden.sum())

    def apply_action(self, action):
        act = (action or {}).get("action")
        if act == "stroke":
            x0 = float(action.get("x0", 0.0)); y0 = float(action.get("y0", 0.0))
            x1 = float(action.get("x1", 0.0)); y1 = float(action.get("y1", 0.0))
            brush = max(1, int(action.get("brush", 3)))
            erase = bool(action.get("erase", False))
            self.apply_stroke(x0, y0, x1, y1, brush, erase)
        elif act == "clear":
            self.forbidden[:] = False
            self.forb_rev += 1
        return {"nogo": self.nogo_count()}

    # --- persistence ---------------------------------------------------------
    def save(self, path):
        """Persist the packed ternary grid + no-go mask + the map-frame anchors
        (rot_from + the seed odom/map-pose tuple, so a reload re-anchors the exact
        odom-frame the grid was drawn in). Atomic .tmp + rename."""
        tmp = path + ".tmp"
        np.savez_compressed(tmp, cells=self.cells, forb=self.forbidden,
                            n=np.int32(self.n), res=np.float32(self.res),
                            rot=np.float32(self.rot_from),
                            s0=np.float32(self.seed_odom_x), s1=np.float32(self.seed_odom_y),
                            s2=np.float32(self.seed_odom_t), s3=np.float32(self.seed_dx),
                            s4=np.float32(self.seed_dy), s5=np.float32(self.seed_pth))
        os.replace(tmp + ".npz" if not tmp.endswith(".npz") else tmp, path)

    def load(self, path):
        """Load a grid written by save(); also imports the legacy float32 log-odds
        format (log+seen) by collapsing it to ternary. Geometry mismatch -> False."""
        try:
            z = np.load(path, allow_pickle=False)
        except (OSError, ValueError, EOFError):
            return False
        try:
            if int(z["n"]) != self.n or abs(float(z["res"]) - self.res) > 1e-9:
                return False
            if "cells" in z:
                cells = np.ascontiguousarray(z["cells"], dtype=np.uint8)
                if cells.size * 4 < self.n * self.n:
                    return False
            elif "log" in z and "seen" in z:
                log = np.asarray(z["log"], dtype=np.float32)
                seen = np.asarray(z["seen"], dtype=bool)
                if log.shape != (self.n, self.n) or seen.shape != (self.n, self.n):
                    return False
                # legacy: seen & positive log-odds = occupied, seen & negative = free
                st = np.where(seen,
                              np.where(log > 0.0, STATE_OCC, STATE_FREE),
                              STATE_UNKNOWN).astype(np.uint8)
                cells = _pack_cells(st)
            else:
                return False
            forb = z["forb"] if "forb" in z else np.zeros((self.n, self.n), dtype=bool)
        except (KeyError, ValueError):
            return False
        self.cells = cells
        self.forbidden = np.ascontiguousarray(forb, dtype=bool)
        # Restore the map-frame anchors (older saves lack them -> all zero, meaning
        # "drawn in the odom frame with seed at odom (0,0,0)" — the old default; the
        # nav node re-anchors from the loaded layout regardless).
        self.rot_from = float(z["rot"]) if "rot" in z else 0.0
        self.seed_odom_x = float(z["s0"]) if "s0" in z else 0.0
        self.seed_odom_y = float(z["s1"]) if "s1" in z else 0.0
        self.seed_odom_t = float(z["s2"]) if "s2" in z else 0.0
        self.seed_dx = float(z["s3"]) if "s3" in z else 0.0
        self.seed_dy = float(z["s4"]) if "s4" in z else 0.0
        self.seed_pth = float(z["s5"]) if "s5" in z else 0.0
        self._state = None
        self._state_rev = -1
        self._dt = None
        self._dt_rev = -1
        self._bleach[:] = 0
        self.rev += 1
        self.forb_rev += 1
        return True

    # --- global planner (Stage 2) -------------------------------------------
    @staticmethod
    def _nearest_free(blocked, c, r, m, maxrad=6):
        if 0 <= r < m and 0 <= c < m and not blocked[r, c]:
            return c, r
        for rad in range(1, maxrad + 1):
            for dr in range(-rad, rad + 1):
                for dc in range(-rad, rad + 1):
                    rr, cc = r + dr, c + dc
                    if 0 <= rr < m and 0 <= cc < m and not blocked[rr, cc]:
                        return cc, rr
        return None, None

    @staticmethod
    def _simplify(path):
        if len(path) < 3:
            return path
        out = [path[0]]
        for i in range(1, len(path) - 1):
            ax, ay = path[i][0] - out[-1][0], path[i][1] - out[-1][1]
            bx, by = path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1]
            if abs(ax * by - ay * bx) > 1e-6:
                out.append(path[i])
        out.append(path[-1])
        return out

    def _coarse(self, downsample, radius_m, allow_unknown):
        """Downsampled obstacle grid shared by plan()/frontiers() (memoised by key
        incl. rev/forb_rev). Occupancy now reads the ternary state directly."""
        key = (downsample, radius_m, allow_unknown, self.rev, self.forb_rev)
        c = self._coarse_cache
        if c is not None and c[0] == key:
            return c[1]
        st = self.state_view()
        ds = max(1, int(downsample))
        m = self.n // ds
        res_c = self.res * ds
        k = m * ds
        occ_c = (st[:k, :k] == STATE_OCC).reshape(m, ds, m, ds).any(axis=(1, 3))
        seen_c = (st[:k, :k] != STATE_UNKNOWN).reshape(m, ds, m, ds).any(axis=(1, 3))
        forb_c = self.forbidden[:k, :k].reshape(m, ds, m, ds).any(axis=(1, 3))

        blocked = occ_c.copy()
        for _ in range(max(1, int(round(radius_m / res_c)))):
            b = blocked.copy()
            b[1:, :] |= blocked[:-1, :]; b[:-1, :] |= blocked[1:, :]
            b[:, 1:] |= blocked[:, :-1]; b[:, :-1] |= blocked[:, 1:]
            blocked = b
        if forb_c.any():
            forb = forb_c.copy()
            for _ in range(max(1, int(round(radius_m / res_c)))):
                b = forb.copy()
                b[1:, :] |= forb[:-1, :]; b[:-1, :] |= forb[1:, :]
                b[:, 1:] |= forb[:, :-1]; b[:, :-1] |= forb[:, 1:]
                forb = b
            blocked |= forb
        if not allow_unknown:
            blocked |= ~seen_c
        self._coarse_cache = (key, (blocked, seen_c, m, res_c))
        return blocked, seen_c, m, res_c

    def frontiers(self, start, radius_m=0.16, downsample=4, k=8):
        blocked, seen_c, m, res_c = self._coarse(downsample, radius_m, True)
        free = seen_c & ~blocked
        unknown = ~seen_c
        fr = np.zeros_like(free)
        fr[1:, :] |= free[1:, :] & unknown[:-1, :]
        fr[:-1, :] |= free[:-1, :] & unknown[1:, :]
        fr[:, 1:] |= free[:, 1:] & unknown[:, :-1]
        fr[:, :-1] |= free[:, :-1] & unknown[:, 1:]
        if not fr.any():
            return []
        sc = int(math.floor((start[0] - self.origin) / res_c))
        sr = int(math.floor((start[1] - self.origin) / res_c))
        rs, cs = np.nonzero(fr)
        order = np.argsort((rs - sr) ** 2 + (cs - sc) ** 2)[:max(1, int(k))]
        return [(self.origin + (cs[i] + 0.5) * res_c, self.origin + (rs[i] + 0.5) * res_c)
                for i in order]

    def plan(self, start, goal, radius_m=0.16, downsample=4, allow_unknown=True,
             max_iter=1000):
        blocked, seen_c, m, res_c = self._coarse(downsample, radius_m, allow_unknown)

        def w2c(x, y):
            return (int(math.floor((x - self.origin) / res_c)),
                    int(math.floor((y - self.origin) / res_c)))

        sc, sr = w2c(*start)
        gc, gr = w2c(*goal)
        if not (0 <= sc < m and 0 <= sr < m and 0 <= gc < m and 0 <= gr < m):
            return None
        gc0, gr0 = gc, gr
        gc, gr = self._nearest_free(blocked, gc, gr, m)
        sc, sr = self._nearest_free(blocked, sc, sr, m)
        if gc is None or sc is None:
            return None
        if (sc == gc and sr == gr) and (gc0 != sc or gr0 != sr):
            return None

        dc, dr = gc - sc, gr - sr
        for i in range(max(abs(dc), abs(dr)) + 1):
            cc = sc + round(dc * i / max(1, abs(dc)))
            rr = sr + round(dr * i / max(1, abs(dr)))
            if blocked[rr, cc]:
                break
        else:
            p0 = (self.origin + (sc + 0.5) * res_c, self.origin + (sr + 0.5) * res_c)
            p1 = (self.origin + (gc + 0.5) * res_c, self.origin + (gr + 0.5) * res_c)
            if math.hypot(p1[0] - p0[0], p1[1] - p0[1]) < res_c * 0.5:
                return [p1]
            return [p0, p1]

        BIG = np.float32(1e9)
        dist = np.full((m, m), BIG, dtype=np.float32)
        dist[gr, gc] = 0.0
        for _ in range(max_iter):
            nb = np.full((m, m), BIG, dtype=np.float32)
            nb[1:, :] = np.minimum(nb[1:, :], dist[:-1, :])
            nb[:-1, :] = np.minimum(nb[:-1, :], dist[1:, :])
            nb[:, 1:] = np.minimum(nb[:, 1:], dist[:, :-1])
            nb[:, :-1] = np.minimum(nb[:, :-1], dist[:, 1:])
            cand = nb + 1.0
            cand[blocked] = BIG
            cand[gr, gc] = 0.0
            newd = np.minimum(dist, cand)
            if np.array_equal(newd, dist):
                break
            dist = newd
        if dist[sr, sc] >= BIG:
            return None

        path, r, c, limit = [], sr, sc, m * m
        for _ in range(limit):
            path.append((self.origin + (c + 0.5) * res_c, self.origin + (r + 0.5) * res_c))
            if r == gr and c == gc:
                break
            best, nr, nc = dist[r, c], r, c
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                rr, cc = r + dr, c + dc
                if 0 <= rr < m and 0 <= cc < m and dist[rr, cc] < best:
                    best, nr, nc = dist[rr, cc], rr, cc
            if (nr, nc) == (r, c):
                break
            r, c = nr, nc
        return self._simplify(path)

    # --- dynamic tolerance helpers (velocity-scaled search / authority gates) ---
    def vel_scale(self, vlin, vang):
        """Velocity-dependent multiplier for match tolerance: expected wheel slip and
        odometry error grow with the distance/rotation between scans. 1.0 when
        parked; capped at 4x. Used by both the search window and the pose-trust gates
        (slow driving = tighter trust, fast driving = more forgiveness)."""
        s = 1.0 + 4.0 * max(0.0, float(vlin)) + 1.5 * abs(float(vang))
        return min(4.0, max(1.0, s))

    def search_window(self, vlin, vang, base_lin, base_ang):
        """Scale the match (lin, ang) half-windows up with wheel velocity (vel_scale):
        a faster-moving prior is less certain, so the matcher must be allowed to look
        further before the wheels win."""
        s = self.vel_scale(vlin, vang)
        return base_lin * s, base_ang * s