"""Tiny 2D occupancy-grid SLAM core — pure numpy, no ROS deps (stays cheap + testable).

Holds a log-odds occupancy grid, integrates LaserScan hits with an inverse sensor
model, and refines the robot pose with a *correlative scan-to-map matcher*. Matching
against the accumulated MAP (not the previous scan) is the lightweight stand-in for
loop closure: when you re-enter an already-mapped area the match snaps the pose back
onto it, which is what keeps a whole-floor map from drifting without a heavy pose graph.

Memory: one float32 grid + one bool 'seen' mask. At 24 m / 5 cm that's 480x480 =
~0.9 MB + 0.23 MB. CPU: integration is O(hit cells); matching is a small coarse-to-fine
search over a subsampled scan (caller decimates), vectorised per candidate angle.
"""
import math
import os

import numpy as np

# Inverse-sensor-model log-odds increments, and the clamp that bounds how "certain"
# a cell can get — clamping keeps the map responsive to a moved chair / opened door.
L_FREE = 0.40
L_OCC = 0.85
L_CLAMP = 4.0


class GridMap:
    def __init__(self, size_m=24.0, res=0.05, rmin=0.12, rmax=6.0):
        self.res = float(res)
        self.n = int(round(size_m / self.res))          # square n x n grid
        self.rmin, self.rmax = float(rmin), float(rmax)
        # World coordinate of cell [0,0] (lower-left). The robot starts at the centre,
        # so the map can grow outward in every direction from the origin.
        self.origin = -0.5 * self.n * self.res
        self.log = np.zeros((self.n, self.n), dtype=np.float32)   # [row=y, col=x]
        self.seen = np.zeros((self.n, self.n), dtype=bool)
        # Bumped on every content mutation (integrate/load) so callers can cache
        # derived exports (occupancy_int8/coverage) instead of recomputing a full-grid
        # np.exp at the map-write rate while nothing is changing.
        self.rev = 0

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

    # --- scan-to-map matching ------------------------------------------------
    def score(self, pose, angles, ranges):
        """Sum of map log-odds at the scan's hit cells (higher = better aligned)."""
        px, py, pth = pose
        a = angles + pth
        c, r = self.w2g(px + ranges * np.cos(a), py + ranges * np.sin(a))
        m = self._inb(c, r)
        if not m.any():
            return -1e18
        return float(self.log[r[m], c[m]].sum())

    def match(self, prior, angles, ranges, lin=0.10, ang=0.12, half=4, refine=2):
        """Correlative scan-to-map match: coarse-to-fine search around `prior` for the
        (x, y, theta) that maximises score. `half` = candidates each side per axis;
        `refine` shrinks the window and re-centres. Caller passes a decimated scan."""
        bx, by, bth = prior
        for it in range(refine):
            scale = 0.35 ** it                       # shrink the window each pass
            xs = bx + np.linspace(-lin * scale, lin * scale, 2 * half + 1)
            ys = by + np.linspace(-lin * scale, lin * scale, 2 * half + 1)
            ths = bth + np.linspace(-ang * scale, ang * scale, 2 * half + 1)
            best_s, best = -1e18, (bx, by, bth)
            for th in ths:
                a = angles + th
                hx = ranges * np.cos(a)              # hit offsets for this heading
                hy = ranges * np.sin(a)
                # cell cols depend only on x, rows only on y -> compute each 2D then
                # broadcast to (Nx, Ny, npts) for a single fancy-indexed lookup.
                cx = np.floor((xs[:, None] + hx[None, :] - self.origin) / self.res).astype(np.int32)
                ry = np.floor((ys[:, None] + hy[None, :] - self.origin) / self.res).astype(np.int32)
                inx = (cx >= 0) & (cx < self.n)
                iny = (ry >= 0) & (ry < self.n)
                cxc = np.clip(cx, 0, self.n - 1)
                ryc = np.clip(ry, 0, self.n - 1)
                vals = self.log[ryc[None, :, :], cxc[:, None, :]]      # (Nx, Ny, npts)
                mask = inx[:, None, :] & iny[None, :, :]
                s = np.where(mask, vals, 0.0).sum(axis=2)              # (Nx, Ny)
                i, j = np.unravel_index(int(np.argmax(s)), s.shape)
                if s[i, j] > best_s:
                    best_s, best = float(s[i, j]), (float(xs[i]), float(ys[j]), float(th))
            bx, by, bth = best
        return bx, by, bth

    # --- map update ----------------------------------------------------------
    def integrate(self, pose, angles, ranges):
        """Ray-cast every valid beam: decrement free cells along it, bump the endpoint."""
        px, py, pth = pose
        v = self._valid(ranges, self.rmin, self.rmax)
        a = (angles + pth)[v]
        rr = ranges[v]
        if rr.size == 0:
            return
        cos, sin = np.cos(a), np.sin(a)

        # occupied endpoints (np.add.at handles repeated cells correctly)
        ec, er = self.w2g(px + rr * cos, py + rr * sin)
        m = self._inb(ec, er)
        np.add.at(self.log, (er[m], ec[m]), L_OCC)
        self.seen[er[m], ec[m]] = True

        # free space: sample each ray at the grid pitch from the robot up to one cell shy of
        # the hit. Vectorised over ALL beams at once — no per-beam Python loop. Each beam
        # contributes `per[b]` samples; we build the ragged step indices (0..per-1 per beam)
        # with a repeat/cumsum trick, so we only ever allocate exactly the kept samples.
        per = np.maximum(0, (rr / self.res).astype(np.int32) - 1)   # samples per beam
        total = int(per.sum())
        if total:
            bi = np.repeat(np.arange(rr.size), per)                 # beam index per sample
            si = np.arange(total) - np.repeat(np.cumsum(per) - per, per)   # 0..per[b]-1
            tt = si * self.res                                      # distance along the ray (f64)
            fc, fr = self.w2g(px + tt * cos[bi], py + tt * sin[bi])
            mf = self._inb(fc, fr)
            np.add.at(self.log, (fr[mf], fc[mf]), -L_FREE)
            self.seen[fr[mf], fc[mf]] = True

        np.clip(self.log, -L_CLAMP, L_CLAMP, out=self.log)
        self.rev += 1

    # --- loop closure: rigid map transform ----------------------------------
    def transform(self, dx, dy, dth):
        """Rigidly shift/rotate the whole grid by (dx, dy, dth) in world metres/rad.
        Used by loop closure to bleed off accumulated global drift: the correction
        found against a far-away re-visited area is applied as a small rotation +
        translation of the accumulated map. Pure-numpy, no scipy. Only called on a
        loop event (rare), so the temporary copy cost (~1.1 MB) is irrelevant."""
        c, s = math.cos(dth), math.sin(dth)
        # world centre of every cell (row r, col c) -> source world point before the
        # transform, then sample the OLD grids there. Inverse of the forward motion:
        # a point that ends up at (wx, wy) came from (R^{-1} ((wx-dx, wy-dy))).
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
        new_log = self.log.copy()
        new_seen = self.seen.copy()
        new_log[tir, tic] = self.log[sir, sic]
        new_seen[tir, tic] = self.seen[sir, sic]
        self.log, self.seen = new_log, new_seen
        self.rev += 1

    # --- export --------------------------------------------------------------
    def occupancy_int8(self):
        """ROS-style occupancy: -1 unknown, 0 free .. 100 occupied. Row 0 = origin_y
        (bottom). Returned row-major as int8, ready to dump to the web map file."""
        p = 1.0 - 1.0 / (1.0 + np.exp(self.log))      # P(occupied)
        out = (p * 100.0).astype(np.int8)
        out[~self.seen] = -1
        return out

    def coverage(self):
        """(seen_fraction, free_m2, occ_m2) — cheap mapping telemetry (two boolean sums
        over the grid). Cheap enough to call at the map-write rate."""
        seen = int(self.seen.sum())
        free = int(((self.log < 0.0) & self.seen).sum())
        occ = int(((self.log > 0.0) & self.seen).sum())
        cell_a = self.res * self.res
        return seen / float(self.n * self.n), free * cell_a, occ * cell_a

    # --- persistence ---------------------------------------------------------
    def save(self, path):
        """Persist the grid (log-odds + seen) compressed. A mostly-empty floor map is a
        few tens of KB — the uniform regions zlib-compress hard. Atomic via a .tmp + rename
        so a reader (or a crash mid-write) never sees a torn file."""
        tmp = path + ".tmp"
        np.savez_compressed(tmp, log=self.log, seen=self.seen,
                            n=np.int32(self.n), res=np.float32(self.res))
        # np.savez appends .npz to a str path; normalise then rename onto the target.
        os.replace(tmp + ".npz" if not tmp.endswith(".npz") else tmp, path)

    def load(self, path):
        """Load a grid written by save(). Returns True on success; False if the file is
        missing/corrupt or its geometry (size/res) doesn't match this map (never load a
        mismatched grid — the indices wouldn't line up)."""
        try:
            z = np.load(path, allow_pickle=False)
        except (OSError, ValueError, EOFError):
            return False
        try:
            if int(z["n"]) != self.n or abs(float(z["res"]) - self.res) > 1e-9:
                return False
            self.log = np.ascontiguousarray(z["log"], dtype=np.float32)
            self.seen = np.ascontiguousarray(z["seen"], dtype=bool)
        except (KeyError, ValueError):
            return False
        self.rev += 1
        return True

    # --- global planner (Stage 2) -------------------------------------------
    OBST_L = 0.62        # log-odds threshold counted as an obstacle (~P>0.65)

    @staticmethod
    def _nearest_free(blocked, c, r, m, maxrad=6):
        """Nearest non-blocked coarse cell to (c, r) in a small spiral (cols, rows)."""
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
        """Drop collinear waypoints so the follower gets corners, not every cell."""
        if len(path) < 3:
            return path
        out = [path[0]]
        for i in range(1, len(path) - 1):
            ax, ay = path[i][0] - out[-1][0], path[i][1] - out[-1][1]
            bx, by = path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1]
            if abs(ax * by - ay * bx) > 1e-6:      # turn here -> keep it
                out.append(path[i])
        out.append(path[-1])
        return out

    def _coarse(self, downsample, radius_m, allow_unknown):
        """Build the downsampled obstacle grid shared by plan() and frontiers(): coarse
        occupied/seen masks + a robot-radius-inflated `blocked` mask. Returns
        (blocked, seen_c, m, res_c)."""
        ds = max(1, int(downsample))
        m = self.n // ds
        res_c = self.res * ds
        k = m * ds
        occ_c = (self.log[:k, :k] > self.OBST_L).reshape(m, ds, m, ds).any(axis=(1, 3))
        seen_c = self.seen[:k, :k].reshape(m, ds, m, ds).any(axis=(1, 3))

        # inflate obstacles by the robot radius (L1 / diamond dilation, a few passes)
        blocked = occ_c.copy()
        for _ in range(max(1, int(round(radius_m / res_c)))):
            b = blocked.copy()
            b[1:, :] |= blocked[:-1, :]; b[:-1, :] |= blocked[1:, :]
            b[:, 1:] |= blocked[:, :-1]; b[:, :-1] |= blocked[:, 1:]
            blocked = b
        if not allow_unknown:
            blocked |= ~seen_c
        return blocked, seen_c, m, res_c

    def frontiers(self, start, radius_m=0.16, downsample=4, k=8):
        """Nearest-first list of up to `k` *frontier* points (world m): free coarse cells
        that border still-unknown space — the classic autonomous-exploration target ("go
        map the edge of what you know"). Vectorised on the same coarse grid as the planner,
        so it's a handful of boolean ops on the 120x120 grid. Caller plans to the first
        reachable one. Returns [] when the map is fully explored / no frontier exists."""
        blocked, seen_c, m, res_c = self._coarse(downsample, radius_m, True)
        free = seen_c & ~blocked
        unknown = ~seen_c
        fr = np.zeros_like(free)                      # free cell 4-adjacent to unknown
        fr[1:, :]  |= free[1:, :]  & unknown[:-1, :]
        fr[:-1, :] |= free[:-1, :] & unknown[1:, :]
        fr[:, 1:]  |= free[:, 1:]  & unknown[:, :-1]
        fr[:, :-1] |= free[:, :-1] & unknown[:, 1:]
        if not fr.any():
            return []
        sc = int((start[0] - self.origin) / res_c)
        sr = int((start[1] - self.origin) / res_c)
        rs, cs = np.nonzero(fr)
        order = np.argsort((rs - sr) ** 2 + (cs - sc) ** 2)[:max(1, int(k))]
        return [(self.origin + (cs[i] + 0.5) * res_c, self.origin + (rs[i] + 0.5) * res_c)
                for i in order]

    def plan(self, start, goal, radius_m=0.16, downsample=4, allow_unknown=True,
             max_iter=1000):
        """Plan a path from `start` to `goal` (world m) over a *downsampled* copy of the
        grid (keeps CPU/RAM tiny: 24 m @ 5 cm / ds=4 -> 120x120 cells). Obstacles are
        inflated by the robot radius; a vectorised wavefront from the goal gives a
        distance field, then we descend it from the start. Returns world waypoints or
        None if unreachable. Cheap enough to re-run ~1 Hz."""
        blocked, seen_c, m, res_c = self._coarse(downsample, radius_m, allow_unknown)

        def w2c(x, y):
            return (int((x - self.origin) / res_c), int((y - self.origin) / res_c))

        sc, sr = w2c(*start)
        gc, gr = w2c(*goal)
        if not (0 <= sc < m and 0 <= sr < m and 0 <= gc < m and 0 <= gr < m):
            return None
        gc, gr = self._nearest_free(blocked, gc, gr, m)     # snap goal off any wall
        sc, sr = self._nearest_free(blocked, sc, sr, m)     # snap start out of inflation
        if gc is None or sc is None:
            return None

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
            if np.array_equal(newd, dist):       # wavefront filled all reachable cells
                break
            dist = newd
        if dist[sr, sc] >= BIG:
            return None                          # goal not reachable from start

        # descend the distance field start -> goal (greedy 4-neighbour steepest)
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
