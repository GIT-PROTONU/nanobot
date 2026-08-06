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
        # No-go mask: cells the PLANNER must never route through / treat as an obstacle.
        # Unlike log-odds it is NOT touched by scan integration (a lidar beam through it
        # won't slowly "free" it back), so a human-marked restricted zone stays put. It's
        # persistence, exposed to the web UI as an overlay, and folded into _coarse().
        self.forbidden = np.zeros((self.n, self.n), dtype=bool)

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

    def relocalize(self, angles, ranges, step=4, n_headings=16, npts=90, keep=6):
        """Global scan-to-map search (kidnap recovery). Scores a decimated scan against
        EVERY grid candidate at `step`-cell spacing across `n_headings` yaw steps, then
        coarse-to-fine refines the top `keep` candidates with the local matcher and keeps
        the best-scoring one. Keeping several hypotheses matters: at a slightly-wrong
        coarse heading even the true location scores poorly, so the true pose can rank
        below a spurious rotated one — refining the top K rescues it. Unlike match(),
        which only hunts around the current pose, this can snap back after the robot is
        carried far away (or boots inside an already-loaded map). Returns
        (x, y, theta, score) or None when the map has little structure and no pose scored
        above the free-space floor. Compatible with the caller's min_score / exit check."""
        if len(ranges) > npts:                       # decimate: keeps the big lookup small
            idx = np.linspace(0, len(ranges) - 1, npts).astype(int)
            angles, ranges = angles[idx], ranges[idx]
        xs = self.origin + (np.arange(0, self.n, step) + 0.5) * self.res
        ys = self.origin + (np.arange(0, self.n, step) + 0.5) * self.res
        ths = np.linspace(-np.pi, np.pi, n_headings, endpoint=False)

        cands = []                                   # (coarse_score, th, x, y)
        for th in ths:                               # per-heading (Nx, Ny, npts) broadcast
            a = angles + th
            hx = ranges * np.cos(a)                  # hit offsets for this heading
            hy = ranges * np.sin(a)
            cx = np.floor((xs[:, None] + hx[None, :] - self.origin) / self.res).astype(np.int32)
            ry = np.floor((ys[:, None] + hy[None, :] - self.origin) / self.res).astype(np.int32)
            inx = (cx >= 0) & (cx < self.n)
            iny = (ry >= 0) & (ry < self.n)
            cxc = np.clip(cx, 0, self.n - 1)
            ryc = np.clip(ry, 0, self.n - 1)
            vals = self.log[ryc[None, :, :], cxc[:, None, :]]     # (Nx, Ny, npts)
            mask = inx[:, None, :] & iny[None, :, :]
            s = np.where(mask, vals, 0.0).sum(axis=2)             # (Nx, Ny)
            # keep the top-2 (x, y) per heading too, so a spurious peak at the true
            # heading doesn't mask the right location for this heading. (Guard the
            # argpartition kth bound: it must stay < size, else tiny maps crash.)
            k2 = min(2, s.size)
            if k2 >= 1:
                for fi in np.argpartition(s.ravel(), -k2)[-k2:]:
                    i, j = np.unravel_index(int(fi), s.shape)
                    cands.append((float(s[i, j]), float(th), float(xs[i]), float(ys[j])))
        cands.sort(key=lambda c: -c[0])

        best = None
        for _, th, x, y in cands[:keep]:
            # 3 refine passes: a 2-pass climb can trap in a neighbouring basin when the
            # coarse heading is a few degrees off, leaving the true peak unreached.
            cand = self.match((x, y, th), angles, ranges,
                              lin=2 * step * self.res, ang=0.35, half=4, refine=3)
            sc = self.score(cand, angles, ranges)
            if best is None or sc > best[3]:
                best = (cand[0], cand[1], cand[2], sc)
        if best is None or best[3] <= 0.0:
            # Free-space cells carry negative log-odds, so a pose that actually hits
            # occupied structure scores positive; anything at/below zero found no match.
            return None
        return best

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

    # --- export --------------------------------------------------------------
    def occupancy_int8(self):
        """ROS-style occupancy: -1 unknown, 0 free .. 100 occupied. Row 0 = origin_y
        (bottom). Returned row-major as int8, ready to dump to the web map file. Only
        cells that have been seen get a probability (exp over the ~mostly-unmapped grid
        dominates the map-write cost early on); unseen cells are -1 directly."""
        out = np.full(self.log.shape, -1, dtype=np.int8)
        s = self.seen
        if s.any():
            p = 1.0 - 1.0 / (1.0 + np.exp(self.log[s]))      # P(occupied) on seen cells
            out[s] = (p * 100.0).astype(np.int8)
        return out

    def coverage(self):
        """(seen_fraction, free_m2, occ_m2) — cheap mapping telemetry (two boolean sums
        over the grid). Cheap enough to call at the map-write rate."""
        seen = int(self.seen.sum())
        free = int(((self.log < 0.0) & self.seen).sum())
        occ = int(((self.log > 0.0) & self.seen).sum())
        cell_a = self.res * self.res
        return seen / float(self.n * self.n), free * cell_a, occ * cell_a

    # --- no-go zones (human edits) -------------------------------------------
    def nogo_count(self):
        """Number of marked no-go cells (for map telemetry)."""
        return int(self.forbidden.sum())

    def _brush(self, c, r, radius, val):
        """Mark the disk of coarse radius `radius` (cells) around (c, r) as val (on/off).
        Vectorised over the disk bounding box so a web stroke costs ~nothing."""
        y0, y1 = max(0, r - radius), min(self.n - 1, r + radius)
        x0, x1 = max(0, c - radius), min(self.n - 1, c + radius)
        yy, xx = np.mgrid[y0:y1 + 1, x0:x1 + 1]
        disk = (xx - c) ** 2 + (yy - r) ** 2 <= radius * radius
        self.forbidden[yy[disk], xx[disk]] = val

    def apply_stroke(self, x0, y0, x1, y1, brush_cells, erase=False):
        """Paint a no-go (or erase) stroke from world (x0,y0) to (x1,y1) with a brush of
        `brush_cells` radius. Walks the line at the grid pitch so a fast drag has no gaps.
        Returns the number of cells changed."""
        c0, r0 = self.w2g(x0, y0)
        c1, r1 = self.w2g(x1, y1)
        dc, dr = float(c1 - c0), float(r1 - r0)
        steps = int(math.hypot(dc, dr) / 2) + 1
        val = False if erase else True              # erase => clear forbidden
        for i in range(steps + 1):
            t = i / max(1, steps)
            c, r = int(round(c0 + dc * t)), int(round(r0 + dr * t))
            self._brush(c, r, brush_cells, val)
        return int(self.forbidden.sum())

    def apply_action(self, action):
        """Apply a single human edit dict (painted by the web map editor). Actions:
          {"action":"stroke","x0":y?,"y0","x1","y1","brush":n,"erase":bool}
          {"action":"clear"}               -> wipe ALL no-go zones
        Any missing/first key -> no-op returning {"nogo": count}. Returns a status dict."""
        act = (action or {}).get("action")
        if act == "stroke":
            x0 = float(action.get("x0", 0.0)); y0 = float(action.get("y0", 0.0))
            x1 = float(action.get("x1", 0.0)); y1 = float(action.get("y1", 0.0))
            brush = max(1, int(action.get("brush", 3)))
            erase = bool(action.get("erase", False))
            self.apply_stroke(x0, y0, x1, y1, brush, erase)
        elif act == "clear":
            self.forbidden[:] = False
        return {"nogo": self.nogo_count()}

    # --- persistence ---------------------------------------------------------
    def save(self, path):
        """Persist the grid (log-odds + seen) compressed. A mostly-empty floor map is a
        few tens of KB — the uniform regions zlib-compress hard. Atomic via a .tmp + rename
        so a reader (or a crash mid-write) never sees a torn file."""
        tmp = path + ".tmp"
        np.savez_compressed(tmp, log=self.log, seen=self.seen,
                            forb=self.forbidden,
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
            # Older maps won't have the forb key — default to an empty mask rather
            # than failing to load (no-go zones are opt-in edits).
            forb = z["forb"] if "forb" in z else np.zeros(self.seen.shape, dtype=bool)
            self.forbidden = np.ascontiguousarray(forb, dtype=bool)
        except (KeyError, ValueError):
            return False
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
        forb_c = self.forbidden[:k, :k].reshape(m, ds, m, ds).any(axis=(1, 3))

        # inflate obstacles by the robot radius (L1 / diamond dilation, a few passes)
        blocked = occ_c.copy()
        for _ in range(max(1, int(round(radius_m / res_c)))):
            b = blocked.copy()
            b[1:, :] |= blocked[:-1, :]; b[:-1, :] |= blocked[1:, :]
            b[:, 1:] |= blocked[:, :-1]; b[:, :-1] |= blocked[:, 1:]
            blocked = b
        # no-go zones: NEVER navigable, regardless of allow_unknown (they only cover a few
        # cells each, so they get the same robot-radius inflation as obstacles). If a
        # forbidden cell is reachable it's plannable around; if it walls a corridor the
        # planner has to route around it like any obstacle.
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
        sc = int(math.floor((start[0] - self.origin) / res_c))
        sr = int(math.floor((start[1] - self.origin) / res_c))
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
            # floor() (not int()) so negative coordinates map like w2g — int() truncates
            # toward zero and would snap a point just below the origin onto cell 0.
            return (int(math.floor((x - self.origin) / res_c)),
                    int(math.floor((y - self.origin) / res_c)))

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
