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
        cos, sin = np.cos(a), np.sin(a)

        # occupied endpoints (np.add.at handles repeated cells correctly)
        ec, er = self.w2g(px + rr * cos, py + rr * sin)
        m = self._inb(ec, er)
        np.add.at(self.log, (er[m], ec[m]), L_OCC)
        self.seen[er[m], ec[m]] = True

        # free space: sample each ray at the grid pitch up to just shy of the hit
        fx, fy = [], []
        for k in range(len(rr)):
            steps = int(rr[k] / self.res)
            if steps <= 1:
                continue
            t = np.arange(0, steps - 1) * self.res    # stop one cell before the hit
            fx.append(px + t * cos[k])
            fy.append(py + t * sin[k])
        if fx:
            fc, fr = self.w2g(np.concatenate(fx), np.concatenate(fy))
            mf = self._inb(fc, fr)
            np.add.at(self.log, (fr[mf], fc[mf]), -L_FREE)
            self.seen[fr[mf], fc[mf]] = True

        np.clip(self.log, -L_CLAMP, L_CLAMP, out=self.log)

    # --- export --------------------------------------------------------------
    def occupancy_int8(self):
        """ROS-style occupancy: -1 unknown, 0 free .. 100 occupied. Row 0 = origin_y
        (bottom). Returned row-major as int8, ready to dump to the web map file."""
        p = 1.0 - 1.0 / (1.0 + np.exp(self.log))      # P(occupied)
        out = (p * 100.0).astype(np.int8)
        out[~self.seen] = -1
        return out
