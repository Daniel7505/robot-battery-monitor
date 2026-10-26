"""Log-odds occupancy grid.

Each depth ray marks the cells up to the hit as free and, for a point in
the height band, the end cell as occupied. One cell is updated at most
once per frame so a long ray does not outvote a short one. The inverse
sensor model is the usual log-odds addend (Moravec and Elfes 1985;
Thrun, Burgard, Fox, Probabilistic Robotics, ch. 9). The grid grows
neither from the apartment size nor from a loaded map: it is a fixed
20 m square centred on the start pose, which is only a memory budget.
"""

from __future__ import annotations

import math

import numpy as np

from src.mapping.body import GRID_RESOLUTION_M, GRID_SIZE_M, ROBOT_RADIUS_M


class OccupancyGrid:
    def __init__(
        self,
        *,
        resolution_m: float = GRID_RESOLUTION_M,
        size_m: float = GRID_SIZE_M,
    ) -> None:
        self.res = float(resolution_m)
        self.n = int(round(float(size_m) / self.res))
        half = 0.5 * self.n * self.res
        self.origin = (-half, -half)
        self.log = np.zeros((self.n, self.n), dtype=np.float32)
        # A contact can reinforce a wall the camera already sees. It does
        # not invent one: a mark dropped ahead of a side-clip lands in the
        # doorway, and a permanent cell there closes the opening. Free
        # rays are allowed to wear a latch back down.
        self.latch = np.zeros((self.n, self.n), dtype=bool)
        self.l_occ = 0.85
        self.l_free = -0.35
        self.l_min = -3.5
        self.l_max = 3.5
        self.occ_thresh = 1.20
        self.free_thresh = -0.50

    def reset(self) -> None:
        """Drop every ray. A loop closure replays keyframes into an empty grid."""
        self.log.fill(0.0)
        self.latch.fill(False)

    def meta(self) -> dict:
        return {
            "resolution_m": self.res,
            "origin_xy_m": [self.origin[0], self.origin[1]],
            "width": self.n,
            "height": self.n,
            "frame": "odometry",
            "labels": {"unknown": -1, "free": 0, "occupied": 1},
            "axis": "row is +y, column is +x; cell (0,0) is the origin corner",
        }

    def labels(self) -> np.ndarray:
        out = np.full(self.log.shape, -1, dtype=np.int8)
        out[self.log <= self.free_thresh] = 0
        out[self.log >= self.occ_thresh] = 1
        return out

    def cell_of(self, x: float, y: float) -> tuple[int, int] | None:
        c = int(math.floor((float(x) - self.origin[0]) / self.res))
        r = int(math.floor((float(y) - self.origin[1]) / self.res))
        if r < 0 or c < 0 or r >= self.n or c >= self.n:
            return None
        return r, c

    def cell_center(self, row: int, col: int) -> tuple[float, float]:
        x = self.origin[0] + (int(col) + 0.5) * self.res
        y = self.origin[1] + (int(row) + 0.5) * self.res
        return x, y

    def stamp_footprint(self, x: float, y: float, radius_m: float = ROBOT_RADIUS_M * 0.75) -> None:
        """The robot is standing here, so this disk is free.

        A cell that is already a confident wall is left alone, otherwise
        brushing a door frame would erase it.
        """
        here = self.cell_of(x, y)
        if here is None:
            return
        r0, c0 = here
        rc = int(math.ceil(radius_m / self.res))
        r2 = radius_m * radius_m
        for dy in range(-rc, rc + 1):
            for dx in range(-rc, rc + 1):
                if (dy * dy + dx * dx) * self.res * self.res > r2:
                    continue
                r, c = r0 + dy, c0 + dx
                if r < 0 or c < 0 or r >= self.n or c >= self.n:
                    continue
                if self.latch[r, c] or self.log[r, c] >= 2.0:
                    continue
                self.log[r, c] = min(float(self.log[r, c]), self.free_thresh - 0.1)

    def mark_contact(self, x: float, y: float) -> None:
        """Reinforce a cell the depth camera has already called a wall.

        Unknown and free cells are left alone. The tire hits the jamb
        while the point straight ahead is the opening, and latching that
        point shut the bedroom door on the real runs.
        """
        cell = self.cell_of(x, y)
        if cell is None:
            return
        r, c = cell
        if float(self.log[r, c]) < self.occ_thresh:
            return
        self.log[r, c] = max(float(self.log[r, c]), 3.2)
        self.latch[r, c] = True

    def integrate(self, origin_xy: tuple[float, float], hits_xy: np.ndarray, floors_xy: np.ndarray) -> None:
        """Ray-cast free space to every endpoint. Hits also occupy the end cell.

        Free and occupied votes share one mask so a hit wins over a floor
        ray that crossed the same cell in this frame.
        """
        mark = np.zeros(self.n * self.n, dtype=np.int8)
        self._paint(origin_xy, floors_xy, occupied=False, mark=mark)
        self._paint(origin_xy, hits_xy, occupied=True, mark=mark)
        log = self.log.ravel()
        # A latched contact decays under a free ray. A real wall is
        # repainted by the hit, which outvotes the free mark in this frame.
        log[mark == 1] += self.l_free
        log[mark == 2] += self.l_occ
        np.clip(log, self.l_min, self.l_max, out=log)
        self.latch[self.log < self.occ_thresh] = False

    def _paint(
        self,
        origin_xy: tuple[float, float],
        ends_xy: np.ndarray,
        *,
        occupied: bool,
        mark: np.ndarray,
    ) -> None:
        ends = np.asarray(ends_xy, dtype=np.float64).reshape(-1, 2)
        if ends.shape[0] == 0:
            return
        for start in range(0, ends.shape[0], 400):
            self._paint_chunk(origin_xy, ends[start : start + 400], occupied, mark)

    def _paint_chunk(
        self,
        origin_xy: tuple[float, float],
        ends: np.ndarray,
        occupied: bool,
        mark: np.ndarray,
    ) -> None:
        ox, oy = float(origin_xy[0]), float(origin_xy[1])
        delta = ends - np.array([ox, oy], dtype=np.float64)
        dist = np.hypot(delta[:, 0], delta[:, 1])
        keep = dist > 1e-3
        if not np.any(keep):
            return
        delta = delta[keep]
        dist = dist[keep]
        n = int(np.ceil(float(dist.max()) / self.res)) + 1
        n = max(2, min(n, 480))
        t = np.linspace(0.0, 1.0, n, dtype=np.float64)
        xs = ox + delta[:, 0:1] * t
        ys = oy + delta[:, 1:2] * t
        cols = np.floor((xs - self.origin[0]) / self.res).astype(np.int32)
        rows = np.floor((ys - self.origin[1]) / self.res).astype(np.int32)
        inside = (rows >= 0) & (rows < self.n) & (cols >= 0) & (cols < self.n)

        def splat(rr: np.ndarray, cc: np.ndarray, valid: np.ndarray, value: int) -> None:
            mask = valid.ravel()
            if not np.any(mask):
                return
            lin = rr.ravel()[mask] * self.n + cc.ravel()[mask]
            mark[lin] = np.maximum(mark[lin], value)

        if occupied:
            splat(rows[:, :-1], cols[:, :-1], inside[:, :-1], 1)
            splat(rows[:, -1:], cols[:, -1:], inside[:, -1:], 2)
        else:
            splat(rows, cols, inside, 1)
