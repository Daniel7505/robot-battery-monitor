"""Frontier selection.

Yamauchi 1997: a frontier cell is free space next to unknown space.
Clusters smaller than ``MIN_FRONTIER_M`` are ignored. The rest are
scored by size divided by the A* path length. The path is planned on
the free grid with occupied cells inflated by the robot radius, so the
robot does not try to squeeze through a door it cannot fit.
"""

from __future__ import annotations

import heapq
import math

import numpy as np

from src.mapping.body import MIN_FRONTIER_M, ROBOT_RADIUS_M
from src.mapping.grid import OccupancyGrid

_NEIGHBORS = (
    (1, 0, 1.0),
    (-1, 0, 1.0),
    (0, 1, 1.0),
    (0, -1, 1.0),
    (1, 1, math.sqrt(2.0)),
    (1, -1, math.sqrt(2.0)),
    (-1, 1, math.sqrt(2.0)),
    (-1, -1, math.sqrt(2.0)),
)


def dilate_disk(mask: np.ndarray, radius_m: float, res: float) -> np.ndarray:
    out = np.zeros_like(mask, dtype=bool)
    if radius_m <= 0.0 or not np.any(mask):
        return mask.astype(bool, copy=True)
    rc = int(math.ceil(radius_m / res))
    limit = (radius_m / res) ** 2 + 1e-9
    h, w = mask.shape
    for dy in range(-rc, rc + 1):
        for dx in range(-rc, rc + 1):
            if dy * dy + dx * dx > limit:
                continue
            y_src0 = max(0, -dy)
            y_src1 = min(h, h - dy)
            x_src0 = max(0, -dx)
            x_src1 = min(w, w - dx)
            if y_src1 <= y_src0 or x_src1 <= x_src0:
                continue
            y_dst0 = max(0, dy)
            x_dst0 = max(0, dx)
            y_dst1 = y_dst0 + (y_src1 - y_src0)
            x_dst1 = x_dst0 + (x_src1 - x_src0)
            out[y_dst0:y_dst1, x_dst0:x_dst1] |= mask[y_src0:y_src1, x_src0:x_src1]
    return out


def frontier_mask(labels: np.ndarray) -> np.ndarray:
    free = labels == 0
    unknown = labels < 0
    neighbor = np.zeros_like(free)
    neighbor[1:, :] |= unknown[:-1, :]
    neighbor[:-1, :] |= unknown[1:, :]
    neighbor[:, 1:] |= unknown[:, :-1]
    neighbor[:, :-1] |= unknown[:, 1:]
    return free & neighbor


def clusters(mask: np.ndarray) -> list[list[tuple[int, int]]]:
    ys, xs = np.nonzero(mask)
    if len(ys) == 0:
        return []
    index = {(int(y), int(x)): True for y, x in zip(ys, xs)}
    seen: set[tuple[int, int]] = set()
    found: list[list[tuple[int, int]]] = []
    for start in index:
        if start in seen:
            continue
        stack = [start]
        seen.add(start)
        comp: list[tuple[int, int]] = []
        while stack:
            y, x = stack.pop()
            comp.append((y, x))
            # Bridge a one-cell gap so depth noise does not chop a wall
            # into specks smaller than the 0.3 m cutoff.
            for dy in range(-2, 3):
                for dx in range(-2, 3):
                    if dy == 0 and dx == 0:
                        continue
                    nxt = (y + dy, x + dx)
                    if nxt in index and nxt not in seen:
                        seen.add(nxt)
                        stack.append(nxt)
        found.append(comp)
    return found


def viewpoint(free: np.ndarray, cells: list[tuple[int, int]], reach: int = 6) -> tuple[int, int] | None:
    """A traversable cell on or just inside a frontier.

    The frontier cell itself often sits inside the inflated obstacle, so
    the robot cannot path to it. Standing a few cells back still sees it.
    """
    h, w = free.shape
    sample = cells
    if len(cells) > 30:
        step = max(1, len(cells) // 30)
        sample = cells[::step]
    best: tuple[int, int] | None = None
    best_d = 10**9
    limit = reach * reach
    for y, x in sample:
        y0 = max(0, y - reach)
        y1 = min(h, y + reach + 1)
        x0 = max(0, x - reach)
        x1 = min(w, x + reach + 1)
        window = free[y0:y1, x0:x1]
        if not np.any(window):
            continue
        ys, xs = np.nonzero(window)
        d2 = (ys + y0 - y) ** 2 + (xs + x0 - x) ** 2
        keep = d2 <= limit
        if not np.any(keep):
            continue
        i = int(np.argmin(np.where(keep, d2, 10**9)))
        dist = int(d2[i])
        if dist < best_d:
            best_d = dist
            best = (int(ys[i] + y0), int(xs[i] + x0))
    return best


def traversable(labels: np.ndarray, res: float, radius_m: float = ROBOT_RADIUS_M) -> np.ndarray:
    blocked = dilate_disk(labels == 1, radius_m, res)
    return (labels == 0) & ~blocked


def astar(
    free: np.ndarray,
    start: tuple[int, int],
    goal: tuple[int, int],
    *,
    step_cost: float,
) -> list[tuple[int, int]] | None:
    """8-connected A* on a boolean free grid. ``start`` may sit just off the grid."""
    h, w = free.shape
    sy, sx = start
    gy, gx = goal
    if not (0 <= gy < h and 0 <= gx < w):
        return None
    if not free[gy, gx]:
        return None

    def heuristic(y: int, x: int) -> float:
        return math.hypot(y - gy, x - gx) * step_cost

    open_heap: list[tuple[float, float, int, int]] = []
    heapq.heappush(open_heap, (heuristic(sy, sx), 0.0, sy, sx))
    came: dict[tuple[int, int], tuple[int, int]] = {}
    best = {start: 0.0}
    seen = 0
    while open_heap:
        _f, g, y, x = heapq.heappop(open_heap)
        if g > best.get((y, x), 1e18) + 1e-9:
            continue
        if (y, x) == (gy, gx):
            path = [(y, x)]
            while (y, x) in came:
                y, x = came[(y, x)]
                path.append((y, x))
            path.reverse()
            return path
        seen += 1
        if seen > 25000:
            return None
        for dy, dx, mul in _NEIGHBORS:
            ny, nx = y + dy, x + dx
            if ny < 0 or nx < 0 or ny >= h or nx >= w:
                continue
            if (ny, nx) != start and not free[ny, nx]:
                continue
            if dy != 0 and dx != 0:
                if not free[y + dy, x] or not free[y, x + dx]:
                    continue
            ng = g + mul * step_cost
            key = (ny, nx)
            if ng + 1e-9 < best.get(key, 1e18):
                best[key] = ng
                came[key] = (y, x)
                heapq.heappush(open_heap, (ng + heuristic(ny, nx), ng, ny, nx))
    return None


def _nearest_free(free: np.ndarray, row: int, col: int, max_cells: int = 40) -> tuple[int, int] | None:
    h, w = free.shape
    if 0 <= row < h and 0 <= col < w and free[row, col]:
        return row, col
    ys, xs = np.nonzero(free)
    if len(ys) == 0:
        return None
    d2 = (ys - row) ** 2 + (xs - col) ** 2
    i = int(np.argmin(d2))
    if d2[i] > max_cells * max_cells:
        return None
    return int(ys[i]), int(xs[i])


def select_goal(
    grid: OccupancyGrid,
    pose_xy: tuple[float, float],
    *,
    radius_m: float = ROBOT_RADIUS_M,
    min_frontier_m: float = MIN_FRONTIER_M,
    banned: set[tuple[int, int]] | None = None,
) -> tuple[tuple[float, float], list[tuple[float, float]]] | None:
    """Best frontier goal and the path to it, in odom metres. None if nothing worth driving to."""
    labels = grid.labels()
    mask = frontier_mask(labels)
    groups = clusters(mask)
    min_cells = max(1, int(math.ceil(min_frontier_m / grid.res)))
    groups = [g for g in groups if len(g) >= min_cells]
    if not groups:
        return None
    groups.sort(key=len, reverse=True)
    free = traversable(labels, grid.res, radius_m)
    here = grid.cell_of(pose_xy[0], pose_xy[1])
    if here is None:
        return None
    start = _nearest_free(free, here[0], here[1])
    if start is None:
        return None
    banned = banned or set()
    best: tuple[float, tuple[float, float], list[tuple[float, float]]] | None = None
    for group in groups[:8]:
        cells = [c for c in group if c not in banned]
        if not cells:
            continue
        # Prefer a cell the robot can stand on. The score still uses the
        # whole cluster's length, so a long wall beats a speck.
        target = viewpoint(free, cells)
        if target is None:
            continue
        path_cells = astar(free, start, target, step_cost=grid.res)
        if not path_cells:
            continue
        length = 0.0
        pts = [grid.cell_center(*rc) for rc in path_cells]
        for a, b in zip(pts, pts[1:]):
            length += math.hypot(b[0] - a[0], b[1] - a[1])
        size_m = len(group) * grid.res
        score = size_m / (length + 0.25)
        goal = grid.cell_center(*target)
        if best is None or score > best[0]:
            best = (score, goal, pts)
    if best is None:
        return None
    return best[1], best[2]
