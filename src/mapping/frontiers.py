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


def viewpoint(
    free: np.ndarray,
    cells: list[tuple[int, int]],
    reach: int = 6,
    occupied: np.ndarray | None = None,
) -> tuple[int, int] | None:
    """A traversable cell on or just inside a frontier.

    The frontier cell itself often sits inside the inflated obstacle, so
    the robot cannot path to it. Standing a few cells back still sees it.
    When the map has walls, the cell is the one nearest the middle of the
    cluster with the most room on both sides, so a doorway is aimed at
    its centre instead of the jamb.
    """
    nearest = _viewpoint_nearest(free, cells, reach)
    if occupied is None or not np.any(occupied) or not cells:
        return nearest
    h, w = free.shape
    cy = float(np.mean([p[0] for p in cells]))
    cx = float(np.mean([p[1] for p in cells]))
    pad = reach + 2
    y0 = max(0, int(math.floor(cy)) - pad)
    y1 = min(h, int(math.floor(cy)) + pad + 1)
    x0 = max(0, int(math.floor(cx)) - pad)
    x1 = min(w, int(math.floor(cx)) + pad + 1)
    if y1 <= y0 or x1 <= x0:
        return nearest
    near = np.zeros((h, w), dtype=bool)
    sample = cells
    if len(cells) > 40:
        sample = cells[:: max(1, len(cells) // 40)]
    for y, x in sample:
        near[max(0, y - reach) : min(h, y + reach + 1), max(0, x - reach) : min(w, x + reach + 1)] = True
    cand = free[y0:y1, x0:x1] & near[y0:y1, x0:x1]
    if not np.any(cand):
        return nearest
    cys, cxs = np.nonzero(cand)
    cys = cys + y0
    cxs = cxs + x0
    oys, oxs = np.nonzero(occupied)
    keep = (oys >= y0 - 16) & (oys < y1 + 16) & (oxs >= x0 - 16) & (oxs < x1 + 16)
    oys, oxs = oys[keep], oxs[keep]
    if len(oys) == 0:
        clear = np.full(len(cys), 16.0)
    else:
        d2 = (cys[:, None] - oys[None, :]) ** 2 + (cxs[:, None] - oxs[None, :]) ** 2
        clear = np.sqrt(np.min(d2, axis=1))
    dist2 = (cys - cy) ** 2 + (cxs - cx) ** 2
    i = int(np.argmax(clear * 1000.0 - dist2))
    return int(cys[i]), int(cxs[i])


def _viewpoint_nearest(free: np.ndarray, cells: list[tuple[int, int]], reach: int) -> tuple[int, int] | None:
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


def line_up_with_openings(grid: OccupancyGrid, path_xy: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Pull a path onto the centre line of a narrow opening.

    A doorway is occupied on both sides and about a metre wide. The
    point is moved to the middle of that gap, and a point is placed
    just before it so the robot turns to face the opening before it
    enters. Wide rooms are left alone.
    """
    if len(path_xy) < 2:
        return list(path_xy)
    occupied = grid.labels() == 1
    if not np.any(occupied):
        return list(path_xy)
    lined: list[tuple[float, float]] = [path_xy[0]]
    i = 1
    while i < len(path_xy):
        prev = lined[-1]
        here = path_xy[i]
        nxt = path_xy[min(i + 1, len(path_xy) - 1)]
        center = _opening_center(grid, occupied, prev, here, nxt)
        if center is None or math.hypot(center[0] - here[0], center[1] - here[1]) < 0.08:
            lined.append(here)
            i += 1
            continue
        cx, cy, tx, ty = center
        # Square up on the centre line before the nose reaches the jambs.
        # 0.40 m was inside the turn, so the body was still angled when it
        # met the 0.81 m opening.
        lined.append((cx - tx * 0.75, cy - ty * 0.75))
        lined.append((cx, cy))
        # Drop later points that belong to the same gap so the path does
        # not wander back to the jamb.
        i += 1
        while i < len(path_xy) and math.hypot(path_xy[i][0] - here[0], path_xy[i][1] - here[1]) <= 0.45:
            i += 1
    return lined


def _opening_center(
    grid: OccupancyGrid,
    occupied: np.ndarray,
    prev: tuple[float, float],
    here: tuple[float, float],
    nxt: tuple[float, float],
) -> tuple[float, float, float, float] | None:
    tx, ty = nxt[0] - prev[0], nxt[1] - prev[1]
    if abs(tx) + abs(ty) < 1e-6:
        tx, ty = here[0] - prev[0], here[1] - prev[1]
    norm = math.hypot(tx, ty)
    if norm < 1e-6:
        return None
    tx, ty = tx / norm, ty / norm
    px, py = -ty, tx
    left = _march(grid, occupied, here[0], here[1], px, py, 1.15)
    right = _march(grid, occupied, here[0], here[1], -px, -py, 1.15)
    if left is None or right is None:
        return None
    gap = left + right
    if gap < 0.55 or gap > 1.30:
        return None
    shift = 0.5 * (left - right)
    cx = here[0] + px * shift
    cy = here[1] + py * shift
    # Point the axis through the opening, in the direction of travel.
    if (nxt[0] - prev[0]) * tx + (nxt[1] - prev[1]) * ty < 0.0:
        tx, ty = -tx, -ty
    return (cx, cy, tx, ty)


def _march(
    grid: OccupancyGrid,
    occupied: np.ndarray,
    x: float,
    y: float,
    dx: float,
    dy: float,
    limit_m: float,
) -> float | None:
    dist = grid.res
    while dist <= limit_m + 1e-9:
        cell = grid.cell_of(x + dx * dist, y + dy * dist)
        if cell is None:
            return None
        if occupied[cell]:
            return dist
        dist += grid.res
    return None


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
        target = viewpoint(free, cells, occupied=(labels == 1))
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
    pts = line_up_with_openings(grid, best[2])
    if not pts:
        return None
    return pts[-1], pts
