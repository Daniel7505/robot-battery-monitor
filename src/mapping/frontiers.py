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

from src.mapping.body import GRID_RESOLUTION_M, MIN_FRONTIER_M, ROBOT_RADIUS_M
from src.mapping.grid import OccupancyGrid
from src.mapping.rooms import crossing_bonus

# Stand far enough back that the depth camera can see the frontier.
# Closer than about 0.75 m is inside the near blind zone (lens at
# z = 0.78 m, pitched down, minimum range 0.35 m).
VIEW_MIN_M = 1.0
VIEW_MAX_M = 3.0
# Close enough that a corner is in the depth image, far enough to be
# outside the near blind zone. Travel is only a tie break.
VIEW_SWEET_M = 1.15

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
    res: float = GRID_RESOLUTION_M,
    pose: tuple[int, int] | None = None,
) -> tuple[int, int] | None:
    """A traversable cell that can see a frontier.

    Prefer a cell 1–3 m back with a clear line on the map, so the depth
    camera is outside its near blind zone. A small room or a tight
    doorway has no such cell: the nearest standable cell is the fallback,
    which is what keeps the robot from stranding itself with no goal.
    """
    ranged = _viewpoint_ranged(free, cells, occupied, res, pose)
    if ranged is not None:
        return ranged
    return _viewpoint_nearest(free, cells, reach)


def _line_clear(occupied: np.ndarray, y0: int, x0: int, y1: int, x1: int) -> bool:
    """True when the cells strictly between the two ends are not occupied."""
    dy = int(y1) - int(y0)
    dx = int(x1) - int(x0)
    steps = max(abs(dy), abs(dx))
    if steps <= 1:
        return True
    height, width = occupied.shape
    for i in range(1, steps):
        y = int(round(y0 + dy * i / steps))
        x = int(round(x0 + dx * i / steps))
        if y < 0 or x < 0 or y >= height or x >= width or occupied[y, x]:
            return False
    return True


def _viewpoint_ranged(
    free: np.ndarray,
    cells: list[tuple[int, int]],
    occupied: np.ndarray | None,
    res: float,
    pose: tuple[int, int] | None = None,
) -> tuple[int, int] | None:
    """Standable cell between ``VIEW_MIN_M`` and ``VIEW_MAX_M`` with line of sight."""
    if not cells or res <= 0.0:
        return None
    height, width = free.shape
    sample = cells
    if len(cells) > 24:
        sample = cells[:: max(1, len(cells) // 24)]
    mean_y = float(np.mean([p[0] for p in sample]))
    mean_x = float(np.mean([p[1] for p in sample]))
    # The middle of the cluster, so a doorway is seen from in front of the
    # gap rather than from beside one jamb.
    anchor = min(sample, key=lambda p: (p[0] - mean_y) ** 2 + (p[1] - mean_x) ** 2)
    best: tuple[int, int] | None = None
    best_score = -1e18
    y, x = anchor
    for dist_m in (1.0, 1.4, 1.8, 2.2, 2.6, 3.0):
        radius = dist_m / res
        for k in range(16):
            ang = k * math.pi / 8.0
            cy = int(round(y + math.sin(ang) * radius))
            cx = int(round(x + math.cos(ang) * radius))
            if cy < 0 or cx < 0 or cy >= height or cx >= width or not free[cy, cx]:
                continue
            near_y, near_x = min(sample, key=lambda p: (cy - p[0]) ** 2 + (cx - p[1]) ** 2)
            dist = math.hypot(cy - near_y, cx - near_x) * res
            if dist < VIEW_MIN_M - 1e-6 or dist > VIEW_MAX_M + 0.08:
                continue
            if occupied is not None and not _line_clear(occupied, cy, cx, near_y, near_x):
                continue
            # Prefer the heading that looks at the middle of the cluster.
            off = math.hypot(near_y - mean_y, near_x - mean_x) * res
            # A stand-off near 1.15 m sees a corner the room centre misses.
            # A little travel cost keeps two equal views from picking the far one.
            travel = 0.0 if pose is None else math.hypot(cy - pose[0], cx - pose[1]) * res
            score = -abs(dist - VIEW_SWEET_M) - 0.05 * travel - 0.15 * off
            if score > best_score:
                best_score = score
                best = (cy, cx)
    return best


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
        if center is None:
            lined.append(here)
            i += 1
            continue
        cx, cy, tx, ty = center
        # Square up before the jambs, pass the centre, and only then stop.
        # Ending on the centre made the robot spin in the 0.81 m opening
        # and back out. A path that is already on the centre line used to
        # skip this, so the goal stayed on the near side of the jambs.
        if math.hypot(cx - here[0], cy - here[1]) >= 0.08:
            lined.append((cx - tx * 0.75, cy - ty * 0.75))
            lined.append((cx, cy))
        lined.append((cx + tx * 0.90, cy + ty * 0.90))
        # Drop points that are still in the gap. The goal has to be the
        # room beyond, or arrival spins the robot in the opening.
        i += 1
        while i < len(path_xy):
            px, py = path_xy[i]
            along = (px - cx) * tx + (py - cy) * ty
            if along > 0.90:
                break
            i += 1
    return _push_past_opening(grid, occupied, lined)


def _push_past_opening(
    grid: OccupancyGrid,
    occupied: np.ndarray,
    path_xy: list[tuple[float, float]],
) -> list[tuple[float, float]]:
    """If the path stops at the mouth of a gap, put the goal in the room beyond.

    A* aims at the last free cell in front of the frontier, which sits on
    the near side of a doorway. Arrival then spins in the opening.
    """
    if len(path_xy) < 2:
        return path_xy
    x0, y0 = path_xy[-1]
    # The last cell of A* is often a diagonal step. The approach over
    # the previous third of a metre is the direction through the gap.
    px, py = path_xy[0]
    for cand in path_xy:
        if math.hypot(x0 - cand[0], y0 - cand[1]) >= 0.35:
            px, py = cand
    tx, ty = x0 - px, y0 - py
    norm = math.hypot(tx, ty)
    if norm < 1e-6:
        return path_xy
    tx, ty = tx / norm, ty / norm
    # The last step can aim at a jamb. Sweep a fan so a gap just off that
    # line is still found.
    headings = [(tx, ty)]
    for ang in (0.5, -0.5, 1.0, -1.0):
        c, s = math.cos(ang), math.sin(ang)
        headings.append((c * tx - s * ty, s * tx + c * ty))
    for hx_dir, hy_dir in headings:
        for step in (0.20, 0.40, 0.60, 0.85):
            hx = x0 + hx_dir * step
            hy = y0 + hy_dir * step
            center = _opening_center(grid, occupied, (x0, y0), (hx, hy), (hx + hx_dir, hy + hy_dir))
            if center is None:
                continue
            cx, cy, ax, ay = center
            if ax * tx + ay * ty < 0.2:
                continue
            far = (cx + ax * 0.90, cy + ay * 0.90)
            if math.hypot(far[0] - x0, far[1] - y0) < 0.35:
                return path_xy
            return [*path_xy, (cx - ax * 0.55, cy - ay * 0.55), (cx, cy), far]
    return path_xy


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


def severed_mouths(
    labels: np.ndarray,
    res: float,
    radius_m: float,
    here: tuple[int, int],
) -> list[tuple[tuple[int, int], tuple[int, int], float]]:
    """Free space the shell cannot reach, just past a thin occupied gap.

    Depth through a doorway paints the next room free and then a smear
    closes the threshold. That room has no frontier, so the planner walks
    the long way around. Each item is the reachable cell at the mouth,
    a cell on the far side, and the far side's area in square metres.
    """
    trav = traversable(labels, res, radius_m)
    height, width = labels.shape
    start = _nearest_free(trav, here[0], here[1], max_cells=30)
    if start is None:
        return []
    reachable = np.zeros_like(trav)
    stack = [start]
    reachable[start] = True
    while stack:
        y, x = stack.pop()
        for ny, nx in ((y + 1, x), (y - 1, x), (y, x + 1), (y, x - 1)):
            if 0 <= ny < height and 0 <= nx < width and trav[ny, nx] and not reachable[ny, nx]:
                reachable[ny, nx] = True
                stack.append((ny, nx))
    limit = 16
    dist = np.full((height, width), 99, dtype=np.int16)
    band: list[tuple[int, int]] = []
    for y, x in zip(*np.nonzero(reachable)):
        dist[int(y), int(x)] = 0
        band.append((int(y), int(x)))
    head = 0
    while head < len(band):
        y, x = band[head]
        head += 1
        base = int(dist[y, x])
        if base >= limit:
            continue
        for ny, nx in ((y + 1, x), (y - 1, x), (y, x + 1), (y, x - 1)):
            if ny < 0 or nx < 0 or ny >= height or nx >= width:
                continue
            if dist[ny, nx] <= base + 1:
                continue
            dist[ny, nx] = base + 1
            band.append((ny, nx))
    free = (labels == 0) & ~reachable & (dist > 0) & (dist <= limit)
    if not np.any(free):
        return []
    found: list[tuple[tuple[int, int], tuple[int, int], float]] = []
    seen = np.zeros(labels.shape, dtype=bool)
    ys, xs = np.nonzero(free)
    for y0, x0 in zip(ys.tolist(), xs.tolist()):
        if seen[y0, x0]:
            continue
        pile = [(int(y0), int(x0))]
        seen[y0, x0] = True
        cells: list[tuple[int, int]] = []
        while pile:
            y, x = pile.pop()
            cells.append((y, x))
            for ny, nx in ((y + 1, x), (y - 1, x), (y, x + 1), (y, x - 1)):
                if ny < 0 or nx < 0 or ny >= height or nx >= width:
                    continue
                if seen[ny, nx] or reachable[ny, nx] or labels[ny, nx] != 0:
                    continue
                seen[ny, nx] = True
                pile.append((ny, nx))
        area = len(cells) * res * res
        if area < 1.0:
            continue
        dists = [int(dist[y, x]) for y, x in cells]
        min_d = min(dists)
        if min_d < 1 or min_d > limit:
            continue
        near = [(y, x) for y, x in cells if int(dist[y, x]) <= min_d + 2]
        if len(near) < 3:
            continue
        rows = [p[0] for p in near]
        cols = [p[1] for p in near]
        span = max(max(rows) - min(rows), max(cols) - min(cols)) * res
        # A doorway is about a metre across. The rim of a whole room is longer.
        if span < 0.45 or span > 1.45:
            continue
        mouth = None
        beyond = None
        best = 10**9
        for y, x in near:
            for dy in range(-(min_d + 1), min_d + 2):
                for dx in range(-(min_d + 1), min_d + 2):
                    ny, nx = y + dy, x + dx
                    if ny < 0 or nx < 0 or ny >= height or nx >= width or not reachable[ny, nx]:
                        continue
                    dd = dy * dy + dx * dx
                    if dd < best:
                        best = dd
                        mouth = (ny, nx)
                        beyond = (y, x)
        if mouth is None or beyond is None:
            continue
        found.append((mouth, beyond, area))
    found.sort(key=lambda item: item[2], reverse=True)
    return found[:4]


def _touches_door(group: list[tuple[int, int]], doors: np.ndarray) -> bool:
    height, width = doors.shape
    step = max(1, len(group) // 8)
    for y, x in group[::step]:
        if doors[y, x]:
            return True
        for ny, nx in ((y + 1, x), (y - 1, x), (y, x + 1), (y, x - 1)):
            if 0 <= ny < height and 0 <= nx < width and doors[ny, nx]:
                return True
    return False


def select_goal(
    grid: OccupancyGrid,
    pose_xy: tuple[float, float],
    *,
    radius_m: float = ROBOT_RADIUS_M,
    min_frontier_m: float = MIN_FRONTIER_M,
    banned: set[tuple[int, int]] | None = None,
    prefer_xy: tuple[float, float] | None = None,
    rooms: dict | None = None,
    heading_xy: tuple[float, float] | None = None,
    report: dict | None = None,
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
    # The longest frontiers are usually the room the robot is already in.
    # A short frontier just past a doorway still has to be scored, or the
    # curiosity bonus never sees it.
    ranked = list(groups[:8])
    if rooms is not None and rooms.get("doors") is not None:
        extra = 0
        for group in groups[8:24]:
            if extra >= 4:
                break
            if _touches_door(group, rooms["doors"]):
                ranked.append(group)
                extra += 1
    free = traversable(labels, grid.res, radius_m)
    here = grid.cell_of(pose_xy[0], pose_xy[1])
    if here is None:
        return None
    start = _nearest_free(free, here[0], here[1])
    if start is None:
        return None
    banned = banned or set()
    best: tuple[float, tuple[float, float], list[tuple[float, float]], list] | None = None
    for group in ranked:
        cells = [c for c in group if c not in banned]
        if not cells:
            continue
        # Prefer a cell the robot can stand on. The score still uses the
        # whole cluster's length, so a long wall beats a speck.
        target = viewpoint(free, cells, occupied=(labels == 1), res=grid.res, pose=start)
        if target is None:
            continue
        path_cells = astar(free, start, target, step_cost=grid.res)
        if not path_cells:
            # The far stand-off can sit across a wall. The nearest safe
            # cell is still a goal, so a tight doorway is not abandoned.
            nearest = _viewpoint_nearest(free, cells, 6)
            if nearest is None or nearest == target:
                continue
            path_cells = astar(free, start, nearest, step_cost=grid.res)
            if not path_cells:
                continue
            target = nearest
        length = 0.0
        pts = [grid.cell_center(*rc) for rc in path_cells]
        for a, b in zip(pts, pts[1:]):
            length += math.hypot(b[0] - a[0], b[1] - a[1])
        size_m = len(group) * grid.res
        score = size_m / (length + 0.25)
        bonus = crossing_bonus(path_cells, rooms, start)
        score *= bonus
        goal = grid.cell_center(*target)
        if heading_xy is not None and bonus > 1.0:
            # Two doorways can both be new. The one in front of the robot
            # is the crossing it is already lined up with.
            dx = goal[0] - float(pose_xy[0])
            dy = goal[1] - float(pose_xy[1])
            norm = math.hypot(dx, dy) or 1.0
            hx, hy = float(heading_xy[0]), float(heading_xy[1])
            hn = math.hypot(hx, hy) or 1.0
            if (dx * hx + dy * hy) / (norm * hn) > 0.6:
                score *= 3.0
        if prefer_xy is not None:
            dx = goal[0] - float(pose_xy[0])
            dy = goal[1] - float(pose_xy[1])
            norm = math.hypot(dx, dy) or 1.0
            ahead = (dx * prefer_xy[0] + dy * prefer_xy[1]) / norm
            # A frontier behind a doorway we just committed to is how the
            # robot turned around in the hall and came back out. A goal
            # off to the side (the bath continuing north after a west
            # entry) is still in the new room, so only the rear half loses.
            if ahead < -0.05:
                score *= 0.25
        if best is None or score > best[0]:
            best = (score, goal, pts, cells)
    # A room already painted free, with the threshold smeared shut, is
    # not a frontier. Score it by its area and aim through the gap.
    for mouth, beyond, area in severed_mouths(labels, grid.res, radius_m, here):
        if mouth in banned:
            continue
        path_cells = astar(free, start, mouth, step_cost=grid.res)
        if not path_cells:
            continue
        pts = [grid.cell_center(*rc) for rc in path_cells]
        length = 0.0
        for a, b in zip(pts, pts[1:]):
            length += math.hypot(b[0] - a[0], b[1] - a[1])
        far = grid.cell_center(*beyond)
        length += math.hypot(far[0] - pts[-1][0], far[1] - pts[-1][1])
        score = area / (length + 0.25) * 6.0
        goal = far
        if heading_xy is not None:
            dx = goal[0] - float(pose_xy[0])
            dy = goal[1] - float(pose_xy[1])
            norm = math.hypot(dx, dy) or 1.0
            hx, hy = float(heading_xy[0]), float(heading_xy[1])
            hn = math.hypot(hx, hy) or 1.0
            if (dx * hx + dy * hy) / (norm * hn) > 0.6:
                score *= 3.0
        if prefer_xy is not None:
            dx = goal[0] - float(pose_xy[0])
            dy = goal[1] - float(pose_xy[1])
            norm = math.hypot(dx, dy) or 1.0
            ahead = (dx * prefer_xy[0] + dy * prefer_xy[1]) / norm
            if ahead < -0.05:
                score *= 0.25
        pts = [*pts, far]
        if best is None or score > best[0]:
            best = (score, goal, pts, [mouth])
    if best is None:
        return None
    pre = best[2]
    pts = line_up_with_openings(grid, pre)
    if not pts:
        return None
    # Lining up must not drag the stand-off into a wall or onto the frontier.
    # A shift of a few decimetres still centres a gap the path already crosses.
    end = grid.cell_of(pts[-1][0], pts[-1][1])
    stand = end is not None and free[end]
    # A shove into a wall, or onto the frontier itself, puts the lens in
    # the near blind zone. A shove through the gap into the next room stays:
    # that end is still a metre or more from this frontier.
    onto_frontier = _within_m(grid, pts[-1], best[3], VIEW_MIN_M) and not _within_m(
        grid, pre[-1], best[3], VIEW_MIN_M
    )
    if not stand or onto_frontier:
        pts = pre
    if report is not None:
        report["cells"] = list(best[3])
    return pts[-1], pts


def _within_m(grid: OccupancyGrid, xy: tuple[float, float], cells: list, limit_m: float) -> bool:
    if not cells:
        return False
    cell = grid.cell_of(xy[0], xy[1])
    if cell is None:
        return False
    limit = (limit_m / grid.res) ** 2
    y, x = cell
    return any((y - cy) ** 2 + (x - cx) ** 2 <= limit for cy, cx in cells)
