"""Frontier selection.

Yamauchi 1997: a frontier cell is free space next to unknown space.
Clusters smaller than ``MIN_FRONTIER_M`` are ignored. The rest are
scored by size divided by the A* path length. The path is planned on
the free grid with occupied cells inflated by the robot radius, so the
robot does not try to squeeze through a door it cannot fit. With
``RBM_FOOTPRINT_ALIGN`` the inflation is the same rectangle the map
veto sweeps, not that disk. The score weights do not change. Turns are
placed where the centre is at least ``TURN_CLEAR_M`` from a mapped
obstacle, and the path prefers ``TURN_AIM_M``. A doorway is a straight
crossing, not a turn. A frontier the robot cannot turn around in, and
that is shorter than 0.8 m, is left alone. A viewpoint stands 1–3 m
off the frontier when a free cell there can see it. A tight room falls
back to the nearest safe cell. A doorway on the path is still lined up.
"""

from __future__ import annotations

import heapq
import math
import os

import numpy as np

from src.mapping.body import GRID_RESOLUTION_M, MIN_FRONTIER_M, ROBOT_RADIUS_M, TURN_AIM_M, TURN_CLEAR_M
from src.mapping.footprint import (
    HALF_M,
    NOSE_M,
    TAIL_M,
    _trajectory,
    footprint_align_enabled,
    footprint_clips,
    shared_footprint_enabled,
)
from src.mapping.grid import OccupancyGrid
from src.mapping.rooms import _nearby_region, crossing_bonus

# Stand far enough back that the depth camera can see the frontier.
# Closer than about 0.75 m is inside the near blind zone (lens at
# z = 0.78 m, pitched down, minimum range 0.35 m).
VIEW_MIN_M = 1.0
VIEW_MAX_M = 3.0
# Close enough that a corner is in the depth image, far enough to be
# outside the near blind zone. Travel is only a tie break.
VIEW_SWEET_M = 1.15

# Expansion limit inside one frontier search. A hit is a failed search,
# not a longer path. The quit dump counts how often this fires.
_ASTAR_CAP = 25000

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
_NEIGHBOR_INDEX = {(dy, dx): i for i, (dy, dx, _mul) in enumerate(_NEIGHBORS)}
# Same sample as the spin sweep. One mask per sample around the circle.
_YAW_STEP = 0.10
_YAW_COUNT = int(round(2.0 * math.pi / _YAW_STEP))
_OFFSETS: dict[tuple[float, float], tuple[tuple[int, int], ...]] = {}
_MASK_CACHE: dict[str, object] = {"key": None, "masks": None}
_FREE_CACHE: dict[str, object] = {"key": None, "mask": None}
# Set when a planned pivot is replaced because the rectangle clips.
_approach_reject = False


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


def _viewpoint_clearest(
    free: np.ndarray,
    cells: list[tuple[int, int]],
    occupied: np.ndarray,
    reach: int = 6,
) -> tuple[int, int] | None:
    """Free cell near the frontier with the most room on both sides.

    Used when a 1–3 m stand-off does not exist and the nearest cell is
    inside the turning radius. A doorway is then aimed at its centre
    instead of the jamb. A cell with no extra room returns None.
    """
    if not cells or not np.any(occupied):
        return None
    height, width = free.shape
    mean_y = float(np.mean([p[0] for p in cells]))
    mean_x = float(np.mean([p[1] for p in cells]))
    pad = reach + 2
    y0 = max(0, int(math.floor(mean_y)) - pad)
    y1 = min(height, int(math.floor(mean_y)) + pad + 1)
    x0 = max(0, int(math.floor(mean_x)) - pad)
    x1 = min(width, int(math.floor(mean_x)) + pad + 1)
    if y1 <= y0 or x1 <= x0:
        return None
    near = np.zeros((height, width), dtype=bool)
    sample = cells if len(cells) <= 40 else cells[:: max(1, len(cells) // 40)]
    for y, x in sample:
        near[max(0, y - reach) : min(height, y + reach + 1), max(0, x - reach) : min(width, x + reach + 1)] = True
    cand = free[y0:y1, x0:x1] & near[y0:y1, x0:x1]
    if not np.any(cand):
        return None
    cys, cxs = np.nonzero(cand)
    cys = cys + y0
    cxs = cxs + x0
    oys, oxs = np.nonzero(occupied)
    keep = (oys >= y0 - 16) & (oys < y1 + 16) & (oxs >= x0 - 16) & (oxs < x1 + 16)
    oys, oxs = oys[keep], oxs[keep]
    if len(oys) == 0:
        return None
    dist2 = (cys[:, None] - oys[None, :]) ** 2 + (cxs[:, None] - oxs[None, :]) ** 2
    room = np.sqrt(np.min(dist2, axis=1))
    centre = (cys - mean_y) ** 2 + (cxs - mean_x) ** 2
    i = int(np.argmax(room * 1000.0 - centre))
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


def _yaw_index(yaw: float, n: int) -> int:
    step = 2.0 * math.pi / n
    return int(round((_wrap(yaw) + math.pi) / step)) % n


def _rect_offsets(yaw: float, res: float) -> tuple[tuple[int, int], ...]:
    """Cell offsets of an obstacle relative to a centre the rectangle covers."""
    key = (round(float(yaw), 5), round(float(res), 5))
    hit = _OFFSETS.get(key)
    if hit is not None:
        return hit
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    reach = int(math.ceil(math.hypot(max(NOSE_M, TAIL_M), HALF_M) / res)) + 1
    found: list[tuple[int, int]] = []
    for dr in range(-reach, reach + 1):
        dy = dr * res
        for dc in range(-reach, reach + 1):
            dx = dc * res
            bx = cos_y * dx + sin_y * dy
            by = -sin_y * dx + cos_y * dy
            if -TAIL_M <= bx <= NOSE_M and -HALF_M <= by <= HALF_M:
                found.append((dr, dc))
    packed = tuple(found)
    _OFFSETS[key] = packed
    return packed


def _stamp_offsets(occupied: np.ndarray, offsets: tuple[tuple[int, int], ...]) -> np.ndarray:
    height, width = occupied.shape
    out = np.zeros((height, width), dtype=bool)
    for dr, dc in offsets:
        if dr >= 0:
            src_r0, src_r1 = dr, height
            dst_r0, dst_r1 = 0, height - dr
        else:
            src_r0, src_r1 = 0, height + dr
            dst_r0, dst_r1 = -dr, height
        if dc >= 0:
            src_c0, src_c1 = dc, width
            dst_c0, dst_c1 = 0, width - dc
        else:
            src_c0, src_c1 = 0, width + dc
            dst_c0, dst_c1 = -dc, width
        if src_r1 <= src_r0 or src_c1 <= src_c0:
            continue
        out[dst_r0:dst_r1, dst_c0:dst_c1] |= occupied[src_r0:src_r1, src_c0:src_c1]
    return out


def rectangle_masks(occupied: np.ndarray, res: float) -> np.ndarray:
    """One mask per yaw sample. True where the shared rectangle hits occupied.

    The outline is the skid shell plus ``FOOTPRINT_MARGIN_M``. A cell the
    0.20 m disk still calls free can be true here, because the corner of
    the rectangle sticks out past that disk.
    """
    key = (occupied.shape, round(float(res), 5), hash(occupied.tobytes()))
    if _MASK_CACHE["key"] == key:
        return _MASK_CACHE["masks"]  # type: ignore[return-value]
    n = _YAW_COUNT
    step = 2.0 * math.pi / n
    masks = np.zeros((n, occupied.shape[0], occupied.shape[1]), dtype=bool)
    if np.any(occupied):
        for i in range(n):
            yaw = -math.pi + i * step
            masks[i] = _stamp_offsets(occupied, _rect_offsets(yaw, res))
    _MASK_CACHE["key"] = key
    _MASK_CACHE["masks"] = masks
    return masks


def neighbor_blocks(masks: np.ndarray) -> np.ndarray:
    """The yaw masks gathered into ``_NEIGHBORS`` order."""
    out = np.empty((len(_NEIGHBORS),) + masks.shape[1:], dtype=bool)
    n = masks.shape[0]
    for i, (dy, dx, _mul) in enumerate(_NEIGHBORS):
        out[i] = masks[_yaw_index(math.atan2(dy, dx), n)]
    return out


def _turn_blocked(masks: np.ndarray, row: int, col: int, yaw_in: float, yaw_out: float) -> bool:
    """True when the rectangle hits while yawing from ``yaw_in`` to ``yaw_out``.

    A step under about 57° is the grid, not a pivot, and is not checked.
    The samples match the spin sweep's 0.10 rad step.
    """
    err = _wrap(yaw_out - yaw_in)
    if abs(err) <= 1.0:
        return False
    n = masks.shape[0]
    steps = max(1, int(math.ceil(abs(err) / _YAW_STEP)))
    for i in range(1, steps + 1):
        if masks[_yaw_index(yaw_in + err * (i / steps), n), row, col]:
            return True
    return False


def traversable(labels: np.ndarray, res: float, radius_m: float = ROBOT_RADIUS_M) -> np.ndarray:
    if footprint_align_enabled():
        blocks = neighbor_blocks(rectangle_masks(labels == 1, res))
        return (labels == 0) & ~np.all(blocks, axis=0)
    blocked = dilate_disk(labels == 1, radius_m, res)
    return (labels == 0) & ~blocked


def astar(
    free: np.ndarray,
    start: tuple[int, int],
    goal: tuple[int, int],
    *,
    step_cost: float,
    costs: np.ndarray | None = None,
    heading_block: np.ndarray | None = None,
    yaw_masks: np.ndarray | None = None,
    start_yaw: float | None = None,
    stats: dict | None = None,
) -> list[tuple[int, int]] | None:
    """8-connected A* on a boolean free grid. ``start`` may sit just off the grid.

    ``heading_block`` and ``yaw_masks`` are the shared rectangle. They are
    absent unless ``RBM_FOOTPRINT_ALIGN`` is on, and they do not change the
    step weights.
    """
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
        if seen > _ASTAR_CAP:
            if stats is not None:
                stats["cap"] = int(stats.get("cap", 0)) + 1
            return None
        for index, (dy, dx, mul) in enumerate(_NEIGHBORS):
            ny, nx = y + dy, x + dx
            if ny < 0 or nx < 0 or ny >= h or nx >= w:
                continue
            if (ny, nx) != start and not free[ny, nx]:
                continue
            if heading_block is not None and bool(heading_block[index, ny, nx]):
                continue
            if dy != 0 and dx != 0:
                if not free[y + dy, x] or not free[y, x + dx]:
                    continue
                if heading_block is not None:
                    side_y = _NEIGHBOR_INDEX[(dy, 0)]
                    side_x = _NEIGHBOR_INDEX[(0, dx)]
                    if bool(heading_block[side_y, y + dy, x]) or bool(heading_block[side_x, y, x + dx]):
                        continue
            if yaw_masks is not None:
                if (y, x) in came:
                    py, px = came[(y, x)]
                    hin = math.atan2(y - py, x - px)
                    if _turn_blocked(yaw_masks, y, x, hin, math.atan2(dy, dx)):
                        continue
                elif start_yaw is not None and (y, x) == start:
                    if _turn_blocked(yaw_masks, y, x, float(start_yaw), math.atan2(dy, dx)):
                        continue
            weight = 1.0 if costs is None else float(costs[ny, nx])
            ng = g + mul * step_cost * weight
            key = (ny, nx)
            if ng + 1e-9 < best.get(key, 1e18):
                best[key] = ng
                came[key] = (y, x)
                heapq.heappush(open_heap, (ng + heuristic(ny, nx), ng, ny, nx))
    return None


def true_start_enabled() -> bool:
    """True when A* starts in the robot's own free component.

    Off unless ``RBM_TRUE_START`` is set. The default keeps the straight-line
    snap of ``_nearest_free``.
    """
    raw = (os.environ.get("RBM_TRUE_START") or "").strip().lower()
    return raw in ("1", "true", "yes", "on")


def _cell_blocked(labels: np.ndarray, row: int, col: int, here: tuple[int, int]) -> bool:
    """Occupied cells block the escape. The robot's own cell does not."""
    if (row, col) == here:
        return False
    return int(labels[row, col]) == 1


def _true_start(
    free: np.ndarray,
    labels: np.ndarray,
    here: tuple[int, int],
    max_cells: int = 40,
) -> tuple[int, int] | None:
    """A planner-free cell in the robot's own component.

    The robot cell is treated as free. The walk does not step on an
    occupied cell, and it uses the same 8-connected corner rule as A*.
    The limit is the 40-cell radius ``_nearest_free`` already uses.
    """
    height, width = free.shape
    y0, x0 = here
    if not (0 <= y0 < height and 0 <= x0 < width):
        return None
    if bool(free[y0, x0]):
        return y0, x0
    seen = {(y0, x0)}
    queue: list[tuple[int, int, int]] = [(y0, x0, 0)]
    head = 0
    while head < len(queue):
        y, x, dist = queue[head]
        head += 1
        if dist >= max_cells:
            continue
        for dy, dx, _mul in _NEIGHBORS:
            ny, nx = y + dy, x + dx
            if ny < 0 or nx < 0 or ny >= height or nx >= width or (ny, nx) in seen:
                continue
            if _cell_blocked(labels, ny, nx, here):
                continue
            if dy != 0 and dx != 0 and (
                _cell_blocked(labels, y + dy, x, here) or _cell_blocked(labels, y, x + dx, here)
            ):
                continue
            seen.add((ny, nx))
            if bool(free[ny, nx]):
                return ny, nx
            queue.append((ny, nx, dist + 1))
    return None


def _planner_start(
    free: np.ndarray,
    labels: np.ndarray,
    here: tuple[int, int],
    max_cells: int = 40,
) -> tuple[int, int] | None:
    if true_start_enabled():
        return _true_start(free, labels, here, max_cells)
    return _nearest_free(free, here[0], here[1], max_cells)


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
    start = _planner_start(trav, labels, here, max_cells=30)
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


def apply_zone_cost(costs: np.ndarray | None, grid: OccupancyGrid, zones) -> np.ndarray | None:
    """Raise the step cost inside each (x, y, radius) disk. Cells stay free.

    A path can still cross the disk when it is the only way out. A goal
    inside the disk is refused separately. This does not write the occupancy
    grid: the camera's map is left alone.
    """
    if not zones:
        return costs
    shape = costs.shape if costs is not None else grid.labels().shape
    out = np.ones(shape, dtype=np.float32) if costs is None else np.array(costs, dtype=np.float32, copy=True)
    res = float(grid.res)
    ox, oy = float(grid.origin[0]), float(grid.origin[1])
    height, width = shape
    for item in zones:
        x, y, radius = float(item[0]), float(item[1]), float(item[2])
        if radius <= 0.0:
            continue
        c_lo = max(0, int(math.floor((x - radius - ox) / res)))
        c_hi = min(width, int(math.floor((x + radius - ox) / res)) + 1)
        r_lo = max(0, int(math.floor((y - radius - oy) / res)))
        r_hi = min(height, int(math.floor((y + radius - oy) / res)) + 1)
        if r_lo >= r_hi or c_lo >= c_hi:
            continue
        cols = np.arange(c_lo, c_hi)
        rows = np.arange(r_lo, r_hi)
        cc, rr = np.meshgrid(cols, rows)
        cx = ox + (cc + 0.5) * res
        cy = oy + (rr + 0.5) * res
        mask = (cx - x) ** 2 + (cy - y) ** 2 <= radius * radius
        window = out[r_lo:r_hi, c_lo:c_hi]
        window[mask] = np.maximum(window[mask], 40.0)
    return out


def point_in_zones(goal: tuple[float, float], zones) -> bool:
    """True when a goal point sits inside a quarantine disk."""
    if not zones:
        return False
    gx, gy = float(goal[0]), float(goal[1])
    for item in zones:
        if math.hypot(gx - float(item[0]), gy - float(item[1])) < float(item[2]):
            return True
    return False


def _region_factor(goal: tuple[float, float], avoid: list[tuple[float, float, float]]) -> float:
    """Scale a score down near a recently attempted region. The frontier stays reachable."""
    factor = 1.0
    for item in avoid:
        weight = float(item[2])
        if weight <= 0.0:
            continue
        dist = math.hypot(float(goal[0]) - float(item[0]), float(goal[1]) - float(item[1]))
        if dist >= 0.60:
            continue
        factor *= math.exp(-weight * (1.0 - dist / 0.60))
    return factor


def clearance_m(occupied: np.ndarray, res: float, reach_m: float = 1.4) -> np.ndarray:
    """Distance in metres from each cell to the nearest occupied cell."""
    dist = np.full(occupied.shape, float(reach_m) + float(res), dtype=np.float32)
    if np.any(occupied):
        dist[occupied] = 0.0
    ortho = np.float32(res)
    diag = np.float32(res * math.sqrt(2.0))
    steps = int(math.ceil(reach_m / res))
    for _ in range(steps):
        nxt = dist.copy()
        nxt[1:, :] = np.minimum(nxt[1:, :], dist[:-1, :] + ortho)
        nxt[:-1, :] = np.minimum(nxt[:-1, :], dist[1:, :] + ortho)
        nxt[:, 1:] = np.minimum(nxt[:, 1:], dist[:, :-1] + ortho)
        nxt[:, :-1] = np.minimum(nxt[:, :-1], dist[:, 1:] + ortho)
        nxt[1:, 1:] = np.minimum(nxt[1:, 1:], dist[:-1, :-1] + diag)
        nxt[1:, :-1] = np.minimum(nxt[1:, :-1], dist[:-1, 1:] + diag)
        nxt[:-1, 1:] = np.minimum(nxt[:-1, 1:], dist[1:, :-1] + diag)
        nxt[:-1, :-1] = np.minimum(nxt[:-1, :-1], dist[1:, 1:] + diag)
        dist = nxt
    return dist


def approach_costs(clear: np.ndarray) -> np.ndarray:
    """Prefer a centre 0.30 m out. Cells inside 0.23 m are a last resort.

    The middle of a narrow opening stays cheap, so a doorway is crossed
    on its centre line instead of being sealed.
    """
    costs = np.ones(clear.shape, dtype=np.float32)
    costs[clear < TURN_AIM_M] = 2.5
    costs[clear < TURN_CLEAR_M] = 6.0
    # A throat centre is the cell with more air than its neighbours.
    peak = clear.copy()
    peak[1:, :] = np.maximum(peak[1:, :], clear[:-1, :])
    peak[:-1, :] = np.maximum(peak[:-1, :], clear[1:, :])
    peak[:, 1:] = np.maximum(peak[:, 1:], clear[:, :-1])
    peak[:, :-1] = np.maximum(peak[:, :-1], clear[:, 1:])
    throat = (clear >= peak - 1e-4) & (clear >= 0.18) & (clear <= 0.70)
    costs[throat] = 1.0
    return costs


def _clearance_at(grid: OccupancyGrid, dist: np.ndarray, x: float, y: float) -> float:
    cell = grid.cell_of(x, y)
    if cell is None:
        return 0.0
    return float(dist[cell])


def hold_turning_clearance(
    grid: OccupancyGrid,
    path_xy: list[tuple[float, float]],
    *,
    aim_m: float = TURN_AIM_M,
    floor_m: float = TURN_CLEAR_M,
) -> list[tuple[float, float]]:
    """Move path points out to the turning aim, except a doorway centre.

    The first point is the robot and stays. A point already on an opening
    centre stays, so the crossing is straight. Anything else closer than
    the aim is pushed away from the nearest mapped obstacle. A creep
    through a seen throat passes a lower aim, the body plus a small margin.
    """
    if len(path_xy) < 2:
        return list(path_xy)
    occupied = grid.labels() == 1
    if not np.any(occupied):
        return list(path_xy)
    dist = clearance_m(occupied, grid.res)
    out: list[tuple[float, float]] = [path_xy[0]]
    for index, here in enumerate(path_xy[1:], start=1):
        prev = out[-1]
        nxt = path_xy[min(index + 1, len(path_xy) - 1)]
        opening = _opening_center(grid, occupied, prev, here, nxt)
        if opening is not None and math.hypot(here[0] - opening[0], here[1] - opening[1]) < 0.20:
            point = (opening[0], opening[1])
        else:
            point = _push_clear(grid, dist, here[0], here[1], aim_m=aim_m, floor_m=floor_m)
            if point is None:
                continue
        if math.hypot(point[0] - out[-1][0], point[1] - out[-1][1]) < 0.05:
            continue
        out.append(point)
    return out if len(out) >= 2 else list(path_xy)


def _push_clear(
    grid: OccupancyGrid,
    dist: np.ndarray,
    x: float,
    y: float,
    *,
    aim_m: float = TURN_AIM_M,
    floor_m: float = TURN_CLEAR_M,
) -> tuple[float, float] | None:
    have = _clearance_at(grid, dist, x, y)
    if have >= aim_m:
        return (x, y)
    cell = grid.cell_of(x, y)
    if cell is None:
        return None
    row, col = cell
    away_r, away_c = 0, 0
    nearest = have
    for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        rr, cc = row + dr, col + dc
        if rr < 0 or cc < 0 or rr >= dist.shape[0] or cc >= dist.shape[1]:
            continue
        if float(dist[rr, cc]) < nearest:
            nearest = float(dist[rr, cc])
            away_r, away_c = -dr, -dc
    if away_r == 0 and away_c == 0:
        return (x, y) if have >= floor_m else None
    step = grid.res
    best = (x, y) if have >= floor_m else None
    px, py = x, y
    for _ in range(12):
        px += away_c * step
        py += away_r * step
        got = _clearance_at(grid, dist, px, py)
        if got >= floor_m:
            best = (px, py)
        if got >= aim_m:
            return (px, py)
    return best


def _opening_persists(
    grid: OccupancyGrid,
    occupied: np.ndarray,
    opening: tuple[float, float, float, float],
    ahead_m: float,
) -> bool:
    """True when the same gap is still there past the mouth. That is a hall."""
    cx, cy, tx, ty = opening
    ahead = (cx + tx * ahead_m, cy + ty * ahead_m)
    further = (ahead[0] + tx * 0.2, ahead[1] + ty * 0.2)
    return _opening_center(grid, occupied, (cx, cy), ahead, further) is not None


def _pose_clearance(grid: OccupancyGrid, x: float, y: float, reach_m: float = 0.45) -> float:
    """Metres from the centre to the nearest mapped obstacle, capped."""
    labels = grid.labels()
    cell = grid.cell_of(x, y)
    if cell is None:
        return 0.0
    res = float(grid.res)
    n = int(math.ceil(reach_m / res))
    row, col = cell
    best = float(reach_m)
    height, width = labels.shape
    for dr in range(-n, n + 1):
        rr = row + dr
        if rr < 0 or rr >= height:
            continue
        for dc in range(-n, n + 1):
            cc = col + dc
            if cc < 0 or cc >= width or int(labels[rr, cc]) != 1:
                continue
            cx, cy = grid.cell_center(rr, cc)
            best = min(best, math.hypot(cx - x, cy - y))
    return best


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def spin_fits(grid: OccupancyGrid, pose: tuple[float, float, float]) -> bool:
    """True when a half-turn in place stays off mapped obstacles.

    Six headings, each swept by the footprint check, cover about 180°.
    Either direction is enough. This is not a search planner.
    """
    x, y, yaw = (float(pose[0]), float(pose[1]), float(pose[2]))
    for sign in (1.0, -1.0):
        heading = yaw
        clear = True
        for _ in range(6):
            if motion_blocked(grid, (x, y, heading), 0.0, sign * 0.70):
                clear = False
                break
            heading += sign * 0.45
        if clear:
            return True
    return False


def free_behind_m(grid: OccupancyGrid, pose: tuple[float, float, float]) -> float:
    """Known-free metres straight behind the tail. Unknown stops the count."""
    labels = grid.labels()
    x, y, yaw = (float(pose[0]), float(pose[1]), float(pose[2]))
    back = (-math.cos(yaw), -math.sin(yaw))
    step = float(grid.res)
    dist = step
    last = 0.0
    while dist <= 1.5 + 1e-9:
        cell = grid.cell_of(x + back[0] * dist, y + back[1] * dist)
        if cell is None or int(labels[cell]) != 0:
            return last
        last = dist
        dist += step
    return last


def wide_escape(
    grid: OccupancyGrid, pose: tuple[float, float, float]
) -> tuple[float, float, str] | None:
    """Drive toward nearby open floor when this spot is too tight to pivot.

    None when the centre already has turning room, or the open cell cannot
    be reached without a pivot. Backing out of a doorway stays the caller's
    choice; this is only the open-room option.
    """
    x, y, yaw = (float(pose[0]), float(pose[1]), float(pose[2]))
    if _pose_clearance(grid, x, y) >= TURN_AIM_M:
        return None
    labels = grid.labels()
    best: tuple[float, float, float] | None = None
    for dist in (0.45, 0.65, 0.85):
        for step in range(12):
            ang = yaw + step * (math.pi / 6.0)
            px = x + dist * math.cos(ang)
            py = y + dist * math.sin(ang)
            mid = grid.cell_of(0.5 * (x + px), 0.5 * (y + py))
            if mid is not None and int(labels[mid]) == 1:
                continue
            got = _pose_clearance(grid, px, py)
            if got < TURN_AIM_M:
                continue
            score = got - 0.15 * dist
            if best is None or score > best[0]:
                best = (score, px, py)
    if best is None:
        return None
    _score, px, py = best
    err = _wrap(math.atan2(py - y, px - x) - yaw)
    if abs(err) > 1.4:
        if motion_blocked(grid, pose, -0.10, 0.0):
            return None
        return (-0.10, 0.0, "wide")
    w = max(-0.40, min(0.40, float(err)))
    if motion_blocked(grid, pose, 0.12, w):
        return None
    return (0.12, w, "wide")


def consume_approach_reject() -> bool:
    """True once after a planned pivot was replaced because the rectangle clips."""
    global _approach_reject
    flag = _approach_reject
    _approach_reject = False
    return flag


def approach_command(
    grid: OccupancyGrid,
    pose: tuple[float, float, float],
    path_xy: list[tuple[float, float]],
) -> tuple[float, float, str] | None:
    """Pivot before a turn or a doorway, then drive the doorway straight.

    None means ordinary pursuit. The pivot happens in the open, before
    the shell is inside the turning radius. A doorway is not an arc.
    """
    global _approach_reject
    _approach_reject = False
    if len(path_xy) < 2:
        return None
    x, y, yaw = (float(pose[0]), float(pose[1]), float(pose[2]))
    nearest = min(range(len(path_xy)), key=lambda i: math.hypot(path_xy[i][0] - x, path_xy[i][1] - y))
    occupied = grid.labels() == 1
    prev = path_xy[max(0, nearest - 1)]
    here = path_xy[nearest]
    travelled = 0.0
    for nxt in path_xy[nearest + 1 :]:
        travelled += math.hypot(nxt[0] - here[0], nxt[1] - here[1])
        opening = _opening_center(grid, occupied, prev, here, nxt)
        # A hall is still a gap a metre past this point. A doorway is not.
        if (
            opening is not None
            and math.hypot(here[0] - x, here[1] - y) <= 1.05
            and not _opening_persists(grid, occupied, opening, 0.85)
        ):
            cx, cy, tx, ty = opening
            along = (x - cx) * tx + (y - cy) * ty
            if along > 0.55:
                return None
            err = _wrap(math.atan2(ty, tx) - yaw)
            if abs(err) <= 0.18:
                return (0.12, 0.0, "straight")
            # Cameras still see the mouth here. A turn starts only with
            # 0.30 m of air. Closer than that, back straight out, or the
            # caller already took the straight crossing above.
            if _pose_clearance(grid, x, y) < TURN_AIM_M:
                return (-0.10, 0.0, "back")
            spin = math.copysign(0.70, err)
            if motion_blocked(grid, pose, 0.0, spin):
                _approach_reject = True
                return (-0.10, 0.0, "back")
            return (0.0, spin, "pivot")
        if travelled > 0.15:
            h_in = math.atan2(here[1] - prev[1], here[0] - prev[0])
            h_out = math.atan2(nxt[1] - here[1], nxt[0] - here[0])
            # A 45° grid step is the path, not a turn. A right angle is a pivot.
            if abs(_wrap(h_out - h_in)) > 1.0:
                dist = math.hypot(here[0] - x, here[1] - y)
                err = _wrap(h_out - yaw)
                if dist < 0.28 and abs(err) > 0.20:
                    if _pose_clearance(grid, x, y) < TURN_AIM_M:
                        wide = wide_escape(grid, pose)
                        if wide is not None:
                            return wide
                        return (-0.10, 0.0, "back")
                    spin = math.copysign(0.70, err)
                    # Centre clearance is a disk. The tire corner sticks out
                    # past it and swings through the turn.
                    if motion_blocked(grid, pose, 0.0, spin):
                        _approach_reject = True
                        return (-0.10, 0.0, "back")
                    return (0.0, spin, "pivot")
        prev, here = here, nxt
        if travelled > 1.1:
            break
    return None


def _unvisited_stand(
    labels: np.ndarray,
    visited: np.ndarray | None,
    res: float,
    here: tuple[int, int],
    rooms: dict | None = None,
) -> tuple[tuple[int, int], float] | None:
    """A stand in free floor the shell has not driven, and that floor's area.

    Depth through a doorway paints the next room free, which removes the
    frontier. Prefer a cell in another region of this robot's own map, at
    least a metre in, so the goal is inside the room rather than on the
    threshold. None when visited is unknown or the patch is only a fringe.
    """
    if visited is None or visited.shape != labels.shape:
        return None
    height, width = labels.shape
    y0, x0 = here
    if not (0 <= y0 < height and 0 <= x0 < width):
        return None
    region = None if rooms is None else rooms.get("labels")
    start_id = _nearby_region(region, y0, x0) if region is not None else 0
    limit = int(8.0 / res)
    inside = int(1.2 / res)
    want = int(2.0 / res)
    queue = [(y0, x0)]
    dist = {(y0, x0): 0}
    head = 0
    other: tuple[int, int] | None = None
    other_miss = want + limit
    unvisited = 0
    while head < len(queue):
        y, x = queue[head]
        head += 1
        d = dist[(y, x)]
        if int(labels[y, x]) == 0 and not bool(visited[y, x]):
            unvisited += 1
        if d >= limit:
            continue
        for ny, nx in ((y + 1, x), (y - 1, x), (y, x + 1), (y, x - 1)):
            if not (0 <= ny < height and 0 <= nx < width) or (ny, nx) in dist:
                continue
            if int(labels[ny, nx]) != 0:
                continue
            dist[(ny, nx)] = d + 1
            queue.append((ny, nx))
            if bool(visited[ny, nx]) or d + 1 < inside:
                continue
            if region is None or start_id <= 0:
                continue
            gid = _nearby_region(region, ny, nx)
            miss = abs((d + 1) - want)
            if gid > 0 and gid != start_id and miss < other_miss:
                other = (ny, nx)
                other_miss = miss
    area = unvisited * res * res
    if other is None or area < 0.8:
        return None
    return other, area


def planner_free_mask(grid: OccupancyGrid) -> np.ndarray:
    """The free mask ``select_goal`` plans on, cached while the grid is unchanged."""
    log = grid.log
    key = (
        id(log),
        int(log.shape[0]),
        round(float(log.sum()), 3),
        int(np.count_nonzero(log >= grid.occ_thresh)),
    )
    if _FREE_CACHE["key"] == key:
        return _FREE_CACHE["mask"]  # type: ignore[return-value]
    mask = traversable(grid.labels(), grid.res, ROBOT_RADIUS_M)
    _FREE_CACHE["key"] = key
    _FREE_CACHE["mask"] = mask
    return mask


def swept_leaves_free(
    grid: OccupancyGrid,
    pose: tuple[float, float, float],
    v_m_s: float,
    w_rad_s: float,
) -> bool:
    """True when the existing sweep carries the centre off the planner mask.

    The sweep distances are the ones ``footprint_clips`` already uses
    (0.35 m of travel, or the spin arc). The mask is the inflated free
    grid the planner searched. A cell the grid has not seen is not a hit.
    Live range remains the check for that space. The rectangle is not
    applied again: it is a second shape, and it is why a path the disk
    accepted was then refused.
    """
    v = float(v_m_s)
    w = float(w_rad_s)
    if abs(v) < 1e-4 and abs(w) < 1e-4:
        return False
    labels = grid.labels()
    free = planner_free_mask(grid)
    x, y, yaw = (float(pose[0]), float(pose[1]), float(pose[2]))
    for px, py, _pyaw in _trajectory(x, y, yaw, v, w):
        cell = grid.cell_of(px, py)
        if cell is None:
            return True
        if bool(free[cell]):
            continue
        if int(labels[cell]) < 0:
            continue
        return True
    return False


def motion_blocked(
    grid: OccupancyGrid,
    pose: tuple[float, float, float],
    v_m_s: float,
    w_rad_s: float,
) -> bool:
    """Map rejection of one commanded arc.

    The rectangle sweep, unless ``RBM_SHARED_FOOTPRINT`` is on. Then the
    planner free mask is the only map test.
    """
    if shared_footprint_enabled():
        return swept_leaves_free(grid, pose, v_m_s, w_rad_s)
    return footprint_clips(grid, pose, v_m_s, w_rad_s)


def _same_free_component(
    free: np.ndarray,
    start: tuple[int, int],
    goal: tuple[int, int],
) -> bool:
    """True when ``goal`` is reachable from ``start`` on the planner mask.

    The neighbour rule matches A*: eight-connected, and a diagonal step
    needs both orthogonal neighbours free.
    """
    if goal == start:
        return True
    height, width = free.shape
    gy, gx = goal
    if not (0 <= gy < height and 0 <= gx < width) or not bool(free[gy, gx]):
        return False
    seen = np.zeros(free.shape, dtype=bool)
    seen[start] = True
    stack = [start]
    while stack:
        y, x = stack.pop()
        for dy, dx, _mul in _NEIGHBORS:
            ny, nx = y + dy, x + dx
            if ny < 0 or nx < 0 or ny >= height or nx >= width or bool(seen[ny, nx]):
                continue
            if not bool(free[ny, nx]):
                continue
            if dy != 0 and dx != 0 and (not bool(free[y + dy, x]) or not bool(free[y, x + dx])):
                continue
            if ny == gy and nx == gx:
                return True
            seen[ny, nx] = True
            stack.append((ny, nx))
    return False


def _frontier_same_component(
    free: np.ndarray,
    start: tuple[int, int],
    group: list[tuple[int, int]],
    labels: np.ndarray,
    res: float,
) -> int:
    """1 when the longest frontier has a stand cell in ``start``'s component."""
    target = viewpoint(free, group, occupied=(labels == 1), res=res, pose=start)
    if target is None:
        for cell in group:
            if bool(free[cell]):
                target = cell
                break
    if target is None:
        return 0
    return 1 if _same_free_component(free, start, target) else 0


def _classify_quit(diag: dict) -> str:
    """One label for a ``select_goal`` miss. Raw counts stay on the line.

    ``start_component`` is a measured split between the snapped start and
    the longest frontier. ``all_banned`` is every scored cluster banned,
    and nothing left past the top-8 cut. ``astar_cap`` is the expansion
    limit. ``cut`` is that top-8 cut dropping clusters before the search.
    Anything else, including a missing start cell, is ``other``.
    """
    if diag.get("return") != "no_candidate":
        return "other"
    if diag.get("same_component") == 0:
        return "start_component"
    kept = int(diag.get("clusters_kept") or 0)
    dropped = int(diag.get("banned_drop") or 0)
    before = int(diag.get("clusters_ge6") or 0)
    fully = kept > 0 and dropped == kept and int(diag.get("astar_attempts") or 0) == 0
    no_cut = before <= kept
    if fully and no_cut:
        return "all_banned"
    if int(diag.get("astar_cap") or 0) > 0:
        return "astar_cap"
    if not no_cut:
        return "cut"
    return "other"


def format_quit(diag: dict) -> str:
    """One parseable line. Missing numbers are ``na``."""

    def field(key: str) -> str:
        value = diag.get(key)
        if value is None:
            return "na"
        if isinstance(value, float):
            return f"{value:.2f}"
        return str(int(value))

    keys = (
        "start_none",
        "jump_cells",
        "jump_m",
        "same_component",
        "raw_cells",
        "clusters_ge6",
        "clusters_kept",
        "banned_drop",
        "banned_cells",
        "banned_regions",
        "viewpoint",
        "astar_attempts",
        "astar_fail",
        "astar_cap",
    )
    parts = [f"{key}={field(key)}" for key in keys]
    parts.append(f"return={diag.get('return') or 'na'}")
    parts.append(f"reason={diag.get('reason') or 'other'}")
    return " ".join(parts)


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
    avoid: list[tuple[float, float, float]] | None = None,
    report: dict | None = None,
    prefer_near: bool = False,
    visited: np.ndarray | None = None,
    turn_aim_m: float | None = None,
    zones: list[tuple[float, float, float]] | None = None,
) -> tuple[tuple[float, float], list[tuple[float, float]]] | None:
    """Best frontier goal and the path to it, in odom metres. None if nothing worth driving to."""
    labels = grid.labels()
    mask = frontier_mask(labels)
    grouped = clusters(mask)
    min_cells = max(1, int(math.ceil(min_frontier_m / grid.res)))
    # On the 5 cm grid this is 6 cells (0.30 m). The top-8 cut sees this list.
    groups = [g for g in grouped if len(g) >= min_cells]
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
    banned_set = set(banned or ())
    diag: dict = {
        "start_none": 1,
        "jump_cells": None,
        "jump_m": None,
        "same_component": None,
        "raw_cells": int(np.count_nonzero(mask)),
        "clusters_ge6": len(groups),
        "clusters_kept": len(ranked),
        "banned_drop": sum(1 for group in ranked if not any(cell not in banned_set for cell in group)),
        "banned_cells": len(banned_set),
        "banned_regions": None,
        "viewpoint": 0,
        "astar_attempts": 0,
        "astar_fail": 0,
        "astar_cap": 0,
        "return": "",
        "reason": "other",
    }
    search_stats = {"cap": 0}

    def _remember_none(fired: str):
        diag["return"] = fired
        diag["astar_cap"] = int(search_stats["cap"])
        if diag["same_component"] is None and start is not None and groups:
            diag["same_component"] = _frontier_same_component(free, start, groups[0], labels, grid.res)
        diag["reason"] = _classify_quit(diag)
        if report is not None:
            report["quit"] = dict(diag)
        return None

    yaw_masks = None
    heading_block = None
    start_yaw = None
    if footprint_align_enabled():
        # Same rectangle as the map veto. The 2.5 / 6.0 clearance weights
        # below are unchanged. The disk radius is not used.
        yaw_masks = rectangle_masks(labels == 1, grid.res)
        heading_block = neighbor_blocks(yaw_masks)
        free = (labels == 0) & ~np.all(heading_block, axis=0)
        if heading_xy is not None:
            start_yaw = math.atan2(float(heading_xy[1]), float(heading_xy[0]))
    else:
        free = traversable(labels, grid.res, radius_m)
    here = grid.cell_of(pose_xy[0], pose_xy[1])
    start = None if here is None else _planner_start(free, labels, here)
    if start is not None and here is not None:
        jump_cells = math.hypot(start[0] - here[0], start[1] - here[1])
        diag["start_none"] = 0
        diag["jump_cells"] = float(jump_cells)
        diag["jump_m"] = float(jump_cells * grid.res)
    if not groups:
        return _remember_none("no_groups")
    if here is None:
        return _remember_none("pose_off_grid")
    if start is None:
        return _remember_none("start_none")
    occupied_now = labels == 1
    clear = clearance_m(occupied_now, grid.res) if np.any(occupied_now) else None
    costs = approach_costs(clear) if clear is not None else None
    costs = apply_zone_cost(costs, grid, zones)
    banned = banned_set
    avoid = list(avoid or [])
    aim_m = TURN_AIM_M if turn_aim_m is None else float(turn_aim_m)
    best: tuple[float, tuple[float, float], list[tuple[float, float]], list] | None = None
    ranked_scores: list[tuple[float, tuple[float, float]]] = []

    def _search(goal_cell: tuple[int, int]):
        diag["astar_attempts"] += 1
        found = astar(
            free,
            start,
            goal_cell,
            step_cost=grid.res,
            costs=costs,
            heading_block=heading_block,
            yaw_masks=yaw_masks,
            start_yaw=start_yaw,
            stats=search_stats,
        )
        if not found:
            diag["astar_fail"] += 1
        return found

    def _keep(
        score: float,
        goal: tuple[float, float],
        pts: list[tuple[float, float]],
        cells: list,
    ) -> None:
        nonlocal best
        if point_in_zones(goal, zones):
            return
        adjusted = score * _region_factor(goal, avoid)
        ranked_scores.append((adjusted, goal))
        if best is None or adjusted > best[0]:
            best = (adjusted, goal, pts, list(cells))

    for group in ranked:
        cells = [c for c in group if c not in banned]
        if not cells:
            continue
        # Prefer a cell the robot can stand on. The score still uses the
        # whole cluster's length, so a long wall beats a speck.
        target = viewpoint(free, cells, occupied=(labels == 1), res=grid.res, pose=start)
        if target is None:
            continue
        diag["viewpoint"] += 1
        # Just failed to move. If the roomiest cell on this frontier is
        # already under the robot, look there. Driving out to the 1–3 m
        # stand-off is how a stalled wheel reading walked the pose along
        # a wall the camera had not left.
        if prefer_near:
            stood = _viewpoint_clearest(free, cells, labels == 1)
            if stood is not None:
                sx, sy = grid.cell_center(*stood)
                if math.hypot(sx - float(pose_xy[0]), sy - float(pose_xy[1])) <= 0.45:
                    target = stood
        # A short frontier with no room to turn is a dead end with nothing
        # worth the trap. A longer one stays, and the drive reverses back out.
        # The 1–3 m stand-off can land in that pocket. The nearest safe cell
        # is tried before the frontier is refused, so a look from the open
        # floor is not dropped because the sweet spot sat in the corner.
        size_m = len(group) * grid.res
        if clear is not None and float(clear[target]) < aim_m and size_m < 0.80:
            safer = _viewpoint_clearest(free, cells, labels == 1)
            nearest = _viewpoint_nearest(free, cells, 6)
            if safer is not None and float(clear[safer]) >= TURN_AIM_M:
                target = safer
            elif nearest is not None and nearest != target and float(clear[nearest]) >= TURN_AIM_M:
                target = nearest
            else:
                if report is not None:
                    xy = grid.cell_center(*target)
                    report.setdefault("refused", []).append(
                        {
                            "xy": (float(xy[0]), float(xy[1])),
                            "reason": "no room to turn",
                            "clear_m": round(float(clear[target]), 2),
                            "size_m": round(size_m, 2),
                        }
                    )
                continue
        path_cells = _search(target)
        if not path_cells:
            # The far stand-off can sit across a wall. The nearest safe
            # cell is still a goal, so a tight doorway is not abandoned.
            nearest = _viewpoint_nearest(free, cells, 6)
            if nearest is None or nearest == target:
                continue
            path_cells = _search(nearest)
            if not path_cells:
                continue
            target = nearest
        length = 0.0
        pts = [grid.cell_center(*rc) for rc in path_cells]
        for a, b in zip(pts, pts[1:]):
            length += math.hypot(b[0] - a[0], b[1] - a[1])
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
        _keep(score, goal, pts, cells)
    # A room already painted free, with the threshold smeared shut, is
    # not a frontier. Score it by its area and aim through the gap.
    for mouth, beyond, area in severed_mouths(labels, grid.res, radius_m, here):
        if mouth in banned:
            continue
        path_cells = _search(mouth)
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
        _keep(score, goal, pts, [mouth])
    stand = _unvisited_stand(labels, visited, grid.res, here, rooms)
    if stand is not None:
        target, area = stand
        if bool(free[target]):
            path_cells = _search(target)
            if path_cells:
                pts = [grid.cell_center(*rc) for rc in path_cells]
                length = 0.0
                for a, b in zip(pts, pts[1:]):
                    length += math.hypot(b[0] - a[0], b[1] - a[1])
                # Capped so the unscanned rest of this room does not beat a
                # real frontier. A path into another region still gets the
                # doorway bonus below.
                score = min(float(area), 4.0) / (length + 0.25) * 0.45
                score *= crossing_bonus(path_cells, rooms, start)
                goal = grid.cell_center(*target)
                _keep(score, goal, pts, [target])
    if best is None:
        return _remember_none("no_candidate")
    pts = line_up_with_openings(grid, best[2])
    pts = hold_turning_clearance(grid, pts, aim_m=aim_m)
    if not pts:
        return _remember_none("lineup_empty")
    if report is not None:
        report["score"] = best[0]
        report["tight"] = _pose_clearance(grid, best[1][0], best[1][1]) < TURN_AIM_M
        report["cells"] = list(best[3])
        report["runner_up"] = None
        report["runner_score"] = None
        ranked_scores.sort(key=lambda item: item[0], reverse=True)
        for scored, goal in ranked_scores:
            if math.hypot(goal[0] - best[1][0], goal[1] - best[1][1]) < 0.25:
                continue
            report["runner_up"] = goal
            report["runner_score"] = scored
            break
    return pts[-1], pts
