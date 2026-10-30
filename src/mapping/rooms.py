"""Split the robot's own map into rooms and doorways.

Nothing here knows how many rooms a building has. A doorway is a thin
throat about a metre across, or a clearance pinch between two wider
regions. A long narrow strip is a hall, not a door. The rest are
room_1, room_2, ... in reading order.
"""

from __future__ import annotations

from collections import deque

import numpy as np


_DOOR_MIN_M = 0.55
_DOOR_MAX_M = 1.40
_DOOR_LONG_M = 0.80
# One occupied cell is a depth speck. A jamb is a run of wall.
_WALL_CELLS = 3
_MIN_ROOM_M2 = 0.45
# A hall is a long passage about a metre wide. The apartment hall is
# 1.02 m by 1.73 m, so the name follows the passage itself: a region
# that is mostly that strip is a hall. Wider rooms stay rooms.
_HALL_MAX_WIDTH_M = 1.70
_HALL_NAME_WIDTH_M = 1.25


def _axis_spans(free: np.ndarray, occupied: np.ndarray, horizontal: bool):
    if not horizontal:
        free = free.T
        occupied = occupied.T
    height, width = free.shape
    left = np.zeros((height, width), dtype=np.int16)
    right = np.zeros((height, width), dtype=np.int16)
    left_occ = np.zeros((height, width), dtype=bool)
    right_occ = np.zeros((height, width), dtype=bool)
    for row in range(height):
        span = 0
        run = 0
        support = 0
        for col in range(width):
            if free[row, col]:
                if span == 0:
                    support = run
                span += 1
                left[row, col] = span
                left_occ[row, col] = support >= _WALL_CELLS
                run = 0
            elif occupied[row, col]:
                span = 0
                run += 1
            else:
                span = 0
                run = 0
        span = 0
        run = 0
        support = 0
        for col in range(width - 1, -1, -1):
            if free[row, col]:
                if span == 0:
                    support = run
                span += 1
                right[row, col] = span
                right_occ[row, col] = support >= _WALL_CELLS
                run = 0
            elif occupied[row, col]:
                span = 0
                run += 1
            else:
                span = 0
                run = 0
    if not horizontal:
        return left.T, right.T, left_occ.T, right_occ.T
    return left, right, left_occ, right_occ


def _narrow(
    free: np.ndarray,
    occupied: np.ndarray,
    res: float,
    horizontal: bool,
    max_width_m: float,
) -> np.ndarray:
    left, right, left_occ, right_occ = _axis_spans(free, occupied, horizontal)
    width_m = (left.astype(np.float64) + right.astype(np.float64) - 1.0) * res
    return free & left_occ & right_occ & (width_m >= _DOOR_MIN_M) & (width_m <= max_width_m)


def _span_cells(cells: list[tuple[int, int]]) -> int:
    rows = [c[0] for c in cells]
    cols = [c[1] for c in cells]
    return max(max(rows) - min(rows), max(cols) - min(cols))


def _passage_components(labels: np.ndarray, resolution_m: float, max_width_m: float):
    free = labels == 0
    occupied = labels == 1
    if not np.any(free) or not np.any(occupied):
        return []
    res = float(resolution_m)
    narrow = _narrow(free, occupied, res, True, max_width_m) | _narrow(free, occupied, res, False, max_width_m)
    return _components(narrow)


def _components(mask: np.ndarray) -> list[list[tuple[int, int]]]:
    seen = np.zeros(mask.shape, dtype=bool)
    found: list[list[tuple[int, int]]] = []
    height, width = mask.shape
    ys, xs = np.nonzero(mask)
    for y0, x0 in zip(ys.tolist(), xs.tolist()):
        if seen[y0, x0]:
            continue
        stack = [(int(y0), int(x0))]
        seen[y0, x0] = True
        cells: list[tuple[int, int]] = []
        while stack:
            y, x = stack.pop()
            cells.append((y, x))
            for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                ny, nx = y + dy, x + dx
                if ny < 0 or nx < 0 or ny >= height or nx >= width:
                    continue
                if seen[ny, nx] or not mask[ny, nx]:
                    continue
                seen[ny, nx] = True
                stack.append((ny, nx))
        found.append(cells)
    return found


def _extent_m(cells: list[tuple[int, int]], res: float) -> tuple[float, float]:
    rows = [c[0] for c in cells]
    cols = [c[1] for c in cells]
    height = (max(rows) - min(rows) + 1) * res
    width = (max(cols) - min(cols) + 1) * res
    return (height, width) if height <= width else (width, height)


def _clearance_cells(free: np.ndarray) -> np.ndarray:
    """4-connected distance in cells from each free cell to a non-free cell."""
    height, width = free.shape
    dist = np.full((height, width), 10**6, dtype=np.int32)
    queue: deque[tuple[int, int]] = deque()
    ys, xs = np.nonzero(~free)
    for y, x in zip(ys.tolist(), xs.tolist()):
        dist[y, x] = 0
        queue.append((int(y), int(x)))
    while queue:
        y, x = queue.popleft()
        base = int(dist[y, x])
        for ny, nx in ((y + 1, x), (y - 1, x), (y, x + 1), (y, x - 1)):
            if ny < 0 or nx < 0 or ny >= height or nx >= width:
                continue
            if dist[ny, nx] <= base + 1:
                continue
            dist[ny, nx] = base + 1
            queue.append((ny, nx))
    return dist


def _pinch_doors(labels: np.ndarray, resolution_m: float) -> np.ndarray:
    """Where two wider regions meet through a passage narrower than about 0.9 m.

    The axis-aligned gap test misses a throat whose narrow cells run
    together along the opening and look long. Clearance still dips there:
    the waves grown from the two regions meet in that dip.
    """
    free = labels == 0
    door = np.zeros(free.shape, dtype=bool)
    if int(np.count_nonzero(free)) < 8:
        return door
    res = float(resolution_m)
    dist = _clearance_cells(free)
    neck = max(2, int(round(0.46 / res)))
    pieces = [cells for cells in _components(free & (dist >= neck)) if len(cells) >= 8]
    if len(pieces) < 2:
        return door
    nearest = np.zeros(free.shape, dtype=np.int16)
    reach = np.full(free.shape, 9999, dtype=np.int16)
    queue: deque[tuple[int, int]] = deque()
    for index, cells in enumerate(pieces, start=1):
        for y, x in cells:
            nearest[y, x] = index
            reach[y, x] = 0
            queue.append((y, x))
    height, width = free.shape
    while queue:
        y, x = queue.popleft()
        cid = int(nearest[y, x])
        base = int(reach[y, x])
        for ny, nx in ((y + 1, x), (y - 1, x), (y, x + 1), (y, x - 1)):
            if ny < 0 or nx < 0 or ny >= height or nx >= width or not free[ny, nx]:
                continue
            other = int(nearest[ny, nx])
            step = base + 1
            if other == 0:
                nearest[ny, nx] = cid
                reach[ny, nx] = step
                queue.append((ny, nx))
            elif other != cid and int(dist[ny, nx]) < neck:
                door[ny, nx] = True
                if int(dist[y, x]) < neck:
                    door[y, x] = True
    # The meeting line of two wide regions can wander into the room and
    # mark a blob a metre across. A doorway is the short throat only.
    keep = np.zeros(free.shape, dtype=bool)
    for cells in _components(door):
        minor, major = _extent_m(cells, res)
        if minor <= 0.55 and 0.45 <= major <= 1.35:
            for y, x in cells:
                keep[y, x] = True
    return keep


def doorway_mask(labels: np.ndarray, resolution_m: float) -> np.ndarray:
    """Thin gaps between occupied cells, plus clearance pinches.

    A corridor is narrow for a long run, so its short side is the
    passage width. A doorway is the opposite: thin along the wall and
    about a metre across. The 0.80 m cap on the long side rejected an
    0.81 m opening once the narrow cells ran the full width of the gap.
    """
    door = np.zeros(labels.shape, dtype=bool)
    res = float(resolution_m)
    for cells in _passage_components(labels, res, _DOOR_MAX_M):
        minor, major = _extent_m(cells, res)
        if minor <= 0.50 and 0.50 <= major <= 1.50:
            for y, x in cells:
                door[y, x] = True
    door |= _pinch_doors(labels, res)
    return door


def hall_mask(labels: np.ndarray, resolution_m: float) -> np.ndarray:
    """Long narrow passages. The same width as a door, but too long to be one."""
    hall = np.zeros(labels.shape, dtype=bool)
    res = float(resolution_m)
    limit = _DOOR_LONG_M / res
    name_cells = _HALL_NAME_WIDTH_M / res
    for cells in _passage_components(labels, res, _HALL_MAX_WIDTH_M):
        if _span_cells(cells) <= limit:
            continue
        rows = [c[0] for c in cells]
        cols = [c[1] for c in cells]
        short = min(max(rows) - min(rows), max(cols) - min(cols))
        if short > name_cells:
            continue
        for y, x in cells:
            hall[y, x] = True
    return hall


def _mouths(free: np.ndarray, hall: np.ndarray, resolution_m: float) -> np.ndarray:
    """Short openings where a passage meets a wider room.

    A hall cell that touches wider free space is a candidate. A run of
    those cells longer than a doorway is the side of the passage, not a
    door, and marking it would make every path look like a crossing.
    """
    if not np.any(hall):
        return np.zeros(hall.shape, dtype=bool)
    touch = np.zeros(hall.shape, dtype=bool)
    touch[1:, :] |= free[:-1, :] & ~hall[:-1, :]
    touch[:-1, :] |= free[1:, :] & ~hall[1:, :]
    touch[:, 1:] |= free[:, :-1] & ~hall[:, :-1]
    touch[:, :-1] |= free[:, 1:] & ~hall[:, 1:]
    raw = hall & touch
    keep = np.zeros(hall.shape, dtype=bool)
    limit = _DOOR_MAX_M / float(resolution_m)
    for cells in _components(raw):
        if _span_cells(cells) <= limit:
            for y, x in cells:
                keep[y, x] = True
    return keep


def segment_map(labels: np.ndarray, resolution_m: float) -> dict:
    """Region ids (0 = none) and names. Door cells are removed before the fill."""
    shape = labels.shape
    hall = hall_mask(labels, resolution_m)
    door = doorway_mask(labels, resolution_m) | _mouths(labels == 0, hall, resolution_m)
    free = (labels == 0) & ~door
    # One cell of doorway is enough to split a 5 cm grid. Dilate so a
    # diagonal leak does not join the two rooms.
    cut = door.copy()
    cut[1:, :] |= door[:-1, :]
    cut[:-1, :] |= door[1:, :]
    cut[:, 1:] |= door[:, :-1]
    cut[:, :-1] |= door[:, 1:]
    free &= ~cut
    res = float(resolution_m)
    min_cells = max(4, int(_MIN_ROOM_M2 / (res * res)))
    pieces = [cells for cells in _components(free) if len(cells) >= min_cells]
    pieces.sort(key=lambda cells: (min(c[0] for c in cells), min(c[1] for c in cells)))
    region = np.zeros(shape, dtype=np.int16)
    stats = []
    for index, cells in enumerate(pieces, start=1):
        rows = np.array([c[0] for c in cells])
        cols = np.array([c[1] for c in cells])
        for y, x in cells:
            region[y, x] = index
        height_m = (float(rows.max() - rows.min()) + 1.0) * res
        width_m = (float(cols.max() - cols.min()) + 1.0) * res
        short = max(res, min(height_m, width_m))
        aspect = max(height_m, width_m) / short
        stats.append((index, aspect, short, len(cells)))
    names: dict[int, str] = {}
    hall_n = 0
    room_n = 0
    for index, aspect, short, count in stats:
        rows, cols = np.nonzero(region == index)
        on_spine = int(np.count_nonzero(hall[rows, cols])) if rows.size else 0
        passage = on_spine * 2 >= count and short <= _HALL_NAME_WIDTH_M
        elongated = aspect >= 2.4 and short <= _HALL_MAX_WIDTH_M and len(stats) > 1
        if passage or elongated:
            hall_n += 1
            names[index] = "hall" if hall_n == 1 else f"hall_{hall_n}"
        else:
            room_n += 1
            names[index] = f"room_{room_n}"
    return {"labels": region, "doors": door, "names": names}


def crossing_bonus(
    path_cells: list[tuple[int, int]],
    rooms: dict | None,
    start_rc: tuple[int, int] | None,
) -> float:
    """Higher when the path leaves the region the robot is standing in.

    A door-shaped speck inside that region is not a new place. Scoring
    every path that touched one made the long frontier in the same room
    beat the opening the robot was already facing.
    """
    if not rooms or not path_cells:
        return 1.0
    doors: np.ndarray = rooms["doors"]
    region: np.ndarray = rooms["labels"]
    crossed = False
    for y, x in path_cells:
        if 0 <= y < doors.shape[0] and 0 <= x < doors.shape[1] and doors[y, x]:
            crossed = True
            break
    if start_rc is None:
        return 3.0 if crossed else 1.0
    sy, sx = start_rc
    gy, gx = path_cells[-1]
    if not (0 <= sy < region.shape[0] and 0 <= sx < region.shape[1]):
        return 3.0 if crossed else 1.0
    if not (0 <= gy < region.shape[0] and 0 <= gx < region.shape[1]):
        return 3.0 if crossed else 1.0
    # Door cells are cut out of the regions, so the robot often stands
    # on an unlabeled cell in the threshold. The room it just left is the
    # nearest labelled cell, not "no room".
    start_id = _nearby_region(region, sy, sx)
    goal_id = _nearby_region(region, gy, gx)
    if start_id > 0 and goal_id > 0 and start_id != goal_id:
        return 6.0
    if start_id > 0 and goal_id == 0 and crossed:
        # Far side is not a labelled room yet. That is a new place.
        return 6.0
    return 1.0


def _nearby_region(region: np.ndarray, row: int, col: int) -> int:
    """Region id at this cell, or the closest labelled cell within 0.25 m."""
    height, width = region.shape
    if not (0 <= row < height and 0 <= col < width):
        return 0
    if int(region[row, col]) > 0:
        return int(region[row, col])
    for radius in range(1, 6):
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if max(abs(dy), abs(dx)) != radius:
                    continue
                ny, nx = row + dy, col + dx
                if 0 <= ny < height and 0 <= nx < width and int(region[ny, nx]) > 0:
                    return int(region[ny, nx])
    return 0
