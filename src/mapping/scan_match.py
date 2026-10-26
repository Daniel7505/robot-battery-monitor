"""Align a depth scan to the occupancy grid.

Correlative scan matching (Olson, ICRA 2009): try rigid poses near the
odometry pose and keep the one whose returns land on occupied cells.
``pose_correction`` searches translation and yaw. A hypothesis is applied
only when it beats the current pose by a margin that grows with the size
of the move, and the applied step is capped so one frame cannot teleport
the robot. ``yaw_nudge`` is the older yaw-only trim. The inertial unit's
yaw is not used: in this simulator it is a world heading.
"""

from __future__ import annotations

import math

import numpy as np

from src.mapping.grid import OccupancyGrid

_OFFSETS = (-0.08, -0.06, -0.04, -0.02, 0.02, 0.04, 0.06, 0.08)


def yaw_nudge(
    grid: OccupancyGrid,
    pose: tuple[float, float, float],
    hits_odom: np.ndarray,
    *,
    labels: np.ndarray | None = None,
) -> float:
    """Return a yaw correction in radians, or 0 when the map cannot say."""
    pts = np.asarray(hits_odom, dtype=np.float64).reshape(-1, 2)
    if pts.shape[0] < 25:
        return 0.0
    step = max(1, pts.shape[0] // 60)
    pts = pts[::step]
    drawn = labels if labels is not None else grid.labels()
    x, y, _yaw = pose
    base = _score(grid, drawn, x, y, 0.0, pts)
    if base < 8:
        return 0.0
    best_off = 0.0
    best = base
    for off in _OFFSETS:
        score = _score(grid, drawn, x, y, off, pts)
        if score > best:
            best = score
            best_off = off
    # A far hypothesis is not applied in one step. The next scan can take
    # a small step if the offset is still there. This stops a biased score
    # from walking the heading while the robot is wedged.
    if abs(best_off) > 0.04 or best < base * 1.2 + 3.0:
        return 0.0
    return max(-0.008, min(0.008, 0.5 * best_off))


# Neighbourhood searched when deciding whether the latest scan still
# belongs at the odometry pose. Radii are metres.
_SHIFTS = tuple(
    (radius * math.cos(math.radians(deg)), radius * math.sin(math.radians(deg)))
    for radius in (0.30, 0.55, 0.80)
    for deg in range(0, 360, 45)
)


def scan_fit(grid: OccupancyGrid, hits_odom: np.ndarray) -> str:
    """How the latest scan sits on the map: support, disagree, or unknown.

    The scan is shifted rigidly. Support means the odometry pose already
    explains it. Disagree means a shift of at least 0.3 m explains it
    much better, so recent wheel travel should not be allowed to finish
    the run. Too few returns, or a map with nothing to match, is unknown.
    """
    pts = np.asarray(hits_odom, dtype=np.float64).reshape(-1, 2)
    if pts.shape[0] < 18:
        return "unknown"
    step = max(1, pts.shape[0] // 60)
    pts = pts[::step]
    labels = grid.labels()
    if int(np.count_nonzero(labels == 1)) < 8:
        return "unknown"
    base = _score_points(grid, labels, pts)
    best = base
    best_d = 0.0
    for dx, dy in _SHIFTS:
        score = _score_points(grid, labels, pts + (dx, dy))
        if score > best:
            best = score
            best_d = math.hypot(dx, dy)
    if best_d >= 0.30 and best >= 10 and best >= base * 1.35 + 4.0:
        return "disagree"
    if base >= 8 and best < base * 1.35 + 4.0:
        return "support"
    return "unknown"


# Search window. A correction bigger than this belongs to a different
# wall, not to the pose that built the map. The applied step is smaller
# still, so one frame cannot jump a cell.
_YAW_LIM = 0.08
_XY_LIM = 0.16
_STEP_M = 0.05
_STEP_YAW = 0.02
_MIN_HITS = 20
_MIN_OCC = 12


def pose_correction(
    grid: OccupancyGrid,
    pose: tuple[float, float, float],
    hits_body: np.ndarray,
) -> tuple[float, float, float]:
    """Return ``(dx, dy, dyaw)`` in the odometry frame, or zeros.

    ``hits_body`` are robot-frame xy returns. The correction is added to
    the odometry pose. Zeros mean the map cannot beat the current pose
    cleanly enough to move.
    """
    pts = np.asarray(hits_body, dtype=np.float64).reshape(-1, 2)
    if pts.shape[0] < _MIN_HITS:
        return (0.0, 0.0, 0.0)
    if pts.shape[0] > 64:
        pick = np.linspace(0, pts.shape[0] - 1, 64).astype(np.int32)
        pts = pts[pick]
    labels = grid.labels()
    if int(np.count_nonzero(labels == 1)) < _MIN_OCC:
        return (0.0, 0.0, 0.0)
    field = _match_field(labels)
    x, y, yaw = (float(v) for v in pose)
    xb = np.ascontiguousarray(pts[:, 0])
    yb = np.ascontiguousarray(pts[:, 1])
    base = float(_score_poses(field, grid, x, y, yaw, np.zeros(1), np.zeros(1), np.zeros(1), xb, yb)[0])

    yaws = np.linspace(-_YAW_LIM, _YAW_LIM, 5)
    offs = np.linspace(-_XY_LIM, _XY_LIM, 7)
    dyaw_g, dx_g, dy_g = np.meshgrid(yaws, offs, offs, indexing="ij")
    dx_g = dx_g.ravel()
    dy_g = dy_g.ravel()
    dyaw_g = dyaw_g.ravel()
    coarse = _score_poses(field, grid, x, y, yaw, dx_g, dy_g, dyaw_g, xb, yb)
    # A straight wall scores the same anywhere along its length. Keep the
    # pose on that ridge closest to the odometry pose, so the match does
    # not slide the robot along a wall it already explains.
    win = _on_ridge(dx_g, dy_g, dyaw_g, coarse)
    # A peak on the window edge means the overlap is already gone, or the
    # score climbs without a mode. Stepping that way walks the pose off
    # the map. Wait until the error sits inside the window.
    if _on_edge(float(dx_g[win]), float(dy_g[win]), float(dyaw_g[win])):
        return (0.0, 0.0, 0.0)

    span_x = np.linspace(float(dx_g[win]) - 0.05, float(dx_g[win]) + 0.05, 5)
    span_y = np.linspace(float(dy_g[win]) - 0.05, float(dy_g[win]) + 0.05, 5)
    span_w = np.linspace(float(dyaw_g[win]) - 0.03, float(dyaw_g[win]) + 0.03, 5)
    dyaw_f, dx_f, dy_f = np.meshgrid(span_w, span_x, span_y, indexing="ij")
    dx_f = dx_f.ravel()
    dy_f = dy_f.ravel()
    dyaw_f = dyaw_f.ravel()
    fine = _score_poses(field, grid, x, y, yaw, dx_f, dy_f, dyaw_f, xb, yb)
    fwin = _on_ridge(dx_f, dy_f, dyaw_f, fine)
    dx, dy, dyaw = _blend(dx_f, dy_f, dyaw_f, fine, fwin)
    dist = math.hypot(dx, dy)
    if dist > _STEP_M:
        scale = _STEP_M / dist
        dx *= scale
        dy *= scale
    dyaw = max(-_STEP_YAW, min(_STEP_YAW, dyaw))
    # The step actually applied has to beat the current pose on its own.
    # A far peak with a valley in front of it is not a correction.
    applied = float(
        _score_poses(
            field,
            grid,
            x,
            y,
            yaw,
            np.array([dx]),
            np.array([dy]),
            np.array([dyaw]),
            xb,
            yb,
        )[0]
    )
    if not _accept(base, applied, dx, dy, dyaw):
        return (0.0, 0.0, 0.0)
    if math.hypot(dx, dy) < 0.02 and abs(dyaw) < 0.008:
        return (0.0, 0.0, 0.0)
    return (dx, dy, dyaw)


def _on_edge(dx: float, dy: float, dyaw: float) -> bool:
    return abs(dx) >= _XY_LIM - 0.02 or abs(dy) >= _XY_LIM - 0.02 or abs(dyaw) >= _YAW_LIM - 0.02


def _accept(base: float, best: float, dx: float, dy: float, dyaw: float) -> bool:
    # The current pose has to already overlap the map. A search that only
    # scores well far away is a different surface.
    if best < 36.0 or base < 24.0:
        return False
    dist = math.hypot(dx, dy)
    yaw = abs(dyaw)
    need = 14.0 + 80.0 * dist + 140.0 * yaw
    need = max(need, 0.18 * base + 40.0 * dist + 80.0 * yaw)
    return best >= base + need


def _on_ridge(dx: np.ndarray, dy: np.ndarray, dyaw: np.ndarray, scores: np.ndarray) -> int:
    """Index of the high-scoring pose closest to the current pose."""
    best = float(scores.max())
    motion = np.hypot(dx, dy) + 0.5 * np.abs(dyaw)
    motion = np.where(scores >= best - 4.0, motion, np.inf)
    return int(np.argmin(motion))


def _blend(
    dx: np.ndarray,
    dy: np.ndarray,
    dyaw: np.ndarray,
    scores: np.ndarray,
    win: int,
) -> tuple[float, float, float]:
    """Average the poses that sit on the winning peak."""
    best = float(scores[win])
    weight = np.clip(scores - (best - 6.0), 0.0, None)
    dist = np.hypot(dx - dx[win], dy - dy[win]) + 0.4 * np.abs(dyaw - dyaw[win])
    weight[dist > 0.12] = 0.0
    total = float(weight.sum())
    if total <= 0.0:
        return float(dx[win]), float(dy[win]), float(dyaw[win])
    weight /= total
    return float(weight @ dx), float(weight @ dy), float(weight @ dyaw)


def _match_field(labels: np.ndarray) -> np.ndarray:
    """Occupied cells score 4, the next ring 2, the one after that 1.

    The ring keeps a hypothesis that lands half a cell off the wall in
    the basin, so the coarse grid can see it.
    """
    occ = labels == 1
    field = np.zeros(labels.shape, dtype=np.float32)
    field[occ] = 4.0
    ring1 = _dilate(occ)
    ring2 = _dilate(ring1)
    field[ring1 & (field == 0.0)] = 2.0
    field[ring2 & (field == 0.0)] = 1.0
    return field


def _dilate(mask: np.ndarray) -> np.ndarray:
    padded = np.pad(mask, 1, constant_values=False)
    return (
        mask
        | padded[:-2, 1:-1]
        | padded[2:, 1:-1]
        | padded[1:-1, :-2]
        | padded[1:-1, 2:]
        | padded[:-2, :-2]
        | padded[:-2, 2:]
        | padded[2:, :-2]
        | padded[2:, 2:]
    )


def _score_poses(
    field: np.ndarray,
    grid: OccupancyGrid,
    x: float,
    y: float,
    yaw: float,
    dx: np.ndarray,
    dy: np.ndarray,
    dyaw: np.ndarray,
    xb: np.ndarray,
    yb: np.ndarray,
) -> np.ndarray:
    c = np.cos(yaw + dyaw)
    s = np.sin(yaw + dyaw)
    wx = (x + dx)[:, None] + c[:, None] * xb[None, :] - s[:, None] * yb[None, :]
    wy = (y + dy)[:, None] + s[:, None] * xb[None, :] + c[:, None] * yb[None, :]
    cols = np.floor((wx - grid.origin[0]) / grid.res).astype(np.int32)
    rows = np.floor((wy - grid.origin[1]) / grid.res).astype(np.int32)
    inside = (rows >= 0) & (cols >= 0) & (rows < grid.n) & (cols < grid.n)
    rows = np.clip(rows, 0, grid.n - 1)
    cols = np.clip(cols, 0, grid.n - 1)
    gathered = field[rows, cols]
    gathered *= inside
    return gathered.sum(axis=1)


def _score_points(grid: OccupancyGrid, labels: np.ndarray, pts: np.ndarray) -> int:
    cols = np.floor((pts[:, 0] - grid.origin[0]) / grid.res).astype(np.int32)
    rows = np.floor((pts[:, 1] - grid.origin[1]) / grid.res).astype(np.int32)
    inside = (rows >= 0) & (cols >= 0) & (rows < grid.n) & (cols < grid.n)
    if not np.any(inside):
        return 0
    return int(np.count_nonzero(labels[rows[inside], cols[inside]] == 1))


def _score(
    grid: OccupancyGrid,
    labels: np.ndarray,
    x: float,
    y: float,
    yaw_off: float,
    pts: np.ndarray,
) -> int:
    c, s = math.cos(yaw_off), math.sin(yaw_off)
    dx = pts[:, 0] - x
    dy = pts[:, 1] - y
    rx = x + c * dx - s * dy
    ry = y + s * dx + c * dy
    cols = np.floor((rx - grid.origin[0]) / grid.res).astype(np.int32)
    rows = np.floor((ry - grid.origin[1]) / grid.res).astype(np.int32)
    inside = (rows >= 0) & (cols >= 0) & (rows < grid.n) & (cols < grid.n)
    if not np.any(inside):
        return 0
    return int(np.count_nonzero(labels[rows[inside], cols[inside]] == 1))
