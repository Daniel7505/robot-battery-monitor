"""Small yaw alignment of a depth scan to the occupancy grid.

A few degree hypotheses around the current heading, scored by how many
returns land on occupied cells. This is the brute-force core of
correlative scan matching (Olson, ICRA 2009), limited to yaw and to a
tenth of a radian so a bad score cannot spin the pose. The inertial
unit's yaw is not used: in this simulator it is a world heading.
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
