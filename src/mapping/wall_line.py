"""Yaw from a depth wall compared with the wall already on the map.

The depth cloud has a line. Occupied cells next to that cloud have a
line. The angle between them is a heading error. Nothing here assumes
the apartment's walls sit on a compass direction: a diagonal wall is a
diagonal on both sides, and only the difference is used. A line that
does not agree with the cells beside it is ignored, so a new wall the
robot has not mapped yet does not pull the heading.
"""

from __future__ import annotations

import math

import numpy as np

from src.mapping.grid import OccupancyGrid


def _fold(angle: float) -> float:
    return (float(angle) + math.pi / 2.0) % math.pi - math.pi / 2.0


def _circ_mean(angles: np.ndarray) -> float:
    doubled = 2.0 * np.asarray(angles, dtype=np.float64)
    return 0.5 * math.atan2(float(np.sin(doubled).mean()), float(np.cos(doubled).mean()))


def dominant_line_angle(points: np.ndarray, *, min_points: int = 10) -> float | None:
    """Direction of the strongest line, folded into (-pi/2, pi/2].

    ``None`` when the points are too few or too scattered to be one wall.
    """
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    if pts.shape[0] < min_points:
        return None
    if pts.shape[0] > 70:
        pick = np.linspace(0, pts.shape[0] - 1, 70).astype(np.int32)
        pts = pts[pick]
    delta = pts[:, None, :] - pts[None, :, :]
    dist = np.hypot(delta[..., 0], delta[..., 1])
    keep = np.triu(np.ones(dist.shape, dtype=bool), 1) & (dist > 0.06) & (dist < 0.45)
    if int(keep.sum()) < 20:
        return None
    angles = np.arctan2(delta[..., 1], delta[..., 0])
    angles = (angles + np.pi / 2.0) % np.pi - np.pi / 2.0
    samples = angles[keep]
    bins = 36
    hist, edges = np.histogram(samples, bins=bins, range=(-np.pi / 2.0, np.pi / 2.0))
    peak = int(np.argmax(hist))
    center = 0.5 * (float(edges[peak]) + float(edges[peak + 1]))
    offset = (samples - center + np.pi / 2.0) % np.pi - np.pi / 2.0
    close = np.abs(offset) <= (np.pi / bins) * 1.6
    if int(close.sum()) < 20 or float(close.mean()) < 0.30:
        return None
    return _fold(_circ_mean(samples[close]))


def _occupied_near(grid: OccupancyGrid, world_xy: np.ndarray, radius_m: float) -> np.ndarray:
    labels = grid.labels()
    res = grid.res
    reach = int(math.ceil(radius_m / res))
    seen: set[tuple[int, int]] = set()
    found: list[tuple[float, float]] = []
    r2 = float(radius_m) * float(radius_m)
    for x, y in world_xy:
        cell = grid.cell_of(float(x), float(y))
        if cell is None:
            continue
        row, col = cell
        for dr in range(-reach, reach + 1):
            for dc in range(-reach, reach + 1):
                rr, cc = row + dr, col + dc
                if (rr, cc) in seen:
                    continue
                if rr < 0 or cc < 0 or rr >= labels.shape[0] or cc >= labels.shape[1]:
                    continue
                if int(labels[rr, cc]) != 1:
                    continue
                cx, cy = grid.cell_center(rr, cc)
                if (cx - float(x)) ** 2 + (cy - float(y)) ** 2 > r2:
                    continue
                seen.add((rr, cc))
                found.append((cx, cy))
    if not found:
        return np.zeros((0, 2))
    return np.asarray(found, dtype=np.float64)


def wall_yaw_delta(
    grid: OccupancyGrid,
    pose: tuple[float, float, float],
    hits_body: np.ndarray,
) -> float:
    """Small yaw to add so the depth line lies on the mapped line.

    Zero when either line is missing, or when they disagree by more than
    the search window. One call is capped so a bad fit cannot spin the pose.
    """
    pts = np.asarray(hits_body, dtype=np.float64).reshape(-1, 2)
    if pts.shape[0] < 12:
        return 0.0
    body_angle = dominant_line_angle(pts)
    if body_angle is None:
        return 0.0
    x, y, yaw = (float(v) for v in pose)
    c, s = math.cos(yaw), math.sin(yaw)
    world = np.column_stack((x + c * pts[:, 0] - s * pts[:, 1], y + s * pts[:, 0] + c * pts[:, 1]))
    cells = _occupied_near(grid, world, 0.40)
    map_angle = dominant_line_angle(cells)
    if map_angle is None:
        return 0.0
    # A handful of cells on a 5 cm lattice can sit a few degrees off the
    # true line. The opening spin paints exactly that, and three nudges
    # then bake the error in before a longer wall exists to contradict it.
    span = max(
        float(cells[:, 0].max() - cells[:, 0].min()),
        float(cells[:, 1].max() - cells[:, 1].min()),
    )
    if span < 0.90:
        return 0.0
    error = _fold(map_angle - (yaw + body_angle))
    # Occupied cells sit on a 5 cm lattice, so a true line and the cells
    # under it differ by a couple of degrees. That is not heading error.
    if abs(error) < 0.05 or abs(error) > 0.30:
        return 0.0
    return max(-0.012, min(0.012, 0.45 * error))
