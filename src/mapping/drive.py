"""Wheel speeds for a differential drive.

Pure pursuit (Coulter, CMU-RI-TR-92-01, 1992): curvature is
``2 y / L^2`` where y is the goal's lateral offset in the robot frame
and L is the look-ahead. Positive yaw rate is left (counter-clockwise),
matching ``controller_wheels``: ``w = (right - left) * radius / track``.
"""

from __future__ import annotations

import math

from src.mapping.body import CRUISE_M_S, LOOKAHEAD_M, WHEEL_RADIUS_M, WHEEL_TRACK_M


def wheel_speeds(
    v_m_s: float,
    w_rad_s: float,
    *,
    radius_m: float = WHEEL_RADIUS_M,
    track_m: float = WHEEL_TRACK_M,
    max_rad_s: float = 8.0,
) -> tuple[float, float]:
    half = 0.5 * float(track_m)
    left = (float(v_m_s) - float(w_rad_s) * half) / float(radius_m)
    right = (float(v_m_s) + float(w_rad_s) * half) / float(radius_m)
    limit = float(max_rad_s)
    return max(-limit, min(limit, left)), max(-limit, min(limit, right))


def pure_pursuit(
    pose: tuple[float, float, float],
    path_xy: list[tuple[float, float]],
    *,
    lookahead_m: float = LOOKAHEAD_M,
    cruise_m_s: float = CRUISE_M_S,
    arrive_m: float = 0.28,
) -> tuple[float, float, bool]:
    """Return ``(v, w, arrived)``. ``w > 0`` turns left."""
    if not path_xy:
        return 0.0, 0.0, True
    x, y, yaw = pose
    # Follow the polyline. A straight look-ahead would cut the corner
    # and drive the body into a wall the grid path had already avoided.
    nearest = min(range(len(path_xy)), key=lambda i: math.hypot(path_xy[i][0] - x, path_xy[i][1] - y))
    travelled = 0.0
    target = path_xy[-1]
    for left, right in zip(path_xy[nearest:], path_xy[nearest + 1 :]):
        travelled += math.hypot(right[0] - left[0], right[1] - left[1])
        target = right
        if travelled >= lookahead_m:
            break
    dx = target[0] - x
    dy = target[1] - y
    dist = math.hypot(dx, dy)
    if dist <= arrive_m and math.hypot(path_xy[-1][0] - x, path_xy[-1][1] - y) <= arrive_m:
        return 0.0, 0.0, True
    c, s = math.cos(yaw), math.sin(yaw)
    xr = c * dx + s * dy
    yr = -s * dx + c * dy
    if xr < 0.05:
        return 0.0, (0.8 if yr >= 0.0 else -0.8), False
    look = max(lookahead_m, dist)
    w = 2.0 * cruise_m_s * yr / (look * look)
    w = max(-1.1, min(1.1, w))
    v = cruise_m_s * max(0.35, 1.0 - abs(w) / 1.3)
    return v, w, False
