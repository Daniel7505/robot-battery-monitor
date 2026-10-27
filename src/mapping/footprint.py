"""Sweep the body along a commanded arc before the wheels take it.

The check is the robot's own occupancy grid. A cell counts only when
this robot has already marked it occupied. Unknown space is not a wall,
and cells the shell already covers are ignored so a latched contact
under a tire does not freeze the next move. Ground truth is not read.
"""

from __future__ import annotations

import math

from src.mapping.bubble import SHELL_HALF_M, SHELL_NOSE_M, SHELL_TAIL_M
from src.mapping.grid import OccupancyGrid
from src.mapping.odometry import wrap_rad

# A few centimetres outside the 0.39 m shell. This is not the planner's
# 0.20 m inflation: that disk would refuse a centred 0.81 m doorway.
# One centimetre was not enough to see a corner swing into a jamb, and
# three centimetres on a 5 cm cell stopped straight passes that only
# grazed the next cell. Two centimetres leaves the centred 0.81 m door open.
FOOTPRINT_MARGIN_M = 0.02
NOSE_M = SHELL_NOSE_M + FOOTPRINT_MARGIN_M
TAIL_M = SHELL_TAIL_M + FOOTPRINT_MARGIN_M
HALF_M = SHELL_HALF_M + FOOTPRINT_MARGIN_M
# A half-metre horizon stopped for a wall the next step would have
# rechecked. One shell length ahead still catches the nose swinging
# into a jamb, which is the corner cut.
SWEEP_AHEAD_M = 0.35
SWEEP_SPIN_RAD = 0.50
STEP_M = 0.05
STEP_SPIN_RAD = 0.10
_REACH_M = math.hypot(max(NOSE_M, TAIL_M), HALF_M)


def footprint_clips(
    grid: OccupancyGrid,
    pose: tuple[float, float, float],
    v_m_s: float,
    w_rad_s: float,
) -> bool:
    """True when the shell along this arc enters an occupied cell."""
    v = float(v_m_s)
    w = float(w_rad_s)
    if abs(v) < 1e-4 and abs(w) < 1e-4:
        return False
    x, y, yaw = (float(pose[0]), float(pose[1]), float(pose[2]))
    ignore = _covered(grid, x, y, yaw)
    for px, py, pyaw in _trajectory(x, y, yaw, v, w):
        if _hits_new_obstacle(grid, px, py, pyaw, ignore):
            return True
    return False


def fit_motion(
    grid: OccupancyGrid,
    pose: tuple[float, float, float],
    v_m_s: float,
    w_rad_s: float,
    target_yaw: float,
) -> tuple[float, float, str]:
    """Return a motion whose sweep misses occupied cells.

    The kind is ``clear`` when the request was already safe, ``wider``
    when a gentler arc is, ``straight`` when only a straight move is,
    ``align`` when the shell has to spin toward ``target_yaw`` first,
    and ``stop`` when every one of those clips.
    """
    v = float(v_m_s)
    w = float(w_rad_s)
    if not footprint_clips(grid, pose, v, w):
        return v, w, "clear"
    for scale in (0.5, 0.25):
        w2 = w * scale
        if not footprint_clips(grid, pose, v, w2):
            return v, w2, "wider"
    if abs(v) > 1e-4:
        v_straight = math.copysign(min(abs(v), 0.12), v)
        if not footprint_clips(grid, pose, v_straight, 0.0):
            return v_straight, 0.0, "straight"
    err = wrap_rad(float(target_yaw) - float(pose[2]))
    if abs(err) > 0.08:
        w_spin = max(-0.55, min(0.55, math.copysign(0.5, err)))
        if not footprint_clips(grid, pose, 0.0, w_spin):
            return 0.0, w_spin, "align"
    return 0.0, 0.0, "stop"


def _trajectory(
    x: float,
    y: float,
    yaw: float,
    v: float,
    w: float,
):
    if abs(v) >= 1e-3:
        n = max(1, int(round(SWEEP_AHEAD_M / STEP_M)))
        dt = (SWEEP_AHEAD_M / abs(v)) / n
    else:
        n = max(1, int(round(SWEEP_SPIN_RAD / STEP_SPIN_RAD)))
        dt = (SWEEP_SPIN_RAD / abs(w)) / n
    for _ in range(n):
        x, y, yaw = _advance(x, y, yaw, v, w, dt)
        yield x, y, yaw


# How long a candidate is swept before it is allowed to swing a corner
# into a wall. Long enough for a hard turn to show the rear, short
# enough that a centred doorway is still a straight pass.
SWING_HORIZON_S = 0.75
SWING_DT_S = 0.05
# Outward travel of the rear corner past the side face, for a yaw change.
# A skid-steer pivots about its centre, so turning away from a wall
# swings this corner into it while the nose camera still sees open floor.
_CORNERS = (
    ("front-left", NOSE_M, HALF_M),
    ("front-right", NOSE_M, -HALF_M),
    ("rear-left", -TAIL_M, HALF_M),
    ("rear-right", -TAIL_M, -HALF_M),
)
# Sonar gaps are measured from the real shell, not from the 2 cm margin.
_SHELL_CORNERS = (
    ("front-left", SHELL_NOSE_M, SHELL_HALF_M),
    ("front-right", SHELL_NOSE_M, -SHELL_HALF_M),
    ("rear-left", -SHELL_TAIL_M, SHELL_HALF_M),
    ("rear-right", -SHELL_TAIL_M, -SHELL_HALF_M),
)


def rear_swing_m(angle_rad: float) -> float:
    """How far the rear corner swings past the side face for a yaw change."""
    angle = abs(float(angle_rad))
    outward = SHELL_TAIL_M * math.sin(angle) - SHELL_HALF_M * (1.0 - math.cos(angle))
    return max(0.0, outward)


def swing_out(
    grid: OccupancyGrid | None,
    pose: tuple[float, float, float],
    v_m_s: float,
    w_rad_s: float,
    *,
    left_m: float | None = None,
    right_m: float | None = None,
    rear_m: float | None = None,
) -> str | None:
    """Corner that this arc drives into a mapped wall or a sonar plane.

    The check uses the robot's own grid, only cells it has already marked
    occupied, plus the live side and rear ranges. Unknown space is not a
    wall. Cells the shell already covers are ignored. None means the
    sweep stays clear. The sensor string names the corner.
    """
    v = float(v_m_s)
    w = float(w_rad_s)
    if abs(v) < 1e-4 and abs(w) < 1e-4:
        return None
    x, y, yaw = (float(pose[0]), float(pose[1]), float(pose[2]))
    ignore = _covered(grid, x, y, yaw) if grid is not None else set()
    samples = max(1, int(round(SWING_HORIZON_S / SWING_DT_S)))
    left_dir = (-math.sin(yaw), math.cos(yaw))
    right_dir = (math.sin(yaw), -math.cos(yaw))
    back_dir = (-math.cos(yaw), -math.sin(yaw))
    for _ in range(samples):
        x, y, yaw = _advance(x, y, yaw, v, w, SWING_DT_S)
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        for name, bx, by in _CORNERS:
            if grid is None:
                break
            wx = x + cos_y * bx - sin_y * by
            wy = y + sin_y * bx + cos_y * by
            if _corner_occupied(grid, wx, wy, ignore):
                return f"swing:{name}"
        for name, bx, by in _SHELL_CORNERS:
            wx = x + cos_y * bx - sin_y * by
            wy = y + sin_y * bx + cos_y * by
            # A slight reverse is how the tire leaves a jamb. A couple of
            # centimetres of rear swing on that backup is the maneuver.
            # A forward turn gets no such slack.
            slack = 0.03 if v < -0.02 else 0.0
            if "left" in name and _past_sonar(wx, wy, pose, left_dir, left_m, SHELL_HALF_M, slack):
                return f"swing:{name}"
            if "right" in name and _past_sonar(wx, wy, pose, right_dir, right_m, SHELL_HALF_M, slack):
                return f"swing:{name}"
            if name.startswith("rear") and _past_sonar(wx, wy, pose, back_dir, rear_m, SHELL_TAIL_M, slack):
                return f"swing:{name}"
    return None


def _past_sonar(
    wx: float,
    wy: float,
    pose: tuple[float, float, float],
    direction: tuple[float, float],
    gap_m: float | None,
    along_m: float,
    slack_m: float = 0.0,
) -> bool:
    """True when this corner has crossed the plane the sonar is reporting.

    ``along_m`` is the shell face the sensor sits on (side half-width, or
    the tail). The gap is air outside that face.
    """
    if gap_m is None or not math.isfinite(float(gap_m)):
        return False
    limit = float(along_m) + float(gap_m) + float(slack_m)
    dx = wx - float(pose[0])
    dy = wy - float(pose[1])
    return dx * direction[0] + dy * direction[1] > limit + 1e-3


def _corner_occupied(
    grid: OccupancyGrid,
    x: float,
    y: float,
    ignore: set[tuple[int, int]],
) -> bool:
    cell = grid.cell_of(x, y)
    if cell is None or cell in ignore:
        return False
    return float(grid.log[cell]) >= grid.occ_thresh


def _advance(x: float, y: float, yaw: float, v: float, w: float, dt: float) -> tuple[float, float, float]:
    if abs(w) < 1e-3:
        return x + v * dt * math.cos(yaw), y + v * dt * math.sin(yaw), yaw
    dth = w * dt
    yaw2 = yaw + dth
    x2 = x + (v / w) * (math.sin(yaw2) - math.sin(yaw))
    y2 = y + (v / w) * (-math.cos(yaw2) + math.cos(yaw))
    return x2, y2, yaw2


def _cell_span(grid: OccupancyGrid, x: float, y: float) -> tuple[int, int, int, int] | None:
    near = grid.cell_of(x - _REACH_M, y - _REACH_M)
    far = grid.cell_of(x + _REACH_M, y + _REACH_M)
    if near is None and far is None:
        return None
    r0 = 0 if near is None else near[0]
    c0 = 0 if near is None else near[1]
    r1 = grid.n - 1 if far is None else far[0]
    c1 = grid.n - 1 if far is None else far[1]
    if r0 > r1:
        r0, r1 = r1, r0
    if c0 > c1:
        c0, c1 = c1, c0
    return r0, r1, c0, c1


def _covered(grid: OccupancyGrid, x: float, y: float, yaw: float) -> set[tuple[int, int]]:
    span = _cell_span(grid, x, y)
    found: set[tuple[int, int]] = set()
    if span is None:
        return found
    r0, r1, c0, c1 = span
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    res = grid.res
    ox, oy = grid.origin
    for r in range(r0, r1 + 1):
        cy = oy + (r + 0.5) * res
        dy = cy - y
        for c in range(c0, c1 + 1):
            cx = ox + (c + 0.5) * res
            dx = cx - x
            bx = cos_y * dx + sin_y * dy
            by = -sin_y * dx + cos_y * dy
            if -TAIL_M <= bx <= NOSE_M and -HALF_M <= by <= HALF_M:
                found.add((r, c))
    return found


def _hits_new_obstacle(
    grid: OccupancyGrid,
    x: float,
    y: float,
    yaw: float,
    ignore: set[tuple[int, int]],
) -> bool:
    span = _cell_span(grid, x, y)
    if span is None:
        return False
    r0, r1, c0, c1 = span
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    res = grid.res
    ox, oy = grid.origin
    log = grid.log
    thresh = grid.occ_thresh
    for r in range(r0, r1 + 1):
        cy = oy + (r + 0.5) * res
        dy = cy - y
        for c in range(c0, c1 + 1):
            if (r, c) in ignore:
                continue
            if float(log[r, c]) < thresh:
                continue
            cx = ox + (c + 0.5) * res
            dx = cx - x
            bx = cos_y * dx + sin_y * dy
            by = -sin_y * dx + cos_y * dy
            if -TAIL_M <= bx <= NOSE_M and -HALF_M <= by <= HALF_M:
                return True
    return False
