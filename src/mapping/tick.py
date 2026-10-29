"""One control tick: live range vetoes, and a map sweep on turns.

Candidates are a handful of forward, arc, spin, and reverse speeds.
A motion is dropped when live range says that speed would meet the
shell, and when a turn's swept corners enter a mapped obstacle. Straight
driving is not a side veto: a centred doorway has about 21 cm per side.
A mapped wall across the nose, or a swept cell behind a backup,
still drops that command. A smear narrower than the shell does not. If every candidate is vetoed, the tick backs
straight out when the rear sonar and the map both allow it. Ground truth
is not read.
"""

from __future__ import annotations

import json
import math
import os

from src.mapping.bubble import (
    BUBBLE_STANDOFF_M,
    BubbleView,
    pivot_ready,
    stop_gap,
)
from src.mapping.footprint import footprint_clips

# Half of cruise. A doorway correction has to finish before the tire meets the frame.
DOOR_CREEP_M_S = 0.12
# Below this yaw rate the command is straight. Arcs and pivots are above it,
# and a map corner on those sweeps is a veto. A few centimetres of shell
# margin (FOOTPRINT_MARGIN_M) covers drift; straight motion does not pay it.
TURN_W_RAD_S = 0.12

# (name, v m/s, w rad/s). Positive w turns left.
CANDIDATES: tuple[tuple[str, float, float], ...] = (
    ("cruise", 0.28, 0.0),
    ("creep", DOOR_CREEP_M_S, 0.0),
    ("slow", 0.06, 0.0),
    ("arc_left", 0.18, 0.40),
    ("arc_right", 0.18, -0.40),
    ("creep_left", DOOR_CREEP_M_S, 0.28),
    ("creep_right", DOOR_CREEP_M_S, -0.28),
    ("hard_left", 0.10, 0.75),
    ("hard_right", 0.10, -0.75),
    ("spin_left", 0.0, 0.70),
    ("spin_right", 0.0, -0.70),
    ("reverse", -0.10, 0.0),
    ("reverse_left", -0.12, 0.40),
    ("reverse_right", -0.12, -0.40),
)


def decision_log_enabled() -> bool:
    return os.environ.get("RBM_DECISION_LOG") == "1"


def log_decision(event: dict) -> None:
    """Append one JSON object. Off unless ``RBM_DECISION_LOG=1``.

    The line goes to ``$RBM_MAP_DIR/decision.jsonl``. It is not printed,
    so a normal mapping log does not grow when the switch is off.
    """
    if not decision_log_enabled():
        return
    folder = os.environ.get("RBM_MAP_DIR")
    if not folder:
        return
    path = os.path.join(folder, "decision.jsonl")
    with open(path, "a", encoding="utf-8") as handle:
        handle.write(json.dumps(event, separators=(",", ":")) + "\n")


def centre_yaw(left_m: float, right_m: float) -> float:
    """Yaw toward the centre of a two-sided gap. A one-sided wall is not a doorway."""
    if left_m > 1.3 or right_m > 1.3:
        return 0.0
    error = float(left_m) - float(right_m)
    if abs(error) < 0.08:
        return 0.0
    return max(-0.45, min(0.45, 1.1 * error))


def live_veto(v: float, w: float, view: BubbleView | None) -> str | None:
    """Sensor reading that makes this motion a contact, or None when it is legal.

    An unmeasured side is not a contact. Only a gap inside the stop
    distance for this speed is a veto.
    """
    if view is None:
        return None
    speed = abs(float(v))
    if v > 0.02:
        need = stop_gap(speed)
        forward = view.forward
        if forward is not None and forward < need:
            return f"forward:{forward:.2f}<{need:.2f}"
        if w > 0.15 and view.left is not None and view.left < need:
            return f"left:{view.left:.2f}<{need:.2f}"
        if w < -0.15 and view.right is not None and view.right < need:
            return f"right:{view.right:.2f}<{need:.2f}"
        return None
    if v < -0.02:
        need = stop_gap(speed)
        rear = view.rear
        if rear is not None and rear < need:
            return f"rear:{rear:.2f}<{need:.2f}"
        return None
    if abs(w) > 0.05 and view.minimum is not None and not pivot_ready(view.minimum):
        return f"pivot:{view.minimum:.2f}"
    return None


def _gap_air(gap: float | None) -> bool:
    return gap is None or float(gap) > BUBBLE_STANDOFF_M


def _desired(
    pursuit_v: float,
    pursuit_w: float,
    heading_err: float,
    view: BubbleView | None,
    left_clear: float,
    right_clear: float,
    radar_blocked: bool,
) -> tuple[float, float]:
    """Pursuit, pulled toward a slow centred crossing when both sides see a gap."""
    v = max(0.0, float(pursuit_v))
    w = float(pursuit_w)
    mouth = False
    if (
        view is not None
        and view.left is not None
        and view.right is not None
        and 0.05 < view.left < 0.50
        and 0.05 < view.right < 0.50
    ):
        mouth = True
        v = min(v if v > 0.02 else DOOR_CREEP_M_S, DOOR_CREEP_M_S)
        w = centre_yaw(view.left, view.right)
    elif 0.12 < left_clear < 1.20 and 0.12 < right_clear < 1.20:
        v = min(v if v > 0.02 else DOOR_CREEP_M_S, DOOR_CREEP_M_S)
        w = centre_yaw(left_clear, right_clear)
        if abs(heading_err) > 0.12:
            w = max(-0.45, min(0.45, w + math.copysign(min(0.35, abs(heading_err)), heading_err)))
    elif radar_blocked:
        v = min(v, DOOR_CREEP_M_S)
    return v, w


def _costs(
    v: float,
    w: float,
    v_des: float,
    w_des: float,
    view: BubbleView | None,
    left_clear: float,
    right_clear: float,
    radar_blocked: bool,
) -> dict[str, float]:
    pursuit = 4.0 * (v - v_des) ** 2 + (w - w_des) ** 2
    door = 0.0
    if 0.12 < left_clear < 1.20 and 0.12 < right_clear < 1.20 and v > 0.16:
        door += 0.8
    side = 0.0
    if view is not None:
        if view.left is not None and view.left < 0.10:
            if v > 0.02:
                side += 1.6
            if w > 0.05:
                side += 2.0
            if v < -0.02 and w < -0.2:
                side -= 0.8
        if view.right is not None and view.right < 0.10:
            if v > 0.02:
                side += 1.6
            if w < -0.05:
                side += 2.0
            if v < -0.02 and w > 0.2:
                side -= 0.8
    if radar_blocked and v > 0.14:
        door += 0.8
    forward = None if view is None else view.forward
    # Air ahead is a reason to go forward. A spin or a reverse is how an
    # open bubble used to sit still.
    if _gap_air(forward) and (forward is None or forward > 0.20):
        if abs(v) < 0.03:
            door += 0.40
        if v < -0.02:
            door += 0.50
    total = pursuit + door + side
    return {
        "pursuit": round(pursuit, 3),
        "door": round(door, 3),
        "side": round(side, 3),
        "total": round(total, 3),
    }


def _fittable_gap(grid, pose: tuple[float, float, float]) -> float | None:
    """Width of a mapped doorway the shell can still pass, or None.

    Both sides have to be occupied, and the gap has to be wider than the
    body. A smear that only vetoes turns is not a wall across the opening.
    """
    if grid is None:
        return None
    labels = grid.labels()
    x, y, yaw = (float(pose[0]), float(pose[1]), float(pose[2]))
    left = (-math.sin(yaw), math.cos(yaw))

    def march(direction: tuple[float, float]) -> float | None:
        dist = float(grid.res)
        while dist <= 0.80:
            cell = grid.cell_of(x + direction[0] * dist, y + direction[1] * dist)
            if cell is None:
                return None
            if int(labels[cell]) == 1:
                return dist
            dist += float(grid.res)
        return None

    east = march(left)
    west = march((-left[0], -left[1]))
    if east is None or west is None:
        return None
    gap = east + west
    if 0.48 <= gap <= 1.35:
        return gap
    return None


def _rear_open(view: BubbleView | None) -> bool:
    """Rear sonar allows a straight backup. No reading is not a wall."""
    if view is None or view.rear is None:
        return True
    return float(view.rear) > 0.06


def _ahead_blocked(grid, pose: tuple[float, float, float]) -> bool:
    """True when a mapped wall spans the nose, not a one-cell smear.

    The centre and both sides at 0.12 m have to be occupied. A centred
    doorway, and a smear narrower than the shell, stays drivable.
    """
    if grid is None:
        return False
    labels = grid.labels()
    x, y, yaw = (float(pose[0]), float(pose[1]), float(pose[2]))
    forward = (math.cos(yaw), math.sin(yaw))
    left = (-forward[1], forward[0])
    step = float(grid.res)
    dist = 0.18
    while dist <= 0.36 + 1e-9:
        cx = x + forward[0] * dist
        cy = y + forward[1] * dist
        blocked = True
        for side in (0.0, 0.12, -0.12):
            cell = grid.cell_of(cx + left[0] * side, cy + left[1] * side)
            if cell is None or int(labels[cell]) != 1:
                blocked = False
                break
        if blocked:
            return True
        dist += step
    return False


def _behind_blocked(grid, pose: tuple[float, float, float]) -> bool:
    """True when a mapped wall spans the tail. A corner graze does not.

    The straight-backup sweep's 2 cm margin kissed one rear corner and
    the tick then held forever. The centre and both sides have to be
    occupied, the same test as a wall across the nose.
    """
    if grid is None:
        return False
    labels = grid.labels()
    x, y, yaw = (float(pose[0]), float(pose[1]), float(pose[2]))
    back = (-math.cos(yaw), -math.sin(yaw))
    left = (-back[1], back[0])
    step = float(grid.res)
    dist = 0.22
    while dist <= 0.42 + 1e-9:
        cx = x + back[0] * dist
        cy = y + back[1] * dist
        blocked = True
        for side in (0.0, 0.12, -0.12):
            cell = grid.cell_of(cx + left[0] * side, cy + left[1] * side)
            if cell is None or int(labels[cell]) != 1:
                blocked = False
                break
        if blocked:
            return True
        dist += step
    return False


def _known_free_ahead(grid, pose: tuple[float, float, float]) -> bool:
    """The robot's own map says the nose path is free, not merely unknown."""
    if grid is None or _ahead_blocked(grid, pose):
        return False
    labels = grid.labels()
    x, y, yaw = (float(pose[0]), float(pose[1]), float(pose[2]))
    step = float(grid.res)
    dist = 0.16
    while dist <= 0.36 + 1e-9:
        cell = grid.cell_of(x + math.cos(yaw) * dist, y + math.sin(yaw) * dist)
        if cell is None or int(labels[cell]) != 0:
            return False
        dist += step
    return True


def _reverse_blocked(grid, pose) -> bool:
    """A mapped wall spans the tail, so a straight backup would enter it."""
    return _behind_blocked(grid, pose)


def _straight_reverse(view, grid, pose, vetoes: list[dict]) -> MotionChoice | None:
    """Back straight out instead of holding. A fittable doorway is a rescan."""
    door = _fittable_gap(grid, pose)
    if not _rear_open(view) or _reverse_blocked(grid, pose):
        if door is None:
            return None
        return MotionChoice("doorway_refusal", 0.0, 0.0, {"total": 0.0, "gap_m": round(door, 2)}, vetoes)
    kind = "doorway_rescan" if door is not None else "back_out"
    costs: dict[str, float] = {"total": 0.0, "fallback": 1.0}
    if door is not None:
        costs["gap_m"] = round(door, 2)
    return MotionChoice(kind, -0.10, 0.0, costs, vetoes)


class MotionChoice:
    def __init__(self, name: str, v: float, w: float, costs: dict[str, float], vetoes: list[dict]) -> None:
        self.name = name
        self.v = float(v)
        self.w = float(w)
        self.costs = costs
        self.vetoes = vetoes

    def why(self) -> str:
        parts = [
            f"{key}={value:.2f}"
            for key, value in self.costs.items()
            if key != "total" and abs(value) > 1e-3
        ]
        return ",".join(parts) if parts else "pursuit"


def choose_motion(
    *,
    view: BubbleView | None,
    pursuit_v: float,
    pursuit_w: float,
    heading_err: float,
    grid,
    pose: tuple[float, float, float],
    left_clear: float,
    right_clear: float,
    radar_blocked: bool = False,
) -> MotionChoice:
    """Score every motion live range still allows. Never zero while a face shows air."""
    v_des, w_des = _desired(
        pursuit_v, pursuit_w, heading_err, view, left_clear, right_clear, radar_blocked
    )
    vetoes: list[dict] = []
    best_name = ""
    best_v = 0.0
    best_w = 0.0
    best_costs: dict[str, float] | None = None
    for name, v, w in CANDIDATES:
        reason = live_veto(v, w, view)
        if reason is None and grid is not None:
            turning = abs(w) > TURN_W_RAD_S
            if turning and footprint_clips(grid, pose, v, w):
                reason = "map:corner"
            elif v < -0.02 and not turning and _behind_blocked(grid, pose):
                reason = "map:rear"
            elif v > 0.02 and not turning and _ahead_blocked(grid, pose):
                reason = "map:ahead"
        if reason is not None:
            vetoes.append({"motion": name, "sensor": reason})
            continue
        costs = _costs(v, w, v_des, w_des, view, left_clear, right_clear, radar_blocked)
        if best_costs is None or costs["total"] < best_costs["total"]:
            best_name, best_v, best_w, best_costs = name, v, w, costs
    if best_costs is not None:
        return MotionChoice(best_name, best_v, best_w, best_costs, vetoes)
    escape = _straight_reverse(view, grid, pose, vetoes)
    if escape is not None:
        return escape
    # A nose gap vetoes every spin, and a hold never becomes a stuck event,
    # so the robot would sit. A spin the map shows as clear is still a move.
    if grid is not None:
        for name, w in (("spin_left", 0.70), ("spin_right", -0.70)):
            if not footprint_clips(grid, pose, 0.0, w):
                return MotionChoice(name, 0.0, w, {"total": 0.0, "fallback": 1.0}, vetoes)
    # Sonar can sit at 6 cm on open floor the map has already marked free.
    # Holding there never counts as stuck, so the seed does not move.
    if _known_free_ahead(grid, pose) and not footprint_clips(grid, pose, 0.06, 0.0):
        return MotionChoice("creep", 0.06, 0.0, {"total": 0.0, "fallback": 1.0}, vetoes)
    return MotionChoice("hold", 0.0, 0.0, {"total": 0.0}, vetoes)
