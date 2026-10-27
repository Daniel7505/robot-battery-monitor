"""One control tick: live range vetoes, everything else is a cost.

Candidates are a handful of forward, arc, spin, and reverse speeds.
A motion is dropped only when a live range says that speed would meet
the shell. Sweep, doorway centring, and a close side are costs, so an
open bubble still produces a command. Ground truth is not read.
"""

from __future__ import annotations

import json
import math
import os

from src.mapping.bubble import (
    BUBBLE_STANDOFF_M,
    BubbleView,
    max_speed_for_gap,
    pivot_ready,
    stop_gap,
)
from src.mapping.footprint import footprint_clips

# Half of cruise. A doorway correction has to finish before the tire meets the frame.
DOOR_CREEP_M_S = 0.12

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
    clips: bool,
    view: BubbleView | None,
    left_clear: float,
    right_clear: float,
    radar_blocked: bool,
) -> dict[str, float]:
    pursuit = 4.0 * (v - v_des) ** 2 + (w - w_des) ** 2
    # Enough to prefer a clear arc over a clipped one, and not enough to
    # turn an open nose into a spin or a reverse.
    footprint = 0.25 if clips else 0.0
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
    total = pursuit + footprint + door + side
    return {
        "pursuit": round(pursuit, 3),
        "footprint": round(footprint, 3),
        "door": round(door, 3),
        "side": round(side, 3),
        "total": round(total, 3),
    }


def _fallback(view: BubbleView | None) -> tuple[str, float, float] | None:
    """A legal-looking motion when every candidate was vetoed but some face still shows air."""
    forward = None if view is None else view.forward
    rear = None if view is None else view.rear
    left = None if view is None else view.left
    right = None if view is None else view.right
    if _gap_air(forward):
        if forward is None:
            speed = 0.06
        else:
            speed = max_speed_for_gap(forward)
            speed = 0.05 if speed < 0.04 else min(0.12, speed)
        return ("air", speed, 0.0)
    if _gap_air(rear):
        return ("reverse", -0.10, 0.0)
    left_m = -1.0 if left is None else left
    right_m = -1.0 if right is None else right
    if _gap_air(left) or _gap_air(right):
        return ("spin_left", 0.0, 0.55) if left_m >= right_m else ("spin_right", 0.0, -0.55)
    return None


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
        if reason is not None:
            vetoes.append({"motion": name, "sensor": reason})
            continue
        clips = False
        if grid is not None and (abs(v) > 1e-4 or abs(w) > 1e-4):
            clips = footprint_clips(grid, pose, v, w)
        costs = _costs(v, w, v_des, w_des, clips, view, left_clear, right_clear, radar_blocked)
        if best_costs is None or costs["total"] < best_costs["total"]:
            best_name, best_v, best_w, best_costs = name, v, w, costs
    if best_costs is not None:
        return MotionChoice(best_name, best_v, best_w, best_costs, vetoes)
    fallen = _fallback(view)
    if fallen is None:
        return MotionChoice("hold", 0.0, 0.0, {"total": 0.0}, vetoes)
    name, v, w = fallen
    return MotionChoice(name, v, w, {"total": 0.0, "fallback": 1.0}, vetoes)
