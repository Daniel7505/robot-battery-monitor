"""One control tick: live range and footprint vetoes, everything else is a cost.

Candidates are a handful of forward, arc, spin, and reverse speeds.
A motion is dropped when live range says that speed would meet the shell,
or when a swept corner or tire would enter a wall this robot has mapped
or that side/rear sonar already sees. The front tires are in that sweep:
they catch a table the outer corner clears and the camera looks over.
Doorway centring and a close side are costs. If no forward motion can
turn away without that contact, a short recovery runs inside this tick:
reverse only on an arc that stays clear, otherwise a clear spin, otherwise
a stop. Ground truth is not read.
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
from src.mapping.footprint import rear_swing_m, swing_out

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


def _near_wall(view: BubbleView | None) -> tuple[str, float] | None:
    """The closer side when the robot is running parallel to one wall."""
    if view is None:
        return None
    left = view.left
    right = view.right
    if left is not None and left < 0.30 and (right is None or left + 0.08 < right):
        return ("left", float(left))
    if right is not None and right < 0.30 and (left is None or right + 0.08 < left):
        return ("right", float(right))
    return None


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
    # Room for the rear corner to come around on the turn this heading
    # will probably ask for. A straight pass inside that radius keeps the
    # tire on the wall; a legal turn away does not pay it.
    swing = 0.0
    near = _near_wall(view)
    if near is not None and v > 0.02:
        side_name, gap = near
        need = max(rear_swing_m(0.50), rear_swing_m(abs(w) * 0.75))
        away = -1.0 if side_name == "left" else 1.0
        if gap < need + 0.01 and w * away > -0.05:
            swing += 1.1
    total = pursuit + door + side + swing
    return {
        "pursuit": round(pursuit, 3),
        "door": round(door, 3),
        "side": round(side, 3),
        "swing": round(swing, 3),
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
    def __init__(
        self,
        name: str,
        v: float,
        w: float,
        costs: dict[str, float],
        vetoes: list[dict],
        maneuver: dict | None = None,
    ) -> None:
        self.name = name
        self.v = float(v)
        self.w = float(w)
        self.costs = costs
        self.vetoes = vetoes
        # Set when the tick should run slow → stop → reverse away → forward
        # instead of the scored command. None on an ordinary choice.
        self.maneuver = maneuver

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
    left_m = None if view is None else view.left
    right_m = None if view is None else view.right
    rear_m = None if view is None else view.rear
    vetoes: list[dict] = []
    best_name = ""
    best_v = 0.0
    best_w = 0.0
    best_costs: dict[str, float] | None = None
    for name, v, w in CANDIDATES:
        reason = live_veto(v, w, view)
        if reason is None:
            reason = swing_out(
                grid, pose, v, w, left_m=left_m, right_m=right_m, rear_m=rear_m
            )
        if reason is not None:
            vetoes.append({"motion": name, "sensor": reason})
            continue
        costs = _costs(v, w, v_des, w_des, view, left_clear, right_clear, radar_blocked)
        if best_costs is None or costs["total"] < best_costs["total"]:
            best_name, best_v, best_w, best_costs = name, v, w, costs
    seed = _three_point(view, vetoes, best_v, best_w)
    if seed is not None:
        plan = safe_recovery(grid, pose, view, seed["wall"])
        costs = best_costs if best_costs is not None else {"total": 0.0}
        return MotionChoice("recovery", plan["v"], plan["w"], costs, vetoes, plan)
    if best_costs is not None:
        return MotionChoice(best_name, best_v, best_w, best_costs, vetoes)
    fallen = _fallback(view)
    if fallen is not None:
        name, v, w = fallen
        blocked = live_veto(v, w, view) or swing_out(
            grid, pose, v, w, left_m=left_m, right_m=right_m, rear_m=rear_m
        )
        if blocked is None:
            return MotionChoice(name, v, w, {"total": 0.0, "fallback": 1.0}, vetoes)
    return MotionChoice("hold", 0.0, 0.0, {"total": 0.0}, vetoes)


def _steers_away(v: float, w: float, wall: str) -> bool:
    if v <= 0.02:
        return False
    if wall == "left":
        return w < -0.12
    return w > 0.12


def _three_point(
    view: BubbleView | None,
    vetoes: list[dict],
    best_v: float,
    best_w: float,
) -> dict | None:
    """Start a three-point turn when every forward steer-away swings into the wall.

    A forward command that already turns the nose away is left alone, and
    so is a reverse that yaws off the wall. No legal forward motion is
    still this case: holding there is how a mapped side wall froze the shell.
    """
    wall = _near_wall(view)
    side = None if wall is None else wall[0]
    if side is None:
        for item in vetoes:
            sensor = str(item.get("sensor", ""))
            if not sensor.startswith("swing:"):
                continue
            if sensor.endswith("left"):
                side = "left"
                break
            if sensor.endswith("right"):
                side = "right"
                break
    if side is None:
        return None
    if best_v > 0.02 and _steers_away(best_v, best_w, side):
        return None
    if best_v < -0.02 and _steers_away(-best_v, best_w, side):
        return None
    saw_swing = False
    blocked = {str(item["motion"]): str(item["sensor"]) for item in vetoes}
    for name, v, w in CANDIDATES:
        if not _steers_away(v, w, side):
            continue
        sensor = blocked.get(name)
        if sensor is None:
            return None
        if sensor.startswith("swing:"):
            saw_swing = True
    if not saw_swing:
        return None
    away_w = -0.28 if side == "left" else 0.28
    return {"wall": side, "away_w": away_w}


# How long the reverse arc is allowed to run. Short on purpose: a longer
# backup walks the tail into whatever is behind the shell.
_REVERSE_S = 0.40
_REVERSE_V = -0.10


def safe_recovery(grid, pose, view: BubbleView | None, wall: str) -> dict:
    """A short reverse angled away, a clear spin, or a stop.

    The reverse is swept against this robot's map and its rear and side
    sonar for ``_REVERSE_S`` seconds. The largest angle that stays clear
    wins. A reverse into the obstacle is not used. A spin is used only
    when its swept footprint is clear. Otherwise the tick stops.
    """
    away = -1.0 if wall == "left" else 1.0
    left_m = None if view is None else view.left
    right_m = None if view is None else view.right
    rear_m = None if view is None else view.rear
    for angle in (0.40, 0.28, 0.15, 0.0):
        w = away * angle
        if live_veto(_REVERSE_V, w, view):
            continue
        if swing_out(
            grid,
            pose,
            _REVERSE_V,
            w,
            left_m=left_m,
            right_m=right_m,
            rear_m=rear_m,
            horizon_s=_REVERSE_S,
        ):
            continue
        return {
            "response": "reverse",
            "wall": wall,
            "away_w": away * 0.28,
            "v": _REVERSE_V,
            "w": w,
            "horizon_s": _REVERSE_S,
        }
    for w in (away * 0.55, -away * 0.55):
        if live_veto(0.0, w, view):
            continue
        if swing_out(
            grid,
            pose,
            0.0,
            w,
            left_m=left_m,
            right_m=right_m,
            rear_m=rear_m,
            horizon_s=0.45,
        ):
            continue
        return {
            "response": "spin",
            "wall": wall,
            "away_w": away * 0.28,
            "v": 0.0,
            "w": w,
            "horizon_s": 0.45,
        }
    return {
        "response": "stop",
        "wall": wall,
        "away_w": away * 0.28,
        "v": 0.0,
        "w": 0.0,
        "horizon_s": 0.0,
    }


class ThreePoint:
    """The recovery maneuver. Inactive until ``start``."""

    def __init__(self) -> None:
        self.phase: str | None = None
        self.until = 0.0
        self.away_w = 0.0
        self.wall = ""
        self.response = ""
        self._index = 0
        self._phases: tuple = ()

    def active(self) -> bool:
        return self.phase is not None

    def start(self, t: float, plan: dict) -> None:
        self.wall = str(plan["wall"])
        self.away_w = float(plan["away_w"])
        self.response = str(plan["response"])
        self._index = 0
        if self.response == "stop":
            self._phases = (("stop", 0.30, 0.0, 0.0),)
        elif self.response == "spin":
            self._phases = (
                ("stop", 0.10, 0.0, 0.0),
                ("spin", float(plan["horizon_s"]), 0.0, float(plan["w"])),
            )
        else:
            self._phases = (
                ("slow", 0.15, 0.06, 0.0),
                ("stop", 0.10, 0.0, 0.0),
                ("reverse", float(plan["horizon_s"]), float(plan["v"]), float(plan["w"])),
                ("forward", 0.35, 0.12, 0.0),
            )
        self.phase = self._phases[0][0]
        self.until = float(t) + self._phases[0][1]

    def poll(self, t: float) -> tuple[float, float] | None:
        if self.phase is None:
            return None
        if float(t) + 1e-9 < self.until:
            return self._command()
        self._index += 1
        if self._index >= len(self._phases):
            self.phase = None
            return None
        name, dur, _v, _w = self._phases[self._index]
        self.phase = name
        self.until = float(t) + dur
        return self._command()

    def _command(self) -> tuple[float, float]:
        _name, _dur, v, w = self._phases[self._index]
        return float(v), float(w)
