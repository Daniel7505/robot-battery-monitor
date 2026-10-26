"""Speed-scaled gap around the whole shell, not only the nose.

The warehouse gate sizes a stop as ``stop_distance_m``: standoff plus
reaction lag plus ``v² / 2a``. That distance is a clearance from the
shell. Counting it from the axle would add the body radius a second
time. The warehouse standoff (0.35 m of air in front of a lane robot)
does not fit through an 0.81 m door, so the standoff here is a few
centimetres. At creep the bubble is smaller than the air beside a
centred tire. At cruise it grows, and the robot slows before a clip.

A spin sweeps the corners, which stick out past the tires. The extra
gap is the pivot clearance. Reverse only until that gap is open, and
never more than a short cap. A fixed two-second reverse pulled the
robot off doorways it had already lined up with.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from src.obstacle_gate import stop_distance_m

# Caster centres sit at x = ±0.11 m with a 0.04 m sphere, so the shell
# reaches about 0.15 m. The front sensor is mounted just ahead of that.
SHELL_NOSE_M = 0.17
SHELL_TAIL_M = 0.16
# Hubs at ±0.17 m, tire 0.05 m wide.
SHELL_HALF_M = 0.195
_CORNER_M = math.hypot(max(SHELL_NOSE_M, SHELL_TAIL_M), SHELL_HALF_M)
# Air past the tire that a spin in place needs.
PIVOT_CLEARANCE_M = _CORNER_M - SHELL_HALF_M
PIVOT_REVERSE_MAX_M = 0.25

# A few centimetres of air, not the warehouse lane standoff.
BUBBLE_STANDOFF_M = 0.04
BUBBLE_LATENCY_S = 0.10
BUBBLE_DECEL_M_S2 = 1.4

_FRONT = ("us_front", "us_front_left", "us_front_right")
_LEFT = ("us_left", "us_front_left", "us_rear_left")
_RIGHT = ("us_right", "us_front_right", "us_rear_right")
_REAR = ("us_rear", "us_rear_left", "us_rear_right")


@dataclass(frozen=True)
class BubbleView:
    """Shell gaps in metres. None means that side was not measured."""

    forward: float | None
    left: float | None
    right: float | None
    rear: float | None
    minimum: float | None


def stop_gap(speed_m_s: float) -> float:
    """Clearance from the shell at which braking must already have started."""
    return stop_distance_m(
        abs(float(speed_m_s)),
        BUBBLE_DECEL_M_S2,
        BUBBLE_LATENCY_S,
        BUBBLE_STANDOFF_M,
    )


def max_speed_for_gap(gap_m: float) -> float:
    """Fastest speed whose stop distance still fits inside ``gap_m``."""
    gap = float(gap_m)
    if not math.isfinite(gap) or gap <= BUBBLE_STANDOFF_M:
        return 0.0
    accel = BUBBLE_DECEL_M_S2
    lag = BUBBLE_LATENCY_S
    inside = (accel * lag) ** 2 + 2.0 * accel * (gap - BUBBLE_STANDOFF_M)
    return max(0.0, -accel * lag + math.sqrt(max(0.0, inside)))


def pivot_ready(gap_m: float | None) -> bool:
    """True when a spin has room, or when no sensor reported a gap."""
    if gap_m is None:
        return True
    return float(gap_m) >= PIVOT_CLEARANCE_M


def reverse_budget_m(gap_m: float | None, *, blind: bool = False, force_min_m: float = 0.0) -> float:
    """How far to back up so the pivot disk is clear. Capped, and zero if it already is."""
    if gap_m is None:
        gap = 0.0 if blind else PIVOT_CLEARANCE_M
    else:
        gap = float(gap_m)
    need = PIVOT_CLEARANCE_M - gap
    if need <= 0.02 and not blind and force_min_m <= 0.0:
        return 0.0
    budget = max(0.0, need + 0.03)
    if blind:
        budget = max(budget, 0.12)
    budget = max(budget, float(force_min_m))
    return min(PIVOT_REVERSE_MAX_M, budget)


def _side(proximity: dict, names: tuple[str, ...]) -> float | None:
    vals: list[float] = []
    for name in names:
        if name not in proximity:
            continue
        try:
            value = float(proximity[name])
        except (TypeError, ValueError):
            continue
        if math.isfinite(value) and 0.0 < value < 1.15:
            vals.append(value)
    if not vals:
        return None
    return min(vals)


def assess_bubble(
    nose_x: float,
    *,
    blind: bool,
    proximity: dict | None,
) -> BubbleView:
    """Shell gaps from the depth nose and any close-range sensors.

    A sensor reading is already the air from the shell, because the
    devices sit on the shell and look outward. A depth hit is a point
    in the body frame, so the nose gap is that x minus the shell.
    Depth that is blind is not an open hallway: with no close-range
    reading it is a zero forward gap. A close-range reading wins over
    that, which is the point of the ring inside the 0.35 m hole.
    """
    prox = proximity or {}
    forward = _side(prox, _FRONT)
    left = _side(prox, _LEFT)
    right = _side(prox, _RIGHT)
    rear = _side(prox, _REAR)
    nose = float(nose_x)
    if (not blind) and math.isfinite(nose) and nose < 50.0:
        shell = max(0.0, nose - SHELL_NOSE_M)
        forward = shell if forward is None else min(forward, shell)
    elif blind and forward is None:
        forward = 0.0
    bits = [gap for gap in (forward, left, right, rear) if gap is not None]
    minimum = min(bits) if bits else None
    return BubbleView(forward, left, right, rear, minimum)
