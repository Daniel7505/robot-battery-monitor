"""
Teleop limits for ButlerBot — WASD drive math, local thermal model, agent throttle.

Used by the Webots controller; logic is testable without Webots.
"""

from __future__ import annotations

# WoW-style WASD — wheel angular velocity (rad/s)
TELEOP_DRIVE_SPEED = 5.5
TELEOP_REVERSE_SPEED = 4.2
TELEOP_TURN_SPEED = 2.6

BATTERY_AGENT_FLOOR_PCT = 15.0
BATTERY_AGENT_CRITICAL_PCT = 10.0
THERMAL_AMBIENT_C = 22.0
THERMAL_WARN_C = 55.0
THERMAL_CRIT_C = 68.0

AGENT_FUN_MESSAGE = "Agent throttling you — having too much fun!"


def normalize_key_code(key: int) -> int:
    """Webots reports lowercase letters; normalize to uppercase ASCII."""
    if ord("a") <= key <= ord("z"):
        return key - 32
    return key


def normalize_key_set(keys: set[int]) -> set[int]:
    return {normalize_key_code(k) for k in keys}


def drive_from_key_set(
    keys: set[int],
    *,
    key_w: int,
    key_a: int,
    key_s: int,
    key_d: int,
) -> tuple[float, float]:
    """Return left/right wheel speeds from active keyboard key codes."""
    keys = normalize_key_set(keys)
    left = 0.0
    right = 0.0
    if key_w in keys:
        left += TELEOP_DRIVE_SPEED
        right += TELEOP_DRIVE_SPEED
    if key_s in keys:
        left -= TELEOP_REVERSE_SPEED
        right -= TELEOP_REVERSE_SPEED
    if key_a in keys:
        left -= TELEOP_TURN_SPEED
        right += TELEOP_TURN_SPEED
    if key_d in keys:
        left += TELEOP_TURN_SPEED
        right -= TELEOP_TURN_SPEED
    return left, right


def update_thermal_c(
    thermal_c: float,
    draw_w: float,
    dt_s: float,
    *,
    motion_factor: float = 1.0,
    ambient_c: float = THERMAL_AMBIENT_C,
    heating_factor: float = 0.55,
    cooling_rate: float = 0.12,
) -> float:
    """Heat rises when the robot is actually working, cools at rest."""
    activity = max(0.0, min(1.0, motion_factor))
    effective_draw = draw_w * activity
    if activity > 0.15 and effective_draw > 10.0:
        thermal_c += (effective_draw - 10.0) * heating_factor * dt_s * 0.035
    elif thermal_c > ambient_c:
        thermal_c -= cooling_rate * dt_s * (1.0 + (thermal_c - ambient_c) * 0.02)
    return round(max(ambient_c, min(85.0, thermal_c)), 2)


def local_agent_throttle(
    battery_pct: float,
    thermal_c: float,
    *,
    battery_floor: float = BATTERY_AGENT_FLOOR_PCT,
    thermal_warn: float = THERMAL_WARN_C,
    thermal_crit: float = THERMAL_CRIT_C,
) -> tuple[float, str | None]:
    """
    Onboard power agent — caps teleop when battery or heat is out of band.
    Returns (throttle_factor 0..1, optional HUD message).
    """
    factor = 1.0
    reasons: list[str] = []

    if battery_pct <= BATTERY_AGENT_CRITICAL_PCT:
        factor = min(factor, 0.25)
        reasons.append("battery")
    elif battery_pct <= battery_floor:
        factor = min(factor, 0.45)
        reasons.append("battery")

    if thermal_c >= thermal_crit:
        factor = min(factor, 0.30)
        reasons.append("heat")
    elif thermal_c >= thermal_warn:
        factor = min(factor, 0.55)
        reasons.append("heat")

    message = AGENT_FUN_MESSAGE if reasons else None
    return round(factor, 3), message


def merge_throttle(local: float, remote: float | None) -> float:
    """Agent always wins — use the stricter (lower) throttle factor."""
    if remote is None:
        return local
    try:
        remote_f = float(remote)
    except (TypeError, ValueError):
        return local
    if not 0.0 < remote_f < 1.0:
        return local
    return round(min(local, remote_f), 3)


def gauge_fill_ratio(value: float, low: float, high: float) -> float:
    if high <= low:
        return 0.0
    return max(0.0, min(1.0, (value - low) / (high - low)))


def gauge_color_hex(ratio: float, *, warn_at: float = 0.55, crit_at: float = 0.82) -> int:
    """Return 0xRRGGBB fill color for a 0..1 gauge ratio."""
    if ratio >= crit_at:
        return 0xFF4444
    if ratio >= warn_at:
        return 0xFFAA22
    return 0x33DD66


def thermal_gauge_ratio(thermal_c: float, ambient: float = THERMAL_AMBIENT_C, crit: float = THERMAL_CRIT_C) -> float:
    return gauge_fill_ratio(thermal_c, ambient, crit)


def battery_gauge_ratio(battery_pct: float) -> float:
    return max(0.0, min(1.0, battery_pct / 100.0))