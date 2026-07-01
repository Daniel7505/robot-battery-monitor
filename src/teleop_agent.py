"""
Teleop limits for ButlerBot — WASD drive math, local thermal model, agent throttle.

Used by the Webots controller; logic is testable without Webots.
"""

from __future__ import annotations

import math

# WoW-style WASD — wheel angular velocity (rad/s)
TELEOP_DRIVE_SPEED = 5.5
TELEOP_REVERSE_SPEED = 4.2
TELEOP_TURN_SPEED = 2.6

BATTERY_AGENT_FLOOR_PCT = 15.0
BATTERY_AGENT_CRITICAL_PCT = 10.0
THERMAL_AMBIENT_C = 22.0
# Relaxed for Webots teleop tuning — heat ramps slowly; throttle only at high temps.
THERMAL_WARN_C = 72.0
THERMAL_CRIT_C = 82.0
THERMAL_HEATING_FACTOR = 0.12
THERMAL_HEATING_RATE = 0.01

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
    heating_factor: float = THERMAL_HEATING_FACTOR,
    cooling_rate: float = 0.12,
) -> float:
    """Heat rises when the robot is actually working, cools at rest."""
    activity = max(0.0, min(1.0, motion_factor))
    effective_draw = draw_w * activity
    if activity > 0.15 and effective_draw > 10.0:
        thermal_c += (effective_draw - 10.0) * heating_factor * dt_s * THERMAL_HEATING_RATE
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


# ABS braking — latched direction; brake magnitude tracks GPS speed.
WHEEL_RADIUS_M = 0.08
MIN_BRAKE_WHEEL_V = 1.5
BRAKE_OPPOSE_FACTOR = 1.05
STOP_SPEED_M_S = 0.02
BRAKE_COAST_SPEED_M_S = 0.12
BRAKE_COAST_PHASE_S = 0.15
CONTROL_SPEED_CAP_M_S = 0.85
MOTION_SETTLED_SPEED_M_S = 0.08
MOTION_SETTLED_WHEEL_RAD_S = 0.2

BATTERY_CAPACITY_WH = 480.0
BATTERY_DRAIN_SCALE = 12.0


def sanitize_motion(speed_m_s: float, forward_m_s: float) -> tuple[float, float]:
    """Ignore GPS spikes from tipping — robot max cruise is ~0.5 m/s."""
    speed = max(0.0, min(speed_m_s, CONTROL_SPEED_CAP_M_S))
    forward = max(-CONTROL_SPEED_CAP_M_S, min(CONTROL_SPEED_CAP_M_S, forward_m_s))
    return round(speed, 3), round(forward, 3)


def motion_settled(
    speed_m_s: float,
    left_wheel_rad_s: float,
    right_wheel_rad_s: float,
) -> bool:
    """True when linear + wheel motion low enough to accept new drive input."""
    return (
        speed_m_s < MOTION_SETTLED_SPEED_M_S
        and abs(left_wheel_rad_s) < MOTION_SETTLED_WHEEL_RAD_S
        and abs(right_wheel_rad_s) < MOTION_SETTLED_WHEEL_RAD_S
        and abs(left_wheel_rad_s - right_wheel_rad_s) < 0.35
    )


def latch_brake_motion_sign(
    forward_m_s: float,
    speed_m_s: float,
    *,
    last_left_v: float = 0.0,
    last_right_v: float = 0.0,
) -> float:
    """+1 forward / -1 reverse / 0 unknown — uses last drive cmd when GPS is ambiguous."""
    if abs(forward_m_s) >= STOP_SPEED_M_S:
        return math.copysign(1.0, forward_m_s)
    net_drive = last_left_v + last_right_v
    if abs(net_drive) > 0.5:
        return math.copysign(1.0, net_drive)
    if speed_m_s >= STOP_SPEED_M_S:
        return 1.0
    return 0.0


def is_spin_brake(
    *,
    last_left_v: float,
    last_right_v: float,
    left_wheel_rad_s: float,
    right_wheel_rad_s: float,
    speed_m_s: float,
) -> bool:
    """Turn-in-place — halt wheels only (per-wheel oppose caused runaway)."""
    turn_cmd = (
        abs(last_left_v - last_right_v) > 1.0
        and abs(last_left_v + last_right_v) < 1.5
    )
    wheel_turn = abs(left_wheel_rad_s - right_wheel_rad_s) > 0.25
    return (turn_cmd or wheel_turn) and speed_m_s < 0.35


def brake_wheel_cap_rad_s(speed_m_s: float) -> float:
    """Speed-tiered cap — must oppose wheel spin at cruise, softer only when crawling."""
    if speed_m_s < 0.15:
        return 5.0
    if speed_m_s < 0.45:
        return 9.0
    return 12.0


def should_coast_before_brake(speed_m_s: float) -> bool:
    return speed_m_s < BRAKE_COAST_SPEED_M_S


def abs_brake_wheel_velocity_latched(
    motion_sign: float,
    speed_m_s: float,
    *,
    min_brake_v: float = MIN_BRAKE_WHEEL_V,
    oppose_factor: float = BRAKE_OPPOSE_FACTOR,
    wheel_radius_m: float = WHEEL_RADIUS_M,
    stop_speed_m_s: float = STOP_SPEED_M_S,
) -> float:
    """Oppose latched direction with ~1:1 wheel-equivalent cmd — works at cruise speed."""
    if motion_sign == 0.0 or speed_m_s < stop_speed_m_s:
        return 0.0
    equiv_rad_s = speed_m_s / max(wheel_radius_m, 1e-6)
    cap = brake_wheel_cap_rad_s(speed_m_s)
    mag = max(min_brake_v, min(cap, equiv_rad_s * oppose_factor))
    return -motion_sign * mag


def abs_brake_wheel_velocity(
    forward_m_s: float,
    speed_m_s: float,
    **kwargs: float,
) -> float:
    sign = latch_brake_motion_sign(forward_m_s, speed_m_s)
    return abs_brake_wheel_velocity_latched(sign, speed_m_s, **kwargs)


def battery_drain_pct(
    draw_w: float,
    dt_s: float,
    *,
    capacity_wh: float = BATTERY_CAPACITY_WH,
    scale: float = BATTERY_DRAIN_SCALE,
    drain_scale: float = 1.0,
) -> float:
    """Physics-based % drop (scaled for gameplay) — 480 Wh pack."""
    if draw_w <= 0 or dt_s <= 0:
        return 0.0
    return (draw_w * dt_s) / (capacity_wh * 3600.0) * 100.0 * scale * drain_scale


def abs_brake_complete(
    forward_m_s: float,
    speed_m_s: float,
    *,
    stop_speed_m_s: float = STOP_SPEED_M_S,
) -> bool:
    return speed_m_s < stop_speed_m_s and abs(forward_m_s) < stop_speed_m_s