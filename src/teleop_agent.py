"""
Teleop limits for ButlerBot — WASD drive math, local thermal model, agent throttle.

Used by the Webots controller; logic is testable without Webots.

Architecture
------------
This module is the *pure* control/math layer the controller imports:

  keyboard  → drive_from_key_set          → left/right rad/s commands
  draw/dt   → update_thermal_c            → local thermal °C
  battery+T → local_agent_throttle        → 0..1 speed cap + HUD message
  local×remote → merge_throttle           → agent-stricter factor wins
  GPS+hubs  → ABS / spin-halt helpers     → stop without residual spin

Residual spin — root cause and lessons encoded here
---------------------------------------------------
Several real Webots failure modes produced "robot keeps circling after stop":

1. **GPS is translation-only.** Body GPS speed ≈ 0 under pure yaw (in-place
   spin). Treating ``speed_m_s < ε`` as "stopped" completes ABS while hubs
   still rotate. Mitigations: ``abs_brake_complete`` requires hub rates calm;
   ``is_spin_brake`` / ``is_turning_motion`` detect differential command or
   opposite wheel rates even when GPS speed is still high.

2. **Symmetric reverse on a spinning base is wrong.** Forward-then-turn used
   to miss spin mode and keep applying equal reverse on both wheels, which
   leaves residual circling. ``is_spin_brake`` prioritizes yaw/diff command
   over linear ABS.

3. **GPS spikes from tipping.** ``sanitize_motion`` caps speed/forward so a
   physics glitch cannot command absurd brake magnitudes.

4. **Drive lockout until settled.** ``motion_settled`` blocks new WASD input
   until linear *and* wheel rates (including L−R diff) are quiet — prevents
   re-commanding drive while residual yaw is still bleeding off.

Related controller lessons (implemented in Webots controller, not here):
  - **NaN encoder → setPosition(NaN) unlocks hubs** in Webots velocity mode;
    always sanitize encoder reads to finite values before setPosition.
  - **Dual hard-zero + stop_epoch**: bridge stamps a new stop_epoch on
    drive_stop; controller hard-zeros hubs and does not treat "already zero
    cmd" as a no-op while residual spin logic is active.
  - **Hub rates + IMU yaw** are the reliable stop sensors; GPS alone is not.
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
    """Return left/right wheel speeds from active keyboard key codes.

    Combinations stack (W+A = forward arc). Speeds are open-loop rad/s setpoints
    for differential drive — not m/s body velocity.
    """
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
    """Heat rises when the robot is actually working, cools at rest.

    ``motion_factor`` prevents parked high-draw electronics from cooking the
    model when the chassis is idle (teleop with zero wheels).
    """
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

    Runs *inside* the Webots controller (no network required) so a lost bridge
    still protects the sim robot. Returns (throttle_factor 0..1, optional HUD
    message). Stricter remote PMS throttle is merged via ``merge_throttle``.
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
    """Agent always wins — use the stricter (lower) throttle factor.

    Remote values outside (0, 1) are ignored so a bad API cannot disable
    local protection by sending 0 or 1.5.
    """
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


# ---------------------------------------------------------------------------
# ABS braking + residual-spin guards
# ---------------------------------------------------------------------------
# Wheel radius converts body m/s → equivalent hub rad/s for oppose magnitude.
WHEEL_RADIUS_M = 0.08
MIN_BRAKE_WHEEL_V = 1.5
BRAKE_OPPOSE_FACTOR = 1.05
STOP_SPEED_M_S = 0.02
BRAKE_COAST_SPEED_M_S = 0.12
BRAKE_COAST_PHASE_S = 0.15
# Cap GPS-derived speeds so tip-over spikes cannot dominate brake math.
CONTROL_SPEED_CAP_M_S = 0.85
MOTION_SETTLED_SPEED_M_S = 0.08
MOTION_SETTLED_WHEEL_RAD_S = 0.2

BATTERY_CAPACITY_WH = 480.0
BATTERY_DRAIN_SCALE = 12.0


def sanitize_motion(speed_m_s: float, forward_m_s: float) -> tuple[float, float]:
    """Ignore GPS spikes from tipping — robot max cruise is ~0.5 m/s.

    Controllers must run raw GPS through this before ABS or "settled" checks;
    uncapped spikes made brake magnitude and lockout thresholds meaningless.
    """
    speed = max(0.0, min(speed_m_s, CONTROL_SPEED_CAP_M_S))
    forward = max(-CONTROL_SPEED_CAP_M_S, min(CONTROL_SPEED_CAP_M_S, forward_m_s))
    return round(speed, 3), round(forward, 3)


def motion_settled(
    speed_m_s: float,
    left_wheel_rad_s: float,
    right_wheel_rad_s: float,
) -> bool:
    """True when linear + wheel motion low enough to accept new drive input.

    Drive lockout: while False, controller should refuse new WASD setpoints so
    residual yaw cannot be re-excited mid-halt. Requires both hubs calm *and*
    small L−R difference (pure spin has low GPS speed but large wheel diff).
    """
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
    """+1 forward / -1 reverse / 0 unknown — uses last drive cmd when GPS is ambiguous.

    GPS forward component can chatter near zero while the chassis still coasts
    in the last commanded direction; last wheel commands break the tie.
    """
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
    """True when stop should kill yaw/spin rather than use linear ABS.

    Differential last command or opposite wheel rates take priority even when
    residual GPS speed is still high (forward-then-turn used to miss spin mode
    and keep applying symmetric reverse, which leaves residual circling).

    Note: ``speed_m_s`` is accepted for API symmetry with the controller; the
    decision intentionally does *not* require low GPS speed — that was the bug.
    """
    net = last_left_v + last_right_v
    diff_cmd = abs(last_left_v - last_right_v)
    turn_cmd = diff_cmd > 1.0 and abs(net) < max(1.5, 0.65 * diff_cmd)
    wheel_diff = abs(left_wheel_rad_s - right_wheel_rad_s)
    wheel_turn = wheel_diff > 0.35 and (
        left_wheel_rad_s * right_wheel_rad_s < 0 or wheel_diff > abs(left_wheel_rad_s + right_wheel_rad_s)
    )
    # Dominant yaw at any linear speed → spin halt path
    if turn_cmd or wheel_turn:
        return True
    return False


def is_turning_motion(
    *,
    left_cmd: float = 0.0,
    right_cmd: float = 0.0,
    left_wheel_rad_s: float = 0.0,
    right_wheel_rad_s: float = 0.0,
    speed_m_s: float = 0.0,
) -> bool:
    """In-place or strong differential turn (GPS speed may be ~0).

    Encodes the translation-only GPS lesson: pure rotation shows little body
    translation while hubs (or turn commands) clearly indicate yaw activity.
    """
    net = left_cmd + right_cmd
    diff = abs(left_cmd - right_cmd)
    if diff > 1.0 and abs(net) < max(1.2, 0.7 * diff):
        return True
    wdiff = abs(left_wheel_rad_s - right_wheel_rad_s)
    if wdiff > 0.45 and (
        left_wheel_rad_s * right_wheel_rad_s < 0 or wdiff > abs(net) * 0.5 + 0.3
    ):
        return True
    # Pure rotation: wheels moving but little body translation
    if (
        speed_m_s < 0.12
        and (abs(left_wheel_rad_s) > 0.4 or abs(right_wheel_rad_s) > 0.4)
        and wdiff > 0.3
    ):
        return True
    return False


def brake_wheel_cap_rad_s(speed_m_s: float) -> float:
    """Speed-tiered cap — must oppose wheel spin at cruise, softer only when crawling."""
    if speed_m_s < 0.15:
        return 5.0
    if speed_m_s < 0.45:
        return 9.0
    return 12.0


def should_coast_before_brake(speed_m_s: float) -> bool:
    """At crawl speeds, a brief coast avoids ABS chatter on near-zero GPS."""
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
    """Oppose latched direction with ~1:1 wheel-equivalent cmd — works at cruise speed.

    Converts body speed to hub rad/s, multiplies by oppose_factor, clamps to a
    speed-tiered cap. Sign is opposite the latched travel direction so both
    wheels get the same reverse command (linear ABS, not spin halt).
    """
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
    """Convenience: latch sign from GPS then compute latched ABS wheel cmd."""
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
    """Physics-based % drop (scaled for gameplay) — 480 Wh pack.

    ``scale`` compresses wall-clock demos so operators see meaningful SOC change
    without waiting hours of sim time.
    """
    if draw_w <= 0 or dt_s <= 0:
        return 0.0
    return (draw_w * dt_s) / (capacity_wh * 3600.0) * 100.0 * scale * drain_scale


def abs_brake_complete(
    forward_m_s: float,
    speed_m_s: float,
    *,
    stop_speed_m_s: float = STOP_SPEED_M_S,
    left_wheel_rad_s: float = 0.0,
    right_wheel_rad_s: float = 0.0,
    require_wheels: bool = True,
) -> bool:
    """Linear GPS calm + (optional) wheel settle — never complete on GPS alone while spinning.

    This is the residual-spin completion gate: GPS-only complete was the classic
    false "stopped" under pure yaw. With ``require_wheels=True`` (default), both
    hub rates and L−R differential must also be below settle thresholds.
    """
    linear_ok = speed_m_s < stop_speed_m_s and abs(forward_m_s) < stop_speed_m_s
    if not require_wheels:
        return linear_ok
    # Stricter than motion_settled alone: keep STOP thresholds for body speed
    # and require wheels calm so residual yaw cannot complete as "stopped".
    return (
        linear_ok
        and abs(left_wheel_rad_s) < MOTION_SETTLED_WHEEL_RAD_S
        and abs(right_wheel_rad_s) < MOTION_SETTLED_WHEEL_RAD_S
        and abs(left_wheel_rad_s - right_wheel_rad_s) < 0.35
    )
