"""
Webots power estimation — map motor/joint telemetry to PMS channel draws.

Grounded in config/hardware_profiles/<id>.yaml (active: butlerbot_wheeled):
drive motors, stabilizers, compute+sensors, optional vision mode, battery Wh.
"""

from __future__ import annotations

from datetime import datetime, timezone

from src.hardware_profile import (
    battery_capacity_wh,
    clamp_motor_power_w,
    get_active_profile,
    motor_idle_and_scale,
    motor_spec,
    normalize_phase_name,
    wheel_radius_m,
)

# ButlerBot Webots motor name → PMS channel
WEBOTS_MOTOR_CHANNELS: dict[str, str] = {
    "left_wheel": "Legs",
    "right_wheel": "Legs",
    "leg_left": "Legs",
    "leg_right": "Legs",
    "leg_motors": "Legs",
    "caster": "Legs",
    "torso_joint": "Torso",
    "torso_motor": "Torso",
    "left_arm": "Arms",
    "right_arm": "Arms",
    "arm_motors": "Arms",
    "gripper": "Arms",
}

# Fallbacks only if profile missing compute block
COMPUTE_IDLE_W = 6.5
COMPUTE_ACTIVE_W = 10.0

_GAIT_STRESS: dict[str, float] = {
    "stand": 1.0,
    "idle": 1.0,
    "standby": 1.0,
    "drive": 1.35,
    "transit": 1.35,
    "walk": 1.35,
    "patrol": 1.2,
    "manipulate": 1.5,
    "high_load": 1.5,
}

_PHASE_STRESS: dict[str, float] = {
    "standby": 1.0,
    "drive_transit": 1.4,
    "walk_transit": 1.4,
    "teleop": 1.35,
    "patrol": 1.2,
    "manipulate": 1.55,
    "return_idle": 1.05,
}

_GAIT_TO_TASK = {
    "stand": "idle",
    "idle": "idle",
    "standby": "idle",
    "drive": "moving",
    "transit": "moving",
    "walk": "moving",
    "patrol": "balanced",
    "balanced": "balanced",
    "manipulate": "high_load",
    "grasp": "high_load",
    "high_load": "high_load",
}


def _resolve_profile(profile: dict | None) -> dict:
    return profile if profile is not None else get_active_profile()


def estimate_motor_power_w(
    velocity: float,
    torque: float,
    *,
    motor_name: str = "",
    motor_idle_w: float | None = None,
    scale: float | None = None,
    profile: dict | None = None,
    speed_m_s: float | None = None,
) -> float:
    """Estimate electrical draw from joint velocity (rad/s) and torque (Nm).

    Wheel motors prefer a cruise curve so partial speeds land mid-band instead of
    always pegging the channel max under full teleop command.
    """
    prof = _resolve_profile(profile)
    name = str(motor_name or "")
    idle_w, motor_scale, tau_proxy = motor_idle_and_scale(name or "joint", prof)
    if motor_idle_w is not None:
        idle_w = float(motor_idle_w)
    if scale is not None:
        motor_scale = float(scale)

    vel = abs(float(velocity))
    tau = abs(float(torque))
    # Missing torque feedback (common in Webots) → synthesize load from |ω|
    if tau < 1e-4 and vel > 0.05:
        tau = vel * tau_proxy

    spec = motor_spec(prof, name) if name else {}
    is_wheel = "wheel" in name.lower()

    # Profile cruise curve for wheels: smooth |ω| (or body speed) → watts
    cruise_w = spec.get("cruise_w")
    if is_wheel and cruise_w is not None and vel > 0.02:
        radius = wheel_radius_m(prof)
        v_cruise = float(spec.get("cruise_speed_m_s") or 0.40)
        omega_cruise = max(v_cruise / max(radius, 1e-4), 0.5)
        # Prefer body speed when provided (smoother than noisy encoder)
        if speed_m_s is not None and float(speed_m_s) > 0.02:
            omega_equiv = float(speed_m_s) / max(radius, 1e-4)
            frac = min(1.35, max(0.0, omega_equiv / omega_cruise))
        else:
            frac = min(1.35, vel / omega_cruise)
        # Gentle power curve: ~linear near cruise, not squared (avoids early peak)
        cruise = float(cruise_w)
        raw = idle_w + (cruise - idle_w) * (frac ** 1.15)
        # Light torque boost only when feedback present and high
        if abs(float(torque)) > 1e-3:
            raw += min(4.0, abs(float(torque)) * vel * 0.15)
    else:
        eff = float(spec.get("efficiency") or 0.0)
        mechanical = tau * vel
        if eff > 0.05 and mechanical > 0:
            raw = idle_w + mechanical / eff
        else:
            raw = idle_w + mechanical * motor_scale

    if name:
        return clamp_motor_power_w(name, raw, prof)
    return round(raw, 2)


def motor_powers_from_joints(
    joints: list[dict],
    profile: dict | None = None,
    *,
    speed_m_s: float = 0.0,
) -> dict[str, float]:
    """Build per-motor watt map from joint state samples."""
    prof = _resolve_profile(profile)
    powers: dict[str, float] = {}
    for joint in joints:
        name = str(joint.get("name", "")).lower()
        if not name:
            continue
        # Recompute from v/τ so profile idle/cruise applies even if controller
        # stamped an older power_w (keeps profile as source of truth).
        powers[name] = estimate_motor_power_w(
            joint.get("velocity", 0.0),
            joint.get("torque", 0.0),
            motor_name=name,
            profile=prof,
            speed_m_s=speed_m_s,
        )
    return powers


def motion_scale(*, speed_m_s: float = 0.0, joints: list[dict] | None = None) -> float:
    """0–1 factor from measured motion — idle teleop should not use full drive stress."""
    wheel_peak = 0.0
    if joints:
        for joint in joints:
            name = str(joint.get("name", "")).lower()
            if "wheel" not in name:
                continue
            wheel_peak = max(wheel_peak, abs(float(joint.get("velocity", 0.0))))
    wheel_factor = min(1.0, wheel_peak / 2.5)
    speed_factor = min(1.0, max(0.0, speed_m_s) / 0.28)
    return max(0.15, max(wheel_factor, speed_factor))


def stress_multiplier(*, gait: str = "stand", phase: str = "") -> float:
    """Combined gait/phase stress factor for Webots twin telemetry."""
    g = str(gait).lower()
    p = normalize_phase_name(phase) if phase else ""
    if p and p in _PHASE_STRESS:
        return _PHASE_STRESS[p]
    if phase and str(phase).lower() in _PHASE_STRESS:
        return _PHASE_STRESS[str(phase).lower()]
    return _GAIT_STRESS.get(g, 1.0)


def _compute_draw_w(
    gait: str,
    profile: dict,
    *,
    speed_m_s: float = 0.0,
    vision: bool = False,
) -> float:
    """SBC + sensors (+ optional vision) from profile."""
    compute = profile.get("compute") or {}
    sensors = profile.get("sensors") or {}
    modes = profile.get("modes") or {}
    idle_w = float(compute.get("idle_w", COMPUTE_IDLE_W))
    active_w = float(compute.get("active_w", COMPUTE_ACTIVE_W))
    sens_idle = float(sensors.get("idle_w", 0.0))
    sens_active = float(sensors.get("active_w", sens_idle))
    moving = not (gait in ("stand", "idle", "standby") and speed_m_s < 0.06)
    if not moving:
        board = idle_w
        sens = sens_idle
    else:
        motion = min(1.0, max(0.0, speed_m_s) / 0.35)
        board = idle_w + (active_w - idle_w) * max(motion, 0.25)
        sens = sens_idle + (sens_active - sens_idle) * max(motion, 0.25)
    vision_w = float(modes.get("vision_active_w", 0.0)) if (vision or moving) else 0.0
    agent_w = float(modes.get("agent_active_w", 0.0))
    return round(board + sens + vision_w + agent_w, 2)


def _stabilizer_draw_w(profile: dict, *, speed_m_s: float = 0.0) -> tuple[str, float]:
    stab = profile.get("stabilizers") or {}
    channel = str(stab.get("channel") or "Torso")
    idle_w = float(stab.get("idle_w", 0.0))
    active_w = float(stab.get("active_w", idle_w))
    v_full = float(stab.get("active_speed_m_s", 0.15) or 0.15)
    if speed_m_s <= 0.02:
        watts = idle_w
    else:
        frac = min(1.0, speed_m_s / max(v_full, 0.05))
        watts = idle_w + (active_w - idle_w) * frac
    return channel, round(watts, 2)


def aggregate_channel_draws(
    motor_power_w: dict[str, float],
    *,
    compute_w: float | None = None,
    gait: str = "stand",
    phase: str = "",
    speed_m_s: float = 0.0,
    joints: list[dict] | None = None,
    profile: dict | None = None,
) -> dict[str, float]:
    """Sum motor draws into Legs / Arms / Torso / Compute (+ stabilizers)."""
    prof = _resolve_profile(profile)
    channels: dict[str, float] = {}
    channel_caps = prof.get("channels") or {}

    for motor, watts in motor_power_w.items():
        ch = WEBOTS_MOTOR_CHANNELS.get(motor.lower())
        if ch:
            channels[ch] = round(channels.get(ch, 0.0) + float(watts), 2)

    stab_ch, stab_w = _stabilizer_draw_w(prof, speed_m_s=speed_m_s)
    if stab_w > 0:
        channels[stab_ch] = round(channels.get(stab_ch, 0.0) + stab_w, 2)

    if compute_w is None:
        compute_w = _compute_draw_w(gait, prof, speed_m_s=speed_m_s)
    channels["Compute"] = round(compute_w, 2)

    # Mild stress from gait/phase (profile cruise already sets base drive level)
    # Keep stress mild — cruise curve already sets drive level (avoids always 28W cap)
    mult = stress_multiplier(gait=gait, phase=phase)
    if mult > 1.0 and speed_m_s > 0.05:
        motion = motion_scale(speed_m_s=speed_m_s, joints=joints)
        effective = 1.0 + (mult - 1.0) * motion * 0.55
        for ch_id in list(channels.keys()):
            if ch_id == "Compute":
                channels[ch_id] = round(channels[ch_id] * min(effective, 1.1), 2)
            elif ch_id == "Legs":
                channels[ch_id] = round(channels[ch_id] * min(effective, 1.2), 2)
            else:
                channels[ch_id] = round(channels[ch_id] * min(effective, 1.15), 2)

    for ch_id, total in list(channels.items()):
        cap = channel_caps.get(ch_id, {})
        max_w = cap.get("max_draw_w")
        if max_w is not None:
            channels[ch_id] = round(min(total, float(max_w)), 2)

    return channels


def gait_to_task(gait: str) -> str:
    return _GAIT_TO_TASK.get(str(gait).lower(), "balanced")


def build_webots_telemetry(
    *,
    joints: list[dict],
    gait: str = "stand",
    phase: str = "",
    speed_m_s: float = 0.0,
    battery_pct: float = 90.0,
    pose: dict | None = None,
    sensors: dict | None = None,
    robot_name: str = "ButlerBot",
    motor_power_w: dict[str, float] | None = None,
    profile: dict | None = None,
) -> dict:
    """Build a DigitalTwinBridge telemetry payload from Webots controller data."""
    prof = _resolve_profile(profile)
    capacity_wh = battery_capacity_wh(prof)
    batt = prof.get("battery") or {}

    motor_power_w = motor_power_w or motor_powers_from_joints(
        joints, prof, speed_m_s=speed_m_s
    )
    channel_draws = aggregate_channel_draws(
        motor_power_w,
        gait=gait,
        phase=phase,
        speed_m_s=speed_m_s,
        joints=joints,
        profile=prof,
    )
    task = gait_to_task(gait)
    norm_phase = normalize_phase_name(phase) if phase else phase

    return {
        "schema_version": "1.1",
        "source": "webots",
        "adapter": "webots",
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "robot": {
            "name": robot_name,
            "main_battery_pct": round(battery_pct, 2),
            "battery_capacity_wh": capacity_wh,
            "battery_voltage_v": batt.get("nominal_voltage_v"),
            "hardware_profile": prof.get("profile_id"),
            "hardware_profile_version": prof.get("version"),
        },
        "mission": {"task": task, "phase": norm_phase or phase},
        "motor_power_w": motor_power_w,
        "channel_draws": channel_draws,
        "joints": joints,
        "locomotion": {
            "gait": gait,
            "speed_m_s": round(speed_m_s, 3),
            "phase": norm_phase or phase,
            "mode": "wheeled",
            "wheel_radius_m": wheel_radius_m(prof),
        },
        "pose": pose or {},
        "sensors": sensors or {},
        "power": {
            "total_draw_w": round(sum(channel_draws.values()), 1),
            "channel_draws": channel_draws,
            "hardware_profile": prof.get("profile_id"),
        },
    }
