"""
Webots power estimation — map motor/joint telemetry to PMS channel draws.

Uses reference hardware profiles when available so sim numbers reflect
realistic actuator classes (BLDC wheels, servos, compute).
"""

from __future__ import annotations

from datetime import datetime, timezone

from src.hardware_profile import (
    clamp_motor_power_w,
    get_active_profile,
    motor_spec,
    normalize_phase_name,
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

COMPUTE_IDLE_W = 8.0
COMPUTE_ACTIVE_W = 12.0

_GAIT_STRESS: dict[str, float] = {
    "stand": 1.0,
    "idle": 1.0,
    "standby": 1.0,
    "drive": 1.85,
    "transit": 1.85,
    "walk": 1.85,
    "patrol": 1.5,
    "manipulate": 2.2,
    "high_load": 2.2,
}

_PHASE_STRESS: dict[str, float] = {
    "standby": 1.0,
    "drive_transit": 2.1,
    "walk_transit": 2.1,
    "patrol": 1.55,
    "manipulate": 2.5,
    "return_idle": 1.1,
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


def _motor_params(motor_name: str, profile: dict) -> tuple[float, float]:
    spec = motor_spec(profile, motor_name)
    idle_w = float(spec.get("idle_w", 1.4 if "wheel" not in motor_name else 2.5))
    scale = float(spec.get("scale", 4.2))
    return idle_w, scale


def estimate_motor_power_w(
    velocity: float,
    torque: float,
    *,
    motor_name: str = "",
    motor_idle_w: float | None = None,
    scale: float | None = None,
    profile: dict | None = None,
) -> float:
    """Estimate electrical draw from joint velocity (rad/s) and torque (Nm)."""
    prof = _resolve_profile(profile)
    if motor_name and (motor_idle_w is None or scale is None):
        idle_w, motor_scale = _motor_params(motor_name, prof)
        motor_idle_w = motor_idle_w if motor_idle_w is not None else idle_w
        scale = scale if scale is not None else motor_scale
    motor_idle_w = motor_idle_w if motor_idle_w is not None else 1.4
    scale = scale if scale is not None else 4.2
    mechanical = abs(float(torque) * float(velocity))
    raw = motor_idle_w + mechanical * scale
    if motor_name:
        return clamp_motor_power_w(motor_name, raw, prof)
    return round(raw, 2)


def motor_powers_from_joints(
    joints: list[dict],
    profile: dict | None = None,
) -> dict[str, float]:
    """Build per-motor watt map from joint state samples."""
    prof = _resolve_profile(profile)
    powers: dict[str, float] = {}
    for joint in joints:
        name = str(joint.get("name", "")).lower()
        if not name:
            continue
        if joint.get("power_w") is not None:
            powers[name] = clamp_motor_power_w(name, float(joint["power_w"]), prof)
            continue
        powers[name] = estimate_motor_power_w(
            joint.get("velocity", 0.0),
            joint.get("torque", 0.0),
            motor_name=name,
            profile=prof,
        )
    return powers


def stress_multiplier(*, gait: str = "stand", phase: str = "") -> float:
    """Combined gait/phase stress factor for Webots twin telemetry."""
    g = str(gait).lower()
    p = normalize_phase_name(phase) if phase else ""
    if p and p in _PHASE_STRESS:
        return _PHASE_STRESS[p]
    if phase and str(phase).lower() in _PHASE_STRESS:
        return _PHASE_STRESS[str(phase).lower()]
    return _GAIT_STRESS.get(g, 1.0)


def _compute_draw_w(gait: str, profile: dict) -> float:
    compute = profile.get("compute") or {}
    idle_w = float(compute.get("idle_w", COMPUTE_IDLE_W))
    active_w = float(compute.get("active_w", COMPUTE_ACTIVE_W))
    if gait in ("stand", "idle", "standby"):
        return idle_w
    return active_w


def aggregate_channel_draws(
    motor_power_w: dict[str, float],
    *,
    compute_w: float | None = None,
    gait: str = "stand",
    phase: str = "",
    profile: dict | None = None,
) -> dict[str, float]:
    """Sum motor draws into Legs / Arms / Torso / Compute channels."""
    prof = _resolve_profile(profile)
    channels: dict[str, float] = {}
    channel_caps = prof.get("channels") or {}

    for motor, watts in motor_power_w.items():
        ch = WEBOTS_MOTOR_CHANNELS.get(motor.lower())
        if ch:
            channels[ch] = round(channels.get(ch, 0.0) + float(watts), 2)

    if compute_w is None:
        compute_w = _compute_draw_w(gait, prof)
    channels["Compute"] = round(compute_w, 2)

    mult = stress_multiplier(gait=gait, phase=phase)
    if mult > 1.0:
        for ch_id in list(channels.keys()):
            if ch_id == "Compute":
                channels[ch_id] = round(channels[ch_id] * min(mult, 1.3), 2)
            else:
                channels[ch_id] = round(channels[ch_id] * mult, 2)

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
    battery = prof.get("battery") or {}
    capacity_wh = float(battery.get("capacity_wh", 480))

    motor_power_w = motor_power_w or motor_powers_from_joints(joints, prof)
    channel_draws = aggregate_channel_draws(
        motor_power_w, gait=gait, phase=phase, profile=prof
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
            "hardware_profile": prof.get("profile_id"),
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
        },
        "pose": pose or {},
        "sensors": sensors or {},
        "power": {
            "total_draw_w": round(sum(channel_draws.values()), 1),
            "channel_draws": channel_draws,
        },
    }