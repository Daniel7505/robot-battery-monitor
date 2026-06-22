"""
Webots power estimation — map motor/joint telemetry to PMS channel draws.

Shared between the Webots controller and unit tests. Servo-scale watts for
ButlerBot (wheeled base + torso + arms + compute).
"""

from __future__ import annotations

from datetime import datetime, timezone

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

COMPUTE_IDLE_W = 7.5
COMPUTE_ACTIVE_W = 10.5

# Stress multipliers — higher reported draw during active Webots phases
_GAIT_STRESS: dict[str, float] = {
    "walk": 2.1,
    "drive": 2.0,
    "transit": 2.0,
    "patrol": 1.55,
    "manipulate": 2.4,
    "high_load": 2.4,
}
_PHASE_STRESS: dict[str, float] = {
    "walk_transit": 2.35,
    "patrol": 1.6,
    "manipulate": 2.75,
    "return_idle": 1.15,
    "standby": 1.0,
}

_GAIT_TO_TASK = {
    "stand": "idle",
    "idle": "idle",
    "standby": "idle",
    "walk": "moving",
    "drive": "moving",
    "transit": "moving",
    "patrol": "balanced",
    "balanced": "balanced",
    "manipulate": "high_load",
    "grasp": "high_load",
    "high_load": "high_load",
}


def estimate_motor_power_w(
    velocity: float,
    torque: float,
    *,
    motor_idle_w: float = 1.2,
    scale: float = 4.2,
) -> float:
    """Estimate electrical draw from joint velocity (rad/s) and torque (Nm)."""
    mechanical = abs(float(torque) * float(velocity))
    return round(motor_idle_w + mechanical * scale, 2)


def motor_powers_from_joints(joints: list[dict]) -> dict[str, float]:
    """Build per-motor watt map from joint state samples."""
    powers: dict[str, float] = {}
    for joint in joints:
        name = str(joint.get("name", "")).lower()
        if not name:
            continue
        powers[name] = estimate_motor_power_w(
            joint.get("velocity", 0.0),
            joint.get("torque", 0.0),
            motor_idle_w=1.0 if "wheel" in name else 1.4,
        )
    return powers


def stress_multiplier(*, gait: str = "stand", phase: str = "") -> float:
    """Combined gait/phase stress factor for Webots twin telemetry."""
    g = str(gait).lower()
    p = str(phase).lower()
    if p and p in _PHASE_STRESS:
        return _PHASE_STRESS[p]
    return _GAIT_STRESS.get(g, 1.0)


def aggregate_channel_draws(
    motor_power_w: dict[str, float],
    *,
    compute_w: float | None = None,
    gait: str = "stand",
    phase: str = "",
) -> dict[str, float]:
    """Sum motor draws into Legs / Arms / Torso / Compute channels."""
    channels: dict[str, float] = {}
    for motor, watts in motor_power_w.items():
        ch = WEBOTS_MOTOR_CHANNELS.get(motor.lower())
        if ch:
            channels[ch] = round(channels.get(ch, 0.0) + float(watts), 2)

    if compute_w is None:
        compute_w = COMPUTE_ACTIVE_W if gait not in ("stand", "idle", "standby") else COMPUTE_IDLE_W
    channels["Compute"] = round(compute_w, 2)

    mult = stress_multiplier(gait=gait, phase=phase)
    if mult > 1.0:
        for ch_id in list(channels.keys()):
            if ch_id == "Compute":
                channels[ch_id] = round(channels[ch_id] * min(mult, 1.35), 2)
            else:
                channels[ch_id] = round(channels[ch_id] * mult, 2)
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
) -> dict:
    """Build a DigitalTwinBridge telemetry payload from Webots controller data."""
    motor_power_w = motor_power_w or motor_powers_from_joints(joints)
    channel_draws = aggregate_channel_draws(motor_power_w, gait=gait, phase=phase)
    task = gait_to_task(gait)

    return {
        "schema_version": "1.1",
        "source": "webots",
        "adapter": "webots",
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "robot": {
            "name": robot_name,
            "main_battery_pct": round(battery_pct, 2),
            "battery_capacity_wh": 480,
        },
        "mission": {"task": task, "phase": phase},
        "motor_power_w": motor_power_w,
        "channel_draws": channel_draws,
        "joints": joints,
        "locomotion": {
            "gait": gait,
            "speed_m_s": round(speed_m_s, 3),
            "phase": phase,
        },
        "pose": pose or {},
        "sensors": sensors or {},
        "power": {
            "total_draw_w": round(sum(channel_draws.values()), 1),
            "channel_draws": channel_draws,
        },
    }