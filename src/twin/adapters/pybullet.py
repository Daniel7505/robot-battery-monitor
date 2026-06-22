"""PyBullet adapter — map joint torques and velocities to PMS channel draws."""

from __future__ import annotations

from src.twin.adapters.base import TwinAdapter
from src.twin.models import TwinTelemetry

# ButlerBot joint groups → power channels (servo-scale watts estimate)
_PYBULLET_JOINT_MAP = {
    "hip": "Legs",
    "knee": "Legs",
    "ankle": "Legs",
    "leg": "Legs",
    "shoulder": "Arms",
    "elbow": "Arms",
    "wrist": "Arms",
    "gripper": "Arms",
    "arm": "Arms",
    "spine": "Torso",
    "torso": "Torso",
    "pelvis": "Torso",
}


class PyBulletAdapter(TwinAdapter):
    """Estimate channel draw from PyBullet joint telemetry."""

    name = "pybullet"

    def normalize(self, payload: dict) -> TwinTelemetry:
        draws = dict(payload.get("channel_draws") or {})
        if not draws:
            draws = _torques_to_draws(payload.get("joints") or payload.get("joint_states") or [])

        locomotion = dict(payload.get("locomotion") or {})
        if payload.get("base_velocity"):
            locomotion["base_velocity"] = payload["base_velocity"]
        if payload.get("contact_points") is not None:
            locomotion["contact_points"] = payload["contact_points"]

        task = payload.get("task")
        if not task and locomotion.get("mode"):
            task = _mode_to_task(locomotion["mode"])

        return TwinTelemetry.from_payload(
            {
                "source": "pybullet",
                "robot": payload.get("robot", {"name": "ButlerBot"}),
                "mission": {"task": task},
                "channel_draws": draws,
                "locomotion": locomotion,
                "pose": payload.get("pose", {}),
                "timestamp": payload.get("timestamp"),
            },
            adapter=self.name,
        )


def _torques_to_draws(joints: list) -> dict[str, float]:
    """Convert joint torque/velocity samples to channel watt estimates."""
    channel_torque: dict[str, float] = {}
    for joint in joints:
        if isinstance(joint, dict):
            name = str(joint.get("name", "")).lower()
            torque = abs(float(joint.get("torque", joint.get("applied_torque", 0))))
            velocity = abs(float(joint.get("velocity", joint.get("joint_velocity", 0))))
        else:
            continue
        power_w = round(torque * velocity * 0.12 + torque * 0.8, 2)
        channel = "Compute"
        for key, ch in _PYBULLET_JOINT_MAP.items():
            if key in name:
                channel = ch
                break
        channel_torque[channel] = round(channel_torque.get(channel, 0.0) + power_w, 2)

    if channel_torque and "Compute" not in channel_torque:
        channel_torque["Compute"] = 8.5
    return channel_torque


def _mode_to_task(mode: str) -> str:
    m = str(mode).lower()
    if m in ("stand", "idle"):
        return "idle"
    if m in ("walk", "step"):
        return "moving"
    if m in ("patrol",):
        return "balanced"
    if m in ("manipulate", "lift"):
        return "high_load"
    return "balanced"