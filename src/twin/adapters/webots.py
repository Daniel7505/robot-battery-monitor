"""Webots adapter — maps Supervisor/controller payloads to twin telemetry."""

from __future__ import annotations

from src.twin.adapters.base import TwinAdapter
from src.twin.models import TwinTelemetry
from src.twin.webots_power import (
    WEBOTS_MOTOR_CHANNELS,
    aggregate_channel_draws,
    gait_to_task,
    motor_powers_from_joints,
)

_WEBOTS_CHANNEL_MAP = dict(WEBOTS_MOTOR_CHANNELS)
_WEBOTS_CHANNEL_MAP.update({
    "leg_motors": "Legs",
    "leg_left": "Legs",
    "leg_right": "Legs",
    "arm_motors": "Arms",
    "torso_motor": "Torso",
    "balance": "Torso",
    "compute": "Compute",
    "onboard_computer": "Compute",
})


class WebotsAdapter(TwinAdapter):
    """Normalize Webots supervisor JSON into PMS channel draws."""

    name = "webots"

    def normalize(self, payload: dict) -> TwinTelemetry:
        motor_power = dict(payload.get("motor_power_w") or payload.get("motors") or {})
        if not motor_power and payload.get("joints"):
            motor_power = motor_powers_from_joints(payload["joints"])

        loc = payload.get("locomotion") or {}
        gait = payload.get("gait") or loc.get("gait", "stand")
        phase = payload.get("phase") or loc.get("phase") or ""
        draws = (
            aggregate_channel_draws(motor_power, gait=gait, phase=phase)
            if motor_power
            else {}
        )
        if not draws:
            for motor_id, watts in motor_power.items():
                channel = _WEBOTS_CHANNEL_MAP.get(str(motor_id).lower(), str(motor_id))
                if channel in {"Legs", "Arms", "Torso", "Compute"}:
                    draws[channel] = round(draws.get(channel, 0.0) + float(watts), 2)

        speed = payload.get("speed_m_s") or (payload.get("locomotion") or {}).get("speed_m_s")
        task = payload.get("task") or (payload.get("mission") or {}).get("task")
        if not task and gait:
            task = gait_to_task(gait)

        return TwinTelemetry.from_payload(
            {
                "source": "webots",
                "robot": payload.get("robot", {"name": "ButlerBot"}),
                "mission": {"task": task},
                "channel_draws": draws or payload.get("channel_draws", {}),
                "locomotion": {
                    "gait": gait,
                    "speed_m_s": speed,
                    "phase": payload.get("phase"),
                },
                "pose": payload.get("pose", {}),
                "timestamp": payload.get("timestamp"),
            },
            adapter=self.name,
        )