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
        loc = payload.get("locomotion") or {}
        gait = payload.get("gait") or loc.get("gait", "stand")
        phase = payload.get("phase") or loc.get("phase") or ""
        speed = payload.get("speed_m_s")
        if speed is None:
            speed = loc.get("speed_m_s")
        try:
            speed_f = float(speed) if speed is not None else 0.0
        except (TypeError, ValueError):
            speed_f = 0.0

        # Prefer pre-built channel_draws from the controller (build_webots_telemetry).
        # Recomputing from motors alone drops motion damping and can disagree with the HUD.
        prebuilt = (
            payload.get("channel_draws")
            or (payload.get("power") or {}).get("channel_draws")
            or {}
        )
        draws: dict[str, float] = {}
        if isinstance(prebuilt, dict) and prebuilt:
            for ch_id, value in prebuilt.items():
                if value is None:
                    continue
                try:
                    draws[str(ch_id)] = round(float(value), 2)
                except (TypeError, ValueError):
                    continue

        if not draws:
            motor_power = dict(payload.get("motor_power_w") or payload.get("motors") or {})
            if not motor_power and payload.get("joints"):
                motor_power = motor_powers_from_joints(payload["joints"])
            if motor_power:
                draws = aggregate_channel_draws(
                    motor_power,
                    gait=gait,
                    phase=phase,
                    speed_m_s=speed_f,
                    joints=payload.get("joints"),
                )
            if not draws:
                for motor_id, watts in motor_power.items():
                    channel = _WEBOTS_CHANNEL_MAP.get(str(motor_id).lower(), str(motor_id))
                    if channel in {"Legs", "Arms", "Torso", "Compute"}:
                        draws[channel] = round(draws.get(channel, 0.0) + float(watts), 2)

        task = payload.get("task") or (payload.get("mission") or {}).get("task")
        if not task and gait:
            task = gait_to_task(gait)

        return TwinTelemetry.from_payload(
            {
                "source": "webots",
                "robot": payload.get("robot", {"name": "ButlerBot"}),
                "mission": {"task": task},
                "channel_draws": draws,
                "locomotion": {
                    "gait": gait,
                    "speed_m_s": speed_f,
                    "phase": phase,
                    "mode": loc.get("mode") or "wheeled",
                },
                "pose": payload.get("pose", {}),
                "timestamp": payload.get("timestamp"),
            },
            adapter=self.name,
        )