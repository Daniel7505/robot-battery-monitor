"""
Webots adapter — maps Supervisor/controller payloads to twin telemetry.

Why prefer pre-built channel_draws
----------------------------------
The Webots controller typically posts output from ``build_webots_telemetry()``,
which already applied profile cruise curves, motion damping, and channel caps.
Recomputing from motors alone can *drop* that damping and disagree with the
on-robot HUD. Therefore:

  1. Use payload channel_draws / power.channel_draws when present.
  2. Else rebuild from motor_power_w or joints via webots_power helpers.
  3. Last resort: map raw motor id → channel with a static name table.

Task inference falls back to gait→task when mission.task is omitted so the
allocator still receives a coherent PMS task under partial payloads.
"""

from __future__ import annotations

from src.twin.adapters.base import TwinAdapter
from src.twin.models import TwinTelemetry
from src.teleop_agent import CONTROL_SPEED_CAP_M_S
from src.twin.webots_power import (
    WEBOTS_MOTOR_CHANNELS,
    aggregate_channel_draws,
    gait_to_task,
    motor_powers_from_joints,
    restamp_active_bom,
)
from src.twin_energy import LOCO_PASSTHROUGH_KEYS

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
        payload = restamp_active_bom(payload)
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
        # GPS spikes above the control cap are not body speed.
        if speed_f > CONTROL_SPEED_CAP_M_S:
            speed_f = CONTROL_SPEED_CAP_M_S
        elif speed_f < 0.0:
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

        loco = {
            "gait": gait,
            "speed_m_s": speed_f,
            "phase": phase,
            "mode": loc.get("mode") or "wheeled",
        }
        hw = payload.get("hardware") if isinstance(payload.get("hardware"), dict) else {}
        for key in ("bom_build", "bom_schema_version", "payload_kg", "wheel_radius_m"):
            if loc.get(key) is not None:
                loco[key] = loc[key]
            elif hw.get(key) is not None:
                loco[key] = hw[key]
        # None is a real value: torque feedback was missing, so measured
        # drive is na. Do not drop the key and let Legs stand in for it.
        for key in ("measured_drive_w", "measured_drive_elec_w"):
            if key in loc or key in payload:
                loco[key] = loc[key] if key in loc else payload.get(key)
        # Controller energy counters and the run id. restamp rebuilds
        # locomotion; these are copied back there and again here.
        for key in LOCO_PASSTHROUGH_KEYS:
            if key in loc:
                loco[key] = loc[key]
            elif key in payload:
                loco[key] = payload[key]
        return TwinTelemetry.from_payload(
            {
                "source": "webots",
                "robot": payload.get("robot", {"name": "ButlerBot"}),
                "mission": {"task": task},
                "channel_draws": draws,
                "subsystem_draws": payload.get("subsystem_draws") or {},
                "locomotion": loco,
                "hardware": hw,
                "pose": payload.get("pose", {}),
                "timestamp": payload.get("timestamp"),
                # Preserve controller sensors/control_diag (hub rates, ABS, locks)
                # — stripping these made residual-spin diagnosis blind.
                "sensors": payload.get("sensors") or {},
                "joints": payload.get("joints") or [],
            },
            adapter=self.name,
        )
