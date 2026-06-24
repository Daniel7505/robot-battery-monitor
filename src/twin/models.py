"""Digital twin data models — versioned telemetry and command contracts."""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any

from src.config import config
from src.mission_tasks import TASK_PROFILES

TWIN_SCHEMA_VERSION = "1.1"
_VALID_TASKS = frozenset(TASK_PROFILES.keys())
_VALID_SOURCES = frozenset({"internal", "external", "webots", "pybullet", "custom", "hardware"})
_DEFAULT_CHANNEL_IDS = frozenset({"Legs", "Arms", "Torso", "Compute", "Cooling"})


def _channel_ids() -> frozenset[str]:
    channels = config.get("power_channels") or []
    ids = {ch.get("id") for ch in channels if ch.get("id")}
    return frozenset(ids) if ids else _DEFAULT_CHANNEL_IDS


@dataclass
class TwinTelemetry:
    """Normalized telemetry from any simulator or hardware feed."""

    source: str = "external"
    adapter: str = "generic"
    timestamp: datetime = field(default_factory=lambda: datetime.now(timezone.utc))
    robot_name: str = "ButlerBot"
    battery_pct: float | None = None
    task: str | None = None
    channel_draws: dict[str, float] = field(default_factory=dict)
    throttle: float | None = None
    locomotion: dict[str, Any] = field(default_factory=dict)
    pose: dict[str, Any] = field(default_factory=dict)
    raw: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_payload(cls, payload: dict, adapter: str = "generic") -> TwinTelemetry:
        robot = payload.get("robot") or {}
        mission = payload.get("mission") or {}
        power = payload.get("power") or {}
        draws = (
            payload.get("channel_draws")
            or payload.get("sensor_draws")
            or power.get("channel_draws")
            or {}
        )
        task = (
            payload.get("task")
            or mission.get("task")
            or payload.get("mission_task")
        )
        if task:
            task = str(task).strip().lower()

        cleaned_draws: dict[str, float] = {}
        for ch_id, value in (draws or {}).items():
            if value is None:
                continue
            cleaned_draws[str(ch_id)] = round(float(value), 2)

        ts_raw = payload.get("timestamp")
        ts = datetime.now(timezone.utc)
        if isinstance(ts_raw, str):
            try:
                ts = datetime.fromisoformat(ts_raw.replace("Z", "+00:00"))
            except ValueError:
                pass

        return cls(
            source=str(payload.get("source", "external")).lower(),
            adapter=adapter,
            timestamp=ts,
            robot_name=robot.get("name") or payload.get("robot_name", "ButlerBot"),
            battery_pct=_optional_float(robot.get("main_battery_pct") or robot.get("battery_pct")),
            task=task,
            channel_draws=cleaned_draws,
            throttle=_optional_float(payload.get("throttle") or power.get("throttle_factor")),
            locomotion=dict(payload.get("locomotion") or {}),
            pose=dict(payload.get("pose") or {}),
            raw=dict(payload),
        )

    def validate(self) -> list[str]:
        errors: list[str] = []
        if self.source not in _VALID_SOURCES:
            errors.append(f"Unknown source: {self.source}")
        if self.task and self.task not in _VALID_TASKS:
            errors.append(f"Invalid task: {self.task}")
        if self.throttle is not None and not 0.0 < self.throttle <= 1.0:
            errors.append("Throttle must be in (0, 1]")
        for ch_id in self.channel_draws:
            if ch_id not in _channel_ids():
                errors.append(f"Unknown channel: {ch_id}")
        return errors

    def to_dict(self) -> dict:
        return {
            "source": self.source,
            "adapter": self.adapter,
            "timestamp": self.timestamp.isoformat(),
            "robot_name": self.robot_name,
            "battery_pct": self.battery_pct,
            "task": self.task,
            "channel_draws": self.channel_draws,
            "throttle": self.throttle,
            "locomotion": self.locomotion,
            "pose": self.pose,
        }


def _optional_float(value) -> float | None:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None