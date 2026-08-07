"""
Digital twin data models — versioned telemetry and command contracts.

Why a normalized model exists
-----------------------------
Webots, PyBullet, custom scripts, and hardware feeds all publish different
JSON shapes. ``TwinTelemetry`` is the single intermediate form every adapter
must produce so ``DigitalTwinBridge`` and the power feed path never branch on
source-specific keys.

Schema version (``TWIN_SCHEMA_VERSION``) is advertised on export/schema
endpoints so external controllers can detect contract drift.

Validation intentionally stays light: unknown sources/tasks/channels become
errors at ingest time rather than silently poisoning the PMS channel map.
"""

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
    """Prefer live config channel IDs; fall back to the wheeled ButlerBot set."""
    channels = config.get("power_channels") or []
    ids = {ch.get("id") for ch in channels if ch.get("id")}
    return frozenset(ids) if ids else _DEFAULT_CHANNEL_IDS


@dataclass
class TwinTelemetry:
    """Normalized telemetry from any simulator or hardware feed.

    Fields map to the PMS view of the robot:
      - battery_pct / capacity: pack state of charge and size
      - channel_draws: watts per PMS channel (Legs, Arms, …)
      - task / throttle: mission posture the allocator should see
      - locomotion / pose: UI + agent context (gait, phase, speed)
      - raw: original payload so controller-only sensors survive export

    Residual-spin observability
    ---------------------------
    GPS on the Webots body is translation-only: pure yaw (in-place spin) can
    report speed ≈ 0 while hubs still rotate. Controllers therefore stamp
    wheel rates, yaw_rate, residual_spin, etc. under ``raw["sensors"]``.
    ``to_dict()`` re-exposes that block so the dashboard can show hub spin even
    when GPS speed looks idle — critical for validating stop/ABS behavior.
    """

    source: str = "external"
    adapter: str = "generic"
    timestamp: datetime = field(default_factory=lambda: datetime.now(timezone.utc))
    robot_name: str = "ButlerBot"
    battery_pct: float | None = None
    battery_capacity_wh: float | None = None
    hardware_profile: str | None = None
    task: str | None = None
    channel_draws: dict[str, float] = field(default_factory=dict)
    throttle: float | None = None
    locomotion: dict[str, Any] = field(default_factory=dict)
    pose: dict[str, Any] = field(default_factory=dict)
    raw: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_payload(cls, payload: dict, adapter: str = "generic") -> TwinTelemetry:
        """Parse a heterogeneous POST body into a ``TwinTelemetry`` instance.

        Accepts several historical key aliases (sensor_draws, mission_task,
        main_battery_pct vs battery_pct) so older controllers keep working
        without dual code paths in the bridge.
        """
        robot = payload.get("robot") or {}
        mission = payload.get("mission") or {}
        power = payload.get("power") or {}
        # Alias cascade: top-level → legacy sensor_draws → nested power block.
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
            battery_capacity_wh=_optional_float(
                robot.get("battery_capacity_wh") or robot.get("capacity_wh")
            ),
            hardware_profile=(
                robot.get("hardware_profile")
                or power.get("hardware_profile")
                or payload.get("hardware_profile")
            ),
            task=task,
            channel_draws=cleaned_draws,
            throttle=_optional_float(payload.get("throttle") or power.get("throttle_factor")),
            locomotion=dict(payload.get("locomotion") or {}),
            pose=dict(payload.get("pose") or {}),
            # Keep the full original dict — sensors and joints live only here
            # until to_dict() selectively re-exports them.
            raw=dict(payload),
        )

    def validate(self) -> list[str]:
        """Return human-readable contract errors (empty list = accept)."""
        errors: list[str] = []
        if self.source not in _VALID_SOURCES:
            errors.append(f"Unknown source: {self.source}")
        if self.task and self.task not in _VALID_TASKS:
            errors.append(f"Invalid task: {self.task}")
        # Throttle is a scale factor, not a percent; 0 is disallowed (would
        # freeze motion without an explicit stop path).
        if self.throttle is not None and not 0.0 < self.throttle <= 1.0:
            errors.append("Throttle must be in (0, 1]")
        for ch_id in self.channel_draws:
            if ch_id not in _channel_ids():
                errors.append(f"Unknown channel: {ch_id}")
        return errors

    def to_dict(self) -> dict:
        """Serialize for bridge status / external_feed export.

        Preserve controller sensors (wheel rates, yaw_rate, residual_spin) so
        dashboard/live checks can see hub spin even when GPS speed is ~0.
        That is the residual-spin observability contract: translation GPS alone
        is not a reliable "robot stopped" signal under pure yaw.
        """
        sensors = {}
        if isinstance(self.raw, dict):
            sensors = dict(self.raw.get("sensors") or {})
        return {
            "source": self.source,
            "adapter": self.adapter,
            "timestamp": self.timestamp.isoformat(),
            "robot_name": self.robot_name,
            "battery_pct": self.battery_pct,
            "battery_capacity_wh": self.battery_capacity_wh,
            "hardware_profile": self.hardware_profile,
            "robot": {
                "name": self.robot_name,
                "main_battery_pct": self.battery_pct,
                "battery_capacity_wh": self.battery_capacity_wh,
                "hardware_profile": self.hardware_profile,
            },
            "task": self.task,
            "channel_draws": self.channel_draws,
            "throttle": self.throttle,
            "locomotion": self.locomotion,
            "pose": self.pose,
            "sensors": sensors,
        }


def _optional_float(value) -> float | None:
    """Coerce numeric-ish values; treat missing/invalid as unset (None)."""
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None
