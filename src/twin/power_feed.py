"""
Power feed contract — stable shape for live battery % + channel watts.

Webots (or any external twin) publishes this shape via POST /api/twin/telemetry.
DigitalTwinBridge normalizes it; ROS2BatterySource writes it into last_readings
so the existing dashboard broadcaster needs no parallel data path.

Schema (minimal):
  {
    "source": "webots",                 # webots | custom | pybullet | hardware | external
    "robot": {"main_battery_pct": 87.5},
    "channel_draws": {
      "Legs": 18.2, "Arms": 6.0, "Torso": 7.5, "Compute": 9.0, "Cooling": 3.0
    },
    "mission": {"task": "moving"}       # optional
  }

Fallback: when no fresh external feed, ROS2BatterySource keeps internal
SimulationDriver / mock ROS2 behavior unchanged.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any


POWER_FEED_SCHEMA_VERSION = "1.0"

# Wheeled ButlerBot channels (matches config power_channels)
WHEELED_CHANNEL_IDS = ("Legs", "Arms", "Torso", "Compute", "Cooling")


@dataclass(frozen=True)
class PowerFeed:
    """Authoritative live power numbers for the dashboard hardware layer."""

    source: str
    battery_pct: float | None = None
    channel_draws: dict[str, float] = field(default_factory=dict)
    task: str | None = None
    throttle: float | None = None
    timestamp: datetime = field(default_factory=lambda: datetime.now(timezone.utc))
    locomotion: dict[str, Any] = field(default_factory=dict)

    def usable(self) -> bool:
        return self.battery_pct is not None or bool(self.channel_draws)

    def to_dict(self) -> dict:
        return {
            "schema_version": POWER_FEED_SCHEMA_VERSION,
            "source": self.source,
            "battery_pct": self.battery_pct,
            "channel_draws": dict(self.channel_draws),
            "task": self.task,
            "throttle": self.throttle,
            "timestamp": self.timestamp.isoformat(),
            "locomotion": dict(self.locomotion),
        }


def power_feed_from_telemetry(tel) -> PowerFeed | None:
    """Build a PowerFeed from a TwinTelemetry instance (or None)."""
    if tel is None:
        return None
    draws = {
        str(k): round(float(v), 2)
        for k, v in (getattr(tel, "channel_draws", None) or {}).items()
        if v is not None
    }
    battery = getattr(tel, "battery_pct", None)
    if battery is not None:
        try:
            battery = max(0.0, min(100.0, float(battery)))
        except (TypeError, ValueError):
            battery = None
    feed = PowerFeed(
        source=str(getattr(tel, "source", "external") or "external"),
        battery_pct=battery,
        channel_draws=draws,
        task=getattr(tel, "task", None),
        throttle=getattr(tel, "throttle", None),
        timestamp=getattr(tel, "timestamp", None) or datetime.now(timezone.utc),
        locomotion=dict(getattr(tel, "locomotion", None) or {}),
    )
    return feed if feed.usable() else None
