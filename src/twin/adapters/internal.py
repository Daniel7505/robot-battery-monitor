"""Internal simulation adapter — backward-compatible passthrough."""

from __future__ import annotations

from src.twin.adapters.base import TwinAdapter
from src.twin.models import TwinTelemetry


class InternalAdapter(TwinAdapter):
    """Represents telemetry produced by the built-in ButlerBot simulation loop."""

    name = "internal"

    def normalize(self, payload: dict) -> TwinTelemetry:
        return TwinTelemetry.from_payload(
            {
                "source": "internal",
                "robot": payload.get("robot", {}),
                "mission": payload.get("mission", {}),
                "channel_draws": payload.get("channel_draws", {}),
                **payload,
            },
            adapter=self.name,
        )