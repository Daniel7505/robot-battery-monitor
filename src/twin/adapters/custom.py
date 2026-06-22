"""Custom / ButlerBot script adapter — direct telemetry payloads."""

from __future__ import annotations

from src.twin.adapters.base import TwinAdapter
from src.twin.models import TwinTelemetry


class CustomAdapter(TwinAdapter):
    """Accept pre-normalized telemetry from custom scripts or ButlerBot examples."""

    name = "custom"

    def normalize(self, payload: dict) -> TwinTelemetry:
        adapter = payload.get("adapter", self.name)
        return TwinTelemetry.from_payload(payload, adapter=str(adapter))