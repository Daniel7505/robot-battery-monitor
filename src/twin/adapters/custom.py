"""
Custom / ButlerBot script adapter — direct telemetry payloads.

Used by ``examples/butlerbot_twin_feed.py`` and any integrator that already
shapes JSON to the twin schema. No motor math is applied — the payload is
handed to ``TwinTelemetry.from_payload`` as-is (with adapter name preserved
when the client set one, e.g. adapter=butlerbot).

Prefer this path for HTTP feed demos; use WebotsAdapter when joint/motor
telemetry must be converted into channel watts.
"""

from __future__ import annotations

from src.twin.adapters.base import TwinAdapter
from src.twin.models import TwinTelemetry


class CustomAdapter(TwinAdapter):
    """Accept pre-normalized telemetry from custom scripts or ButlerBot examples."""

    name = "custom"

    def normalize(self, payload: dict) -> TwinTelemetry:
        adapter = payload.get("adapter", self.name)
        return TwinTelemetry.from_payload(payload, adapter=str(adapter))
