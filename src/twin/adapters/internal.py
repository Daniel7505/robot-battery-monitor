"""
Internal simulation adapter — backward-compatible passthrough.

When the PMS built-in ButlerBot loop (or tests) posts telemetry with
source/adapter ``internal`` or ``generic``, this adapter tags the payload
as internal and forwards fields through ``TwinTelemetry.from_payload``.

External-active logic in the bridge *excludes* source=internal, so internal
posts never steal the power feed from a live Webots connection, and never
activate the twin UI as "external" on their own.
"""

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
