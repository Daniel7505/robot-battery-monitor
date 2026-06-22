"""Twin adapter base — normalize external simulator payloads to TwinTelemetry."""

from __future__ import annotations

from abc import ABC, abstractmethod

from src.twin.models import TwinTelemetry


class TwinAdapter(ABC):
    """Pluggable adapter for Webots, PyBullet, custom scripts, or internal passthrough."""

    name: str = "generic"

    @abstractmethod
    def normalize(self, payload: dict) -> TwinTelemetry:
        """Convert adapter-specific payload into standard twin telemetry."""

    def validate_payload(self, payload: dict) -> list[str]:
        return self.normalize(payload).validate()