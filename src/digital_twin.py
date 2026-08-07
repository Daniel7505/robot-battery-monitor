"""
Backward-compatible aliases for the DigitalTwinBridge package.

Architecture
------------
The live digital-twin implementation lives under ``src/twin/``:

  Webots / custom feed  →  adapters  →  TwinTelemetry  →  DigitalTwinBridge
                                                   ↘ PowerFeed → hardware layer
                                                   ↘ teleop commands → Webots poll

This module exists so older imports keep working::

    from src.digital_twin import DigitalTwinBridge, get_twin_bridge

Prefer the package path for new code::

    from src.twin import DigitalTwinBridge, get_twin_bridge

``DigitalTwinInterface`` / ``get_twin_interface`` are historical names for the
same bridge singleton; they do not implement a separate interface layer.
"""

from src.twin.bridge import DigitalTwinBridge, get_twin_bridge, reset_twin_bridge
from src.twin.models import TWIN_SCHEMA_VERSION

# Historical aliases — same objects, different names used in early docs/tests.
DigitalTwinInterface = DigitalTwinBridge
get_twin_interface = get_twin_bridge
reset_twin_interface = reset_twin_bridge

__all__ = [
    "DigitalTwinBridge",
    "DigitalTwinInterface",
    "TWIN_SCHEMA_VERSION",
    "get_twin_bridge",
    "get_twin_interface",
    "reset_twin_bridge",
    "reset_twin_interface",
]
