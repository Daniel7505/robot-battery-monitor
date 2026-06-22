"""
Backward-compatible aliases for the DigitalTwinBridge package.

Prefer: from src.twin import DigitalTwinBridge, get_twin_bridge
"""

from src.twin.bridge import DigitalTwinBridge, get_twin_bridge, reset_twin_bridge
from src.twin.models import TWIN_SCHEMA_VERSION

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