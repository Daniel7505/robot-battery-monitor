"""
Digital twin package — bridge, adapters, power contract, and ButlerBot examples.

What this package does
----------------------
Connects an external robot simulation (primarily Webots ButlerBot) to the
onboard Power Management System (PMS) without rewriting the dashboard or
hardware layer.

Data path (inbound telemetry)::

    Simulator controller
        → POST /api/twin/telemetry  (JSON payload)
        → adapter.normalize()       (source-specific → TwinTelemetry)
        → DigitalTwinBridge.ingest
        → PowerFeed (battery % + channel watts)
        → ROS2BatterySource / hardware.apply_power_feed
        → dashboard last_readings (same path as internal sim)

Data path (outbound control)::

    Dashboard / API / onboard agent
        → POST /api/twin/command
        → DigitalTwinBridge.apply_command
        → mission/throttle inject into PMS
        → teleop left/right + stop_epoch for Webots to poll via GET state

Why a bridge exists
-------------------
Internal SimulationDriver and external Webots must share one dashboard. The
bridge arbitrates: when a fresh non-stale external feed is present and
``prefer_external`` is set, live watts come from the twin; otherwise the
built-in ButlerBot simulation is unchanged.

Key types re-exported here are the public integration surface for scripts and
tests. Adapters and Webots power math stay as submodules so integrators can
import only what they need.
"""

from src.twin.bridge import DigitalTwinBridge, get_twin_bridge, reset_twin_bridge
from src.twin.models import TWIN_SCHEMA_VERSION, TwinTelemetry
from src.twin.butlerbot import BUTLERBOT_WALKING_FLOW, butlerbot_telemetry_step
from src.twin.power_feed import POWER_FEED_SCHEMA_VERSION, PowerFeed, power_feed_from_telemetry

__all__ = [
    "DigitalTwinBridge",
    "TwinTelemetry",
    "TWIN_SCHEMA_VERSION",
    "get_twin_bridge",
    "reset_twin_bridge",
    "BUTLERBOT_WALKING_FLOW",
    "butlerbot_telemetry_step",
    "PowerFeed",
    "POWER_FEED_SCHEMA_VERSION",
    "power_feed_from_telemetry",
]
