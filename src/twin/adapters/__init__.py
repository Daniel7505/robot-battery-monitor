"""
Twin adapter registry — map adapter name → normalizer implementation.

How adapters fit the architecture
---------------------------------
``DigitalTwinBridge.ingest_telemetry`` never parses source-specific JSON itself.
It resolves an adapter by name and calls ``normalize(payload) → TwinTelemetry``.

Registered names::

    internal / generic  → InternalAdapter   (built-in sim passthrough)
    custom / butlerbot  → CustomAdapter     (pre-shaped script payloads)
    webots              → WebotsAdapter     (joints / motor maps / prebuilt draws)
    pybullet            → PyBulletAdapter   (torque·velocity estimates)

Unknown names fall back to ``generic`` (InternalAdapter) so a misconfigured
client still gets a best-effort parse rather than a hard crash.
"""

from src.twin.adapters.base import TwinAdapter
from src.twin.adapters.custom import CustomAdapter
from src.twin.adapters.internal import InternalAdapter
from src.twin.adapters.pybullet import PyBulletAdapter
from src.twin.adapters.webots import WebotsAdapter

_custom = CustomAdapter()

_ADAPTERS: dict[str, TwinAdapter] = {
    "internal": InternalAdapter(),
    "generic": InternalAdapter(),
    "custom": _custom,
    "butlerbot": _custom,
    "webots": WebotsAdapter(),
    "pybullet": PyBulletAdapter(),
}


def get_adapter(name: str) -> TwinAdapter:
    """Return the adapter for ``name``, or the generic passthrough."""
    return _ADAPTERS.get(name, _ADAPTERS["generic"])


def registered_adapters() -> list[str]:
    """Sorted adapter names for schema/status discovery."""
    return sorted(_ADAPTERS.keys())
