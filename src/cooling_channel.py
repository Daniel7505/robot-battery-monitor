"""
Active cooling channel — dry-ice / fan loop tied to thermal stress and twin phase.
"""

from __future__ import annotations

from src.config import config


def estimate_cooling_draw_w(
    thermal_c: float,
    phase: str | None = None,
    *,
    ambient_c: float | None = None,
) -> float:
    """Estimate Cooling channel draw from temperature and mission phase."""
    safety = config.get("safety") or {}
    ambient = float(ambient_c if ambient_c is not None else safety.get("thermal_ambient_c", 22.0))
    warn = float(safety.get("thermal_warning_c", 55.0))
    crit = float(safety.get("thermal_critical_c", 68.0))
    max_w = float((config.get("cooling") or {}).get("max_draw_w", 10.0))

    base = 1.5
    key = (phase or "").lower()
    if key in ("drive_transit", "walk_transit", "patrol"):
        base += 1.5
    elif key == "manipulate":
        base += 4.0
    elif key in ("standby", "return_idle"):
        base += 0.5

    if thermal_c >= crit:
        base += 5.0
    elif thermal_c >= warn:
        base += 3.0
    elif thermal_c > ambient + 12:
        base += 1.5

    return round(min(max_w, max(0.5, base)), 1)