"""
Active cooling channel — thermal- and phase-aware draw estimate.

Role
----
ButlerBot can include a ``Cooling`` power channel (fans / dry-ice loop). This
module estimates the watts that channel should request based on:

  - Estimated system temperature (from SafetyMonitor thermal model)
  - Optional twin mission phase (transit / manipulate heat differently)
  - Config limits (``cooling.max_draw_w``, safety thermal thresholds)

ROS2BatterySource takes ``max(mission Cooling target, this estimate)`` so
cooling never under-responds during hot high-load phases.

Output
------
A single float watts value, clamped to [0.5, max_draw_w].
"""

from __future__ import annotations

from src.config import config


def estimate_cooling_draw_w(
    thermal_c: float,
    phase: str | None = None,
    *,
    ambient_c: float | None = None,
) -> float:
    """
    Estimate Cooling channel draw from temperature and mission phase.

    Base load is always on (electronics fans); phase and thermal bands add
    stepwise boosts rather than a continuous PID (sufficient for the sim pack).
    """
    safety = config.get("safety") or {}
    ambient = float(ambient_c if ambient_c is not None else safety.get("thermal_ambient_c", 22.0))
    warn = float(safety.get("thermal_warning_c", 55.0))
    crit = float(safety.get("thermal_critical_c", 68.0))
    max_w = float((config.get("cooling") or {}).get("max_draw_w", 10.0))

    base = 1.5
    key = (phase or "").lower()
    # Locomotion and manipulation dump more heat into the chassis.
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
        # Elevated but not yet at warning band — mild boost.
        base += 1.5

    return round(min(max_w, max(0.5, base)), 1)
