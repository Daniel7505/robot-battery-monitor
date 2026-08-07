"""
Reference hardware profiles for grounded power estimation.

Role in the system
------------------
YAML profiles under ``config/hardware_profiles/<id>.yaml`` describe ButlerBot
variants (e.g. wheeled vs biped): pack capacity, motor specs, phase reference
draws/durations, geometry. Consumers include:

  - src/twin/webots_power.py  (physics-based channel draws)
  - battery_capacity_wh()    (SOC integration / forecasts)
  - mission_forecast         (loop energy using phase_draw_w / phase_duration_s)

Selection
---------
``hardware_profile`` top-level config, or ``robot.hardware_profile``, default
``butlerbot_wheeled``.

Invariants
----------
- Missing/invalid YAML → ``_FALLBACK_PROFILE`` (never raises to callers).
- ``load_hardware_profile`` is LRU-cached; call ``clear_profile_cache()`` in tests.
"""

from __future__ import annotations

from functools import lru_cache
from pathlib import Path

import yaml

from src.config import config
from src.logger import logger

_PROFILES_DIR = Path(__file__).resolve().parent.parent / "config" / "hardware_profiles"

_FALLBACK_PROFILE = {
    "profile_id": "default",
    "label": "Default (config.yaml channels)",
    "battery": {"capacity_wh": 480, "nominal_voltage_v": 48},
    "compute": {"idle_w": 7.5, "active_w": 10.5, "peak_w": 22.0},
    "sensors": {"idle_w": 0.0, "active_w": 0.0},
    "stabilizers": {"idle_w": 0.0, "active_w": 0.0, "channel": "Torso"},
    "modes": {},
    "geometry": {"wheel_radius_m": 0.08},
    "motors": {},
    "channels": {},
    "phase_draw_w": {},
    "phase_duration_s": {},
}


@lru_cache(maxsize=8)
def load_hardware_profile(profile_id: str) -> dict:
    """Load a hardware profile YAML by id (cached)."""
    path = _PROFILES_DIR / f"{profile_id}.yaml"
    if not path.is_file():
        logger.warning(f"Hardware profile not found: {path}")
        return dict(_FALLBACK_PROFILE)
    try:
        with open(path, encoding="utf-8") as fh:
            data = yaml.safe_load(fh) or {}
        if not isinstance(data, dict):
            return dict(_FALLBACK_PROFILE)
        return data
    except Exception as exc:
        logger.error(f"Failed to load hardware profile {profile_id}: {exc}")
        return dict(_FALLBACK_PROFILE)


def get_active_profile_id() -> str:
    """Resolved profile id from config (wheeled ButlerBot by default)."""
    return (
        config.get("hardware_profile")
        or config.get("robot", "hardware_profile")
        or "butlerbot_wheeled"
    )


def get_active_profile() -> dict:
    return load_hardware_profile(get_active_profile_id())


def clear_profile_cache() -> None:
    """Drop cached YAML (tests / hot-reload)."""
    load_hardware_profile.cache_clear()


def motor_spec(profile: dict, motor_name: str) -> dict:
    """Lookup motor block; tries exact then lowercased key."""
    motors = profile.get("motors") or {}
    return dict(motors.get(motor_name) or motors.get(motor_name.lower()) or {})


def battery_capacity_wh(profile: dict | None = None) -> float:
    """Pack energy (Wh) — profile first, then config robot/simulation."""
    prof = profile or get_active_profile()
    batt = prof.get("battery") or {}
    if batt.get("capacity_wh") is not None:
        return float(batt["capacity_wh"])
    sim = config.get("simulation") or {}
    if sim.get("battery_capacity_wh") is not None:
        return float(sim["battery_capacity_wh"])
    return float(config.get("robot", "main_battery_capacity_wh", 480) or 480)


def battery_nominal_voltage_v(profile: dict | None = None) -> float:
    prof = profile or get_active_profile()
    batt = prof.get("battery") or {}
    return float(batt.get("nominal_voltage_v", 48))


def wheel_radius_m(profile: dict | None = None) -> float:
    prof = profile or get_active_profile()
    geom = prof.get("geometry") or {}
    return float(geom.get("wheel_radius_m", 0.08) or 0.08)


def clamp_motor_power_w(motor_name: str, watts: float, profile: dict | None = None) -> float:
    """Clamp estimated draw to reference motor peak power."""
    prof = profile or get_active_profile()
    spec = motor_spec(prof, motor_name)
    peak = float(spec.get("peak_w") or spec.get("peak_power_w") or 120)
    return round(min(max(0.0, watts), peak), 2)


def motor_idle_and_scale(
    motor_name: str, profile: dict | None = None
) -> tuple[float, float, float]:
    """
    Return (idle_w, scale, torque_proxy_coeff) for the velocity/torque power model.

    Prefer cruise_w + cruise_speed_m_s to derive scale so watts match the profile
    at a known speed instead of opaque hard-coded constants.

    Model (wheels): P ≈ idle + tau_proxy · scale · ω²  with τ ≈ tau_proxy · ω.
    """
    prof = profile or get_active_profile()
    spec = motor_spec(prof, motor_name)
    is_wheel = "wheel" in motor_name.lower()
    idle_w = float(spec.get("idle_w", 2.0 if is_wheel else 1.4))
    tau_proxy = float(spec.get("torque_proxy_coeff", 0.38 if is_wheel else 0.30))
    if "scale" in spec and spec.get("cruise_w") is None:
        return idle_w, float(spec["scale"]), tau_proxy

    cruise_w = spec.get("cruise_w")
    cruise_speed = spec.get("cruise_speed_m_s")
    if cruise_w is not None and is_wheel:
        radius = wheel_radius_m(prof)
        v_cruise = float(cruise_speed if cruise_speed is not None else 0.40)
        omega = max(v_cruise / max(radius, 1e-4), 0.5)
        # Solve scale so P(ω_cruise) ≈ cruise_w.
        target = max(float(cruise_w) - idle_w, 0.5)
        scale = target / (tau_proxy * omega * omega + 1e-6)
        return idle_w, float(scale), tau_proxy

    if "scale" in spec:
        return idle_w, float(spec["scale"]), tau_proxy
    return idle_w, (3.6 if is_wheel else 4.0), tau_proxy


def phase_reference_draw_w(phase: str, profile: dict | None = None) -> float:
    """Nominal total system draw for a twin mission phase (forecast fallback)."""
    prof = profile or get_active_profile()
    draws = prof.get("phase_draw_w") or {}
    key = normalize_phase_name(phase)
    return float(draws.get(key) or draws.get(phase) or 30.0)


def phase_reference_duration_s(phase: str, profile: dict | None = None) -> float:
    """Nominal duration for a twin mission phase (forecast fallback)."""
    prof = profile or get_active_profile()
    durations = prof.get("phase_duration_s") or {}
    key = normalize_phase_name(phase)
    return float(durations.get(key) or durations.get(phase) or 10.0)


# Biped walk_* names collapse to wheeled drive_transit for power models.
_PHASE_ALIASES = {
    "walk_transit": "drive_transit",
    "walk": "drive_transit",
    "teleop": "teleop",
}


def normalize_phase_name(phase: str) -> str:
    """Canonical twin phase key (aliases → wheeled naming)."""
    p = str(phase or "").lower()
    return _PHASE_ALIASES.get(p, p)
