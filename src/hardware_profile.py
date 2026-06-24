"""
Load reference hardware profiles for grounded power estimation.
"""

from __future__ import annotations

import os
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
    "motors": {},
    "channels": {},
    "phase_draw_w": {},
    "phase_duration_s": {},
}


@lru_cache(maxsize=8)
def load_hardware_profile(profile_id: str) -> dict:
    """Load a hardware profile YAML by id."""
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
    return (
        config.get("hardware_profile")
        or config.get("robot", "hardware_profile")
        or "butlerbot_wheeled"
    )


def get_active_profile() -> dict:
    return load_hardware_profile(get_active_profile_id())


def motor_spec(profile: dict, motor_name: str) -> dict:
    motors = profile.get("motors") or {}
    return dict(motors.get(motor_name) or motors.get(motor_name.lower()) or {})


def clamp_motor_power_w(motor_name: str, watts: float, profile: dict | None = None) -> float:
    """Clamp estimated draw to reference motor peak."""
    prof = profile or get_active_profile()
    spec = motor_spec(prof, motor_name)
    peak = float(spec.get("peak_w", 120))
    return round(min(max(0.0, watts), peak), 2)


def phase_reference_draw_w(phase: str, profile: dict | None = None) -> float:
    prof = profile or get_active_profile()
    draws = prof.get("phase_draw_w") or {}
    key = normalize_phase_name(phase)
    return float(draws.get(key) or draws.get(phase) or 30.0)


def phase_reference_duration_s(phase: str, profile: dict | None = None) -> float:
    prof = profile or get_active_profile()
    durations = prof.get("phase_duration_s") or {}
    key = normalize_phase_name(phase)
    return float(durations.get(key) or durations.get(phase) or 10.0)


_PHASE_ALIASES = {
    "walk_transit": "drive_transit",
    "walk": "drive_transit",
}


def normalize_phase_name(phase: str) -> str:
    p = str(phase or "").lower()
    return _PHASE_ALIASES.get(p, p)