"""Sensor noise models that match the oak_* devices.

Webots applies RangeFinder noise and the gyro lookup table itself.
The apartment controller does not call this module. The kinematic
stand-in and the unit tests do, so a run without Webots is still noisy.
"""

from __future__ import annotations

import math

import numpy as np

from src.mapping.body import (
    GYRO_BIAS_RAD_S,
    GYRO_NOISE_RAD_S,
    IMU_NOISE_FIELD,
    OAK_DEPTH_NOISE,
)


def depth_sigma_m(max_range_m: float, fraction: float = OAK_DEPTH_NOISE) -> float:
    """Webots R2025a: RangeFinder noise 1.0 means σ = maxRange."""
    return float(fraction) * float(max_range_m)


def noisy_depth(depth: np.ndarray, rng: np.random.Generator, max_range_m: float) -> np.ndarray:
    """Add gaussian depth noise. Non-finite pixels (no return) stay non-finite."""
    sigma = depth_sigma_m(max_range_m)
    out = np.array(depth, dtype=np.float64, copy=True)
    finite = np.isfinite(out)
    if sigma > 0.0 and np.any(finite):
        out[finite] += rng.normal(0.0, sigma, size=int(np.count_nonzero(finite)))
    return out


def noisy_yaw_rate(yaw_rate_rad_s: float, rng: np.random.Generator) -> float:
    """Bias plus white noise on a yaw rate, in rad/s."""
    return float(yaw_rate_rad_s) + GYRO_BIAS_RAD_S + float(rng.normal(0.0, GYRO_NOISE_RAD_S))


def noisy_tilt(roll_rad: float, pitch_rad: float, rng: np.random.Generator) -> tuple[float, float]:
    """Inertial-unit noise on roll and pitch. Heading is not touched."""
    sigma = IMU_NOISE_FIELD * math.pi / 2.0
    return (
        float(roll_rad) + float(rng.normal(0.0, sigma)),
        float(pitch_rad) + float(rng.normal(0.0, sigma)),
    )
