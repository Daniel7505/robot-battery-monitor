"""Sensor noise models that match the oak_* devices.

Webots applies the gyro lookup table itself. The apartment controller
does not call this module. The kinematic stand-in and the unit tests do,
so a run without Webots is still noisy.

Depth error grows with range squared. σ(3 m) is 2% of range, the
Luxonis median bound for an OAK-D Lite (480P, 75 mm baseline):
https://docs.luxonis.com/hardware/platform/depth/depth-accuracy
The device noise on oak_depth is 0. Webots cannot express a
range-squared fraction, so this function is the noise, in the
kinematic stand-in and in the apartment controller.
"""

from __future__ import annotations

import math

import numpy as np

from src.mapping.body import (
    GYRO_BIAS_RAD_S,
    GYRO_NOISE_RAD_S,
    IMU_NOISE_FIELD,
    OAK_DEPTH_SIGMA_AT_1M,
)


def depth_sigma_m(range_m: float, at_1m: float = OAK_DEPTH_SIGMA_AT_1M) -> float:
    """σ at ``range_m``. Stereo error scales with range squared.

    ``at_1m`` is σ at 1 m. At 0.5 m the error is a quarter of that, at
    2 m it is four times, and at 3 m it is nine times.
    """
    z = max(0.0, float(range_m))
    return float(at_1m) * z * z


def noisy_depth(depth: np.ndarray, rng: np.random.Generator, max_range_m: float) -> np.ndarray:
    """Add range-squared gaussian depth noise.

    Non-finite pixels (no return) stay non-finite. ``max_range_m`` caps
    the range the quadratic is evaluated at, so a glitch past the sensor
    maximum does not explode the draw.
    """
    out = np.array(depth, dtype=np.float64, copy=True)
    finite = np.isfinite(out)
    if not np.any(finite):
        return out
    z = np.clip(out[finite], 0.0, float(max_range_m))
    sigma = float(OAK_DEPTH_SIGMA_AT_1M) * np.square(z)
    out[finite] += rng.normal(0.0, 1.0, size=int(z.shape[0])) * sigma
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
