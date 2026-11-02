"""Sensor noise models that match the oak_* devices.

Webots applies the gyro lookup table itself. The apartment controller
applies ``noisy_depth`` because RangeFinder noise is a fraction of max
range, and the OAK-D Lite error is not. The kinematic stand-in calls
the same function, so a run without Webots is still noisy.

Depth error grows with range squared. oak_depth device noise on
ButlerBotSkid is 0.
"""

from __future__ import annotations

import math

import numpy as np

from src.mapping.body import (
    ACCEL_BIAS_M_S2,
    ACCEL_DRIFT_LIMIT_M_S2,
    ACCEL_DRIFT_M_S2_PER_SQRT_S,
    ACCEL_NOISE_M_S2,
    GYRO_BIAS_RAD_S,
    GYRO_NOISE_RAD_S,
    IMU_NOISE_FIELD,
    OAK_DEPTH_SIGMA_AT_1M,
)


def depth_sigma_m(range_m: float, at_1m: float = OAK_DEPTH_SIGMA_AT_1M) -> float:
    """σ at ``range_m``. Stereo error scales with range squared.

    ``at_1m`` is σ at 1 m (6.7 mm for the published OAK-D Lite band).
    At 0.5 m the error is a quarter of that, at 2 m it is four times,
    and at 5 m it is about 3.3% of range.
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


def noisy_accel(
    ax: float,
    ay: float,
    az: float,
    rng: np.random.Generator,
    bias: list[float],
    dt_s: float,
) -> tuple[float, float, float]:
    """Bias, white noise, and a slow drift on a 3-axis accelerometer.

    ``bias`` is a three-element list the caller keeps. Each call walks it
    by a random step and clamps it. The device lookup already shifts the
    raw sample; this is the absolute noise a fractional lookup cannot
    put on a near-zero axis.
    """
    step = ACCEL_DRIFT_M_S2_PER_SQRT_S * math.sqrt(max(0.0, float(dt_s)))
    noisy = []
    for i, raw in enumerate((float(ax), float(ay), float(az))):
        bias[i] = float(bias[i]) + float(rng.normal(0.0, step))
        bias[i] = max(-ACCEL_DRIFT_LIMIT_M_S2, min(ACCEL_DRIFT_LIMIT_M_S2, bias[i]))
        noisy.append(raw + ACCEL_BIAS_M_S2 + bias[i] + float(rng.normal(0.0, ACCEL_NOISE_M_S2)))
    return (noisy[0], noisy[1], noisy[2])


def noisy_tilt(roll_rad: float, pitch_rad: float, rng: np.random.Generator) -> tuple[float, float]:
    """Inertial-unit noise on roll and pitch. Heading is not touched."""
    sigma = IMU_NOISE_FIELD * math.pi / 2.0
    return (
        float(roll_rad) + float(rng.normal(0.0, sigma)),
        float(pitch_rad) + float(rng.normal(0.0, sigma)),
    )
