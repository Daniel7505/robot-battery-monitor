"""Gyro read helpers and pivot heading integration (Webots + unit tests)."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class GyroReading:
    yaw_rate_rad_s: float
    source: str  # "gyro" | "imu_delta" | "missing"


class FakeGyro:
    """Minimal Webots Gyro stand-in for tests."""

    def __init__(self, *, yaw_rate_rad_s: float = 0.0) -> None:
        self._values = [0.0, 0.0, float(yaw_rate_rad_s)]

    def getValues(self) -> list[float]:
        return list(self._values)

    def set_yaw_rate(self, yaw_rate_rad_s: float) -> None:
        self._values[2] = float(yaw_rate_rad_s)


def read_yaw_rate_rad_s(
    gyro,
    *,
    imu_yaw_rad: float | None = None,
    prev_imu_yaw_rad: float | None = None,
    dt_s: float = 0.0,
    axis: int = 2,
) -> GyroReading:
    """Yaw rate (+ = left / CCW) from Gyro, else finite-difference IMU yaw."""
    if gyro is not None:
        try:
            values = gyro.getValues()
            rate = float(values[int(axis)])
            if math.isfinite(rate):
                return GyroReading(rate, "gyro")
        except Exception:
            pass
    if (
        imu_yaw_rad is not None
        and prev_imu_yaw_rad is not None
        and dt_s > 1e-6
    ):
        dy = imu_yaw_rad - prev_imu_yaw_rad
        while dy > math.pi:
            dy -= 2.0 * math.pi
        while dy < -math.pi:
            dy += 2.0 * math.pi
        return GyroReading(dy / dt_s, "imu_delta")
    return GyroReading(0.0, "missing")


def integrate_heading_rad(integrated_rad: float, yaw_rate_rad_s: float, dt_s: float) -> float:
    """Integrate measured yaw rate (rad)."""
    return float(integrated_rad) + float(yaw_rate_rad_s) * max(0.0, float(dt_s))


def pivot_turn_progress_rad(integrated_rad: float, corner_dir: int) -> float:
    """Signed turn progress toward a ``corner_dir`` pivot (+1 left, −1 right)."""
    d = int(corner_dir) if corner_dir else 1
    return float(integrated_rad) * float(d)


def rate_tracking_adjustment(
    want_w_rad_s: float,
    measured_w_rad_s: float,
    *,
    gain: float = 0.35,
) -> float:
    """Small PI-less correction to the commanded pivot rate."""
    return float(gain) * (float(want_w_rad_s) - float(measured_w_rad_s))
