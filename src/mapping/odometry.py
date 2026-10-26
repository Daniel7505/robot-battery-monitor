"""Differential-drive odometry: wheel travel plus gyro yaw.

Translation comes from the average of the two hub angles. Heading comes
from the integrated gyro, not from the difference of the wheels, so a
skid on carpet does not have to agree with the yaw gyro. Encoder ticks
are applied here (0.0015 rad) even when the simulator hubs are finer.
"""

from __future__ import annotations

import math

from src.mapping.body import ENCODER_RESOLUTION_RAD, WHEEL_RADIUS_M, WHEEL_TRACK_M


def wrap_rad(angle: float) -> float:
    a = float(angle)
    return math.atan2(math.sin(a), math.cos(a))


class WheelGyroOdometry:
    def __init__(
        self,
        *,
        radius_m: float = WHEEL_RADIUS_M,
        track_m: float = WHEEL_TRACK_M,
        resolution_rad: float = ENCODER_RESOLUTION_RAD,
    ) -> None:
        self.radius_m = float(radius_m)
        self.track_m = float(track_m)
        self.resolution_rad = float(resolution_rad)
        self.x_m = 0.0
        self.y_m = 0.0
        self.yaw_rad = 0.0
        self.yaw_unwrapped_rad = 0.0
        self._prev_left: float | None = None
        self._prev_right: float | None = None

    def quantize(self, angle_rad: float) -> float:
        step = self.resolution_rad
        if step <= 0.0:
            return float(angle_rad)
        return round(float(angle_rad) / step) * step

    def pose(self) -> tuple[float, float, float]:
        return (self.x_m, self.y_m, self.yaw_rad)

    def update(
        self,
        left_rad: float,
        right_rad: float,
        yaw_rate_rad_s: float,
        dt_s: float,
    ) -> tuple[float, float, float]:
        left_q = self.quantize(left_rad)
        right_q = self.quantize(right_rad)
        if self._prev_left is None or dt_s <= 0.0:
            self._prev_left = left_q
            self._prev_right = right_q
            return self.pose()
        ds = 0.5 * ((left_q - self._prev_left) + (right_q - self._prev_right)) * self.radius_m
        self._prev_left = left_q
        self._prev_right = right_q
        rate = float(yaw_rate_rad_s)
        mid = self.yaw_unwrapped_rad + 0.5 * rate * float(dt_s)
        self.yaw_unwrapped_rad += rate * float(dt_s)
        self.yaw_rad = wrap_rad(self.yaw_unwrapped_rad)
        self.x_m += ds * math.cos(mid)
        self.y_m += ds * math.sin(mid)
        return self.pose()
