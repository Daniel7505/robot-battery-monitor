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
        self.bias_rad_s = 0.0
        self._prev_left: float | None = None
        self._prev_right: float | None = None

    def quantize(self, angle_rad: float) -> float:
        step = self.resolution_rad
        if step <= 0.0:
            return float(angle_rad)
        return round(float(angle_rad) / step) * step

    def pose(self) -> tuple[float, float, float]:
        return (self.x_m, self.y_m, self.yaw_rad)

    def pending_translation(self, left_rad: float, right_rad: float) -> float:
        """Wheel travel since the last update, before that update is applied."""
        if self._prev_left is None:
            return 0.0
        left_q = self.quantize(left_rad)
        right_q = self.quantize(right_rad)
        return 0.5 * ((left_q - self._prev_left) + (right_q - self._prev_right)) * self.radius_m

    def nudge_yaw(self, delta_rad: float) -> None:
        self.yaw_unwrapped_rad += float(delta_rad)
        self.yaw_rad = wrap_rad(self.yaw_unwrapped_rad)

    def apply_correction(self, dx_m: float, dy_m: float, dyaw_rad: float) -> None:
        """Shift the pose in the odometry frame. Used by scan matching."""
        self.x_m += float(dx_m)
        self.y_m += float(dy_m)
        self.nudge_yaw(dyaw_rad)

    def update(
        self,
        left_rad: float,
        right_rad: float,
        yaw_rate_rad_s: float,
        dt_s: float,
        *,
        trust_translation: bool = True,
    ) -> tuple[float, float, float]:
        left_q = self.quantize(left_rad)
        right_q = self.quantize(right_rad)
        if self._prev_left is None or dt_s <= 0.0:
            self._prev_left = left_q
            self._prev_right = right_q
            return self.pose()
        d_left = left_q - self._prev_left
        d_right = right_q - self._prev_right
        self._prev_left = left_q
        self._prev_right = right_q
        ds = 0.5 * (d_left + d_right) * self.radius_m
        wheel_dyaw = (d_right - d_left) * self.radius_m / self.track_m
        rate_in = float(yaw_rate_rad_s)
        # oak_gyro reads +0.001 rad/s at rest (lookupTable output column).
        # A wedged robot sits for minutes; integrating that bias is the
        # heading walk seen on the first Webots run. Learn it only while
        # the hubs are still and the rate is too small to be a real turn.
        wheels_still = abs(ds) < 1e-4 and abs(wheel_dyaw) < 1e-4
        if wheels_still and abs(rate_in) < 0.02:
            alpha = min(1.0, max(0.0, float(dt_s)) * 0.8)
            self.bias_rad_s += alpha * (rate_in - self.bias_rad_s)
            self.bias_rad_s = max(-0.02, min(0.02, self.bias_rad_s))
            return self.pose()
        rate = rate_in - self.bias_rad_s
        if not trust_translation:
            ds = 0.0
        mid = self.yaw_unwrapped_rad + 0.5 * rate * float(dt_s)
        self.yaw_unwrapped_rad += rate * float(dt_s)
        self.yaw_rad = wrap_rad(self.yaw_unwrapped_rad)
        self.x_m += ds * math.cos(mid)
        self.y_m += ds * math.sin(mid)
        return self.pose()
