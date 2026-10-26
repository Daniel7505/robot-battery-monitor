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
        # Integrals of the raw sources, for the heading breakdown. These
        # do not move the pose by themselves.
        self.gyro_raw_yaw = 0.0
        self.gyro_unbiased_yaw = 0.0
        self.wheel_yaw = 0.0
        self.still_s = 0.0
        self.moving_s = 0.0
        self._win_t = 0.0
        self._win_wheel = 0.0
        self._win_gyro = 0.0
        self._win_path = 0.0
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
        dt = float(dt_s)
        self.gyro_raw_yaw += rate_in * dt
        self.wheel_yaw += wheel_dyaw
        # oak_gyro reads +0.001 rad/s at rest (lookupTable output column).
        # A wedged robot sits for minutes; integrating that bias is the
        # heading walk seen on the first Webots run. Learn it only while
        # the hubs are still and the rate is too small to be a real turn.
        wheels_still = abs(ds) < 1e-4 and abs(wheel_dyaw) < 1e-4
        if wheels_still and abs(rate_in) < 0.02:
            self.still_s += dt
            alpha = min(1.0, max(0.0, dt) * 0.8)
            self.bias_rad_s += alpha * (rate_in - self.bias_rad_s)
            self.bias_rad_s = max(-0.02, min(0.02, self.bias_rad_s))
            self.gyro_unbiased_yaw += (rate_in - self.bias_rad_s) * dt
            return self.pose()
        self.moving_s += dt
        # One encoder tick in an 8 ms step looks like a 0.04 rad/s turn, so
        # a per-step "straight" test never fires. Average half a second.
        # The lookup bias is about +0.001 rad/s. A residual of several
        # thousandths is hub scrub, and learning that is what walked the
        # heading on the first seed.
        self._win_t += dt
        self._win_wheel += wheel_dyaw
        self._win_gyro += rate_in * dt
        self._win_path += abs(ds)
        if self._win_t >= 0.50:
            wheel_rate = self._win_wheel / self._win_t
            gyro_rate = self._win_gyro / self._win_t
            residual = gyro_rate - wheel_rate
            straight = (
                self._win_path > 0.04
                and abs(wheel_rate) < 0.03
                and abs(gyro_rate) < 0.05
                and abs(residual) < 0.0035
            )
            if straight:
                alpha = min(1.0, self._win_t * 0.12)
                self.bias_rad_s += alpha * (residual - self.bias_rad_s)
                self.bias_rad_s = max(-0.004, min(0.004, self.bias_rad_s))
            self._win_t = 0.0
            self._win_wheel = 0.0
            self._win_gyro = 0.0
            self._win_path = 0.0
        rate = rate_in - self.bias_rad_s
        self.gyro_unbiased_yaw += rate * dt
        if not trust_translation:
            ds = 0.0
        mid = self.yaw_unwrapped_rad + 0.5 * rate * float(dt_s)
        self.yaw_unwrapped_rad += rate * float(dt_s)
        self.yaw_rad = wrap_rad(self.yaw_unwrapped_rad)
        self.x_m += ds * math.cos(mid)
        self.y_m += ds * math.sin(mid)
        return self.pose()
