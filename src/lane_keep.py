"""Shared wheel math and the painted-S centre line.

The pixel-gap nadir keeper (``lane_keep_command``, ``NadirGuard``, the
33/33 fan) is gone. It was tuned for the old high camera and logged a
false "nadir stop" from empty gap pixels while row-fit was driving.
Row-fit still uses :class:`SteerFilter` and :func:`wheels_from_steer`.

:func:`yellow_score` stays because ``lane_vision``'s ``lane_keep`` colour
mask matches it. The live mask is ``rg``. :func:`track_centerline` stays
as the S fallback when a world has no track file.
"""

from __future__ import annotations

import math

DEFAULT_CRUISE_RAD_S = 5.5
DEFAULT_K_STEER = 2.0
DEFAULT_STEER_SLEW_PER_S = 12.0
DEFAULT_STEER_RELEASE_PER_S = 4.0
DEFAULT_TURN_SLOW = 0.24
MAX_WHEEL_RAD_S = 8.0
WHEEL_RADIUS_M = 0.08
WHEEL_Y_LEFT_M = 0.17

_TRACK_START_M = 3.0
_TRACK_LOBE_M = 5.0
_TRACK_AMP_M = 1.0
_TRACK_SIGN = -1.0
FINISH_X_M = 16.5


def track_centerline(x: float) -> tuple[float, float]:
    """Lane center (y, heading_rad) at world x."""
    x = float(x)
    if x <= _TRACK_START_M:
        return 0.0, 0.0
    if x <= _TRACK_START_M + _TRACK_LOBE_M:
        u = x - _TRACK_START_M
        y = _TRACK_SIGN * (_TRACK_AMP_M / 2.0) * (
            1.0 - math.cos(2.0 * math.pi * u / _TRACK_LOBE_M)
        )
        yp = (
            _TRACK_SIGN
            * (_TRACK_AMP_M / 2.0)
            * (2.0 * math.pi / _TRACK_LOBE_M)
            * math.sin(2.0 * math.pi * u / _TRACK_LOBE_M)
        )
        return y, math.atan(yp)
    if x <= _TRACK_START_M + 2 * _TRACK_LOBE_M:
        u = x - _TRACK_START_M - _TRACK_LOBE_M
        y = -_TRACK_SIGN * (_TRACK_AMP_M / 2.0) * (
            1.0 - math.cos(2.0 * math.pi * u / _TRACK_LOBE_M)
        )
        yp = (
            -_TRACK_SIGN
            * (_TRACK_AMP_M / 2.0)
            * (2.0 * math.pi / _TRACK_LOBE_M)
            * math.sin(2.0 * math.pi * u / _TRACK_LOBE_M)
        )
        return y, math.atan(yp)
    return 0.0, 0.0


def track_cross_track_m(x: float, y: float) -> float:
    """Meters left of centerline (+ = robot left of paint center)."""
    cy, th = track_centerline(x)
    return (float(y) - cy) * math.cos(th)


def yellow_score(rgb: tuple[float, float, float]) -> float:
    """0–1: yellow paint (high R+G, low B) vs gray floor."""
    r, g, b = rgb
    return max(0.0, min(1.0, (r + g) * 0.5 - b))


class SteerFilter:
    """Slew the virtual wheel so both hubs ease in together."""

    def __init__(
        self,
        slew_per_s: float = DEFAULT_STEER_SLEW_PER_S,
        release_per_s: float | None = None,
    ) -> None:
        self.value = 0.0
        self.slew_per_s = float(slew_per_s)
        if release_per_s is None:
            self.release_per_s = min(self.slew_per_s, DEFAULT_STEER_RELEASE_PER_S)
        else:
            self.release_per_s = float(release_per_s)

    def step(self, target: float | None, dt: float) -> float:
        want = 0.0 if target is None else max(-1.0, min(1.0, float(target)))
        if abs(want) < 1e-9:
            grabbing = abs(self.value) < 1e-9
        elif self.value * want < 0.0:
            grabbing = True
        else:
            grabbing = abs(want) + 1e-9 >= abs(self.value)
        rate = self.slew_per_s if grabbing else self.release_per_s
        max_step = max(0.0, float(rate) * max(0.0, float(dt)))
        delta = want - self.value
        if abs(delta) > max_step:
            delta = max_step if delta > 0.0 else -max_step
        self.value += delta
        return self.value


def wheels_from_steer(
    steer: float,
    *,
    cruise: float = DEFAULT_CRUISE_RAD_S,
    k_steer: float = DEFAULT_K_STEER,
    turn_slow: float = DEFAULT_TURN_SLOW,
) -> tuple[float, float]:
    """Both hubs stay forward. steer < 0 → left slower (yaw left)."""
    s = float(steer)
    slow = max(0.0, min(0.6, float(turn_slow))) * min(1.0, abs(s))
    cruise_eff = float(cruise) * (1.0 - slow)
    left = cruise_eff + k_steer * s
    right = cruise_eff - k_steer * s
    floor = 0.35 * abs(float(cruise))
    left = max(floor, min(MAX_WHEEL_RAD_S, left))
    right = max(floor, min(MAX_WHEEL_RAD_S, right))
    return left, right
