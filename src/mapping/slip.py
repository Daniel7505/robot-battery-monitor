"""Encoder travel the depth camera does not agree with.

Wheel slip spins the hubs while the body stays put. Gyro yaw does not
see a straight skid, so the check is the forward range: if the encoders
claim the robot moved and a wall in view did not get closer or farther,
the travel is dropped.
"""

from __future__ import annotations

import math


class MotionCheck:
    def __init__(self) -> None:
        self._path_m = 0.0
        self._t_s = 0.0
        self._clear0_m: float | None = None
        self.cooldown_s = 0.0
        self.events = 0

    def observe(self, dt_s: float, ds_m: float, clearance_m: float) -> bool:
        """True when this step's wheel translation should not move the pose."""
        dt = max(0.0, float(dt_s))
        if self.cooldown_s > 0.0:
            self.cooldown_s = max(0.0, self.cooldown_s - dt)
            return True
        useful = math.isfinite(clearance_m) and 0.2 < float(clearance_m) < 3.0
        if not useful:
            self._reset(None)
            return False
        if self._clear0_m is None:
            self._reset(float(clearance_m))
            return False
        self._path_m += abs(float(ds_m))
        self._t_s += dt
        # Wait until the hubs claim a real shift. A short reverse is only
        # ~7 cm in half a second, and splitting the window there hid the slip.
        if self._path_m < 0.08:
            if self._t_s > 1.0:
                self._reset(float(clearance_m))
            return False
        delta = abs(float(clearance_m) - self._clear0_m)
        slipped = delta < 0.30 * self._path_m
        self._reset(float(clearance_m))
        if not slipped:
            return False
        self.events += 1
        self.cooldown_s = 0.5
        return True

    def _reset(self, clear: float | None) -> None:
        self._clear0_m = clear
        self._path_m = 0.0
        self._t_s = 0.0
