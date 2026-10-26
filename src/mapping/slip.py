"""Encoder travel the depth camera does not agree with.

Wheel slip spins the hubs while the body stays put. Gyro yaw does not
see a straight skid, so the check is the forward range: if the encoders
claim the robot moved and a wall in view did not get closer or farther,
the travel is dropped.

A flat floor looks the same after a pure translation, so a frame that
merely "barely changes" is not evidence of slip. The cap applies when
the forward view is unusable (inside the camera minimum, so there is
no wall range to compare) or when a wall range fails to move with the
wheels. Both drop the step's translation.
"""

from __future__ import annotations

import math


class MotionCheck:
    def __init__(self) -> None:
        self._path_m = 0.0
        self._t_s = 0.0
        self._clear0_m: float | None = None
        self._blind = False
        self.cooldown_s = 0.0
        self.events = 0

    def observe(self, dt_s: float, ds_m: float, clearance_m: float, *, blind: bool = False) -> bool:
        """True when this step's wheel translation should not move the pose.

        ``blind`` means the forward depth is mostly empty. That happens
        when the body is closer than the camera minimum, so there is no
        wall range to watch. Encoder travel in that state is not motion.
        """
        dt = max(0.0, float(dt_s))
        if self.cooldown_s > 0.0:
            self.cooldown_s = max(0.0, self.cooldown_s - dt)
            return True
        if blind:
            if not self._blind:
                self._reset(None)
                self._blind = True
            self._path_m += abs(float(ds_m))
            self._t_s += dt
            if abs(float(ds_m)) < 1e-4 and self._path_m < 0.08:
                if self._t_s > 1.0:
                    self._path_m = 0.0
                    self._t_s = 0.0
                return False
            if self._path_m >= 0.08:
                self._path_m = 0.0
                self._t_s = 0.0
                self.events += 1
                self.cooldown_s = 0.5
            return True
        if self._blind:
            self._reset(None)
            self._blind = False
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
