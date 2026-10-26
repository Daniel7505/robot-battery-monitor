"""Body-frame drives for short checks.

A name such as ``square`` or ``plunge`` is the whole interface. Distances
and yaw are in the robot frame. Nothing here is a place in a building.
"""

from __future__ import annotations

import math
import os

from src.mapping.body import CRUISE_M_S, SPIN_RAD_S
from src.mapping.bubble import max_speed_for_gap
from src.mapping.drive import wheel_speeds


_MODES = ("square", "plunge", "backup")


def drive_mode(explicit: str | None = None) -> str:
    raw = explicit if explicit is not None else os.environ.get("RBM_DRIVE", "")
    mode = str(raw or "").strip().lower()
    return mode if mode in _MODES else ""


class BodyScript:
    """Square, a straight plunge, or a straight backup. Stops itself."""

    def __init__(self, mode: str) -> None:
        self.mode = str(mode)
        self.done = False
        self.phase = "drive"
        self.leg = 0
        self.progress_m = 0.0
        self.yaw_mark: float | None = None
        self.legs = 4
        self.leg_m = 0.80
        self.turn_rad = math.pi / 2.0
        self.backup_m_s = 0.18

    def command(
        self,
        *,
        ds_m: float,
        yaw_unwrapped: float,
        forward_gap_m: float | None,
        rear_gap_m: float | None,
    ) -> tuple[float, float]:
        if self.done:
            return 0.0, 0.0
        yaw = float(yaw_unwrapped)
        if self.yaw_mark is None:
            self.yaw_mark = yaw
        if self.mode == "square":
            return self._square(float(ds_m), yaw, forward_gap_m)
        if self.mode == "plunge":
            return self._along(CRUISE_M_S, forward_gap_m)
        if self.mode == "backup":
            return self._backup(rear_gap_m)
        self.done = True
        return 0.0, 0.0

    def _cap(self, gap_m: float | None, wanted: float) -> float:
        if gap_m is None:
            return float(wanted)
        room = max_speed_for_gap(float(gap_m))
        return min(float(wanted), room)

    def _square(self, ds_m: float, yaw: float, forward_gap_m: float | None) -> tuple[float, float]:
        if self.phase == "turn":
            turned = yaw - float(self.yaw_mark)
            if turned >= self.turn_rad - 0.05:
                self.leg += 1
                self.phase = "drive"
                self.progress_m = 0.0
                self.yaw_mark = yaw
                if self.leg >= self.legs:
                    self.done = True
                    return 0.0, 0.0
            else:
                return wheel_speeds(0.0, SPIN_RAD_S)
        self.progress_m += max(0.0, ds_m)
        if self.progress_m >= self.leg_m:
            self.phase = "turn"
            self.yaw_mark = yaw
            self.progress_m = 0.0
            return wheel_speeds(0.0, SPIN_RAD_S)
        speed = self._cap(forward_gap_m, 0.22)
        if speed < 0.02 and forward_gap_m is not None and forward_gap_m <= 0.04:
            self.done = True
            return 0.0, 0.0
        return wheel_speeds(speed, 0.0)

    def _along(self, wanted: float, gap_m: float | None) -> tuple[float, float]:
        speed = self._cap(gap_m, wanted)
        if gap_m is not None and speed < 0.02:
            self.done = True
            return 0.0, 0.0
        return wheel_speeds(speed, 0.0)

    def _backup(self, rear_gap_m: float | None) -> tuple[float, float]:
        speed = self._cap(rear_gap_m, self.backup_m_s)
        if rear_gap_m is not None and speed < 0.02:
            self.done = True
            return 0.0, 0.0
        # No rear reading yet: creep, do not dive blind at the cruise speed.
        if rear_gap_m is None:
            speed = min(speed, 0.08)
        return wheel_speeds(-speed, 0.0)
