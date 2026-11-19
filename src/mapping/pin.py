"""Wheels commanded, and every motion sensor says the body did not translate.

The depth scene, the floor flow, and the visual odometry each have to
produce their own still reading. A missing sample is not stillness: one
sensor that cannot see does not get to freeze the pose. Yaw is allowed.
A spin against a door jamb turns the shell while the tires scrub, and
that scrub is what walked the odometry.
"""

from __future__ import annotations


class PinWatch:
    """Watch one stretch of commanded translation.

    The window closes once the wheels have claimed ``WHEEL_M``. It is a
    pin when all three sensors reported during that stretch and each one
    moved less than ``RATIO`` of the wheel claim (and less than a small
    floor, so sensor noise on a short window is not travel).
    """

    WHEEL_M = 0.05
    RATIO = 0.45
    FLOOR_M = 0.012
    # One floor pixel of noise, or one 8 ms step at a creep. Counted as
    # a sample so a still image is a vote, and not added as travel.
    # Summing the absolute value of every frame turned a rocking tire
    # into metres of "motion" and the pin never closed. Samples under
    # this floor add nothing. The window still needs each sensor.
    QUIET_M = 0.0015

    def __init__(self) -> None:
        self.x0 = 0.0
        self.y0 = 0.0
        self.wheel_m = 0.0
        self.depth_m = 0.0
        self.floor_m = 0.0
        self.visual_m = 0.0
        self.depth_n = 0
        self.floor_n = 0
        self.visual_n = 0

    def anchor(self, x_m: float, y_m: float) -> None:
        self.x0 = float(x_m)
        self.y0 = float(y_m)

    def reset(self) -> None:
        self.wheel_m = 0.0
        self.depth_m = 0.0
        self.floor_m = 0.0
        self.visual_m = 0.0
        self.depth_n = 0
        self.floor_n = 0
        self.visual_n = 0

    def add(
        self,
        wheel_m: float,
        depth_m: float | None,
        floor_m: float | None,
        visual_m: float | None,
    ) -> None:
        self.wheel_m += abs(float(wheel_m))
        self._vote(depth_m, "depth")
        self._vote(floor_m, "floor")
        self._vote(visual_m, "visual")

    def _vote(self, sample_m: float | None, kind: str) -> None:
        if sample_m is None:
            return
        moved = abs(float(sample_m))
        if moved < self.QUIET_M:
            moved = 0.0
        if kind == "depth":
            self.depth_m += moved
            self.depth_n += 1
        elif kind == "floor":
            self.floor_m += moved
            self.floor_n += 1
        else:
            self.visual_m += moved
            self.visual_n += 1

    def ready(self) -> bool:
        return self.wheel_m >= self.WHEEL_M

    def pinned(self) -> bool:
        """True when the wheels claimed travel and all three scenes stayed put."""
        if not self.ready():
            return False
        if self.depth_n < 1 or self.floor_n < 1 or self.visual_n < 1:
            return False
        limit = max(self.FLOOR_M, self.RATIO * self.wheel_m)
        return self.depth_m < limit and self.floor_m < limit and self.visual_m < limit
