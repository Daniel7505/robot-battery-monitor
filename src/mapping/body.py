"""ButlerBot body constants the mapper is allowed to know.

These are the robot's own wheels and the depth camera bolted to it.
They are not the apartment. Values match ``ButlerBot.proto``: wheel
radius 0.08 m, hub anchors at y = ±0.17 m, depth camera at
(0.14, 0, 0.78) with a 5° down pitch. Tests check the proto still
says the same numbers.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


WHEEL_RADIUS_M = 0.08
WHEEL_TRACK_M = 0.34
# Software stand-in for a 4096-count encoder. The shared PositionSensors
# stay at infinite resolution so warehouse energy and ABS are unchanged.
ENCODER_RESOLUTION_RAD = 0.0015
# Planning inflation. Hubs are at ±0.17 m. 0.24 m still leaves a centre
# line through the 0.81 m bedroom door.
ROBOT_RADIUS_M = 0.24
GRID_RESOLUTION_M = 0.05
GRID_SIZE_M = 20.0
MIN_FRONTIER_M = 0.30
HEIGHT_MIN_M = 0.05
HEIGHT_MAX_M = 1.50
# Webots RangeFinder.noise is a fraction of maxRange, not of the reading.
OAK_DEPTH_NOISE = 0.002
OAK_DEPTH_MAX_M = 8.0
OAK_DEPTH_MIN_M = 0.35
# Kinematic stand-in for the oak_gyro lookup table: a small bias plus
# absolute white noise. Webots itself applies the lookup table.
GYRO_BIAS_RAD_S = 0.001
GYRO_NOISE_RAD_S = 0.005
# Webots InertialUnit.noise of 1.0 is σ = π/2. 0.01 is about 0.9°.
IMU_NOISE_FIELD = 0.01
CRUISE_M_S = 0.28
SPIN_RAD_S = 0.70
LOOKAHEAD_M = 0.25
ARRIVE_M = 0.28
# Body-frame x of the nearest forward hit. The lens is at x = 0.14 m and
# ignores returns closer than 0.35 m, so the nearest visible obstacle sits
# near x = 0.50 m. Stopping inside that hole means the robot never sees
# the wall it is about to touch.
STOP_CLEARANCE_M = 0.62


@dataclass(frozen=True)
class DepthCamera:
    """Pinhole depth camera in the robot frame (x forward, y left, z up)."""

    name: str
    translation: tuple[float, float, float]
    rotation: tuple[float, float, float, float]
    width: int
    height: int
    fov_rad: float
    min_range_m: float
    max_range_m: float

    @property
    def focal_px(self) -> float:
        return (self.width / 2.0) / math.tan(self.fov_rad / 2.0)

    @property
    def cx(self) -> float:
        return (self.width - 1) / 2.0

    @property
    def cy(self) -> float:
        return (self.height - 1) / 2.0


# 80° HFOV, 320×200, pitched 0.0873 rad (5°) down. Same numbers as the proto.
OAK_DEPTH = DepthCamera(
    name="oak_depth",
    translation=(0.14, 0.0, 0.78),
    rotation=(0.0, 1.0, 0.0, 0.0873),
    width=320,
    height=200,
    fov_rad=1.396,
    min_range_m=OAK_DEPTH_MIN_M,
    max_range_m=OAK_DEPTH_MAX_M,
)
