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
# Planning inflation. Hubs sit at ±0.17 m and the tire is 0.05 m wide, so
# the body is 0.195 m from the centreline to the outside of a tire.
# 0.26 m of inflation leaves about 0.29 m of traversable width in an
# 0.81 m opening, and one extra occupied cell on a jamb closes it. That
# is the bedroom doorway the robot clipped on every Webots run. 0.20 m
# is the tire plus a centimetre, and still leaves a centre line after
# that extra cell. Furniture is handled by the stop range, not by sealing
# the opening.
ROBOT_RADIUS_M = 0.20
GRID_RESOLUTION_M = 0.05
GRID_SIZE_M = 20.0
MIN_FRONTIER_M = 0.30
HEIGHT_MIN_M = 0.05
HEIGHT_MAX_M = 1.50
# Webots RangeFinder.noise is a fraction of maxRange, not of the reading.
# The proto keeps this value. Warehouse laps never enable the device.
# The kinematic depth path uses the quadratic model below instead.
OAK_DEPTH_NOISE = 0.002
OAK_DEPTH_MAX_M = 8.0
OAK_DEPTH_MIN_M = 0.35
# Stereo disparity error makes range error grow with distance squared.
# σ(1 m) = 1 cm, so 0.5–3 m stays a few centimetres (OAK-D Lite band).
OAK_DEPTH_SIGMA_AT_1M = 0.01
# Past this, a return is only a probable surface. It must not occupy a
# cell: a far smear was closing doorways the shell can still fit through.
OAK_FIRM_RANGE_M = 3.0
# Share of the forward view with no usable range. A surface closer than
# minRange fills the frame with empty pixels, and that is contact.
BLIND_VIEW_FRACTION = 0.65
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
STOP_CLEARANCE_M = 0.70
# Reported clearance when the forward view is inside the camera minimum.
# It has to sit under STOP_CLEARANCE_M so the drive does not treat "no
# return" as an open corridor.
BLIND_CLEARANCE_M = 0.12
# Downward floor camera, a few centimetres above the laminate. The scale
# starts 3% high on purpose: the metres-per-pixel factor is this nominal
# height, and a thicker carpet is not known ahead of time.
FLOOR_SCALE_ERROR = 1.03
FLOOR_NOISE_PX = 0.35


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


# Looks down: rotation about +Y by π/2 sends the optical axis to -Z.
# 32×32 is the same order as a mouse-sensor window, not a scene camera.
FLOOR_CAM = DepthCamera(
    name="floor_cam",
    translation=(0.0, 0.0, 0.04),
    rotation=(0.0, 1.0, 0.0, 1.5708),
    width=32,
    height=32,
    fov_rad=0.90,
    min_range_m=0.01,
    max_range_m=2.0,
)

# Backup camera, yawed 180° so the optical axis is robot -X. Small and
# short-range: it is only used while reversing or when the forward view
# has nothing to track. Same numbers as the mappingSensors block.
REAR_DEPTH = DepthCamera(
    name="rear_depth",
    translation=(-0.16, 0.0, 0.28),
    rotation=(0.0, 0.0, 1.0, math.pi),
    width=80,
    height=48,
    fov_rad=1.2,
    min_range_m=0.08,
    max_range_m=2.5,
)

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
