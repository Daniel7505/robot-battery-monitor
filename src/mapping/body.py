"""Skid-steer body constants the mapper is allowed to know.

Four driven wheels, no casters. Values match ``ButlerBotSkid.proto``:
wheel radius 0.08 m (160 mm), hubs at y = ±0.17 m so the track is
0.34 m, front axle at x = 0.11 m so the tire face is the 0.19 m nose.
The depth camera is at (0.14, 0, 0.78) with a 5° down pitch. The
two-wheel ``ButlerBot.proto`` shares this hub spacing and still has
casters; the warehouse controller uses that proto, not these comments.
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
# the colliding body is 0.195 m from the centreline. 0.20 m is that tire
# plus a centimetre. The sonar face is 0.227 m (SHELL_HALF_M) and has no
# bounding object, so it is not the inflation: a disk that wide treats a
# sensor as a wall. 0.26 m already closes a smeared 0.81 m doorway.
# 0.20 m still leaves a centre line after one extra jamb cell, and it is
# wider than the tire, so a gap the planner accepts is a gap the wheels fit.
ROBOT_RADIUS_M = 0.20
# Where the centre may turn. 0.23 m clears the side face. 0.30 m is the
# outer corner, so a pivot does not swing a tire into the wall. A doorway
# is not a turn: the robot stops, lines up, pivots in the open, and drives
# straight through. Smooth-arc turning (forklift-style rear steer) is a
# later option for a payload on a warehouse route. It is not implemented.
TURN_CLEAR_M = 0.23
TURN_AIM_M = 0.30
GRID_RESOLUTION_M = 0.05
GRID_SIZE_M = 20.0
MIN_FRONTIER_M = 0.30
HEIGHT_MIN_M = 0.05
HEIGHT_MAX_M = 1.50
# Webots RangeFinder.noise is a fraction of maxRange, not of the reading.
# ButlerBotSkid sets oak_depth noise to 0. The mapper applies the quadratic
# below. The caster proto still carries 0.002 because the warehouse robot
# shares that node and never enables it.
OAK_DEPTH_MAX_M = 8.0
OAK_DEPTH_MIN_M = 0.35
# Stereo error grows with range squared. Calibrated to the Luxonis median
# OAK-D Lite figure (480P, 75 mm baseline): below 3 m, below 2% absolute
# depth error; from 3 m to 6 m, below 4%.
# https://docs.luxonis.com/hardware/platform/depth/depth-accuracy
# σ(3 m) = 0.02 × 3 m, so σ(1 m) = 6.7 mm. σ(5 m) is 3.3%, inside the 4% band.
OAK_DEPTH_SIGMA_AT_1M = 0.06 / 9.0
# Full-confidence hits out to 4 m. 4–5 m still occupy, at a lower log-odds,
# because the published error is still a few percent there. Past 5 m the
# return is only probable and must not close a doorway.
OAK_FIRM_RANGE_M = 4.0
OAK_SOFT_RANGE_M = 5.0
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
