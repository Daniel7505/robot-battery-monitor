"""Numpy depth images for tests and the kinematic stand-in.

Rays use the same camera frame as ``src.mapping.depth_cloud``. The
returned image is camera-frame X (the RangeFinder value), in metres.
This is not a Webots render. It does not load the apartment.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from src.mapping.body import DepthCamera
from src.mapping.depth_cloud import axis_angle_matrix


@dataclass(frozen=True)
class Box:
    x0: float
    y0: float
    z0: float
    x1: float
    y1: float
    z1: float

    def overlaps_height(self, z0: float, z1: float) -> bool:
        return self.z1 > z0 and self.z0 < z1


def _rot_x(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=np.float64)


def _rot_y(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=np.float64)


def _rot_z(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def render_xdepth(
    cam: DepthCamera,
    pose: tuple[float, float, float],
    boxes: list[Box],
    *,
    floor_z: float = 0.0,
    roll_rad: float = 0.0,
    pitch_rad: float = 0.0,
) -> np.ndarray:
    """Range image shaped (height, width). No hit is +inf."""
    x, y, yaw = (float(v) for v in pose)
    cols = np.arange(cam.width, dtype=np.float64)
    rows = np.arange(cam.height, dtype=np.float64)
    cc, rr = np.meshgrid(cols, rows)
    d_cam = np.stack(
        [
            np.full(cc.shape, cam.focal_px, dtype=np.float64),
            cam.cx - cc,
            cam.cy - rr,
        ],
        axis=-1,
    )
    r_mount = axis_angle_matrix(cam.rotation)
    r_tilt = _rot_y(pitch_rad) @ _rot_x(roll_rad)
    r_yaw = _rot_z(yaw)
    d_world = (d_cam @ r_mount.T) @ (r_yaw @ r_tilt).T
    origin = r_yaw @ (r_tilt @ np.asarray(cam.translation, dtype=np.float64))
    origin = origin + np.array([x, y, 0.0], dtype=np.float64)
    t_hit = np.full(cc.shape, np.inf, dtype=np.float64)
    dz = d_world[..., 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        t_floor = np.where(dz < -1e-8, (float(floor_z) - origin[2]) / dz, np.inf)
    t_hit = np.minimum(t_hit, np.where(t_floor > 1e-5, t_floor, np.inf))
    for box in boxes:
        t_hit = np.minimum(t_hit, _ray_box(origin, d_world, box))
    return t_hit * cam.focal_px


def _ray_box(origin: np.ndarray, direction: np.ndarray, box: Box) -> np.ndarray:
    bmin = np.array([box.x0, box.y0, box.z0], dtype=np.float64)
    bmax = np.array([box.x1, box.y1, box.z1], dtype=np.float64)
    with np.errstate(divide="ignore", invalid="ignore"):
        inv = 1.0 / direction
        t0 = (bmin - origin) * inv
        t1 = (bmax - origin) * inv
    tnear = np.max(np.minimum(t0, t1), axis=-1)
    tfar = np.min(np.maximum(t0, t1), axis=-1)
    hit = (tfar + 1e-8 >= tnear) & (tnear > 1e-5) & np.isfinite(tnear)
    return np.where(hit, tnear, np.inf)


def circle_hits(x: float, y: float, boxes: list[Box], radius: float) -> bool:
    r2 = float(radius) * float(radius)
    for box in boxes:
        if not box.overlaps_height(0.05, 1.4):
            continue
        cx = min(max(float(x), box.x0), box.x1)
        cy = min(max(float(y), box.y0), box.y1)
        if (float(x) - cx) ** 2 + (float(y) - cy) ** 2 < r2:
            return True
    return False
