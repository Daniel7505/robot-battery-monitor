"""Depth image to robot-frame points.

Webots R2025a cameras look along local +X. Image left is +Y and image
up is +Z (the same frame as ``CameraModel.ray`` in ``src/lane_camera.py``:
direction ``(focal, cx - col, cy - row)``). A RangeFinder pixel is the
hit's camera-frame X, the distance to the plane through the lens, not
the length of the ray. ``rangefinder.md`` calls that plane z = 0; the
camera frame used in this repo puts the optical axis on +X, and that is
the convention implemented here.

Mount pitch uses the proto rotation. Roll and pitch of the body come
from the inertial unit and tilt the cloud. The inertial unit's yaw is
a compass in Webots, so this module never reads it.
"""

from __future__ import annotations

import math

import numpy as np

from src.mapping.body import HEIGHT_MAX_M, HEIGHT_MIN_M, DepthCamera


def axis_angle_matrix(rotation: tuple[float, float, float, float]) -> np.ndarray:
    ax, ay, az, ang = (float(v) for v in rotation)
    n = math.sqrt(ax * ax + ay * ay + az * az)
    if n < 1e-12 or abs(ang) < 1e-12:
        return np.eye(3)
    x, y, z = ax / n, ay / n, az / n
    c, s = math.cos(ang), math.sin(ang)
    t = 1.0 - c
    return np.array(
        [
            [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
            [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
            [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
        ],
        dtype=np.float64,
    )


def _rot_x(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=np.float64)


def _rot_y(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=np.float64)


def body_points(
    cam: DepthCamera,
    depth_x: np.ndarray,
    *,
    roll_rad: float = 0.0,
    pitch_rad: float = 0.0,
    stride: int = 1,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Unproject a range image.

    Returns ``(hits, floors, sensor_xyz)`` in the robot frame. ``hits``
    are points whose height is in the mapping band. ``floors`` are points
    below that band (the ray is empty space). Pixels with no return, or
    outside the camera's range, are dropped.
    """
    image = np.asarray(depth_x, dtype=np.float64)
    if image.ndim != 2:
        raise ValueError("depth image must be height × width")
    height, width = image.shape
    step = max(1, int(stride))
    rows = np.arange(0, height, step, dtype=np.float64)
    cols = np.arange(0, width, step, dtype=np.float64)
    cc, rr = np.meshgrid(cols, rows)
    sampled = image[rr.astype(np.int32), cc.astype(np.int32)]
    d_cam = np.stack(
        [
            np.full(cc.shape, cam.focal_px, dtype=np.float64),
            cam.cx - cc,
            cam.cy - rr,
        ],
        axis=-1,
    )
    finite = np.isfinite(sampled) & (sampled >= cam.min_range_m) & (sampled <= cam.max_range_m)
    if not np.any(finite):
        origin = _sensor_origin(cam, roll_rad, pitch_rad)
        empty = np.zeros((0, 3), dtype=np.float64)
        return empty, empty, origin
    # Invalid pixels stay at t = 0 so an inf depth does not become NaN.
    t = np.zeros(sampled.shape, dtype=np.float64)
    t[finite] = sampled[finite] / cam.focal_px
    p_cam = d_cam * t[..., None]
    r_mount = axis_angle_matrix(cam.rotation)
    p_level = p_cam @ r_mount.T + np.asarray(cam.translation, dtype=np.float64)
    r_tilt = _rot_y(float(pitch_rad)) @ _rot_x(float(roll_rad))
    p_body = p_level @ r_tilt.T
    pts = p_body[finite]
    z = pts[:, 2]
    hits = pts[(z >= HEIGHT_MIN_M) & (z <= HEIGHT_MAX_M)]
    floors = pts[(z < HEIGHT_MIN_M) & (z > -0.25)]
    return hits, floors, _sensor_origin(cam, roll_rad, pitch_rad)


def _sensor_origin(cam: DepthCamera, roll_rad: float, pitch_rad: float) -> np.ndarray:
    r_tilt = _rot_y(float(pitch_rad)) @ _rot_x(float(roll_rad))
    return r_tilt @ np.asarray(cam.translation, dtype=np.float64)


def to_odom(points_body: np.ndarray, pose: tuple[float, float, float]) -> np.ndarray:
    """Robot-frame xyz → odom xy. Yaw is about z; z itself is unchanged."""
    if points_body.size == 0:
        return np.zeros((0, 2), dtype=np.float64)
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    xb = points_body[:, 0]
    yb = points_body[:, 1]
    return np.column_stack((x + c * xb - s * yb, y + s * xb + c * yb))


def blind_fraction(cam: DepthCamera, depth_x: np.ndarray, *, stride: int = 1) -> float:
    """Share of the centre view that has no usable range.

    Webots writes a non-finite value both inside ``minRange`` and past
    ``maxRange``. A wall against the lens fills this window. A floor in
    view does not, so an open room stays drivable.
    """
    image = np.asarray(depth_x, dtype=np.float64)
    if image.ndim != 2 or image.size == 0:
        return 0.0
    height, width = image.shape
    step = max(1, int(stride))
    c0 = int(width * 0.28)
    c1 = max(c0 + 1, int(width * 0.72))
    r0 = int(height * 0.08)
    r1 = max(r0 + 1, int(height * 0.92))
    sampled = image[r0:r1:step, c0:c1:step]
    if sampled.size == 0:
        return 0.0
    usable = np.isfinite(sampled) & (sampled >= cam.min_range_m) & (sampled <= cam.max_range_m)
    return float(1.0 - np.count_nonzero(usable) / float(sampled.size))


def depth_scene_motion_m(
    prev,
    curr,
    camera: DepthCamera,
    dyaw_rad: float,
) -> float | None:
    """Translation of the depth scene, in metres, after the gyro yaw is taken out.

    A pure turn slides the image sideways and is not travel. The previous
    frame is shifted by ``focal * dyaw`` so the residual is the range change
    a translation would make. ``None`` means there were not enough pixels
    in both frames: that is not a reading of "still".
    """
    if abs(float(dyaw_rad)) > 0.40:
        return None
    previous = np.asarray(prev, dtype=np.float64)
    current = np.asarray(curr, dtype=np.float64)
    if previous.shape != current.shape or previous.ndim != 2 or previous.size == 0:
        return None
    height, width = current.shape
    shift = int(round(float(camera.focal_px) * float(dyaw_rad)))
    aligned = np.full_like(previous, np.inf)
    if shift >= 0:
        if shift < width:
            aligned[:, shift:] = previous[:, : width - shift]
    elif -shift < width:
        aligned[:, : width + shift] = previous[:, -shift:]
    r0, r1 = int(height * 0.25), max(int(height * 0.25) + 1, int(height * 0.70))
    c0, c1 = int(width * 0.20), max(int(width * 0.20) + 1, int(width * 0.80))
    left = aligned[r0:r1, c0:c1]
    right = current[r0:r1, c0:c1]
    usable = (
        np.isfinite(left)
        & np.isfinite(right)
        & (left >= camera.min_range_m)
        & (right >= camera.min_range_m)
        & (left <= camera.max_range_m)
        & (right <= camera.max_range_m)
    )
    if int(np.count_nonzero(usable)) < 30:
        return None
    return float(np.median(np.abs(left[usable] - right[usable])))


def forward_clearance(hits_body: np.ndarray) -> tuple[float, float, float]:
    """Nearest range in the nose, the left shoulder, and the right shoulder.

    A miss (no points) is a large range, so the side with no return is
    treated as open. The nose is |y| < 0.12 m: a wall the body will meet
    head-on. The shoulders are 0.12–0.45 m off the centre line, which is
    where a door jamb sits in an 0.81 m opening. Reporting that jamb as
    the nose made the drive stop and turn away from a gap it can fit
    through. The tires reach 0.195 m, so a shoulder that is much closer
    than the other one is a steer, not a stop.
    """
    if hits_body.size == 0:
        return (99.0, 99.0, 99.0)
    x = hits_body[:, 0]
    y = hits_body[:, 1]
    front = (x > 0.15) & (x < 2.5)
    centre = front & (np.abs(y) < 0.12)
    left = front & (y >= 0.12) & (y < 0.45)
    right = front & (y <= -0.12) & (y > -0.45)

    def _min(mask: np.ndarray) -> float:
        if not np.any(mask):
            return 99.0
        return float(np.min(x[mask]))

    return (_min(centre), _min(left), _min(right))


def rear_clearance_m(hits_body: np.ndarray, *, tail_m: float = 0.16, half_m: float = 0.195) -> float | None:
    """Air from the tail shell to the nearest point behind the robot."""
    if hits_body is None or hits_body.size == 0:
        return None
    pts = np.asarray(hits_body, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[0] == 0:
        return None
    x = pts[:, 0]
    y = pts[:, 1]
    behind = (x < -float(tail_m)) & (np.abs(y) < float(half_m) + 0.05)
    if not np.any(behind):
        return None
    return float(np.min(-x[behind]) - float(tail_m))
