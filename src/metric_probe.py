"""20 cm shove probe — dead-proxy gate for any lateral meter metric.

Harness owns the perturbation (body +Y) and the noise floor. A metric
that does not move ~20 cm is unvalidated and must not steer.

No Webots required. Synthetic floor + existing unprojection.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable

from src.lane_keep import (
    LANE_HALF_M,
    NADIR_FOV_RAD,
    NADIR_IMAGE,
    NADIR_MOUNT_POS,
    NADIR_MOUNT_ROT,
    PAINT_HALF_W_M,
    PAINT_Y_LEFT_M,
    WHEEL_Y_LEFT_M,
    nadir_lateral_m,
    nadir_wheel_to_tape,
    pixel_to_ground_robot_m,
)

# Same kill as the SIDELOOK pin. SIDELOOK moved 4–7 mm → below this floor.
SHOVE_M = 0.20
PROBE_NOISE_FLOOR_M = 0.03
PROBE_ACCURACY_M = 0.05

LINE_CAM_L_POS = (0.30, 0.65, 0.12)
LINE_CAM_IDENTITY = (0.0, 1.0, 0.0, 0.0)
LINE_CAM_FOV_RAD = 1.2

PAINT_RGB = (0.91, 0.59, 0.22)
FLOOR_RGB = (0.50, 0.50, 0.52)
WHEEL_RGB = (0.12, 0.12, 0.12)
SKY_RGB = (0.72, 0.76, 0.82)

MetricFn = Callable[[float], float | None]


def _rgb_to_bgra(rgb: tuple[float, float, float]) -> bytes:
    r, g, b = rgb
    return bytes(
        (
            max(0, min(255, int(round(b * 255.0)))),
            max(0, min(255, int(round(g * 255.0)))),
            max(0, min(255, int(round(r * 255.0)))),
            255,
        )
    )


def _world_xy(
    robot_xy: tuple[float, float],
    yaw_rad: float,
    ahead_m: float,
    lateral_m: float,
) -> tuple[float, float]:
    c = math.cos(float(yaw_rad))
    s = math.sin(float(yaw_rad))
    dx = float(ahead_m) * c - float(lateral_m) * s
    dy = float(ahead_m) * s + float(lateral_m) * c
    return (float(robot_xy[0]) + dx, float(robot_xy[1]) + dy)


def render_floor_bgra(
    *,
    robot_xy: tuple[float, float] = (0.0, 0.0),
    yaw_rad: float = 0.0,
    cam_pos: tuple[float, float, float],
    cam_rot: tuple[float, float, float, float],
    width: int,
    height: int,
    fov_rad: float,
    paint_world_y: float = PAINT_Y_LEFT_M,
    paint_half_m: float = PAINT_HALF_W_M,
    wheel_body_y: float = WHEEL_Y_LEFT_M,
) -> bytearray:
    """Project left paint + left wheel into a camera BGRA buffer."""
    w = int(width)
    h = int(height)
    img = bytearray(w * h * 4)
    paint = _rgb_to_bgra(PAINT_RGB)
    floor = _rgb_to_bgra(FLOOR_RGB)
    wheel = _rgb_to_bgra(WHEEL_RGB)
    sky = _rgb_to_bgra(SKY_RGB)
    for row in range(h):
        for col in range(w):
            o = (row * w + col) * 4
            hit = pixel_to_ground_robot_m(
                robot_xy,
                yaw_rad,
                cam_pos,
                cam_rot,
                float(col),
                float(row),
                w,
                h,
                fov_rad,
            )
            if hit is None:
                img[o : o + 4] = sky
                continue
            ahead, lat = hit
            _wx, wy = _world_xy(robot_xy, yaw_rad, ahead, lat)
            if abs(wy - float(paint_world_y)) <= float(paint_half_m):
                img[o : o + 4] = paint
            elif abs(ahead) <= 0.10 and abs(lat - float(wheel_body_y)) <= 0.08:
                img[o : o + 4] = wheel
            else:
                img[o : o + 4] = floor
    return img


@dataclass(frozen=True)
class ProbeResult:
    center: float | None
    shoved: float | None
    delta_m: float | None
    shove_m: float
    sensitive: bool
    accurate: bool
    validated: bool
    reason: str


def probe_lateral_metric(
    metric_fn: MetricFn,
    *,
    shove_m: float = SHOVE_M,
    noise_floor_m: float = PROBE_NOISE_FLOOR_M,
    accuracy_m: float = PROBE_ACCURACY_M,
    toward: float = 1.0,
) -> ProbeResult:
    """Parked body +Y shove. Metric must change by ~shove meters.

    ``toward`` is the body-Y sign that closes the seen tape (+1 = left).
    Sign of the metric is free; we only require |Δ| ≈ shove.
    """
    body = float(toward) * float(shove_m)
    center = metric_fn(0.0)
    shoved = metric_fn(body)
    if center is None or shoved is None:
        return ProbeResult(
            center=center,
            shoved=shoved,
            delta_m=None,
            shove_m=float(shove_m),
            sensitive=False,
            accurate=False,
            validated=False,
            reason="unvalidated: metric missing at center or after shove",
        )
    delta = float(shoved) - float(center)
    sensitive = abs(delta) >= float(noise_floor_m)
    accurate = abs(abs(delta) - float(shove_m)) <= float(accuracy_m)
    if not sensitive:
        return ProbeResult(
            center=float(center),
            shoved=float(shoved),
            delta_m=delta,
            shove_m=float(shove_m),
            sensitive=False,
            accurate=False,
            validated=False,
            reason=(
                f"unvalidated: flat under {shove_m:.2f} m shove "
                f"(Δ={delta:.4f} m, floor={noise_floor_m:.3f} m)"
            ),
        )
    if not accurate:
        return ProbeResult(
            center=float(center),
            shoved=float(shoved),
            delta_m=delta,
            shove_m=float(shove_m),
            sensitive=True,
            accurate=False,
            validated=False,
            reason=(
                f"unvalidated: moved {delta:.4f} m, want |Δ|≈{shove_m:.2f} m "
                f"(±{accuracy_m:.2f} m)"
            ),
        )
    return ProbeResult(
        center=float(center),
        shoved=float(shoved),
        delta_m=delta,
        shove_m=float(shove_m),
        sensitive=True,
        accurate=True,
        validated=True,
        reason=f"ok: Δ={delta:.4f} m on {shove_m:.2f} m shove",
    )


def line_cam_identity_dist_m(body_y: float) -> float | None:
    """Old LINE_CAM operator on a synthetic left stripe."""
    robot_xy = (0.0, float(body_y))
    img = render_floor_bgra(
        robot_xy=robot_xy,
        cam_pos=LINE_CAM_L_POS,
        cam_rot=LINE_CAM_IDENTITY,
        width=64,
        height=64,
        fov_rad=LINE_CAM_FOV_RAD,
    )
    del img, robot_xy
    # LINE_CAM wall hit is not a live operator. Probe must fail.
    return None


def frozen_spawn_dist_m(body_y: float) -> float | None:
    """Classic dead proxy: ignore body Y, report spawn mount distance."""
    _ = body_y
    return float(LANE_HALF_M)


def picture_offset_as_meters(body_y: float) -> float | None:
    """Picture-wins column offset, falsely treated as meters."""
    robot_xy = (0.0, float(body_y))
    img = render_floor_bgra(
        robot_xy=robot_xy,
        cam_pos=LINE_CAM_L_POS,
        cam_rot=LINE_CAM_IDENTITY,
        width=64,
        height=64,
        fov_rad=LINE_CAM_FOV_RAD,
    )
    del img
    # Picture-wins column offset is not a meter metric. Probe must fail.
    return None


def nadir_gap_m(body_y: float) -> float | None:
    """Yellow-ruler meters: tape inner edge to drive wheel."""
    robot_xy = (0.0, float(body_y))
    img = render_floor_bgra(
        robot_xy=robot_xy,
        cam_pos=NADIR_MOUNT_POS,
        cam_rot=NADIR_MOUNT_ROT,
        width=NADIR_IMAGE,
        height=NADIR_IMAGE,
        fov_rad=NADIR_FOV_RAD,
    )
    hit = nadir_wheel_to_tape(img, NADIR_IMAGE, NADIR_IMAGE)
    if not hit:
        return None
    return float(hit["m"])


def nadir_paint_y_m(body_y: float) -> float | None:
    """Drawing-2 left nadir: robot-frame Y of the paint at the axle row."""
    robot_xy = (0.0, float(body_y))
    img = render_floor_bgra(
        robot_xy=robot_xy,
        cam_pos=NADIR_MOUNT_POS,
        cam_rot=NADIR_MOUNT_ROT,
        width=NADIR_IMAGE,
        height=NADIR_IMAGE,
        fov_rad=NADIR_FOV_RAD,
    )
    return nadir_lateral_m(
        img,
        NADIR_IMAGE,
        NADIR_IMAGE,
        cam_pos=NADIR_MOUNT_POS,
        cam_rot=NADIR_MOUNT_ROT,
        fov_rad=NADIR_FOV_RAD,
        robot_xy=robot_xy,
        yaw_rad=0.0,
    )


def low_nadir_paint_y_m(body_y: float) -> float | None:
    """Same gap mount at LINE_CAM height — too short to hold wheel + tape."""
    cam = (NADIR_MOUNT_POS[0], NADIR_MOUNT_POS[1], 0.12)
    robot_xy = (0.0, float(body_y))
    img = render_floor_bgra(
        robot_xy=robot_xy,
        cam_pos=cam,
        cam_rot=NADIR_MOUNT_ROT,
        width=NADIR_IMAGE,
        height=NADIR_IMAGE,
        fov_rad=NADIR_FOV_RAD,
    )
    return nadir_lateral_m(
        img,
        NADIR_IMAGE,
        NADIR_IMAGE,
        cam_pos=cam,
        cam_rot=NADIR_MOUNT_ROT,
        fov_rad=NADIR_FOV_RAD,
        robot_xy=robot_xy,
        yaw_rad=0.0,
    )


