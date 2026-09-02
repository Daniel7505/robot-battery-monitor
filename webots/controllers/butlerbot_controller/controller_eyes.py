"""Shoulder nadir harvest. No physics.

LINE_CAM / finish / SIDELOOK / forecast aim, look-at, and shove-diag live
in archives/controller_eyes_choir_2026-09-02.py. Restore that file to undo.

The controller only calls ``_nadir_lateral_from_cam``. Other cameras may
still exist in the world. They are not read here.
"""
from __future__ import annotations

from controller import Camera


def _nadir_lateral_from_cam(
    cam: Camera | None,
    node,
    robot_xy: tuple[float, float] | None,
    yaw_rad: float | None,
    nadir_fn,
    *,
    side: str = "left",
) -> dict | None:
    """Yellow-ruler: tape inner edge → that side's drive wheel.

    nadir_fn is nadir_wheel_to_tape (pixel count × 6 cm / stripe_px).
    Live fight is 32 / 29, not a frozen 31.
    """
    del node, robot_xy, yaw_rad
    if cam is None or nadir_fn is None:
        return None
    try:
        image = cam.getImage()
        w = int(cam.getWidth())
        h = int(cam.getHeight())
    except Exception:
        return None
    if not image or w < 4 or h < 4:
        return None
    try:
        hit = nadir_fn(image, w, h, side=side)
    except Exception:
        return None
    if not hit:
        return None
    out = dict(hit)
    if out.get("m") is not None:
        out["m"] = round(float(out["m"]), 4)
    return out
