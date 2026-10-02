#!/usr/bin/env python3
"""Tilt calculator for a Webots lane camera (pitch, optional toe-out yaw).

Given the mount translation (robot frame: x forward, y left, z up, origin on
the floor under the drive axle), FOV and resolution, find the pitch that puts
the TOP image row (centre column) on the floor ``--reach`` metres ahead of the
axle, and print the Webots ``rotation`` line plus the floor coverage. Uses
``src.lane_vision.CameraModel``, the same math the controller runs.

    python scripts/aim_camera.py                       # left nadir, 1.96 m reach
    python scripts/aim_camera.py --mount 0.03542 0.41808 0.70419 --reach 2.2
    python scripts/aim_camera.py --box-center 0.01042 0.41808 0.72919 --box-size 0.05
    python scripts/aim_camera.py --yaw 10            # toe the camera 10 deg outward

Webots: ``rotation 0 1 0 theta`` pitches the view down by theta. Mirror a
pure-pitch shoulder camera by negating translation Y only (same rotation line).
With ``--yaw`` the camera is yawed first, then pitched (Rz(yaw) * Ry(pitch),
horizon level), the pitch is re-solved so the top-row centre still lands
``--reach`` m ahead (robot x), and the mirrored camera's rotation is printed
too: the yaw flips sign, i.e. axis (x, y, z) -> (-x, y, -z), same angle.
``--yaw`` > 0 = toe OUT (away from the robot's centre line) for either side.
"""
from __future__ import annotations

import argparse
import itertools
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from src.lane_vision import (  # noqa: E402
    FALLBACK_LANE_W_M,
    NADIR_LEFT_POSE,
    PAINT_TOP_Z_M,
    CameraModel,
    aim_pitch_for_reach,
    camera_coverage,
    yaw_pitch_rotation,
)

DEFAULT_REACH_M = 1.96  # top row reach of the original z 1.31 m mount


def line_in_image(cam: CameraModel, y_m: float, x_from: float = -1.0, x_to: float = 4.0, step: float = 0.01):
    """Pixels (col,row) where the floor line y = y_m (along +x) shows in the image."""
    out = []
    n = int(round((x_to - x_from) / step))
    for i in range(n + 1):
        x = x_from + i * step
        px = cam.ground_to_pixel(x, y_m)
        if px is None:
            continue
        c, r = px
        if -0.5 <= c <= cam.width - 0.5 and -0.5 <= r <= cam.height - 0.5:
            out.append((x, c, r))
    return out


def box_inside_near(cam: CameraModel, center, size, near: float) -> dict:
    """Is the whole mount box closer than the near plane (depth along the optical axis)?

    Webots clips everything with depth < near, so if every box corner has
    depth < near the camera can never render its own box.
    """
    R = cam.R
    ax = (R[0][0], R[1][0], R[2][0])  # camera +X in the robot frame
    tx, ty, tz = cam.translation
    half = [0.5 * float(s) for s in size]
    depths = []
    for sx, sy, sz in itertools.product((-1, 1), repeat=3):
        v = (center[0] + sx * half[0] - tx, center[1] + sy * half[1] - ty, center[2] + sz * half[2] - tz)
        depths.append(v[0] * ax[0] + v[1] * ax[1] + v[2] * ax[2])
    return {"max_depth_m": max(depths), "near_m": near, "box_invisible": max(depths) < near}


def _rot_line(rot) -> str:
    x, y, z, a = rot
    return "rotation " + " ".join(f"{v:.6g}" if abs(v) > 5e-7 else "0" for v in (x, y, z)) + f" {a:.4f}"


def aim(mount, fov, width, height, reach, *, lane_w=FALLBACK_LANE_W_M, plane_z=PAINT_TOP_Z_M,
        yaw_deg: float = 0.0) -> dict:
    side = 1.0 if mount[1] >= 0 else -1.0
    yaw = math.radians(yaw_deg) * side  # toe-out: left camera turns left, right camera turns right
    pitch = aim_pitch_for_reach(mount, fov, width, height, reach, plane_z=plane_z, yaw_rad=yaw)
    pitch_r = round(pitch, 4)
    rot = yaw_pitch_rotation(yaw, pitch_r)
    rot_m = yaw_pitch_rotation(-yaw, pitch_r)
    cam = CameraModel("aim", mount, rot, width, height, fov, plane_z)
    cov = camera_coverage(cam)
    near_line = line_in_image(cam, side * 0.5 * lane_w)
    far_line = line_in_image(cam, -side * 0.5 * lane_w)
    return {
        "pitch_rad": pitch,
        "pitch_rad_rounded": pitch_r,
        "pitch_deg": math.degrees(pitch),
        "yaw_deg": yaw_deg,
        "rotation": rot,
        "rotation_line": f"rotation 0 1 0 {pitch_r}" if yaw_deg == 0 else _rot_line(rot),
        "mirror_rotation_line": f"rotation 0 1 0 {pitch_r}" if yaw_deg == 0 else _rot_line(rot_m),
        "translation_line": f"translation {mount[0]:g} {mount[1]:g} {mount[2]:g}",
        "mirror_translation_line": f"translation {mount[0]:g} {-mount[1]:g} {mount[2]:g}",
        "camera": cam,
        "coverage": cov,
        "own_side_line": near_line,
        "other_side_line": far_line,
    }


def _span(pts) -> str:
    if not pts:
        return "NOT visible"
    a, b = pts[0], pts[-1]
    return (
        f"visible from x={a[0]:.2f} m (col {a[1]:.0f}, row {a[2]:.0f}) "
        f"to x={b[0]:.2f} m (col {b[1]:.0f}, row {b[2]:.0f})"
    )


def main(argv=None) -> int:
    p0 = NADIR_LEFT_POSE
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--mount", nargs=3, type=float, default=list(p0["translation"]), metavar=("X", "Y", "Z"))
    ap.add_argument("--fov", type=float, default=p0["fov"], help="horizontal fieldOfView, rad")
    ap.add_argument("--width", type=int, default=p0["width"])
    ap.add_argument("--height", type=int, default=p0["height"])
    ap.add_argument("--reach", type=float, default=DEFAULT_REACH_M, help="top row floor x ahead of the axle, m")
    ap.add_argument("--lane-width", type=float, default=FALLBACK_LANE_W_M)
    ap.add_argument("--yaw", type=float, default=0.0, help="toe-out yaw, degrees (+ = away from the centre line)")
    ap.add_argument("--near", type=float, default=0.03, help="Camera near, m (for --box-center)")
    ap.add_argument("--box-center", nargs=3, type=float, metavar=("X", "Y", "Z"))
    ap.add_argument("--box-size", nargs=3, type=float, default=[0.05, 0.05, 0.05], metavar=("SX", "SY", "SZ"))
    a = ap.parse_args(argv)

    r = aim(tuple(a.mount), a.fov, a.width, a.height, a.reach, lane_w=a.lane_width, yaw_deg=a.yaw)
    cov = r["coverage"]
    cam = r["camera"]
    print(f"mount {tuple(a.mount)}  fov {a.fov} rad  {a.width}x{a.height}  target top-row reach {a.reach} m")
    print(f"pitch = {r['pitch_rad']:.5f} rad = {r['pitch_deg']:.2f} deg down"
          + (f", toe-out yaw {a.yaw:g} deg (yaw first, then pitch)" if a.yaw else ""))
    print(f"  {r['translation_line']}")
    print(f"  {r['rotation_line']}")
    if a.yaw:
        print(f"  mirror: {r['mirror_translation_line']}")
        print(f"          {r['mirror_rotation_line']}  (yaw flipped: axis x and z negated)")
    else:
        print(f"  mirror: {r['mirror_translation_line']}  (same rotation)")
    lb, ll = cov["bottom_m_per_px_lat_long"]
    tb, tl = cov["top_m_per_px_lat_long"]
    print("coverage (centre column, floor x from the axle):")
    print(f"  bottom row {cov['bottom_row_x_m']:+.3f} m   {lb * 100:.2f} cm/px sideways x {ll * 100:.2f} cm/px along")
    print(f"  mid row    {cov['mid_row_x_m']:+.3f} m")
    print(f"  top row    {cov['top_row_x_m']:+.3f} m   {tb * 100:.2f} cm/px sideways x {tl * 100:.2f} cm/px along")
    print(f"  6 cm stripe ~{0.06 / lb:.1f} px wide at the bottom, ~{0.06 / tb:.1f} px at the top")
    side = "left" if a.mount[1] >= 0 else "right"
    other = "right" if side == "left" else "left"
    print(f"lane width {a.lane_width} m:")
    print(f"  {side} line (y={0.5 * a.lane_width * (1 if side == 'left' else -1):+.2f}): {_span(r['own_side_line'])}")
    print(f"  {other} line: {_span(r['other_side_line'])}")
    if a.box_center:
        chk = box_inside_near(cam, a.box_center, a.box_size, a.near)
        verdict = "box never rendered" if chk["box_invisible"] else "BOX MAY SHOW (raise near or move lens to the box face)"
        print(f"own box: farthest corner depth {chk['max_depth_m']:.4f} m vs near {a.near} m -> {verdict}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
