#!/usr/bin/env python3
"""Camera toe-out (yaw) study for the box-mounted nadir cameras (analysis only).

For each yaw the pitch is re-optimised so the top-row centre still lands
``--reach`` m ahead of the axle (robot forward x), then the same CameraModel
the controller uses reports where lane lines land in the image, how wide a
lane stays visible, what near / far coverage is lost, and when the crossing
line of a sharp 90 deg corner first enters view.

Sign convention: yaw > 0 = toe OUT (left camera turns left, right camera
turns right, mirror images). yaw < 0 = toe IN.

    python scripts/camera_yaw_study.py                 # markdown tables
    python scripts/camera_yaw_study.py --yaws -15 -10 -5 0 5 10 15 20 25
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from src.lane_vision import (  # noqa: E402
    NADIR_LEFT_POSE,
    CameraModel,
    aim_pitch_for_reach,
    camera_coverage,
    yaw_pitch_rotation,
)

MOUNT = tuple(NADIR_LEFT_POSE["translation"])  # left box lens (0.03542, 0.41808, 0.70419)
FOV = 1.2
W = H = 128
REACH_M = 1.96
LANE_WIDTHS = (1.0, 1.3, 1.6, 2.0)
AHEAD_M = (0.3, 0.5, 1.0, 1.5)
MARGIN_PX = 2.0  # a 6 cm stripe is ~3-10 px wide; keep its centre 2 px inside the frame


def cams_for(yaw_deg: float, reach: float = REACH_M):
    """(left, right, pitch_rad) for a mirrored pair toed out by yaw_deg."""
    y = math.radians(yaw_deg)
    pitch = aim_pitch_for_reach(MOUNT, FOV, W, H, reach, yaw_rad=y)
    left = CameraModel("nadir_left", MOUNT, yaw_pitch_rotation(y, pitch), W, H, FOV)
    right = CameraModel("nadir_right", (MOUNT[0], -MOUNT[1], MOUNT[2]), yaw_pitch_rotation(-y, pitch), W, H, FOV)
    return left, right, pitch


def in_frame(cam, x, y, margin=MARGIN_PX):
    px = cam.ground_to_pixel(x, y)
    if px is None:
        return None
    c, r = px
    if margin - 0.5 <= c <= cam.width - 0.5 - margin and margin - 0.5 <= r <= cam.height - 0.5 - margin:
        return c, r
    return None


def line_col(cam, x, y):
    p = in_frame(cam, x, y)
    return "out" if p is None else f"{p[0]:.0f}"


def max_visible_width(cam, x, w_hi=4.0):
    """Widest lane whose own-side line (y = +w/2 for the left camera) is in frame at x."""
    best = None
    w = 0.6
    while w <= w_hi + 1e-9:
        if in_frame(cam, x, 0.5 * w) is not None:
            best = w
        w = round(w + 0.01, 2)
    return best


def ground_samples(cam, step=2):
    pts = []
    for r in range(0, cam.height, step):
        for c in range(0, cam.width, step):
            g = cam.pixel_to_ground(c, r)
            if g is not None:
                pts.append(g)
    return pts


def corner_first_seen(left, right, lane_w=1.3, turn="left"):
    """Farthest distance ahead (robot centred, heading straight at the corner)
    at which the corner's crossing (outer) line is in either camera's view.

    Left turn: the outer line crosses ahead at x = d, from y = -w/2 out to +inf
    (it becomes the right line of the new leg). Right turn: mirror.
    Returns (distance_m, cams that see it first).
    """
    best, who = 0.0, []
    for cam in (left, right):
        far = 0.0
        for gx, gy in ground_samples(cam, 1):
            ok = gy >= -0.5 * lane_w if turn == "left" else gy <= 0.5 * lane_w
            if ok and gy <= 4.0 and gy >= -4.0 and in_frame(cam, gx, gy) is not None:
                far = max(far, gx)
        if far > best + 1e-6:
            best, who = far, [cam.name]
        elif abs(far - best) <= 1e-6:
            who.append(cam.name)
    return best, who


def crossing_view_at(cam, d, lane_w=1.3, turn="left"):
    """Lateral span (y) of the crossing line x = d visible in this camera."""
    ys = []
    y = -3.0
    while y <= 3.0:
        ok = y >= -0.5 * lane_w if turn == "left" else y <= 0.5 * lane_w
        if ok and in_frame(cam, d, y) is not None:
            ys.append(y)
        y = round(y + 0.01, 2)
    return (min(ys), max(ys)) if ys else None


def study(yaws):
    rows = []
    for yd in yaws:
        left, right, pitch = cams_for(yd)
        cov = camera_coverage(left)
        bottom = left.pixel_to_ground(left.cx, left.height - 1)
        # nearest floor point of the own line (w=1.3) still in frame
        near_line = None
        x = -0.5
        while x <= 3.0:
            if in_frame(left, x, 0.65) is not None:
                near_line = x
                break
            x = round(x + 0.01, 2)
        far_line = None
        x = 3.0
        while x >= -0.5:
            if in_frame(left, x, 0.65) is not None:
                far_line = x
                break
            x = round(x - 0.01, 2)
        cl, wl = corner_first_seen(left, right, 1.3, "left")
        overlap = {}
        for d in (0.5, 1.0, 1.5):
            sl = crossing_view_at(left, d, 99.0, "left")  # whole cross line, no lane clip
            sr = crossing_view_at(right, d, 99.0, "left")
            overlap[d] = None if not (sl and sr) else round(sr[1] - sl[0], 2)
        rows.append({
            "yaw_deg": yd,
            "pitch_deg": math.degrees(pitch),
            "rotation_left": yaw_pitch_rotation(math.radians(yd), round(pitch, 4)),
            "rotation_right": yaw_pitch_rotation(math.radians(-yd), round(pitch, 4)),
            "cols": {w: [line_col(left, a, 0.5 * w) for a in AHEAD_M] for w in LANE_WIDTHS},
            "max_w_1m": max_visible_width(left, 1.0),
            "max_w_05m": max_visible_width(left, 0.5),
            "max_w_15m": max_visible_width(left, 1.5),
            "bottom_centre": bottom,
            "top_centre": left.pixel_to_ground(left.cx, 0),
            "line_13_from_to": (near_line, far_line),
            "m_per_px_bottom": cov["bottom_m_per_px_lat_long"],
            "centre_overlap_m": overlap,
            "corner_left_first_seen_m": cl,
            "corner_left_cams": wl,
            "corner_left_span_at_1m_L": crossing_view_at(left, 1.0, 1.3, "left"),
            "corner_left_span_at_1m_R": crossing_view_at(right, 1.0, 1.3, "left"),
            "corner_left_span_at_15_L": crossing_view_at(left, 1.5, 1.3, "left"),
            "corner_left_span_at_15_R": crossing_view_at(right, 1.5, 1.3, "left"),
        })
    return rows


def _fmt_span(sp):
    return "-" if sp is None else f"{sp[0]:+.2f}..{sp[1]:+.2f}"


def markdown(rows) -> str:
    out = []
    out.append("| yaw (out +) | pitch | left rotation (axis-angle) | max lane w @0.5 / 1.0 / 1.5 m | own line (w 1.3) seen x | bottom-centre floor x, y | top-centre floor x, y | L/R overlap across the centre @0.5 / 1.0 / 1.5 m | sharp-corner crossing line first seen |")
    out.append("|---|---|---|---|---|---|---|---|---|")
    for r in rows:
        ax = r["rotation_left"]
        rot = f"`{ax[0]:.4f} {ax[1]:.4f} {ax[2]:.4f} {ax[3]:.4f}`"
        nl, fl = r["line_13_from_to"]
        b, t = r["bottom_centre"], r["top_centre"]
        out.append(
            f"| {r['yaw_deg']:+g}° | {r['pitch_deg']:.1f}° | {rot} | "
            + " / ".join("-" if v is None else (f">={v:g}" if v >= 4.0 else f"{v:g}")
                         for v in (r["max_w_05m"], r["max_w_1m"], r["max_w_15m"])) + " m | "
            + f"{nl:.2f}..{fl:.2f} m | {b[0]:+.2f}, {b[1]:+.2f} | {t[0]:.2f}, {t[1]:+.2f} | "
            + " / ".join("-" if r["centre_overlap_m"][d] is None else f"{r['centre_overlap_m'][d]:+.2f}" for d in (0.5, 1.0, 1.5))
            + " m | "
            + f"{r['corner_left_first_seen_m']:.2f} m ({', '.join(c.replace('nadir_', '') for c in r['corner_left_cams'])}) |"
        )
    out.append("")
    out.append("Left-camera image column of the left line (robot centred; 0 = left edge, 127 = right edge, 'out' = outside the frame or within 2 px of the edge):")
    out.append("")
    out.append("| yaw | " + " | ".join(f"w {w:.1f} m @ " + "/".join(f"{a:g}" for a in AHEAD_M) for w in LANE_WIDTHS) + " |")
    out.append("|---|" + "---|" * len(LANE_WIDTHS))
    for r in rows:
        out.append(f"| {r['yaw_deg']:+g}° | " + " | ".join(" / ".join(r["cols"][w]) for w in LANE_WIDTHS) + " |")
    out.append("")
    out.append("Sharp 90° LEFT corner, lane 1.30 m, robot centred: visible y-span of the crossing (outer) line when it is 1.0 m / 1.5 m ahead:")
    out.append("")
    out.append("| yaw | left cam @1.0 m | right cam @1.0 m | left cam @1.5 m | right cam @1.5 m |")
    out.append("|---|---|---|---|---|")
    for r in rows:
        out.append(f"| {r['yaw_deg']:+g}° | {_fmt_span(r['corner_left_span_at_1m_L'])} | {_fmt_span(r['corner_left_span_at_1m_R'])} | "
                   f"{_fmt_span(r['corner_left_span_at_15_L'])} | {_fmt_span(r['corner_left_span_at_15_R'])} |")
    return "\n".join(out)


def tracker_points(yaws, cases=((1.3, 0.0), (1.6, 0.0), (2.0, 0.0), (2.0, 0.3), (2.0, -0.3))):
    """Straight lane, rendered and fitted by the real LaneTracker: points per side
    (left / right line) and the fitted width, per yaw. offset + = robot left of centre."""
    from src.lane_vision import LaneTracker, lane_polyline_robot, offset_polyline, render_lane_bgra

    straight = [(-2 + 0.1 * i, 0.0) for i in range(80)]
    out = {}
    for yd in yaws:
        left, right, _ = cams_for(yd)
        cams = {"nadir_left": left, "nadir_right": right}
        for w, off in cases:
            c = lane_polyline_robot(straight, (0.0, off), 0.0)
            lines = [offset_polyline(c, w / 2), offset_polyline(c, -w / 2)]
            imgs = {n: (render_lane_bgra(cam, lines), W, H) for n, cam in cams.items()}
            e = LaneTracker(cams).process(imgs)
            out[(yd, w, off)] = (e.n_left, e.n_right, e.lane_width_m if e.valid else None, e.offset_m if e.valid else None)
    return out


def tracker_markdown(res, yaws, cases=((1.3, 0.0), (1.6, 0.0), (2.0, 0.0), (2.0, 0.3), (2.0, -0.3))) -> str:
    out = ["| yaw | " + " | ".join(f"w {w:.1f} m, robot {off:+.1f} m" for w, off in cases) + " |",
           "|---|" + "---|" * len(cases)]
    for yd in yaws:
        cells = []
        for w, off in cases:
            nl, nr, fw, fo = res[(yd, w, off)]
            cells.append(f"{nl} / {nr} pts" + ("" if fw is None else f", w {fw:.2f}, off {fo:+.2f}"))
        out.append(f"| {yd:+g}° | " + " | ".join(cells) + " |")
    return "\n".join(out)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--yaws", nargs="*", type=float, default=[-15, -10, -5, 0, 5, 10, 15, 20, 25])
    ap.add_argument("--tracker", action="store_true", help="also run LaneTracker on rendered straight lanes")
    a = ap.parse_args(argv)
    print(markdown(study(a.yaws)))
    if a.tracker:
        ty = [y for y in a.yaws if y in (-10, 0, 10, 15, 20)] or a.yaws
        print("\nLaneTracker on a straight lane (points left / right line, fitted width and offset):\n")
        print(tracker_markdown(tracker_points(ty), ty))
    return 0


if __name__ == "__main__":
    sys.exit(main())
