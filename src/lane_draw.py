"""Synthetic lane renderer for tests and offline tuning."""

from __future__ import annotations

import math

from src.lane_camera import STRIPE_W_M, CameraModel

# --------------------------------------------------------------------------
# Synthetic renderer (tests / offline tuning). Same camera model.
# --------------------------------------------------------------------------


def lane_polyline_robot(
    centre_world: list[tuple[float, float]],
    robot_xy: tuple[float, float],
    robot_yaw: float,
) -> list[tuple[float, float]]:
    c, s = math.cos(robot_yaw), math.sin(robot_yaw)
    out = []
    for wx, wy in centre_world:
        dx, dy = wx - robot_xy[0], wy - robot_xy[1]
        out.append((dx * c + dy * s, -dx * s + dy * c))
    return out


def offset_polyline(poly: list[tuple[float, float]], d: float) -> list[tuple[float, float]]:
    """Shift a polyline sideways by d (+ = left of travel direction)."""
    out = []
    n = len(poly)
    for i in range(n):
        a = poly[max(0, i - 1)]
        b = poly[min(n - 1, i + 1)]
        tx, ty = b[0] - a[0], b[1] - a[1]
        L = math.hypot(tx, ty) or 1.0
        out.append((poly[i][0] - d * ty / L, poly[i][1] + d * tx / L))
    return out


def render_lane_bgra(
    cam: CameraModel,
    lines: list[list[tuple[float, float]]],
    *,
    stripe_w: float = STRIPE_W_M,
    paint=(0.85, 0.85, 0.18),
    floor=(0.55, 0.56, 0.58),
    noise: float = 0.04,
    marks=(),
) -> bytes:
    """Render paint polylines (robot frame) into a BGRA buffer, nearest-sample.

    ``marks``: extra (polyline, width_m, rgb) strokes in other colours, e.g. a
    red finish bar (drawn first, so lane paint wins where they overlap).
    """
    cell = 0.1
    grid: dict[tuple[int, int], list] = {}
    strokes = [(m[0], 0.5 * float(m[1]), tuple(m[2])) for m in marks]
    strokes += [(line, stripe_w / 2.0, tuple(paint)) for line in lines]
    for line, half, stroke_rgb in strokes:
        for i in range(len(line) - 1):
            (x0, y0), (x1, y1) = line[i], line[i + 1]
            for gx in range(int(math.floor((min(x0, x1) - half) / cell)), int(math.floor((max(x0, x1) + half) / cell)) + 1):
                for gy in range(int(math.floor((min(y0, y1) - half) / cell)), int(math.floor((max(y0, y1) + half) / cell)) + 1):
                    grid.setdefault((gx, gy), []).append((x0, y0, x1, y1, half, stroke_rgb))
    w, h = cam.width, cam.height
    out = bytearray(w * h * 4)
    seed = 12345
    for row in range(h):
        for col in range(w):
            seed = (1103515245 * seed + 12345) & 0x7FFFFFFF
            jitter = ((seed / 0x7FFFFFFF) - 0.5) * 2.0 * noise
            color = floor
            p = cam.pixel_to_ground(col, row)
            if p is not None:
                segs = grid.get((int(math.floor(p[0] / cell)), int(math.floor(p[1] / cell))), ())
                for x0, y0, x1, y1, half, stroke_rgb in reversed(segs):
                    vx, vy = x1 - x0, y1 - y0
                    L2 = vx * vx + vy * vy
                    t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((p[0] - x0) * vx + (p[1] - y0) * vy) / L2))
                    if math.hypot(p[0] - (x0 + t * vx), p[1] - (y0 + t * vy)) <= half:
                        color = stroke_rgb
                        break
            else:
                color = (0.3, 0.35, 0.45)
            o = (row * w + col) * 4
            r, g, b = (max(0.0, min(1.0, v + jitter)) for v in color)
            out[o] = int(b * 255)
            out[o + 1] = int(g * 255)
            out[o + 2] = int(r * 255)
            out[o + 3] = 255
    return bytes(out)
