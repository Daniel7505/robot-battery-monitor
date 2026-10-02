"""Gentle S-curve centerline + VRML paint for butlerbot.wbt.

Two cosine lobes (flat slope at the junctions) so the last stretch is
straight +X for the red-eye. SIGN flips the first bend so we can tell
line-follow from a memorized +Y-then-−Y lap. Width and paint stay put.
"""

from __future__ import annotations

import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

START_STRAIGHT_M = 3.0
LOBE_M = 5.0
AMP_M = 1.0
# −1 = first lobe goes south (−Y). Same difficulty, opposite of the scored S.
SIGN = -1.0
FINISH_STRAIGHT_M = 3.5
HALF_WIDTH_M = 0.65
SEG_M = 0.38
PAINT_Z = 0.008
FINISH_X_M = START_STRAIGHT_M + 2 * LOBE_M + FINISH_STRAIGHT_M  # 16.5
# Track history: before 2026-09-04 (commit 1ff49a5) LOBE_M was 9.0 and the
# finish was x=24.5 m (path ~25.0 m). Steer logs with x ≈ 24 m are from that
# older, longer S. Current double-S: finish x=16.5 m, path ~17.4 m.


def min_radius_m() -> float:
    """Tightest centerline radius (cosine-lobe crest/trough)."""
    return 1.0 / ((AMP_M / 2.0) * (2.0 * math.pi / LOBE_M) ** 2)


def centerline(x: float) -> tuple[float, float]:
    """Return (y, heading_rad) of the lane center at world x."""
    x = float(x)
    if x <= START_STRAIGHT_M:
        return 0.0, 0.0
    if x <= START_STRAIGHT_M + LOBE_M:
        u = x - START_STRAIGHT_M
        y = SIGN * (AMP_M / 2.0) * (1.0 - math.cos(2.0 * math.pi * u / LOBE_M))
        yp = SIGN * (AMP_M / 2.0) * (2.0 * math.pi / LOBE_M) * math.sin(
            2.0 * math.pi * u / LOBE_M
        )
        return y, math.atan(yp)
    if x <= START_STRAIGHT_M + 2 * LOBE_M:
        u = x - START_STRAIGHT_M - LOBE_M
        y = -SIGN * (AMP_M / 2.0) * (1.0 - math.cos(2.0 * math.pi * u / LOBE_M))
        yp = -SIGN * (AMP_M / 2.0) * (2.0 * math.pi / LOBE_M) * math.sin(
            2.0 * math.pi * u / LOBE_M
        )
        return y, math.atan(yp)
    return 0.0, 0.0


def path_length_m(step: float = 0.05) -> float:
    n = int(FINISH_X_M / step)
    acc = 0.0
    prev = (0.0, 0.0)
    for i in range(1, n + 1):
        x = i * step
        y, _ = centerline(x)
        acc += math.hypot(x - prev[0], y - prev[1])
        prev = (x, y)
    return acc


def cross_track_m(x: float, y: float) -> float:
    """Signed distance to centerline (robot left / +normal is positive)."""
    cy, th = centerline(x)
    nx, ny = -math.sin(th), math.cos(th)
    return (x - x) * nx + (y - cy) * ny  # (0)*nx + (y-cy)*ny


# Paint recipe (2026-10-01: one mesh per line instead of ~100 box nodes).
# The mesh code lives in scripts/track_builder.py (any waypoint track); the S is
# just its waypoints (tracks/s.json, written by ``--write-track``).
from track_builder import (  # noqa: E402
    _APPEARANCE,
    _fmt_mesh,
    _quad,
    box_strip,
    emit_track_vrml,
    strip_prism,
)

LINE_W_M = 0.06  # painted stripe width
LINE_H_M = 0.01  # stripe thickness; top at PAINT_Z + LINE_H_M / 2 = 0.013 m
PAINT_STEP_M = 0.34  # legacy box spacing (boxes were SEG_M = 0.38 long, overlapping)
MESH_STEP_X_M = 0.02  # waypoint / ribbon sample spacing along x
BAR_SIZE = (0.08, 1.4, 0.02)  # start / finish bar (x, y, z), centre z BAR_Z
BAR_Z = 0.015
TICK_SIZE = (0.04, 0.45, 0.01)
TICK_Z = 0.01
TICK_XS = (3.0, 5.5, 8.0, 10.5, 13.0)
TRACK_JSON = os.path.join(os.path.dirname(_HERE), "tracks", "s.json")


def paint_x_range() -> tuple[float, float]:
    """x span of each lane line: the legacy boxes ran from the first box
    (centre 0, half-length SEG_M/2) to the last centre <= FINISH_X_M."""
    n_last = int(math.floor(FINISH_X_M / PAINT_STEP_M + 1e-9))
    return -SEG_M / 2.0, n_last * PAINT_STEP_M + SEG_M / 2.0


def lane_line_samples(side: float, step: float = MESH_STEP_X_M):
    """[(x, (left_edge_xy), (right_edge_xy), centre_xy)] along one lane line.

    side +1 = left line, -1 = right line. Each line is the centreline offset
    by side * HALF_WIDTH_M along the path normal; the stripe edges are
    +/- LINE_W_M / 2 further along the same normal (offset curves share normals).
    The start/finish straights are straight, so extending to the legacy box
    ends (x = -0.19 .. 16.51) is exact.
    """
    x0, x1 = paint_x_range()
    n = int(math.ceil((x1 - x0) / step))
    out = []
    for i in range(n + 1):
        x = x0 + (x1 - x0) * i / n
        y, th = centerline(x)
        nx, ny = -math.sin(th), math.cos(th)
        cx, cy = x + side * HALF_WIDTH_M * nx, y + side * HALF_WIDTH_M * ny
        h = LINE_W_M / 2.0
        out.append((x, (cx + h * nx, cy + h * ny), (cx - h * nx, cy - h * ny), (cx, cy)))
    return out


def _quad(pts, a, b, c, d, want):
    """Face a-b-c-d, reversed if needed so its normal points along ``want``."""
    ax, ay, az = pts[a]
    bx, by, bz = pts[b]
    cx, cy, cz = pts[c]
    u = (bx - ax, by - ay, bz - az)
    v = (cx - ax, cy - ay, cz - az)
    nrm = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
    dot = sum(p * q for p, q in zip(nrm, want))
    return (a, b, c, d) if dot >= 0 else (d, c, b, a)


def strip_prism(left_edge, right_edge, z_top: float, height: float, pts: list, faces: list) -> None:
    """Append a stripe prism (top + both walls + end caps) to pts/faces.

    left_edge/right_edge: [(x, y), ...] matching samples along the stripe.
    Top faces point +Z (counter-clockwise seen from above), walls outward.
    """
    zb = z_top - height
    base = len(pts)
    for (lx, ly), (rx, ry) in zip(left_edge, right_edge):
        pts.extend([(lx, ly, z_top), (rx, ry, z_top), (lx, ly, zb), (rx, ry, zb)])
    n = len(left_edge)
    for k in range(n - 1):
        i, j = base + 4 * k, base + 4 * (k + 1)
        # outward side normals from the edge geometry
        lx, ly = left_edge[k]
        rx, ry = right_edge[k]
        out_l = (lx - rx, ly - ry, 0.0)
        faces.append(_quad(pts, i + 1, j + 1, j, i, (0.0, 0.0, 1.0)))  # top
        faces.append(_quad(pts, i, j, j + 2, i + 2, out_l))  # left wall
        faces.append(_quad(pts, i + 1, i + 3, j + 3, j + 1, (-out_l[0], -out_l[1], 0.0)))  # right wall
    # end caps
    first, last = base, base + 4 * (n - 1)
    t0 = (left_edge[0][0] - left_edge[1][0], left_edge[0][1] - left_edge[1][1], 0.0)
    t1 = (left_edge[-1][0] - left_edge[-2][0], left_edge[-1][1] - left_edge[-2][1], 0.0)
    faces.append(_quad(pts, first, first + 1, first + 3, first + 2, t0))
    faces.append(_quad(pts, last, last + 1, last + 3, last + 2, t1))


def box_strip(cx: float, cy: float, cz: float, yaw: float, sx: float, sy: float, sz: float, pts, faces) -> None:
    """A legacy Box (centre, yaw, size) as one strip prism."""
    c, s_ = math.cos(yaw), math.sin(yaw)

    def w(dx, dy):
        return (cx + dx * c - dy * s_, cy + dx * s_ + dy * c)

    hx, hy = sx / 2.0, sy / 2.0
    strip_prism([w(-hx, hy), w(hx, hy)], [w(-hx, -hy), w(hx, -hy)], cz + sz / 2.0, sz, pts, faces)


def _fmt_mesh(name: str, color: str, pts, faces, comment: str) -> str:
    def _f(v):
        t = f"{v:.4f}".rstrip("0").rstrip(".")
        return "0" if t in ("-0", "") else t

    lines = [f"    # {comment}", f"    DEF {name} Shape {{"]
    lines += ["      appearance " + _APPEARANCE[color].replace("\n", "\n      ")]
    lines += ["      geometry IndexedFaceSet {", "        coord Coordinate {", "          point ["]
    for i in range(0, len(pts), 4):
        lines.append("            " + ", ".join(" ".join(_f(v) for v in p) for p in pts[i:i + 4]))
    lines += ["          ]", "        }", "        coordIndex ["]
    for i in range(0, len(faces), 6):
        lines.append("          " + " ".join(" ".join(str(k) for k in f) + " -1" for f in faces[i:i + 6]))
    lines += ["        ]", "        ccw TRUE", "      }", "    }"]
    return "\n".join(lines) + "\n"


def track_dict() -> dict:
    """The S as a waypoint track (same format as every tracks/*.json)."""
    n = int(round(FINISH_X_M / MESH_STEP_X_M))
    wps = []
    for i in range(n + 1):
        x = round(i * MESH_STEP_X_M, 6)
        wps.append([x, round(centerline(x)[0], 6)])
    x0, x1 = paint_x_range()
    return {
        "name": "s",
        "description": (f"Gentle double S: {START_STRAIGHT_M:g} m straight, two {LOBE_M:g} m cosine lobes "
                        f"(amplitude {AMP_M:g} m, first lobe {'south' if SIGN < 0 else 'north'}), "
                        f"{FINISH_STRAIGHT_M:g} m straight. Generated by scripts/s_track.py --write-track."),
        "lane_width_m": 2 * HALF_WIDTH_M,
        "stripe_width_m": LINE_W_M,
        "paint_before_start_m": round(-x0, 6),
        "paint_after_finish_m": round(x1 - FINISH_X_M, 6),
        "start_bar": True,
        "finish_bar": True,
        "ticks": [[tx, round(centerline(tx)[0], 6), round(centerline(tx)[1], 6)] for tx in TICK_XS],
        "waypoints": wps,
    }


def track_json() -> str:
    """tracks/s.json text: one waypoint per line so diffs stay readable."""
    import json

    d = track_dict()
    wps = d.pop("waypoints")
    ticks = d.pop("ticks")
    body = json.dumps(d, indent=2)[:-2]
    body += ',\n  "ticks": [' + ", ".join("[" + ", ".join(f"{v:g}" for v in t) + "]" for t in ticks) + "]"
    body += ',\n  "waypoints": [\n' + ",\n".join(f"    [{x:g}, {y:g}]" for x, y in wps) + "\n  ]\n}\n"
    return body


def geometry():
    from src.track_geometry import TrackGeometry, spec_from_dict

    return TrackGeometry(spec_from_dict(track_dict(), TRACK_JSON))


S_HEADER = [
    "Gentle S-curve track (visual only - no boundingObject)",
    f"Start (0,0)  first lobe SIGN={SIGN:g}  finish {FINISH_X_M:g}",
    f"Amplitude {AMP_M:g} m  min radius ~{min_radius_m():.1f} m  last {FINISH_STRAIGHT_M:g} m straight",
]


def emit_vrml(line_mode: str | None = None) -> str:
    """The S track block for butlerbot.wbt (ribbon lines; the old box paint is
    emit_vrml_boxes, archived in archives/butlerbot_track_boxes_2026-10-01.wbt)."""
    if line_mode not in (None, "ribbon"):
        raise ValueError("only ribbon lines are generated now; see emit_vrml_boxes for the archive")
    return emit_track_vrml(geometry(), S_HEADER, "scripts/s_track.py (python scripts/s_track.py --write-world)")


def emit_vrml_boxes() -> str:
    """Legacy generator (before 2026-10-01): one Transform + Box per paint piece.

    Kept for the archive and for the mesh-vs-boxes geometry check.
    """
    def _box(x, y, z, yaw, sx, sy, sz, color):
        app = "        appearance " + _APPEARANCE[color].replace("\n", "\n        ")
        return (
            f"Transform {{\n  translation {x:.4f} {y:.4f} {z:.3f}\n  rotation 0 0 1 {yaw:.5f}\n"
            f"  children [\n    Shape {{\n{app}\n      geometry Box {{\n"
            f"        size {sx:.3f} {sy:.3f} {sz:.3f}\n      }}\n    }}\n  ]\n}}\n"
        )

    chunks = [
        "# =============================================================================\n",
        "# Gentle S-curve track (visual only - no boundingObject)\n",
        f"# Start (0,0)  first lobe SIGN={SIGN:g}  finish {FINISH_X_M:g}\n",
        f"# Amplitude {AMP_M:g} m  min radius ~{min_radius_m():.1f} m  last {FINISH_STRAIGHT_M:g} m straight\n",
        "# Same yellow / red recipe as the old straight. Edges follow path normals.\n",
        "# =============================================================================\n\n",
        "# START line (green)\n",
        _box(0.0, 0.0, BAR_Z, 0.0, *BAR_SIZE, "green"),
        f"# FINISH line (red) - GPS ({FINISH_X_M:g}, 0)\n",
        _box(FINISH_X_M, 0.0, BAR_Z, 0.0, *BAR_SIZE, "red"),
        "\n# Lane edges (short tangent boxes)\n",
    ]
    for x, y, th, side in legacy_paint_boxes():
        chunks.append(_box(x, y, PAINT_Z, th, SEG_M, LINE_W_M, LINE_H_M, "yellow"))
    chunks.append("\n# Centerline ticks (human range posts)\n")
    for tx in TICK_XS:
        y, th = centerline(tx)
        chunks.append(_box(tx, y, TICK_Z, th, *TICK_SIZE, "tick"))
    return "".join(chunks)


def legacy_paint_boxes():
    """(x, y, yaw, side) of every legacy 0.38 m lane box, in emit order."""
    out = []
    x = 0.0
    while x <= FINISH_X_M + 1e-6:
        y, th = centerline(x)
        nx, ny = -math.sin(th), math.cos(th)
        out.append((x + HALF_WIDTH_M * nx, y + HALF_WIDTH_M * ny, th, 1))
        out.append((x - HALF_WIDTH_M * nx, y - HALF_WIDTH_M * ny, th, -1))
        x += PAINT_STEP_M
    return out


def write_world(path=None, line_mode: str | None = None) -> int:
    """Replace the track block in butlerbot.wbt with emit_vrml(). Returns 0."""
    from track_builder import replace_track_block

    path = path or os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                "webots", "worlds", "butlerbot.wbt")
    with open(path, encoding="utf-8") as fh:
        text = fh.read()
    text = replace_track_block(text, emit_vrml(line_mode))
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(text)
    return 0


def emit_path_csv(step: float = 0.25) -> str:
    """Static GPS map of the painted centerline (x, y, heading_rad)."""
    lines = ["x_m,y_m,heading_rad,kind"]
    x = 0.0
    while x <= FINISH_X_M + 1e-9:
        y, th = centerline(x)
        kind = "center"
        if abs(x) < 1e-6:
            kind = "start"
        elif abs(x - FINISH_X_M) < 1e-6:
            kind = "finish"
        lines.append(f"{x:.3f},{y:.4f},{th:.5f},{kind}")
        x = round(x + step, 6)
    return "\n".join(lines) + "\n"


if __name__ == "__main__":
    import sys as _sys

    if "--write-track" in _sys.argv:
        with open(TRACK_JSON, "w", encoding="utf-8", newline="\n") as _fh:
            _fh.write(track_json())
        print(f"wrote {TRACK_JSON}")
    if "--write-world" in _sys.argv:
        _sys.exit(write_world())
    if "--write-track" not in _sys.argv:
        print(emit_vrml())
    print(f"# path_length≈{path_length_m():.2f} m  finish_x={FINISH_X_M}", file=_sys.stderr)
