"""Gentle S-curve centerline + VRML paint for butlerbot.wbt.

Two cosine lobes (flat slope at the junctions) so the last stretch is
straight +X for the red-eye. SIGN flips the first bend so we can tell
line-follow from a memorized +Y-then-−Y lap. Width and paint stay put.
"""

from __future__ import annotations

import math

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
LINE_W_M = 0.06  # painted stripe width
LINE_H_M = 0.01  # stripe thickness; top at PAINT_Z + LINE_H_M / 2 = 0.013 m
PAINT_STEP_M = 0.34  # legacy box spacing (boxes were SEG_M = 0.38 long, overlapping)
MESH_STEP_X_M = 0.02  # ribbon sample spacing along x
BAR_SIZE = (0.08, 1.4, 0.02)  # start / finish bar (x, y, z), centre z BAR_Z
BAR_Z = 0.015
TICK_SIZE = (0.04, 0.45, 0.01)
TICK_Z = 0.01
TICK_XS = (3.0, 5.5, 8.0, 10.5, 13.0)

_APPEARANCE = {
    "yellow": """PBRAppearance {
  baseColor 0.95 0.95 0.2
  roughness 0.8
  metalness 0
}""",
    "red": """PBRAppearance {
  baseColor 0.9 0.15 0.12
  roughness 0.7
  metalness 0
  emissiveColor 0.25 0.04 0.03
}""",
    "green": """PBRAppearance {
  baseColor 0.1 0.85 0.25
  roughness 0.7
  metalness 0
  emissiveColor 0.05 0.2 0.05
}""",
    "tick": """PBRAppearance {
  baseColor 0.9 0.9 0.95
  roughness 0.9
}""",
}


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


# "boxes" (default): every legacy 0.38 m paint piece becomes one strip in the
# line's mesh, so the painted footprint is IDENTICAL to the old Box nodes,
# including the short gaps (up to ~13 cm) on the outer side of the bends where
# 0.34 m x-spacing stretches past 0.38 m along the line.
# "ribbon": one continuous strip along the exact offset curve (no gaps).
LINE_MODE = "ribbon"  # Dan 2026-10-01: smooth continuous lines


def track_meshes(line_mode: str | None = None) -> dict:
    """name -> (color, pts, faces, comment) for every painted part."""
    mode = line_mode or LINE_MODE
    top = PAINT_Z + LINE_H_M / 2.0
    out = {}
    for name, side, label in (("TRACK_LINE_L", 1.0, "Left lane line"), ("TRACK_LINE_R", -1.0, "Right lane line")):
        pts, faces = [], []
        x0, x1 = paint_x_range()
        if mode == "ribbon":
            sm = lane_line_samples(side)
            strip_prism([p[1] for p in sm], [p[2] for p in sm], top, LINE_H_M, pts, faces)
            how = "one continuous ribbon along the offset curve"
        else:
            n = 0
            for bx, by, th, sd in legacy_paint_boxes():
                if sd == side:
                    box_strip(bx, by, PAINT_Z, th, SEG_M, LINE_W_M, LINE_H_M, pts, faces)
                    n += 1
            how = f"{n} strips ({SEG_M} m tangent pieces every {PAINT_STEP_M} m of x, same as the old boxes)"
        out[name] = ("yellow", pts, faces,
                     f"{label} (yellow, {LINE_W_M * 100:.0f} cm): centreline offset "
                     f"{side * HALF_WIDTH_M:+.2f} m, x {x0:.2f}..{x1:.2f}, top z {top:.3f}; {how}")
    pts, faces = [], []
    box_strip(0.0, 0.0, BAR_Z, 0.0, *BAR_SIZE, pts, faces)
    out["TRACK_START"] = ("green", pts, faces, "START line (green) at x=0")
    pts, faces = [], []
    box_strip(FINISH_X_M, 0.0, BAR_Z, 0.0, *BAR_SIZE, pts, faces)
    out["TRACK_FINISH"] = ("red", pts, faces, f"FINISH line (red) - GPS ({FINISH_X_M:g}, 0)")
    pts, faces = [], []
    for tx in TICK_XS:
        y, th = centerline(tx)
        box_strip(tx, y, TICK_Z, th, *TICK_SIZE, pts, faces)
    out["TRACK_TICKS"] = ("tick", pts, faces, "Centerline ticks (human range posts), one strip each at x=" +
                          ", ".join(f"{t:g}" for t in TICK_XS))
    return out


def emit_vrml(line_mode: str | None = None) -> str:
    head = [
        "# =============================================================================\n",
        "# Gentle S-curve track (visual only - no boundingObject)\n",
        f"# Start (0,0)  first lobe SIGN={SIGN:g}  finish {FINISH_X_M:g}\n",
        f"# Amplitude {AMP_M:g} m  min radius ~{min_radius_m():.1f} m  last {FINISH_STRAIGHT_M:g} m straight\n",
        "# Generated by scripts/s_track.py (python scripts/s_track.py --write-world).\n",
        "# One DEF TRACK Pose; each painted part is ONE Shape / IndexedFaceSet (top,\n",
        "# walls, end caps; two-sided via walls). Replaces the 105 top-level Transform+Box\n",
        "# nodes (archived in archives/butlerbot_track_boxes_2026-10-01.wbt) with the\n",
        "# same footprint, colours, 6 cm stripe and heights (lines top 0.013, ticks\n",
        "# 0.015, bars 0.025 above the floor).\n",
        "# =============================================================================\n",
        "DEF TRACK Pose {\n",
        "  children [\n",
    ]
    body = [_fmt_mesh(n, c, p, f, cm) for n, (c, p, f, cm) in track_meshes(line_mode).items()]
    return "".join(head) + "".join(body) + "  ]\n}\n"


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
    import os

    path = path or os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                "webots", "worlds", "butlerbot.wbt")
    with open(path, encoding="utf-8") as fh:
        text = fh.read()
    start = text.index("# =============================================================================\n# Gentle S-curve track")
    end = text.index("\nRobot {", start) + 1
    text = text[:start] + emit_vrml(line_mode) + "\n" + text[end:]
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

    _mode = "boxes" if "--boxes" in _sys.argv else ("ribbon" if "--ribbon" in _sys.argv else None)
    if "--write-world" in _sys.argv:
        _sys.exit(write_world(line_mode=_mode))
    print(emit_vrml(_mode))
    print(f"# path_length≈{path_length_m():.2f} m  finish_x={FINISH_X_M}", file=__import__("sys").stderr)
