#!/usr/bin/env python3
"""Paint any waypoint track (tracks/*.json) into a Webots world as one DEF TRACK Pose.

Each painted part is ONE Shape with an IndexedFaceSet (top, walls and end caps
of the 6 cm stripe, ``ccw TRUE``, top faces wound +Z; no ``solid`` field, it
is not an IndexedFaceSet field in R2025a):

    TRACK_LINE_L / TRACK_LINE_R   continuous ribbons along the exact offset curves
                                  (true mitres at sharp corners, see src/track_geometry.py)
    TRACK_START / TRACK_FINISH    green / red bars across the lane (lane width + 0.10 m)
    TRACK_TICKS                   optional white centreline ticks
    TRACK_LINES (networks)        all lane-line pieces of a track with ``roads`` (intersections):
                                  lines are cut where another road's carriageway joins
    TRACK_EXIT_<NAME> (networks)  one red bar per exit (each is a GPS finish line)

    python scripts/track_builder.py corner90                        # print the TRACK block
    python scripts/track_builder.py corner90 --world webots/worlds/butlerbot_corner90.wbt
        (creates the world from butlerbot.wbt if it does not exist, else replaces its
         track block; robot, cameras, HUDs and lights stay as in the template)

The S in butlerbot.wbt is written by scripts/s_track.py through the same code.
"""
from __future__ import annotations

import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from src.track_geometry import TRACK_FILE_TAG, TrackGeometry, TrackNetwork, load_track  # noqa: E402

PAINT_Z = 0.008  # stripe centre height (top at 0.013, above the floor, no z-fighting)
LINE_H_M = 0.01
BAR_ALONG_M = 0.08
BAR_EXTRA_M = 0.10  # bar = lane width + this
BAR_H_M = 0.02
BAR_Z = 0.015
TICK_SIZE = (0.04, 0.45, 0.01)
TICK_Z = 0.01
TEMPLATE_WORLD = os.path.join(ROOT, "webots", "worlds", "butlerbot.wbt")
BLOCK_RULE = "# =============================================================================\n"

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


# --------------------------------------------------------------------------
# Mesh primitives
# --------------------------------------------------------------------------


def _quad(pts, a, b, c, d, want):
    """Face a-b-c-d, reversed if needed so its normal points along ``want``.

    Repeated corners (a collapsed inner fillet) are dropped; a face with fewer
    than 3 distinct corners returns None. Orientation uses the Newell normal,
    so it is right even with a repeated corner.
    """
    idx = []
    for k in (a, b, c, d):
        if idx and pts[idx[-1]] == pts[k]:
            continue
        idx.append(k)
    if len(idx) > 1 and pts[idx[0]] == pts[idx[-1]]:
        idx.pop()
    if len(idx) < 3:
        return None
    nx = ny = nz = 0.0
    for m in range(len(idx)):
        p, q = pts[idx[m]], pts[idx[(m + 1) % len(idx)]]
        nx += (p[1] - q[1]) * (p[2] + q[2])
        ny += (p[2] - q[2]) * (p[0] + q[0])
        nz += (p[0] - q[0]) * (p[1] + q[1])
    if abs(nx) + abs(ny) + abs(nz) < 1e-14:
        return None
    dot = nx * want[0] + ny * want[1] + nz * want[2]
    return tuple(idx) if dot >= 0 else tuple(reversed(idx))


def strip_prism(left_edge, right_edge, z_top: float, height: float, pts: list, faces: list) -> None:
    """Append a stripe prism (top + both walls + end caps) to pts/faces.

    left_edge/right_edge: [(x, y), ...] paired samples along the stripe.
    Top faces point +Z (counter-clockwise seen from above), walls outward.
    """
    L, R = [left_edge[0]], [right_edge[0]]
    for l, r in zip(left_edge[1:], right_edge[1:]):
        if l == L[-1] and r == R[-1]:
            continue  # sharp mitre sampled k times on both edges
        L.append(l)
        R.append(r)
    zb = z_top - height
    base = len(pts)
    for (lx, ly), (rx, ry) in zip(L, R):
        pts.extend([(lx, ly, z_top), (rx, ry, z_top), (lx, ly, zb), (rx, ry, zb)])
    n = len(L)

    def add(f):
        if f is not None:
            faces.append(f)

    for k in range(n - 1):
        i, j = base + 4 * k, base + 4 * (k + 1)
        lx, ly = L[k]
        rx, ry = R[k]
        out_l = (lx - rx, ly - ry, 0.0)
        add(_quad(pts, i + 1, j + 1, j, i, (0.0, 0.0, 1.0)))  # top
        add(_quad(pts, i, j, j + 2, i + 2, out_l))  # left wall
        add(_quad(pts, i + 1, i + 3, j + 3, j + 1, (-out_l[0], -out_l[1], 0.0)))  # right wall
    first, last = base, base + 4 * (n - 1)
    t0 = (L[0][0] - L[1][0], L[0][1] - L[1][1], 0.0)
    t1 = (L[-1][0] - L[-2][0], L[-1][1] - L[-2][1], 0.0)
    add(_quad(pts, first, first + 1, first + 3, first + 2, t0))
    add(_quad(pts, last, last + 1, last + 3, last + 2, t1))


def box_strip(cx: float, cy: float, cz: float, yaw: float, sx: float, sy: float, sz: float, pts, faces) -> None:
    """A Box (centre, yaw, size) as one strip prism."""
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


# --------------------------------------------------------------------------
# Track -> meshes -> VRML
# --------------------------------------------------------------------------


def _tick_pose(geom: TrackGeometry, x: float, y: float):
    pr = geom.project(x, y)
    c = geom.centerline
    i = pr["index"]
    a, b = c[i], c[i + 1]
    L = math.hypot(b[0] - a[0], b[1] - a[1]) or 1.0
    t = ((x - a[0]) * (b[0] - a[0]) + (y - a[1]) * (b[1] - a[1])) / (L * L)
    t = max(0.0, min(1.0, t))
    px, py = a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])
    if t < 1e-6:
        th = geom.heading_at_vertex(i)
    elif t > 1 - 1e-6:
        th = geom.heading_at_vertex(i + 1)
    else:
        th = pr["heading_rad"]
    return px, py, th


def network_meshes(net: TrackNetwork) -> dict:
    top = PAINT_Z + LINE_H_M / 2.0
    out = {}
    pts, faces = [], []
    segs = net.paint_segments()
    for _c, le, re_ in segs:
        strip_prism(le, re_, top, LINE_H_M, pts, faces)
    out["TRACK_LINES"] = ("yellow", pts, faces,
                          f"Lane lines (yellow, {net.stripe_w_m * 100:.0f} cm) of {len(net.roads)} roads, "
                          f"{len(segs)} pieces: cut where another road joins; top z {top:.3f}")
    for name, color, xy, th, w in net.bars():
        pts, faces = [], []
        box_strip(xy[0], xy[1], BAR_Z, th, BAR_ALONG_M, w + BAR_EXTRA_M, BAR_H_M, pts, faces)
        label = "START line (green)" if color == "green" else f"EXIT {name[11:].lower()} (red) - GPS finish"
        out[name] = (color, pts, faces, f"{label} at ({xy[0]:g}, {xy[1]:g})")
    return out


def track_meshes(geom: TrackGeometry) -> dict:
    """name -> (color, pts, faces, comment) for every painted part."""
    if isinstance(geom, TrackNetwork):
        return network_meshes(geom)
    sp = geom.spec
    top = PAINT_Z + LINE_H_M / 2.0
    out = {}
    for name, side, label in (("TRACK_LINE_L", 1.0, "Left lane line"), ("TRACK_LINE_R", -1.0, "Right lane line")):
        le, re_ = geom.stripe_edges(side)
        pts, faces = [], []
        strip_prism(le, re_, top, LINE_H_M, pts, faces)
        w0, w1 = min(sp.widths), max(sp.widths)
        wtxt = f"{side * w0 / 2:+.2f} m" if abs(w1 - w0) < 1e-9 else f"{side * w0 / 2:+.2f}..{side * w1 / 2:+.2f} m"
        out[name] = ("yellow", pts, faces,
                     f"{label} (yellow, {sp.stripe_w_m * 100:.0f} cm): centreline offset {wtxt}, "
                     f"top z {top:.3f}; one continuous ribbon along the offset curve")
    for name, color, xy, th, w, label in (
        ("TRACK_START", "green", geom.start_xy, geom.start_heading, sp.widths[0], "START line (green)"),
        ("TRACK_FINISH", "red", geom.finish_xy, geom.finish_heading, sp.widths[-1], "FINISH line (red) - GPS"),
    ):
        if not (sp.start_bar if name == "TRACK_START" else sp.finish_bar):
            continue
        pts, faces = [], []
        box_strip(xy[0], xy[1], BAR_Z, th, BAR_ALONG_M, w + BAR_EXTRA_M, BAR_H_M, pts, faces)
        out[name] = (color, pts, faces, f"{label} at ({xy[0]:g}, {xy[1]:g})")
    if sp.ticks:
        pts, faces = [], []
        for tk in sp.ticks:
            px, py, th = _tick_pose(geom, tk[0], tk[1])
            if len(tk) > 2:
                th = tk[2]  # explicit heading (the S gives the exact curve tangent)
            box_strip(px, py, TICK_Z, th, *TICK_SIZE, pts, faces)
        out["TRACK_TICKS"] = ("tick", pts, faces, "Centerline ticks (human range posts), one strip each at x=" +
                              ", ".join(f"{t[0]:g}" for t in sp.ticks))
    return out


def track_file_rel(geom: TrackGeometry) -> str | None:
    if not geom.spec.path:
        return None
    return os.path.relpath(os.path.abspath(geom.spec.path), ROOT).replace(os.sep, "/")


def default_header(geom: TrackGeometry) -> list:
    if isinstance(geom, TrackNetwork):
        ws = sorted({round(w, 2) for g in geom.roads for w in g.spec.widths})
        return [
            f"Track {geom.name} (visual only - no boundingObject)",
            geom.spec.description or f"{len(geom.roads)} roads",
            f"Start ({geom.start_xy[0]:g},{geom.start_xy[1]:g})  exits: " +
            ", ".join(f"{e.name} ({e.xy[0]:g},{e.xy[1]:g})" for e in geom.exits),
            f"Roads {len(geom.roads)}, lane widths {'/'.join(f'{w:.2f}' for w in ws)} m, "
            f"stripe {geom.stripe_w_m * 100:.0f} cm",
        ]
    sp = geom.spec
    fx, fy = geom.finish_xy
    w0, w1 = min(sp.widths), max(sp.widths)
    sharp = sum(1 for j in range(1, len(sp.waypoints) - 1) if sp.radii[j] <= 0 and abs(geom._phi[j]) > 1e-6)
    rounded = sum(1 for j in range(1, len(sp.waypoints) - 1) if sp.radii[j] > 0 and abs(geom._phi[j]) > 1e-6)
    return [
        f"Track {sp.name} (visual only - no boundingObject)",
        f"{sp.description}" if sp.description else f"{len(sp.waypoints)} waypoints",
        f"Start ({sp.waypoints[0][0]:g},{sp.waypoints[0][1]:g})  finish ({fx:g},{fy:g})  centreline {geom.length_m:.2f} m",
        f"Lane width {w0:.2f}" + (f"..{w1:.2f}" if w1 - w0 > 1e-9 else "") +
        f" m  stripe {sp.stripe_w_m * 100:.0f} cm  corners: {sharp} sharp, {rounded} rounded",
    ]


def emit_track_vrml(geom: TrackGeometry, header: list | None = None, generator: str | None = None) -> str:
    """The whole track block: rule, comment header, DEF TRACK Pose."""
    header = default_header(geom) if header is None else header
    rel = track_file_rel(geom)
    gen = generator or (f"python scripts/track_builder.py {rel} --world <this world>" if rel else "scripts/track_builder.py")
    lines = [BLOCK_RULE]
    lines += [f"# {h}\n" for h in header]
    if rel:
        lines.append(f"{TRACK_FILE_TAG}{rel}\n")
    lines += [
        f"# Generated by {gen}.\n",
        "# One DEF TRACK Pose; each painted part is ONE Shape / IndexedFaceSet (top,\n",
        "# walls, end caps; two-sided via walls). Lines top 0.013, ticks 0.015, bars\n",
        "# 0.025 above the floor. Do not hand-edit: change the track file and regenerate.\n",
        BLOCK_RULE,
        "DEF TRACK Pose {\n",
        "  children [\n",
    ]
    body = [_fmt_mesh(n, c, p, f, cm) for n, (c, p, f, cm) in track_meshes(geom).items()]
    return "".join(lines) + "".join(body) + "  ]\n}\n"


def replace_track_block(text: str, block: str) -> str:
    """Swap the generated track block (rule ... DEF TRACK Pose {...}) for ``block``."""
    pose = text.index("\nDEF TRACK Pose {") + 1
    start = text.rindex(BLOCK_RULE, 0, text.rindex(BLOCK_RULE, 0, pose))
    end = text.index("\nRobot {", pose) + 1
    return text[:start] + block + "\n" + text[end:]


def info_line(geom: TrackGeometry) -> str:
    if isinstance(geom, TrackNetwork):
        return (f'    "TRACK (ENU): {geom.name}  start ({geom.start_xy[0]:g},{geom.start_xy[1]:g})  exits '
                + " ".join(e.name for e in geom.exits) + f'  file {track_file_rel(geom)}"')
    fx, fy = geom.finish_xy
    w0, w1 = min(geom.spec.widths), max(geom.spec.widths)
    wt = f"{w0:.2f} m" if w1 - w0 < 1e-9 else f"{w0:.2f}..{w1:.2f} m"
    return (f'    "TRACK (ENU): {geom.spec.name}  start ({geom.start_xy[0]:g},{geom.start_xy[1]:g})  '
            f'finish ({fx:g},{fy:g})  lane {wt}  file {track_file_rel(geom)}"')


def write_world(geom: TrackGeometry, world: str, template: str = TEMPLATE_WORLD, header=None) -> str:
    """Create ``world`` from the template (if missing) and paint ``geom`` into it."""
    src = world if os.path.isfile(world) else template
    with open(src, encoding="utf-8") as fh:
        text = fh.read()
    text = replace_track_block(text, emit_track_vrml(geom, header))
    if src == template and os.path.abspath(world) != os.path.abspath(template):
        lines = text.split("\n")
        for k, ln in enumerate(lines):
            if ln.lstrip().startswith('"TRACK (ENU):'):
                lines[k] = info_line(geom)
                break
        text = "\n".join(lines)
    with open(world, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(text)
    return world


def main(argv=None) -> int:
    import argparse

    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("track", help="track name (tracks/<name>.json) or path")
    ap.add_argument("--world", help="world file to create/update")
    ap.add_argument("--template", default=TEMPLATE_WORLD)
    a = ap.parse_args(argv)
    geom = load_track(a.track)
    if a.world:
        print(f"wrote {write_world(geom, a.world, a.template)}  ({geom.spec.name}, {geom.length_m:.2f} m)")
    else:
        print(emit_track_vrml(geom))
    return 0


if __name__ == "__main__":
    sys.exit(main())
