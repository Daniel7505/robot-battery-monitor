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


def _box(x: float, y: float, z: float, yaw: float, sx: float, sy: float, sz: float, color: str) -> str:
    if color == "yellow":
        app = """        appearance PBRAppearance {
          baseColor 0.95 0.95 0.2
          roughness 0.8
          metalness 0
        }"""
    elif color == "red":
        app = """        appearance PBRAppearance {
          baseColor 0.9 0.15 0.12
          roughness 0.7
          metalness 0
          emissiveColor 0.25 0.04 0.03
        }"""
    elif color == "green":
        app = """        appearance PBRAppearance {
          baseColor 0.1 0.85 0.25
          roughness 0.7
          metalness 0
          emissiveColor 0.05 0.2 0.05
        }"""
    else:
        app = """        appearance PBRAppearance {
          baseColor 0.9 0.9 0.95
          roughness 0.9
        }"""
    return (
        f"Transform {{\n"
        f"  translation {x:.4f} {y:.4f} {z:.3f}\n"
        f"  rotation 0 0 1 {yaw:.5f}\n"
        f"  children [\n"
        f"    Shape {{\n"
        f"{app}\n"
        f"      geometry Box {{\n"
        f"        size {sx:.3f} {sy:.3f} {sz:.3f}\n"
        f"      }}\n"
        f"    }}\n"
        f"  ]\n"
        f"}}\n"
    )


def emit_vrml() -> str:
    chunks = [
        "# =============================================================================\n",
        "# Gentle S-curve track (visual only - no boundingObject)\n",
        f"# Start (0,0)  first lobe SIGN={SIGN:g}  finish {FINISH_X_M:g}\n",
        f"# Amplitude {AMP_M:g} m  min radius ~1.9 m  last {FINISH_STRAIGHT_M:g} m straight\n",
        "# Same yellow / red recipe as the old straight. Edges follow path normals.\n",
        "# =============================================================================\n\n",
        "# START line (green)\n",
        _box(0.0, 0.0, 0.015, 0.0, 0.08, 1.4, 0.02, "green"),
        f"# FINISH line (red) - GPS ({FINISH_X_M:g}, 0)\n",
        _box(FINISH_X_M, 0.0, 0.015, 0.0, 0.08, 1.4, 0.02, "red"),
        "\n# Lane edges (short tangent boxes)\n",
    ]
    x = 0.0
    while x <= FINISH_X_M + 1e-6:
        y, th = centerline(x)
        nx, ny = -math.sin(th), math.cos(th)
        lx, ly = x + HALF_WIDTH_M * nx, y + HALF_WIDTH_M * ny
        rx, ry = x - HALF_WIDTH_M * nx, y - HALF_WIDTH_M * ny
        chunks.append(_box(lx, ly, PAINT_Z, th, SEG_M, 0.06, 0.01, "yellow"))
        chunks.append(_box(rx, ry, PAINT_Z, th, SEG_M, 0.06, 0.01, "yellow"))
        x += 0.34
    chunks.append("\n# Centerline ticks (human range posts)\n")
    for tx in (3.0, 5.5, 8.0, 10.5, 13.0):
        y, th = centerline(tx)
        chunks.append(_box(tx, y, 0.01, th, 0.04, 0.45, 0.01, "tick"))
    return "".join(chunks)


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
    print(emit_vrml())
    print(f"# path_length≈{path_length_m():.2f} m  finish_x={FINISH_X_M}", file=__import__("sys").stderr)
