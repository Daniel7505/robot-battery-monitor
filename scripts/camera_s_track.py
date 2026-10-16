"""S-curve the memorized track cannot follow.

tracks/s.json is a south-first cosine: amplitude 1 m, lobes 5 m.
This course is north-first: amplitude 1.5 m, lobes 6 m. At the old
trough the two centerlines are more than a lane apart, so a robot
that drove the s.json waypoints would leave the paint.

The world carries ``# STEER_TRACK off``. The finish referee still
loads this file to score the finish line, cross-track, and the true
lane width. The lane controller does not receive it.

Lane width is 1.30 m plus a 0.15 m sine over the course (1.15 m to
1.45 m). That is only ±7.5 cm on each line, so it can look even on
screen. ``RBM_CAMERA_S_WIDTH=1.0,1.6`` (or ``--width 1.0,1.6``) writes
a sibling world with a 0.30 m sine. Both yellow lines still stay inside
the nadir cameras at that range. The shipped world is left at 1.15–1.45 m.

    python3 scripts/camera_s_track.py
    RBM_CAMERA_S_WIDTH=1.0,1.6 python3 scripts/camera_s_track.py
"""

from __future__ import annotations

import math
import os
import re
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

START_STRAIGHT_M = 3.0
LOBE_M = 6.0
AMP_M = 1.5
SIGN = 1.0  # first lobe north. The scored S (tracks/s.json) goes south.
FINISH_STRAIGHT_M = 3.5
HALF_WIDTH_M = 0.65  # mean half-width of the shipped paint
WIDTH_M = 2 * HALF_WIDTH_M
WIDTH_AMP_M = 0.15  # shipped sine: 1.15 m to 1.45 m
# Opt-in stronger paint. "1.0,1.6" / "1.0-1.6" / "1.0:1.6".
WIDTH_ENV = "RBM_CAMERA_S_WIDTH"
DEFAULT_WIDTH_LO_M = WIDTH_M - WIDTH_AMP_M  # 1.15
DEFAULT_WIDTH_HI_M = WIDTH_M + WIDTH_AMP_M  # 1.45
FINISH_X_M = START_STRAIGHT_M + 2 * LOBE_M + FINISH_STRAIGHT_M  # 18.5
MESH_STEP_X_M = 0.02
LINE_W_M = 0.06
PAINT_BEFORE_M = 0.19
PAINT_AFTER_M = 0.01
TRACK_JSON = os.path.join(_ROOT, "tracks", "camera_s.json")
WORLD = os.path.join(_ROOT, "webots", "worlds", "butlerbot_camera_s.wbt")


def parse_width_range(text: str | None) -> tuple[float, float]:
    """``1.0,1.6`` / ``1.0-1.6`` / ``1.0:1.6``. Blank is the shipped 1.15–1.45 m."""
    raw = (text or "").strip().replace(" ", "")
    if not raw:
        return DEFAULT_WIDTH_LO_M, DEFAULT_WIDTH_HI_M
    match = re.match(r"^(\d+(?:\.\d+)?)[,:\-](\d+(?:\.\d+)?)$", raw)
    if match is None:
        raise ValueError(
            f"{WIDTH_ENV}={text!r} is not a min,max pair (example 1.0,1.6)"
        )
    lo, hi = float(match.group(1)), float(match.group(2))
    if not (0.4 <= lo < hi <= 2.5):
        raise ValueError(
            f"{WIDTH_ENV}={text!r} is outside 0.4–2.5 m with min < max"
        )
    return lo, hi


def is_default_width(lo: float, hi: float) -> bool:
    return abs(lo - DEFAULT_WIDTH_LO_M) < 1e-9 and abs(hi - DEFAULT_WIDTH_HI_M) < 1e-9


def width_m(
    x: float,
    lo: float = DEFAULT_WIDTH_LO_M,
    hi: float = DEFAULT_WIDTH_HI_M,
) -> float:
    """Lane width (m) at world x. One sine over the course."""
    mean = 0.5 * (float(lo) + float(hi))
    amp = 0.5 * (float(hi) - float(lo))
    return mean + amp * math.sin(2.0 * math.pi * float(x) / FINISH_X_M)


def centerline(x: float) -> tuple[float, float]:
    """(y, heading_rad) at world x. Flat slope where the straights meet the lobes."""
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


def curvature_1pm(x: float) -> float:
    """Signed centerline curvature (1/m, + = left) from the cosine slope."""
    x = float(x)
    if x <= START_STRAIGHT_M or x >= START_STRAIGHT_M + 2 * LOBE_M:
        return 0.0
    if x <= START_STRAIGHT_M + LOBE_M:
        u = x - START_STRAIGHT_M
        sign = SIGN
    else:
        u = x - START_STRAIGHT_M - LOBE_M
        sign = -SIGN
    w = 2.0 * math.pi / LOBE_M
    yp = sign * (AMP_M / 2.0) * w * math.sin(w * u)
    ypp = sign * (AMP_M / 2.0) * w * w * math.cos(w * u)
    return ypp / (1.0 + yp * yp) ** 1.5


def min_radius_m() -> float:
    return 1.0 / ((AMP_M / 2.0) * (2.0 * math.pi / LOBE_M) ** 2)


def track_dict(
    lo: float = DEFAULT_WIDTH_LO_M,
    hi: float = DEFAULT_WIDTH_HI_M,
) -> dict:
    n = int(round(FINISH_X_M / MESH_STEP_X_M))
    wps = []
    for i in range(n + 1):
        x = round(i * MESH_STEP_X_M, 6)
        wps.append([x, round(centerline(x)[0], 6), round(width_m(x, lo, hi), 4)])
    ticks_x = (
        START_STRAIGHT_M,
        START_STRAIGHT_M + 0.5 * LOBE_M,
        START_STRAIGHT_M + LOBE_M,
        START_STRAIGHT_M + 1.5 * LOBE_M,
        START_STRAIGHT_M + 2 * LOBE_M,
    )
    return {
        "name": "camera_s",
        "description": (
            f"Camera-proof double S: {START_STRAIGHT_M:g} m straight, two {LOBE_M:g} m "
            f"cosine lobes (amplitude {AMP_M:g} m, first lobe north), "
            f"{FINISH_STRAIGHT_M:g} m straight. Lane width {lo:.2f}–"
            f"{hi:.2f} m (sine, mean {0.5 * (lo + hi):.2f} m). Not tracks/s.json "
            f"(amplitude 1 m, lobes 5 m, first lobe south). "
            "Generated by scripts/camera_s_track.py."
        ),
        "lane_width_m": round(0.5 * (lo + hi), 4),
        "stripe_width_m": LINE_W_M,
        "paint_before_start_m": PAINT_BEFORE_M,
        "paint_after_finish_m": PAINT_AFTER_M,
        "start_bar": True,
        "finish_bar": True,
        "ticks": [
            [tx, round(centerline(tx)[0], 6), round(centerline(tx)[1], 6)] for tx in ticks_x
        ],
        "waypoints": wps,
    }


def track_json(
    lo: float = DEFAULT_WIDTH_LO_M,
    hi: float = DEFAULT_WIDTH_HI_M,
) -> str:
    import json

    d = track_dict(lo, hi)
    wps = d.pop("waypoints")
    ticks = d.pop("ticks")
    body = json.dumps(d, indent=2)[:-2]
    body += ',\n  "ticks": [' + ", ".join(
        "[" + ", ".join(f"{v:g}" for v in t) + "]" for t in ticks
    ) + "]"
    body += ',\n  "waypoints": [\n' + ",\n".join(
        f"    [{x:g}, {y:g}, {w:.4f}]" for x, y, w in wps
    ) + "\n  ]\n}\n"
    return body


def output_paths(
    lo: float = DEFAULT_WIDTH_LO_M,
    hi: float = DEFAULT_WIDTH_HI_M,
    root: str | None = None,
) -> tuple[str, str]:
    """Shipped files for 1.15–1.45 m. Any other range is a sibling."""
    root = _ROOT if root is None else root
    if is_default_width(lo, hi) and os.path.abspath(root) == os.path.abspath(_ROOT):
        return TRACK_JSON, WORLD
    tag = f"w{lo:.2f}-{hi:.2f}"
    return (
        os.path.join(root, "tracks", f"camera_s_{tag}.json"),
        os.path.join(root, "webots", "worlds", f"butlerbot_camera_s_{tag}.wbt"),
    )


def write_files(
    lo: float = DEFAULT_WIDTH_LO_M,
    hi: float = DEFAULT_WIDTH_HI_M,
    root: str | None = None,
) -> str:
    """Write the track and world. Return the world path."""
    from src.track_geometry import TrackGeometry, spec_from_dict
    from track_builder import TEMPLATE_WORLD, write_world

    track_path, world_path = output_paths(lo, hi, root)
    os.makedirs(os.path.dirname(track_path), exist_ok=True)
    os.makedirs(os.path.dirname(world_path), exist_ok=True)
    with open(track_path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(track_json(lo, hi))
    geom = TrackGeometry(spec_from_dict(track_dict(lo, hi), track_path))
    header = [
        "Camera-proof S (visual only - no boundingObject)",
        "STEER_TRACK off",
        "The waypoint file scores the finish and cross-track. It is not given to the wheel command.",
        (
            f"Amplitude {AMP_M:g} m, lobe {LOBE_M:g} m, first lobe north, "
            f"min radius ~{min_radius_m():.2f} m. "
            f"Lane width {lo:.2f} to {hi:.2f} m. "
            "tracks/s.json is amplitude 1 m, lobe 5 m, first lobe south, width 1.30 m."
        ),
    ]
    template = TEMPLATE_WORLD
    if os.path.abspath(world_path) != os.path.abspath(WORLD) and os.path.isfile(WORLD):
        template = WORLD
    write_world(geom, world_path, template, header)
    print(f"wrote {track_path}")
    print(f"wrote {world_path}  ({geom.length_m:.2f} m, finish x={FINISH_X_M:g})")
    return world_path


def main(argv: list[str] | None = None) -> int:
    import argparse

    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument(
        "--width",
        default=None,
        help=f"min,max metres ({WIDTH_ENV}). Example: 1.0,1.6",
    )
    ap.add_argument("--root", default=None, help="output root (default: repo)")
    args = ap.parse_args(argv)
    requested = args.width if args.width is not None else os.environ.get(WIDTH_ENV)
    if args.width is None and not os.environ.get(WIDTH_ENV) and args.root is None:
        world = write_files()
    else:
        lo, hi = parse_width_range(requested)
        if is_default_width(lo, hi) and args.root is None:
            world = WORLD
            print(f"shipped paint {lo:.2f} to {hi:.2f} m, leaving {WORLD}")
        else:
            world = write_files(lo, hi, root=args.root)
    print(f"CAMERA_S_WORLD {world}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except ValueError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
