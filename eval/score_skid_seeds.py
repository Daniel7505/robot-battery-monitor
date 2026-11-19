"""Compare ButlerBotSkid seeds with the published caster baseline.

Reads /tmp/map_seeds/skid and /tmp/map_seeds/828e01d. Scoring geometry
(spawn, lip boxes) lives here. The mapper does not import this file.
"""

from __future__ import annotations

import csv
import math
import sys
from pathlib import Path

ROOTS = {
    "baseline": Path("/tmp/map_seeds/828e01d"),
    "skid": Path(sys.argv[1]) if len(sys.argv) > 1 else Path("/tmp/map_seeds/skid"),
}
SEEDS = (
    "stock",
    "p40",
    "m35",
    "east",
    "west",
    "sw",
    "north",
    "south",
)
SPAWNS = {
    "stock": (6.10, 4.70, 1.5708),
    "p40": (6.10, 4.70, 1.9708),
    "m35": (6.10, 4.70, 1.2208),
    "east": (5.40, 3.80, 0.0),
    "west": (6.60, 3.90, math.pi),
    "sw": (4.90, 4.50, 0.7),
    "north": (5.80, 5.60, 2.2),
    "south": (6.40, 6.30, -1.2),
}
# Vertical carpet steps the casters used to meet. The walkway gap and
# letter G. A row counts when the body is in the box.
LIPS = (
    ("walkway", 0.70, 1.80, 4.20, 4.85),
    ("letter_g", 3.45, 4.20, 4.65, 5.65),
)
BEDROOM = (0.0, 3.54, 4.74, 8.33)


def read_csv(path: Path):
    if not path.is_file():
        return []
    with path.open(encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def num(row, key, default=float("nan")):
    raw = row.get(key, "")
    if raw is None or raw == "":
        return default
    try:
        return float(raw)
    except ValueError:
        return default


def wrap(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


def unwrap(values):
    out = []
    prev = 0.0
    for index, value in enumerate(values):
        if index == 0:
            prev = value
            out.append(value)
            continue
        prev = prev + wrap(value - prev)
        out.append(prev)
    return out


def world_xy(ox, oy, sx, sy, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return sx + c * ox - s * oy, sy + s * ox + c * oy


def nearest(rows, t):
    return min(rows, key=lambda row: abs(num(row, "t_s") - t))


def episodes(rows, gap=0.2):
    if not rows:
        return []
    start = num(rows[0], "t_s")
    prev = start
    bursts = []
    for row in rows[1:]:
        t = num(row, "t_s")
        if t - prev > gap:
            bursts.append(prev - start)
            start = t
        prev = t
    bursts.append(prev - start)
    return bursts


def in_box(x, y, box) -> bool:
    _name, x0, x1, y0, y1 = box
    return x0 <= x <= x1 and y0 <= y <= y1


def score_one(folder: Path, name: str) -> dict:
    sx, sy, yaw0 = SPAWNS[name]
    gt = read_csv(folder / "gt_pose_log.csv")
    pose = read_csv(folder / "pose_log.csv")
    yaw_rows = read_csv(folder / "yaw_sources.csv")
    walls = read_csv(folder / "wall_contacts.csv")
    physics = read_csv(folder / "physics_log.csv")
    log = (folder / "webots.log").read_text(encoding="utf-8", errors="replace") if (folder / "webots.log").is_file() else ""
    out = {
        "name": name,
        "rows": len(gt),
        "cap": "time cap" in log,
        "floor_slip": log.count("floor slip, wheel motion rejected"),
        "lifted": log.count("wheels lifted"),
        "floor_backup": log.count("floor backup"),
        "still_backup": log.count("still backup"),
    }
    presses = episodes(walls)
    out["press_n"] = len(walls)
    out["press_episodes"] = len(presses)
    out["press_longest"] = max(presses) if presses else 0.0
    if not gt or not pose:
        out["head"] = float("nan")
        out["pos"] = float("nan")
        out["bed"] = False
        out["lip_stall_s"] = float("nan")
        out["lip_stall_n"] = 0
        out["lip_air_s"] = float("nan")
        return out
    gt_yaw = unwrap([num(row, "yaw_rad") for row in gt])
    end_t = num(gt[-1], "t_s")
    pr = nearest(pose, end_t)
    tr = nearest(gt, num(pr, "t_s"))
    wx, wy = world_xy(num(pr, "x_m"), num(pr, "y_m"), sx, sy, yaw0)
    out["pos"] = math.hypot(wx - num(tr, "x_m"), wy - num(tr, "y_m"))
    fused = num(pr, "yaw_rad")
    if yaw_rows:
        fused = num(nearest(yaw_rows, num(pr, "t_s")), "fused_yaw")
    ti = min(range(len(gt)), key=lambda i: abs(num(gt[i], "t_s") - num(pr, "t_s")))
    out["head"] = math.degrees(wrap(fused - (gt_yaw[ti] - yaw0)))
    out["end_xy"] = (num(tr, "x_m"), num(tr, "y_m"))
    out["bed"] = any(
        BEDROOM[0] < num(row, "x_m") < BEDROOM[1] and BEDROOM[2] < num(row, "y_m") < BEDROOM[3]
        for row in gt
    )
    stall_rows = []
    air_rows = []
    for row in physics:
        x, y = num(row, "x_m"), num(row, "y_m")
        if not any(in_box(x, y, box) for box in LIPS):
            continue
        speed = num(row, "speed_m_s")
        walls_n = num(row, "left_wall", 0) + num(row, "right_wall", 0) + num(row, "caster_wall", 0)
        if speed < 0.05 and walls_n == 0:
            stall_rows.append(row)
        zs = [num(row, "left_z"), num(row, "right_z"), num(row, "front_left_z"), num(row, "front_right_z")]
        zs = [value for value in zs if value == value]
        if zs and max(zs) > 0.086 and speed < 0.08:
            air_rows.append(row)
    stall_eps = episodes(stall_rows)
    long = [item for item in stall_eps if item >= 0.5]
    out["lip_stall_n"] = len(long)
    out["lip_stall_s"] = sum(long)
    out["lip_stall_max"] = max(long) if long else 0.0
    air_eps = [item for item in episodes(air_rows) if item >= 0.3]
    out["lip_air_s"] = sum(air_eps)
    out["lip_air_n"] = len(air_eps)
    return out


def fmt(value, digits=2):
    if value != value:
        return "nan"
    return f"{value:.{digits}f}"


def main() -> None:
    print(f"{'seed':8} {'robot':10} {'head':7} {'pos':7} {'press_s':8} {'press_n':8} {'lip_s':7} {'lip_n':6} {'air_s':7} {'slip':5} {'bed'}")
    for name in SEEDS:
        for label, root in ROOTS.items():
            folder = root / name
            if not folder.is_dir():
                print(f"{name:8} {label:10} MISSING")
                continue
            row = score_one(folder, name)
            print(
                f"{name:8} {label:10} {fmt(row['head']):>7} {fmt(row['pos'], 3):>7} "
                f"{fmt(row['press_longest']):>8} {row['press_n']:8d} "
                f"{fmt(row.get('lip_stall_s', float('nan'))):>7} {row.get('lip_stall_n', 0):6d} "
                f"{fmt(row.get('lip_air_s', float('nan'))):>7} {row['floor_slip']:5d} "
                f"{'yes' if row.get('bed') else 'no'}"
            )


if __name__ == "__main__":
    main()
