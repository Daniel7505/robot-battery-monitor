"""Compare the robot's own room labels with the apartment floors.

Scoring only. The mapper writes ``rooms.npy`` in the odometry frame and
does not know these floor names. The four rooms and the hallway are the
living room, the kitchen, the bedroom, the bath, and the hall. Closets
and the laundry are not part of that count.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from eval.apartment_gt_grid import load_solids, rasterize, spawn_pose
from eval.score_apartment_map import warp_to_ground_truth


EXPECTED = (
    ("APT_FLOOR_LIVING", "room"),
    ("APT_FLOOR_KITCHEN", "room"),
    ("APT_FLOOR_BEDROOM", "room"),
    ("APT_FLOOR_BATH", "room"),
    ("APT_FLOOR_HALL", "hall"),
)


def score_rooms(run_dir: Path, *, world: Path | None = None) -> dict:
    folder = Path(run_dir)
    rooms_path = folder / "rooms.npy"
    meta_path = folder / "map_meta.json"
    names_path = folder / "rooms_meta.json"
    if not rooms_path.is_file() or not meta_path.is_file():
        return {"found": [], "missing": [name for name, _kind in EXPECTED], "labels": []}
    regions = np.load(rooms_path)
    meta = json.loads(meta_path.read_text(encoding="utf-8"))
    names = {}
    if names_path.is_file():
        raw = json.loads(names_path.read_text(encoding="utf-8")).get("names", {})
        names = {int(key): value for key, value in raw.items()}
    sx, sy, yaw = spawn_pose(world=world)
    gt = rasterize(load_solids(world) if world is not None else load_solids())
    warped = warp_to_ground_truth(regions.astype(np.int16), meta, gt, (sx, sy), yaw)
    solids = load_solids(world) if world is not None else load_solids()
    found = []
    missing = []
    for solid_name, _kind in EXPECTED:
        solid = next((item for item in solids if item.name == solid_name), None)
        if solid is None:
            missing.append(solid_name)
            continue
        # Floor cells inside this solid, inset by one ground-truth cell.
        res = gt.resolution_m
        cols = np.arange(gt.labels.shape[1])
        rows = np.arange(gt.labels.shape[0])
        cc, rr = np.meshgrid(cols, rows)
        xw = gt.origin_xy[0] + (cc + 0.5) * res
        yw = gt.origin_xy[1] + (rr + 0.5) * res
        inside = (
            (xw > solid.xmin + 0.15)
            & (xw < solid.xmax - 0.15)
            & (yw > solid.ymin + 0.15)
            & (yw < solid.ymax - 0.15)
        )
        ids = warped[inside]
        ids = ids[ids > 0]
        if ids.size == 0:
            missing.append(solid_name)
            continue
        values, counts = np.unique(ids, return_counts=True)
        best = int(values[int(np.argmax(counts))])
        overlap = float(counts.max()) / float(max(1, int(np.count_nonzero(inside))))
        if overlap < 0.15:
            missing.append(solid_name)
            continue
        found.append({"floor": solid_name, "label": names.get(best, str(best)), "overlap": round(overlap, 3)})
    return {
        "found": found,
        "missing": missing,
        "labels": sorted(set(names.values())),
    }
