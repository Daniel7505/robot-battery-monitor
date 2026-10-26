"""Raster of the apartment for scoring only.

Floor, wall, door, and furniture boxes come from ``apartment.wbt``.
The spawn translation comes from ``eval/apartment_ground_truth.json``.
The spawn heading comes from the world's ButlerBot rotation. The mapper
does not import this module.
"""

from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
WORLD = ROOT / "webots" / "worlds" / "apartment.wbt"
GROUND_TRUTH = ROOT / "eval" / "apartment_ground_truth.json"


@dataclass(frozen=True)
class Solid:
    name: str
    xmin: float
    xmax: float
    ymin: float
    ymax: float
    zmin: float
    zmax: float


@dataclass
class GtGrid:
    labels: np.ndarray  # -1 unknown, 0 free, 1 occupied
    origin_xy: tuple[float, float]
    resolution_m: float

    @property
    def free(self) -> np.ndarray:
        return self.labels == 0

    @property
    def occupied(self) -> np.ndarray:
        return self.labels == 1


def _strip_comments(text: str) -> str:
    return re.sub(r"#[^\n]*", "", text)


def _brace_body(text: str, open_idx: int) -> str:
    depth = 0
    for i in range(open_idx, len(text)):
        ch = text[i]
        if ch == "{":
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0:
                return text[open_idx + 1 : i]
    raise ValueError("unbalanced brace")


def load_solids(path: Path | None = None) -> list[Solid]:
    raw = _strip_comments((path or WORLD).read_text(encoding="utf-8"))
    solids: list[Solid] = []
    for match in re.finditer(r"\bDEF\s+(APT_\w+)\s+Solid\s*\{", raw):
        body = _brace_body(raw, match.end() - 1)
        trans = re.search(
            r"translation\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)",
            body,
        )
        box = re.search(
            r"boundingObject\s+Box\s*\{\s*size\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)",
            body,
        )
        if not trans or not box:
            continue
        cx, cy, cz = (float(v) for v in trans.groups())
        sx, sy, sz = (float(v) for v in box.groups())
        solids.append(
            Solid(
                match.group(1),
                cx - sx / 2.0,
                cx + sx / 2.0,
                cy - sy / 2.0,
                cy + sy / 2.0,
                cz - sz / 2.0,
                cz + sz / 2.0,
            )
        )
    return solids


def spawn_pose(world: Path | None = None, ground_truth: Path | None = None) -> tuple[float, float, float]:
    """World xy from the scoring JSON, heading from the ButlerBot rotation."""
    gt = json.loads((ground_truth or GROUND_TRUTH).read_text(encoding="utf-8"))
    sx, sy, _sz = (float(v) for v in gt["spawn"]["translation_m"])
    text = _strip_comments((world or WORLD).read_text(encoding="utf-8"))
    match = re.search(
        r"ButlerBot\s*\{.*?rotation\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)",
        text,
        re.S,
    )
    if match is None:
        raise ValueError("ButlerBot rotation not found")
    ax, ay, az, ang = (float(v) for v in match.groups())
    # The apartment instance is a yaw about +Z. Any other axis falls back to 0.
    yaw = float(ang) if abs(az) >= 0.9 and abs(ax) < 0.1 and abs(ay) < 0.1 else 0.0
    if az < 0:
        yaw = -yaw
    return sx, sy, yaw


def rasterize(solids: list[Solid] | None = None, resolution_m: float = 0.05) -> GtGrid:
    """Free = floor solids. Occupied = walls, doors, and furniture in the height band."""
    solids = list(solids if solids is not None else load_solids())
    floors = [s for s in solids if s.name.startswith("APT_FLOOR_")]
    blockers = [
        s
        for s in solids
        if s.name.startswith(("APT_WALL_", "APT_DOOR_", "APT_FURN_")) and s.zmax > 0.05 and s.zmin < 1.5
    ]
    if not floors:
        raise ValueError("no floor solids")
    res = float(resolution_m)
    xmin = min(s.xmin for s in floors + blockers) - res
    ymin = min(s.ymin for s in floors + blockers) - res
    xmax = max(s.xmax for s in floors + blockers) + res
    ymax = max(s.ymax for s in floors + blockers) + res
    width = int(math.ceil((xmax - xmin) / res))
    height = int(math.ceil((ymax - ymin) / res))
    labels = np.full((height, width), -1, dtype=np.int8)

    def _fill(solid: Solid, value: int) -> None:
        c0 = max(0, int(math.floor((solid.xmin - xmin) / res)))
        c1 = min(width, int(math.ceil((solid.xmax - xmin) / res)))
        r0 = max(0, int(math.floor((solid.ymin - ymin) / res)))
        r1 = min(height, int(math.ceil((solid.ymax - ymin) / res)))
        if c1 > c0 and r1 > r0:
            labels[r0:r1, c0:c1] = value

    for solid in floors:
        _fill(solid, 0)
    for solid in blockers:
        _fill(solid, 1)
    return GtGrid(labels=labels, origin_xy=(xmin, ymin), resolution_m=res)
