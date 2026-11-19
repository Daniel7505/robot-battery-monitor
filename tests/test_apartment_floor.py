"""Walkable apartment floor has support. Closed doors and the outdoors do not.

Room plates stop at each wall face. A doorway in that 4 in partition is a
hole unless a plate fills the slot. The outdoors, a wall footprint, and a
closed door are not walkable floor.
"""

from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
WORLD = ROOT / "webots" / "worlds" / "apartment.wbt"
WALL_M = 0.1016


def _strip(text: str) -> str:
    return re.sub(r"#[^\n]*", "", text)


def _body(text: str, open_idx: int) -> str:
    depth = 0
    for i in range(open_idx, len(text)):
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                return text[open_idx + 1 : i]
    raise AssertionError("unbalanced brace")


def _solids(text: str) -> dict[str, dict]:
    raw = _strip(text)
    solids: dict[str, dict] = {}
    for match in re.finditer(r"\bDEF\s+(APT_\w+)\s+Solid\s*\{", raw):
        body = _body(raw, match.end() - 1)
        trans = re.search(
            r"translation\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)",
            body,
        )
        cx, cy, cz = (float(v) for v in trans.groups())
        box = re.search(
            r"boundingObject\s+Box\s*\{\s*size\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)",
            body,
        )
        vis = re.search(
            r"geometry\s+Box\s*\{\s*size\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)",
            body,
        )
        sx, sy, sz = (float(v) for v in box.groups())
        solids[match.group(1)] = {
            "xmin": cx - sx / 2,
            "xmax": cx + sx / 2,
            "ymin": cy - sy / 2,
            "ymax": cy + sy / 2,
            "zmin": cz - sz / 2,
            "zmax": cz + sz / 2,
            "sx": sx,
            "sy": sy,
            "sz": sz,
            "cx": cx,
            "cy": cy,
            "vis": tuple(float(v) for v in vis.groups()) if vis else None,
        }
    return solids


def _covers(floors: list[dict], x: float, y: float) -> bool:
    return any(
        floor["xmin"] - 1e-4 <= x <= floor["xmax"] + 1e-4
        and floor["ymin"] - 1e-4 <= y <= floor["ymax"] + 1e-4
        and floor["zmax"] >= -0.001
        for floor in floors
    )


def _openings(solids: dict) -> list[dict]:
    """Gaps between collinear wall pieces, including a 4 in post on that line."""
    walls = {name: box for name, box in solids.items() if name.startswith("APT_WALL")}
    vertical: dict[float, list[tuple[str, dict]]] = {}
    horizontal: dict[float, list[tuple[str, dict]]] = {}
    posts: list[tuple[str, dict]] = []
    for name, box in walls.items():
        thin_x = box["sx"] <= 0.15
        thin_y = box["sy"] <= 0.15
        if thin_x and not thin_y:
            vertical.setdefault(round(box["cx"], 3), []).append((name, box))
        elif thin_y and not thin_x:
            horizontal.setdefault(round(box["cy"], 3), []).append((name, box))
        elif thin_x and thin_y:
            posts.append((name, box))
    for name, box in posts:
        key = round(box["cx"], 3)
        if key in vertical:
            vertical[key].append((name, box))
        key = round(box["cy"], 3)
        if key in horizontal:
            horizontal[key].append((name, box))
    found = []
    for key, segs in vertical.items():
        segs.sort(key=lambda item: item[1]["ymin"])
        for (left_name, left), (right_name, right) in zip(segs, segs[1:]):
            gap = right["ymin"] - left["ymax"]
            if gap <= 0.05:
                continue
            found.append(
                {
                    "axis": "y",
                    "a": left_name,
                    "b": right_name,
                    "gap": gap,
                    "x0": max(left["xmin"], right["xmin"]),
                    "x1": min(left["xmax"], right["xmax"]),
                    "y0": left["ymax"],
                    "y1": right["ymin"],
                    "cx": key,
                    "cy": (left["ymax"] + right["ymin"]) / 2,
                }
            )
    for key, segs in horizontal.items():
        segs.sort(key=lambda item: item[1]["xmin"])
        for (left_name, left), (right_name, right) in zip(segs, segs[1:]):
            gap = right["xmin"] - left["xmax"]
            if gap <= 0.05:
                continue
            found.append(
                {
                    "axis": "x",
                    "a": left_name,
                    "b": right_name,
                    "gap": gap,
                    "x0": left["xmax"],
                    "x1": right["xmin"],
                    "y0": max(left["ymin"], right["ymin"]),
                    "y1": min(left["ymax"], right["ymax"]),
                    "cx": (left["xmax"] + right["xmin"]) / 2,
                    "cy": key,
                }
            )
    return found


def _door_covers(doors: list[dict], opening: dict) -> bool:
    x, y = opening["cx"], opening["cy"]
    return any(
        door["xmin"] <= x <= door["xmax"]
        and door["ymin"] <= y <= door["ymax"]
        and door["zmin"] <= 0.0
        and door["zmax"] > 0.5
        for door in doors
    )


def _unsupported(floors: list[dict], opening: dict) -> int:
    inset = 0.01
    x0, x1 = opening["x0"], opening["x1"]
    y0, y1 = opening["y0"] + inset, opening["y1"] - inset
    if opening["axis"] == "x":
        x0, x1 = opening["x0"] + inset, opening["x1"] - inset
        y0, y1 = opening["y0"], opening["y1"]
    if x1 <= x0 or y1 <= y0:
        return 0
    missing = 0
    y = y0
    while y < y1:
        x = x0
        while x < x1:
            if not _covers(floors, x + 0.005, y + 0.005):
                missing += 1
            x += 0.01
        y += 0.01
    return missing


def test_floor_boxes_match_the_visible_surface_and_sit_flush():
    solids = _solids(WORLD.read_text(encoding="utf-8"))
    floors = {name: box for name, box in solids.items() if name.startswith("APT_FLOOR")}
    assert floors
    for name, box in floors.items():
        assert box["vis"] is not None, name
        assert abs(box["vis"][0] - box["sx"]) < 1e-6
        assert abs(box["vis"][1] - box["sy"]) < 1e-6
        assert abs(box["vis"][2] - box["sz"]) < 1e-6
        assert abs(box["zmax"]) <= 0.001
        assert abs(box["sz"] - 0.02) < 1e-6


def test_open_interior_doorways_have_floor_and_closed_doors_do_not_need_it():
    solids = _solids(WORLD.read_text(encoding="utf-8"))
    floors = [box for name, box in solids.items() if name.startswith("APT_FLOOR")]
    doors = [box for name, box in solids.items() if name.startswith("APT_DOOR")]
    openings = _openings(solids)
    assert openings, "no wall openings parsed"
    open_names = set()
    closed = []
    for opening in openings:
        if _door_covers(doors, opening):
            closed.append(opening)
            continue
        missing = _unsupported(floors, opening)
        assert missing == 0, (
            f"{opening['a']} .. {opening['b']} gap={opening['gap']:.3f} m "
            f"has {missing} unsupported samples"
        )
        open_names.add((opening["a"], opening["b"]))
    # The two slots that dropped a wheel, plus the doorways that already had plates.
    assert ("APT_WALL_SHARED_B", "APT_WALL_SHARED_C") in open_names
    assert ("APT_WALL_BED_SOUTH_A", "APT_WALL_BED_SOUTH_B") in open_names
    assert ("APT_WALL_BATH_EAST_A", "APT_WALL_BATH_EAST_B") in open_names
    assert ("APT_WALL_SHARED_A", "APT_WALL_SHARED_B") in open_names
    assert ("APT_WALL_LAUNDRY_N_A", "APT_WALL_LAUNDRY_N_B") in open_names
    assert closed, "front and patio doors should close their openings"
    # A point in the old 0.102 m slot is on a plate. A point outside is not required.
    assert _covers(floors, 3.7084, 5.1308)
    assert _covers(floors, 1.2192, 4.572)
    assert not _covers(floors, 9.0, 4.0)


def test_the_bedroom_partition_slot_is_the_four_inch_wall():
    """The hole was the partition thickness, not a measured opening in the floor."""
    solids = _solids(WORLD.read_text(encoding="utf-8"))
    slot = solids["APT_FLOOR_LIVING"]["xmin"] - solids["APT_FLOOR_BEDROOM"]["xmax"]
    assert abs(slot - WALL_M) < 1e-4
    bridge = solids["APT_FLOOR_BRIDGE_BEDROOM"]
    assert bridge["ymin"] >= solids["APT_WALL_SHARED_B"]["ymax"] - 1e-4
    assert bridge["ymax"] <= solids["APT_WALL_SHARED_C"]["ymin"] + 1e-4
    walk = solids["APT_FLOOR_BRIDGE_BED_BATH"]
    assert walk["xmin"] >= solids["APT_WALL_BED_SOUTH_A"]["xmax"] - 1e-4
    assert walk["xmax"] <= solids["APT_WALL_BED_SOUTH_B"]["xmin"] + 1e-4
