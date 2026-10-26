"""Apartment twin: measured shell, estimated furniture, no controller foresight.

Scoring numbers live in ``eval/apartment_ground_truth.json``. The Webots
controller must not import or open that file. This test reads it. The robot
does not.
"""

from __future__ import annotations

import ast
import json
import math
import re
from collections import deque
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
WORLD = ROOT / "webots" / "worlds" / "apartment.wbt"
GROUND_TRUTH = ROOT / "eval" / "apartment_ground_truth.json"
CONTROLLER_ROOT = ROOT / "webots" / "controllers"
GT_NAME = "apartment_ground_truth.json"

# Letters on Dan's tape. Door letters are minimum clear openings.
_ROOM_LETTERS = {
    "A_living_width": "A",
    "B_living_kitchen_length": "B",
    "C_bedroom_width": "C",
    "D_bedroom_length": "D",
    "E_hallway_width": "E",
    "F_kitchen_aisle": "F",
}
_DOOR_LETTERS = {
    "G_bedroom_door": "G",
    "H_bathroom_door": "H",
    "I_front_door": "I",
    "J_patio_door": "J",
}


def _ground_truth() -> dict:
    return json.loads(GROUND_TRUTH.read_text(encoding="utf-8"))


def _inches(gt: dict, key: str) -> float:
    return float(gt["measured_in"][key]) * float(gt["inch_to_metre"])


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
    raise AssertionError("unbalanced brace in apartment world")


def _parse_solids(text: str) -> dict[str, dict]:
    raw = _strip_comments(text)
    solids: dict[str, dict] = {}
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
        assert trans and box, match.group(1)
        cx, cy, cz = (float(v) for v in trans.groups())
        sx, sy, sz = (float(v) for v in box.groups())
        material = re.search(r'contactMaterial\s+"([^"]+)"', body)
        solids[match.group(1)] = {
            "xmin": cx - sx / 2.0,
            "xmax": cx + sx / 2.0,
            "ymin": cy - sy / 2.0,
            "ymax": cy + sy / 2.0,
            "zmin": cz - sz / 2.0,
            "zmax": cz + sz / 2.0,
            "material": material.group(1) if material else None,
        }
    return solids


def _prefixed(solids: dict, prefix: str) -> list[dict]:
    return [solids[name] for name in solids if name.startswith(prefix)]


def _agg(solids: dict, prefix: str, field: str, how: str) -> float:
    vals = [item[field] for item in _prefixed(solids, prefix)]
    assert vals, prefix
    return min(vals) if how == "min" else max(vals)


def _gaps_x(solids: dict, prefix: str) -> list[float]:
    segs = sorted(_prefixed(solids, prefix), key=lambda item: item["xmin"])
    gaps = []
    for left, right in zip(segs, segs[1:]):
        gap = right["xmin"] - left["xmax"]
        if gap > 0.05:
            gaps.append(gap)
    return gaps


def _gaps_y(solids: dict, prefix: str) -> list[float]:
    segs = sorted(_prefixed(solids, prefix), key=lambda item: item["ymin"])
    gaps = []
    for low, high in zip(segs, segs[1:]):
        gap = high["ymin"] - low["ymax"]
        if gap > 0.05:
            gaps.append(gap)
    return gaps


def _measured(solids: dict) -> dict[str, float]:
    return {
        "A_living_width": _agg(solids, "APT_WALL_EAST", "xmin", "min")
        - _agg(solids, "APT_WALL_SHARED", "xmax", "max"),
        "B_living_kitchen_length": _agg(solids, "APT_WALL_NORTH", "ymin", "min")
        - _agg(solids, "APT_WALL_SOUTH", "ymax", "max"),
        "C_bedroom_width": _agg(solids, "APT_WALL_SHARED", "xmin", "min")
        - _agg(solids, "APT_WALL_WEST", "xmax", "max"),
        "D_bedroom_length": _agg(solids, "APT_WALL_NORTH", "ymin", "min")
        - _agg(solids, "APT_WALL_BED_SOUTH", "ymax", "max"),
        # E is the entry hall: left-block east face to the fridge, open to the kitchen.
        "E_hallway_width": solids["APT_FURN_FRIDGE"]["xmin"]
        - _agg(solids, "APT_WALL_SHARED", "xmax", "max"),
        "F_kitchen_aisle": solids["APT_FURN_ISLAND"]["ymin"] - solids["APT_FURN_COUNTER"]["ymax"],
    }


def _door_widths(solids: dict) -> dict[str, float]:
    bed = _gaps_x(solids, "APT_WALL_BED_SOUTH")
    assert len(bed) == 1, bed
    # Shared wall: lower opening is the west hallway, upper opening is door G.
    shared = _gaps_y(solids, "APT_WALL_SHARED")
    assert len(shared) == 2, shared
    shared_segs = sorted(_prefixed(solids, "APT_WALL_SHARED"), key=lambda item: item["ymin"])
    g_ymin = shared_segs[1]["ymax"]
    bed_ymax = _agg(solids, "APT_WALL_BED_SOUTH", "ymax", "max")
    assert g_ymin > shared_segs[0]["ymax"]
    assert 0.0 < g_ymin - bed_ymax <= 0.16
    assert shared[0] + 1e-4 >= 0.8128
    faces = {(round(seg["xmin"], 4), round(seg["xmax"], 4)) for seg in shared_segs}
    assert faces == {(round(shared_segs[0]["xmin"], 4), round(shared_segs[0]["xmax"], 4))}
    # Walkway is the left end of the bedroom's inner wall, directly above the bath.
    bed_segs = sorted(_prefixed(solids, "APT_WALL_BED_SOUTH"), key=lambda item: item["xmin"])
    walk_center = (bed_segs[0]["xmax"] + bed_segs[1]["xmin"]) / 2.0
    west = _agg(solids, "APT_WALL_WEST", "xmax", "max")
    bedroom_width = _agg(solids, "APT_WALL_SHARED", "xmin", "min") - west
    assert walk_center < west + 0.35 * bedroom_width
    # Door H is the lower opening in the bath partition, into the west hallway.
    # The upper opening is the walk-in door.
    bath = _gaps_y(solids, "APT_WALL_BATH_EAST")
    assert len(bath) == 2, bath
    assert bath[1] + 1e-4 >= 0.8128
    laundry = _gaps_x(solids, "APT_WALL_LAUNDRY")
    assert len(laundry) == 1 and laundry[0] + 1e-4 >= 0.9144
    south = _gaps_x(solids, "APT_WALL_SOUTH")
    north = _gaps_x(solids, "APT_WALL_NORTH")
    assert len(south) == 1 and len(north) == 1
    # Closets stay inside the left block. Nothing from that block stands in the main room.
    banned = ("APT_WALL_PANTRY", "APT_WALL_STORAGE", "APT_WALL_HALL_WEST", "APT_DOOR_PANTRY", "APT_DOOR_STORAGE")
    assert not any(name.startswith(banned) for name in solids)
    fridge = solids["APT_FURN_FRIDGE"]
    island = solids["APT_FURN_ISLAND"]
    counter = solids["APT_FURN_COUNTER"]
    east = _agg(solids, "APT_WALL_EAST", "xmin", "min")
    assert abs(island["xmin"] - fridge["xmin"]) <= 0.02
    assert 0.05 < east - island["xmax"] <= 0.30
    assert abs(counter["xmax"] - east) <= 0.02
    assert abs(counter["ymin"] - fridge["ymin"]) <= 0.02
    return {
        "H_bathroom_door": bath[0],
        "G_bedroom_door": shared[1],
        "I_front_door": south[0],
        "J_patio_door": north[0],
        "bedroom_bath_walkway": bed[0],
    }


def _contact_fds(text: str) -> dict[tuple[str, str], float]:
    raw = _strip_comments(text)
    pairs: dict[tuple[str, str], float] = {}
    for match in re.finditer(r"\bContactProperties\s*\{", raw):
        body = _brace_body(raw, match.end() - 1)
        mats = re.findall(r'material[12]\s+"([^"]+)"', body)
        slip = re.search(r"forceDependentSlip\s*\[\s*([-+0-9.eE]+)", body)
        if len(mats) == 2 and slip:
            pairs[(mats[0], mats[1])] = float(slip.group(1))
    return pairs


def _interior_bounds(solids: dict) -> tuple[float, float, float, float]:
    return (
        _agg(solids, "APT_WALL_WEST", "xmax", "max"),
        _agg(solids, "APT_WALL_SOUTH", "ymax", "max"),
        _agg(solids, "APT_WALL_EAST", "xmin", "min"),
        _agg(solids, "APT_WALL_NORTH", "ymin", "min"),
    )


def _obstacle_rects(solids: dict) -> list[tuple[float, float, float, float]]:
    rects = []
    for name, solid in solids.items():
        if name.startswith("APT_FLOOR_"):
            continue
        rects.append((solid["xmin"], solid["ymin"], solid["xmax"], solid["ymax"]))
    return rects


def _sample_points(solids: dict, spawn: tuple[float, float]) -> dict[str, tuple[float, float]]:
    island = solids["APT_FURN_ISLAND"]
    counter = solids["APT_FURN_COUNTER"]
    hall = solids["APT_FLOOR_HALL"]
    closet = solids["APT_FLOOR_CLOSET"]
    laundry = solids["APT_FLOOR_LAUNDRY"]
    bed_wall = sorted(_prefixed(solids, "APT_WALL_BED_SOUTH"), key=lambda item: item["xmin"])
    shared = sorted(_prefixed(solids, "APT_WALL_SHARED"), key=lambda item: item["ymin"])
    south = sorted(_prefixed(solids, "APT_WALL_SOUTH"), key=lambda item: item["xmin"])
    walk_x = (bed_wall[0]["xmax"] + bed_wall[1]["xmin"]) / 2.0
    # Door G is the upper shared-wall opening. The lower one is the west hallway.
    g_y = None
    best = -1.0
    for low, high in zip(shared, shared[1:]):
        if high["ymin"] - low["ymax"] > 0.05 and low["ymax"] > best:
            best = low["ymax"]
            g_y = (low["ymax"] + high["ymin"]) / 2.0
    assert g_y is not None
    return {
        "main_room": spawn,
        "entry": ((south[0]["xmax"] + south[1]["xmin"]) / 2.0, south[0]["ymax"] + 0.45),
        "kitchen_aisle": ((island["xmin"] + island["xmax"]) / 2.0, (counter["ymax"] + island["ymin"]) / 2.0),
        "hallway": ((hall["xmin"] + hall["xmax"]) / 2.0, (hall["ymin"] + hall["ymax"]) / 2.0),
        "bedroom": (shared[0]["xmin"] - 0.40, g_y),
        "bathroom": (walk_x, bed_wall[0]["ymin"] - 0.45),
        "walk_in": ((closet["xmin"] + closet["xmax"]) / 2.0, (closet["ymin"] + closet["ymax"]) / 2.0),
        "laundry": ((laundry["xmin"] + laundry["xmax"]) / 2.0, (laundry["ymin"] + laundry["ymax"]) / 2.0),
    }


def _path_connects(solids: dict, points: dict[str, tuple[float, float]], radius: float) -> None:
    x0, y0, x1, y1 = _interior_bounds(solids)
    step = 0.02
    nx = int(math.ceil((x1 - x0) / step)) + 1
    ny = int(math.ceil((y1 - y0) / step)) + 1
    blocked = bytearray(nx * ny)
    radius2 = radius * radius
    for rect in _obstacle_rects(solids):
        a, b, c, d = rect
        ix0 = max(0, int((a - radius - x0) / step) - 1)
        ix1 = min(nx - 1, int(math.ceil((c + radius - x0) / step)) + 1)
        iy0 = max(0, int((b - radius - y0) / step) - 1)
        iy1 = min(ny - 1, int(math.ceil((d + radius - y0) / step)) + 1)
        for iy in range(iy0, iy1 + 1):
            py = y0 + iy * step
            row = iy * nx
            for ix in range(ix0, ix1 + 1):
                px = x0 + ix * step
                dx = max(a - px, 0.0, px - c)
                dy = max(b - py, 0.0, py - d)
                if dx * dx + dy * dy < radius2:
                    blocked[row + ix] = 1

    def cell(px: float, py: float) -> tuple[int, int]:
        ix = min(nx - 1, max(0, int(round((px - x0) / step))))
        iy = min(ny - 1, max(0, int(round((py - y0) / step))))
        return ix, iy

    named = {name: cell(px, py) for name, (px, py) in points.items()}
    for name, (ix, iy) in named.items():
        assert not blocked[iy * nx + ix], f"{name} has less than the required clearance at {points[name]}"

    start = named["main_room"]
    seen = bytearray(nx * ny)
    queue: deque[tuple[int, int]] = deque([start])
    seen[start[1] * nx + start[0]] = 1
    while queue:
        ix, iy = queue.popleft()
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            jx, jy = ix + dx, iy + dy
            if jx < 0 or jy < 0 or jx >= nx or jy >= ny:
                continue
            flat = jy * nx + jx
            if seen[flat] or blocked[flat]:
                continue
            seen[flat] = 1
            queue.append((jx, jy))
    missing = [name for name, (ix, iy) in named.items() if not seen[iy * nx + ix]]
    assert not missing, f"no {radius * 2:.3f} m path from the main room to {missing}"


def _loaded_from(seeds: list[Path]) -> list[Path]:
    """Python files reachable from ``seeds`` by import or a path join into ``src``."""
    seen: set[Path] = set()
    queue = list(seeds)
    while queue:
        path = queue.pop()
        path = path.resolve()
        if path in seen or not path.is_file():
            continue
        if "controllers" not in path.parts and "src" not in path.parts:
            continue
        seen.add(path)
        text = path.read_text(encoding="utf-8")
        try:
            tree = ast.parse(text)
        except SyntaxError:
            tree = None
        if tree is not None:
            for node in ast.walk(tree):
                module = None
                if isinstance(node, ast.ImportFrom):
                    module = node.module or ""
                elif isinstance(node, ast.Import):
                    module = node.names[0].name if node.names else ""
                if not module or not (module == "src" or module.startswith("src.")):
                    continue
                rel = Path(*module.split("."))
                for cand in (ROOT / rel.with_suffix(".py"), ROOT / rel / "__init__.py"):
                    if cand.is_file():
                        queue.append(cand)
        for call in re.finditer(r"os\.path\.join\((.*?)\)", text, re.S):
            parts = re.findall(r'["\']([^"\']+)["\']', call.group(1))
            if not parts or parts[0] != "src":
                continue
            cand = ROOT.joinpath(*parts)
            if cand.is_file():
                queue.append(cand)
    return sorted(seen)


def _loaded_by_controller() -> list[Path]:
    """Controller package plus every module those files import or load by path."""
    seeds = [path for path in CONTROLLER_ROOT.rglob("*.py") if path.is_file()]
    return _loaded_from(seeds)


# Tokens that would hand the mapper the apartment or the simulator pose.
_MAPPER_BANNED = (
    "apartment_ground_truth",
    "apartment.wbt",
    "worlds/apartment",
    "getPosition",
    "getFromDef",
    "getSelf",
    "getRoot",
    "getOrientation",
    "Supervisor",
    "supervisor",
    "gps",
)


@pytest.fixture(scope="module")
def world_text() -> str:
    return WORLD.read_text(encoding="utf-8")


@pytest.fixture(scope="module")
def solids(world_text: str) -> dict:
    return _parse_solids(world_text)


def test_world_is_r2025a_and_does_not_steer_a_track(world_text: str):
    assert world_text.startswith("#VRML_SIM R2025a utf8\n")
    assert 'EXTERNPROTO "../protos/ButlerBot.proto"' in world_text
    assert "\n# STEER_TRACK off\n" in world_text
    assert "\n# TRACK_FILE " not in world_text
    from src.track_geometry import steer_track_enabled, track_file_from_world

    assert steer_track_enabled(str(WORLD), env={}) is False
    assert track_file_from_world(str(WORLD)) is None
    assert GT_NAME not in world_text


def test_measured_rooms_match_the_tape_within_one_centimetre(solids: dict):
    gt = _ground_truth()
    tol = float(gt["tolerance_m"])
    got = _measured(solids)
    for key, letter in _ROOM_LETTERS.items():
        expect = _inches(gt, key)
        assert abs(got[key] - expect) <= tol, f"{letter} {got[key]:.4f} m vs {expect:.4f} m"


def test_doorways_are_at_least_the_measured_widths(solids: dict):
    gt = _ground_truth()
    got = _door_widths(solids)
    for key, letter in _DOOR_LETTERS.items():
        expect = _inches(gt, key)
        assert got[key] + 1e-4 >= expect, f"{letter} opening {got[key]:.4f} m is under {expect:.4f} m"
    walk = float(gt["estimates"]["bedroom_bath_walkway_in"]) * float(gt["inch_to_metre"])
    assert got["bedroom_bath_walkway"] + 1e-4 >= walk


def test_floor_zones_and_carpet_step(solids: dict):
    gt = _ground_truth()
    tol = float(gt["tolerance_m"])
    step = float(gt["estimates"]["carpet_step_in"]) * float(gt["inch_to_metre"])
    floor_def = {
        "living": "APT_FLOOR_LIVING",
        "kitchen": "APT_FLOOR_KITCHEN",
        "hallway": "APT_FLOOR_HALL",
        "bathroom": "APT_FLOOR_BATH",
        "closet": "APT_FLOOR_CLOSET",
        "laundry": "APT_FLOOR_LAUNDRY",
        "pantry": "APT_FLOOR_PANTRY",
        "storage": "APT_FLOOR_STORAGE",
        "bedroom": "APT_FLOOR_BEDROOM",
    }
    for zone, box in gt["zones_m"].items():
        solid = solids[floor_def[zone]]
        xmin, ymin, _zmin = box["min_m"]
        xmax, ymax, zmax = box["max_m"]
        assert abs(solid["xmin"] - xmin) <= tol
        assert abs(solid["ymin"] - ymin) <= tol
        assert abs(solid["xmax"] - xmax) <= tol
        assert abs(solid["ymax"] - ymax) <= tol
        assert abs(solid["zmax"] - zmax) <= tol
    lip = solids["APT_FLOOR_BEDROOM"]["zmax"] - solids["APT_FLOOR_HALL"]["zmax"]
    assert abs(lip - step) <= tol
    assert solids["APT_FLOOR_BEDROOM"]["material"] == "carpet"
    for name in ("APT_FLOOR_LIVING", "APT_FLOOR_KITCHEN", "APT_FLOOR_HALL", "APT_FLOOR_BATH"):
        assert solids[name]["material"] == "laminate"


def test_carpet_slip_is_higher_and_energy_is_not_wired(world_text: str):
    gt = _ground_truth()
    pairs = _contact_fds(world_text)
    laminate = pairs[("drive_wheel", "laminate")]
    carpet = pairs[("drive_wheel", "carpet")]
    assert carpet > laminate
    assert laminate == pytest.approx(gt["floors"]["contact_force_dependent_slip"]["drive_wheel_laminate"])
    assert carpet == pytest.approx(gt["floors"]["contact_force_dependent_slip"]["drive_wheel_carpet"])
    assert gt["floors"]["energy_model_wired"] is False
    energy = (ROOT / "src" / "twin_energy_cruise.py").read_text(encoding="utf-8")
    assert GT_NAME not in energy
    assert "apartment_ground_truth" not in energy


def test_furniture_boxes_match_scoring_ground_truth(solids: dict):
    gt = _ground_truth()
    tol = float(gt["tolerance_m"])
    for item in gt["furniture"]:
        assert item["estimated"] is True
        solid = solids["APT_FURN_" + item["id"].upper()]
        xmin, ymin, zmin = item["aabb_m"]["min_m"]
        xmax, ymax, zmax = item["aabb_m"]["max_m"]
        assert abs(solid["xmin"] - xmin) <= tol
        assert abs(solid["ymin"] - ymin) <= tol
        assert abs(solid["zmin"] - zmin) <= tol
        assert abs(solid["xmax"] - xmax) <= tol
        assert abs(solid["ymax"] - ymax) <= tol
        assert abs(solid["zmax"] - zmax) <= tol
    assert "APT_FURN_DINING" not in solids
    assert "APT_FURN_RUG" not in solids
    assert world_text_pendants() == 0
    # Desk group stays flush to the far wall. The chair may pass the desk face by a few inches.
    east = _agg(solids, "APT_WALL_EAST", "xmin", "min")
    desk = solids["APT_FURN_DESK"]
    chair = solids["APT_FURN_OFFICE_CHAIR"]
    assert abs(desk["xmax"] - east) <= 0.01
    assert chair["xmin"] >= desk["xmin"] - 0.22
    for name in ("APT_FURN_TV_STAND", "APT_FURN_TRASH"):
        assert solids[name]["xmin"] >= desk["xmin"] - 0.01
        assert abs(solids[name]["xmax"] - east) <= 0.01
    assert solids["APT_FURN_ISLAND_CHAIR"]["ymin"] >= solids["APT_FURN_ISLAND"]["ymax"] - 0.01


def world_text_pendants() -> int:
    text = _strip_comments(WORLD.read_text(encoding="utf-8"))
    return len(re.findall(r"\bDEF\s+APT_PENDANT_\d+\s+Transform\b", text))


def test_spawn_is_in_the_main_room_on_a_24_inch_path(world_text: str, solids: dict):
    gt = _ground_truth()
    match = re.search(
        r"ButlerBot\s*\{.*?translation\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)",
        _strip_comments(world_text),
        re.S,
    )
    assert match
    spawn = (float(match.group(1)), float(match.group(2)))
    sx, sy, _sz = gt["spawn"]["translation_m"]
    assert abs(spawn[0] - sx) <= 0.01 and abs(spawn[1] - sy) <= 0.01
    x0, y0, x1, y1 = _interior_bounds(solids)
    assert x0 < spawn[0] < x1 and y0 < spawn[1] < y1
    # Main room is the right-hand block, not the bedroom.
    assert spawn[0] > _agg(solids, "APT_WALL_SHARED", "xmax", "max")
    # One connected 24 in path covers every requested walk:
    # front door -> entry hall -> living room; entry hall -> bedroom via G;
    # entry hall -> west hallway -> bath; bath -> bedroom via the walkway;
    # hallway -> laundry; plus the walk-in and the kitchen aisle.
    radius = float(gt["drivable_path_in"]) * float(gt["inch_to_metre"]) / 2.0
    _path_connects(solids, _sample_points(solids, spawn), radius)


def test_controller_does_not_import_or_open_apartment_ground_truth():
    """No foresight: controller code must not name the scoring file."""
    assert GROUND_TRUTH.is_file()
    loaded = _loaded_by_controller()
    assert loaded, "controller package was not found"
    hits = []
    for path in loaded:
        text = path.read_text(encoding="utf-8")
        if GT_NAME in text or "apartment_ground_truth" in text:
            hits.append(str(path.relative_to(ROOT)))
    assert hits == []
    # The controller entry is in that set, so an import of the eval file would have shown up.
    assert any(path.name == "butlerbot_controller.py" for path in loaded)
    assert any(path.name == "apartment_mapper.py" for path in loaded)


def test_mapper_has_no_pose_or_map_foresight():
    """The explorer and everything it imports stay off the scoring files and the true pose.

    The pose logger is a different controller. It may call getPosition. The
    mapper's import closure must not include it.
    """
    mapper = CONTROLLER_ROOT / "apartment_mapper"
    seeds = [path for path in mapper.rglob("*.py") if path.is_file()]
    assert seeds, "apartment_mapper controller is missing"
    loaded = _loaded_from(seeds)
    names = {path.name for path in loaded}
    assert "apartment_mapper.py" in names
    assert "apartment_pose_logger.py" not in names
    hits = []
    for path in loaded:
        text = path.read_text(encoding="utf-8")
        for token in _MAPPER_BANNED:
            if token in text or token in text.lower():
                hits.append(f"{path.relative_to(ROOT)}: {token}")
    assert hits == []


def test_pose_logger_is_separate_and_reads_the_simulator_pose():
    logger = CONTROLLER_ROOT / "apartment_pose_logger" / "apartment_pose_logger.py"
    text = logger.read_text(encoding="utf-8")
    assert "getPosition" in text
    assert "getFromDef" in text
    assert GT_NAME not in text
    assert "apartment.wbt" not in text
    mapper = (CONTROLLER_ROOT / "apartment_mapper" / "apartment_mapper.py").read_text(encoding="utf-8")
    assert "apartment_pose_logger" not in mapper
