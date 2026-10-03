"""Prototype warehouse course: layout, generated files, road-graph planner, one closed-loop mission."""
import importlib.util
import json
import math
import re
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

from src.route_planner import RoadGraph  # noqa: E402
from src.track_geometry import TrackNetwork, _poly_dist, load_track, referee_for  # noqa: E402

import warehouse_builder as wb  # noqa: E402

_spec = importlib.util.spec_from_file_location("rowfit_sim", ROOT / "scripts" / "rowfit_sim.py")
sim = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sim)

WORLD = ROOT / "webots" / "worlds" / "butlerbot_warehouse.wbt"
TRACK = ROOT / "tracks" / "warehouse.json"


@pytest.fixture(scope="module")
def built():
    return wb.build_all(write=False)


def test_files_are_generator_output(built):
    """tracks/warehouse.json and the world are exactly what the builder writes (regenerate, don't hand-edit)."""
    d = json.loads(TRACK.read_text(encoding="utf-8"))
    assert d == json.loads(json.dumps(built["track"]))
    net = load_track("warehouse")
    text = WORLD.read_text(encoding="utf-8")
    assert text == wb.build_world(built["layout"], net, built["staging"])


def test_layout_matches_the_brief(built):
    lay = built["layout"]
    sp = lay.sp
    assert sp.width_m * sp.depth_m <= 4650.0  # ~50k sq ft or smaller
    assert 10 <= len(lay.rack_rows()) <= 20
    assert 3.5 <= sp.aisle_w_m <= 4.0 and 3.5 <= sp.cross_aisle_w_m <= 4.5
    assert len(lay.xM) >= 1  # middle cross aisle
    # aisles really are aisle_w wide between rack faces
    racks = lay.racks()
    for y in lay.aisle_y:
        faces = [b for b in racks if b.rect[1] < y < b.rect[3]]
        assert not faces
        right = max(b.rect[3] for b in racks if b.rect[3] <= y)
        left = min(b.rect[1] for b in racks if b.rect[1] >= y)
        assert left - right == pytest.approx(sp.aisle_w_m)
    for b in racks:
        assert b.h == pytest.approx(6.0)
        assert lay.x0 < b.rect[0] and b.rect[2] < lay.x1 and lay.y0 < b.rect[1] and b.rect[3] < lay.y1
    doors = [n for n, _r, _y in lay.doors]
    assert any(n.startswith("RCV") for n in doors) and any(n.startswith("SHP") for n in doors)
    tables = [b for b in built["staging"] if b.kind == "table"]
    pallets = [b for b in built["staging"] if b.kind == "pallet"]
    assert tables and pallets and all(0.8 <= b.h <= 1.5 for b in tables + pallets)


def test_nothing_solid_on_a_lane(built):
    net, lay = built["net"], built["layout"]
    items = lay.racks() + built["staging"]
    for b in items:
        x0, y0, x1, y1 = b.rect
        pts = [(x0 + (x1 - x0) * i / 6, y0 + (y1 - y0) * j / 6) for i in range(7) for j in range(7)]
        for g in net.roads:
            w = max(g.spec.widths) / 2
            assert min(_poly_dist(g.centerline, *p) for p in pts) > w + 0.5, (b.name, g.spec.name)
    # start just inside RCV-1, facing into the building; exits inside the dock wall
    assert net.start_xy == (0.0, 0.0) and net.start_heading == 0.0
    assert all(lay.x0 < e.xy[0] < lay.x0 + 2.0 for e in net.exits)


def test_track_uses_the_features_the_bot_handles(built):
    net, g = built["net"], built["graph"]
    assert isinstance(load_track("warehouse"), TrackNetwork)
    kinds = set()
    for n in g.junctions:
        for e in g.edges:
            if e.b == n.id:
                kinds.add(g._kind(g.options(n.id, e.h_in)))
    assert {"plus", "t_end", "side_L", "side_R"} <= kinds
    assert len(g.corners()) >= 4 and all(abs(abs(a) - 90) < 1e-6 for _k, _p, a in g.corners())
    plaza = next(r for r in net.roads if r.spec.name == "forklift_crossing")
    assert plaza.spec.widths[0] > 2.0  # the tape gap (GAP_CROSS)
    assert {e.name for e in net.exits} == {"receiving_2", "shipping_1", "shipping_2"}
    ws = {round(w, 2) for r in net.roads if r.spec.name != "forklift_crossing" for w in r.spec.widths}
    assert ws == {1.30}


def test_missions_route_strings_reach_their_exits(built):
    net, g = built["net"], built["graph"]
    d = json.loads(TRACK.read_text(encoding="utf-8"))
    assert len(d["missions"]) >= 2
    for m in d["missions"]:
        p = g.follow(m["route"])
        assert p.ok and p.exit == m["to"], (m["name"], p.note)
        assert sim.expected_exit(net, m["route"]) == m["to"]
        assert sim.expected_exit(net, m["route"].replace(",", ", ")) == m["to"]
    far = next(m for m in d["missions"] if m["name"] == "far_aisle_to_shipping")
    assert far["length_m"] > 100 and far["junctions"] >= 8
    assert d["routes"]["default"] is None  # default straight-else-left circles the perimeter


@pytest.mark.parametrize("course,exit_,route", [
    ("plus", "north", "L"), ("plus", "south", "R"), ("plus", "east", "S"), ("t_end", "south", "R"),
    ("t_left", "north", "L"), ("t_right", "south", "R"), ("gap45", "east", "S"),
])
def test_planner_agrees_with_the_hand_written_routes(course, exit_, route):
    g = RoadGraph(load_track(course))
    p = g.plan(exit_)
    assert p.ok and p.route == route and p.exit == exit_
    assert g.follow(route).exit == exit_


def test_planner_no_path():
    p = RoadGraph(load_track("t_end")).plan("east")
    assert not p.ok and "no path" in p.note


def test_world_shape(built):
    text = WORLD.read_text(encoding="utf-8")
    base = (ROOT / "webots" / "worlds" / "butlerbot.wbt").read_text(encoding="utf-8")
    assert text.split("\nRobot {", 1)[1] == base.split("\nRobot {", 1)[1]  # robot / spawn byte-identical
    assert text.count("{") == text.count("}") and text.count("[") == text.count("]")
    assert re.findall(r"^DEF TRACK Pose \{", text, re.M) == ["DEF TRACK Pose {"]
    assert not re.search(r"^(Transform|Pose) \{", text, re.M)
    assert "# TRACK_FILE tracks/warehouse.json" in text and '"TRACK (ENU): warehouse ' in text
    assert "castShadows TRUE" not in text
    wh = text.split(wb.WH_BEGIN, 1)[1].split(wb.WH_END, 1)[0]
    solids = re.findall(r"DEF (WH_\w+) Solid \{", wh)
    lay = built["layout"]
    assert len(solids) == len(lay.racks()) + len(lay.walls_and_doors()) + len(built["staging"])
    assert len(set(solids)) == len(solids)
    assert wh.count("boundingObject Box") == len(solids)
    names = re.findall(r'^\s+name "([^"]+)"', wh, re.M)
    assert len(names) == len(set(names)) == len(solids)
    assert "DEF WH_OBSTACLES Group" in wh  # placeholder, empty
    for m in re.finditer(r"baseColor ([\d.]+) ([\d.]+) ([\d.]+)", wh):
        r, g_, b = map(float, m.groups())
        assert min(r, g_) - b < 0.22  # nothing in the building passes the lane yellow test


def test_referee_from_the_world_tag():
    r = referee_for(str(WORLD), env={})
    assert r.name == "warehouse" and r.exit_name(-3.2, 36.0) == "shipping_1"
    assert r.exit_name(30.0, 0.0) is None


def test_closed_loop_dock_shuttle():
    d = json.loads(TRACK.read_text(encoding="utf-8"))
    m = next(m for m in d["missions"] if m["name"] == "dock_shuttle")
    r = sim.run("warehouse", route=m["route"], max_s=200.0)
    assert r["ok"] and r["exit"] == "shipping_2", (r["exit"], r["junctions"], r["braked_at"])
    assert r["max_lateral_m"] < 0.08
    assert [j["choice"] for j in r["junctions"]] == m["route"].split(",")
    assert all(abs(p["true_err_deg"]) < 3.0 for p in r["pivots"])
