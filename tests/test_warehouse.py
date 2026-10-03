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


def test_every_aisle_is_taped_and_joined_to_the_cross_aisles(built):
    """Every aisle has a lane, and each one meets the front, middle and back cross aisles in a T / plus."""
    lay, g = built["layout"], built["graph"]
    assert lay.taped == list(range(lay.n_aisles))
    xs = [lay.xF] + lay.xM + [lay.xB]
    nodes = {(round(n.xy[0], 2), round(n.xy[1], 2)): n for n in g.junctions}
    corners = {(round(p[0], 2), round(p[1], 2)) for _k, p, _a in g.corners()}
    for i, y in enumerate(lay.aisle_y):
        for x in xs:
            edge_corner = i in (0, lay.n_aisles - 1) and x in (lay.xF, lay.xB)  # perimeter corner, not a junction
            assert (round(x, 2), round(y, 2)) in (corners if edge_corner else nodes), (i, x, y)
    # the forklift crossing is still a tape gap on the start lane
    assert any(r.spec.name == "forklift_crossing" and r.spec.widths[0] > 2.0 for r in built["net"].roads)
    # a drive down an odd aisle (untaped before) reaches its exit
    d = json.loads(TRACK.read_text(encoding="utf-8"))
    odd = next(m for m in d["missions"] if m["name"] == "odd_aisle_to_shipping")
    ys = {round(v[1], 3) for v in odd["via"][:-1]}
    assert len(ys) == 1 and (lay.aisle_y.index(ys.pop()) - lay.sp.main_aisle) % 2 == 1
    assert g.follow(odd["route"]).exit == "shipping_1"


def test_viewpoint_is_a_fixed_top_down_overview(built):
    """Main 3D view: no follow, straight down, whole floor in frame, nothing above it."""
    lay = built["layout"]
    sp = lay.sp
    text = WORLD.read_text(encoding="utf-8")
    vp = re.search(r"^DEF VIEWPOINT Viewpoint \{\n(.*?)^\}", text, re.M | re.S).group(1)
    body = "\n".join(ln for ln in vp.splitlines() if not ln.strip().startswith("#"))
    assert re.search(r'follow ""', body) and 'followType "None"' in body
    assert "ButlerBot" not in body
    v = wb.overview_viewpoint(lay)
    ax, ay, az, ang = v["orientation"]
    assert (ax, ay, az) == (0.0, 1.0, 0.0) and ang == pytest.approx(math.pi / 2)  # straight down (ENU)
    x, y, z = v["position"]
    assert x == pytest.approx((lay.x0 + lay.x1) / 2) and y == pytest.approx((lay.y0 + lay.y1) / 2)
    tan_h = math.tan(v["fov"] / 2)
    for aspect in (1.0, 16 / 9, sp.overview_aspect):  # fov on the wider side; floor depth runs up the screen
        tan_v = tan_h / aspect
        assert z * tan_v >= sp.depth_m / 2 and z * tan_h >= sp.width_m / 2
    assert v["near"] < z - sp.wall_h_m and (v["far"] == 0 or v["far"] > z)
    # nothing (roof / ceiling) above the walls between the eye and the floor
    for b in lay.walls_and_doors() + lay.racks() + built["staging"]:
        assert b.z0 + b.h <= sp.wall_h_m + 1e-9
    assert not re.search(r"(?i)(ceiling|roof)\s+(Solid|Shape|Pose)", text)
