"""Intersections: track networks, junction classifier, route policy, GAP_CROSS."""
import importlib.util
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

from src.junction_vision import JunctionTracker, classify_junction  # noqa: E402
from src.lane_vision import (  # noqa: E402
    LaneTracker,
    corner_cues,
    lane_polyline_robot,
    render_lane_bgra,
    trace_lines,
    yellow_mask,
)
from src.route_policy import RoutePolicy, default_choice, parse_route  # noqa: E402
from src.track_geometry import TrackGeometry, TrackNetwork, load_track, referee_for  # noqa: E402

_spec = importlib.util.spec_from_file_location("rowfit_sim", ROOT / "scripts" / "rowfit_sim.py")
sim = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sim)
CAMS = sim.sim_cameras()
NEW = ["plus", "t_end", "t_left", "t_right", "gap15", "gap25", "gap35", "gap45"]


# ---------------------------------------------------------------- networks
def test_old_tracks_stay_single_roads_and_new_ones_are_networks():
    for n in ("corner90", "corner_mix", "widen", "corner90_right", "corner90_wide"):
        assert isinstance(load_track(n), TrackGeometry)
    for n in NEW:
        assert isinstance(load_track(n), TrackNetwork)


def test_plus_lines_have_gaps_exactly_where_the_cross_road_joins():
    g = load_track("plus")
    pieces = g.paint_lines()
    assert len(pieces) == 8
    main = sorted((p for p in pieces if abs(p[0][1] - p[-1][1]) < 1e-9), key=lambda p: (p[0][1], p[0][0]))
    assert len(main) == 4
    for p in main:  # main road lines stop at the cross road's lines (x 3.35 / 4.65 +/- stripe/2)
        xs = sorted((p[0][0], p[-1][0]))
        assert xs[1] == pytest.approx(3.38, abs=0.011) or xs[0] == pytest.approx(4.62, abs=0.011)
    cross = [p for p in pieces if p not in main]
    for p in cross:
        ys = sorted((p[0][1], p[-1][1]))
        assert ys[1] == pytest.approx(-0.62, abs=0.011) or ys[0] == pytest.approx(0.62, abs=0.011)
    assert [e.name for e in g.exits] == ["east", "south", "north"]
    assert g.exit_crossed(8.05, 0.1) == "east" and g.exit_crossed(4.1, 4.05) == "north"
    assert g.exit_crossed(4.0, 0.0) is None


def test_t_end_has_no_straight_exit_and_gap_series_widths():
    t = load_track("t_end")
    assert [e.name for e in t.exits] == ["south", "north"]
    assert t.routes["default"] == "north"  # straight impossible -> left
    for gap in (1.5, 2.5, 3.5, 4.5):
        g = load_track(f"gap{int(gap * 10)}")
        ends = sorted(round(p[-1][0], 2) for p in g.paint_lines() if p[0][1] == pytest.approx(0.65))
        starts = sorted(round(p[0][0], 2) for p in g.paint_lines() if p[0][1] == pytest.approx(0.65))
        assert starts[1] - ends[0] == pytest.approx(gap - 0.06, abs=0.03)  # line-to-line gap = road width


def test_referee_reports_the_exit():
    r = referee_for(None, env={"RBM_TRACK": "plus"})
    assert r.crossed(4.0, 4.1) and r.exit_name(4.0, 4.1) == "north"
    assert r.exit_name(8.1, 0.0) == "east" and not r.crossed(5.0, 0.0)
    assert "exits east" in r.describe()


# ---------------------------------------------------------------- classifier
def _view(name, x, y=0.0, yaw=0.0, bars=False):
    geom = sim.track_geometry(name)
    if geom is None:
        centre = sim.course(name)
        from src.lane_vision import offset_polyline

        polys, marks = [offset_polyline(centre, 0.65), offset_polyline(centre, -0.65)], []
    else:
        _c, left, right, barl = sim.track_course(geom)
        polys = (list(left) + list(right)) if (left and isinstance(left[0], list)) else [left, right]
        marks = barl if bars else []
    lines = [lane_polyline_robot([p for p in poly if math.hypot(p[0] - x, p[1] - y) < 3.5], (x, y), yaw)
             for poly in polys]
    lines = [ln for ln in lines if len(ln) > 1]
    mk = [(lane_polyline_robot(b, (x, y), yaw), w, c) for b, w, c in marks]
    return {n: (render_lane_bgra(c, lines, marks=mk), c.width, c.height) for n, c in CAMS.items()}


def _classify(imgs, lane_w=1.30):
    per = []
    for n, c in CAMS.items():
        m = yellow_mask(imgs[n][0], c.width, c.height)
        tr = trace_lines(c, m, c.width, c.height)
        per.append((c, tr, corner_cues(c, m, c.width, c.height, traced=tr)))
    return classify_junction(per, lane_w)


@pytest.mark.parametrize("name,x,kind,openings", [
    ("plus", 2.0, "plus", "L S? R"),  # far side beyond the camera reach: straight unknown
    ("plus", 2.9, "plus", "L S R"),  # resumed lines / far pieces seen
    ("t_end", 2.9, "t_end", "L S- R".replace("S-", "-")),
    ("t_left", 3.0, "side_L", "L S -"),
    ("t_right", 3.0, "side_R", "- S R"),
    ("gap45", 3.0, "plus", "L S? R"),  # Dan's case: plaza wider than the look-ahead
])
def test_junction_kinds(name, x, kind, openings):
    j = _classify(_view(name, x))
    assert j is not None and j.kind == kind and j.openings() == openings, j


def test_junction_centre_is_the_crossing_road_centre():
    j = _classify(_view("plus", 2.9))
    assert j.center_m == pytest.approx(4.0 - 2.9, abs=0.08)  # far side seen: measured crossing width
    j = _classify(_view("plus", 2.0))
    assert j.center_m == pytest.approx(2.0, abs=0.08)  # far side unseen: own lane width
    j = _classify(_view("t_left", 3.0))
    assert j.center_m == pytest.approx(1.0, abs=0.08)


@pytest.mark.parametrize("name,x0,x1", [("corner90", 0.0, 3.6), ("corner_mix", 0.0, 3.0), ("widen", 0.0, 9.0),
                                         ("corner90_wide", 0.0, 3.6)])
def test_no_turn_junction_on_old_courses_or_bars(name, x0, x1):
    g = sim.track_geometry(name)
    x = x0
    while x <= x1:
        pr_xy = g.dense_centerline(0.04)
        # walk the centre line by arc length
        acc, px, py, yaw = 0.0, *pr_xy[0], 0.0
        for (ax, ay), (bx, by) in zip(pr_xy, pr_xy[1:]):
            d = math.hypot(bx - ax, by - ay)
            if acc + d >= x:
                f = (x - acc) / d if d else 0.0
                px, py, yaw = ax + f * (bx - ax), ay + f * (by - ay), math.atan2(by - ay, bx - ax)
                break
            acc += d
        j = _classify(_view(name, px, py, yaw, bars=True), g.width_at_s(x))
        assert j is None or j.kind.startswith(("corner", "opening")), (name, x, j)
        x += 0.25


def test_persistence_needs_two_consistent_frames():
    jt = JunctionTracker()
    a = _classify(_view("plus", 2.0))
    assert not jt.update(a, 0.0).seen
    b = _classify(_view("plus", 2.1))
    assert jt.update(b, 0.1).seen and b.frames == 2
    assert jt.update(None, 0.1) is None
    c = _classify(_view("plus", 2.2))
    assert not jt.update(c, 0.1).seen  # a gap in the sequence restarts the count


def test_tracker_clips_the_fit_before_the_crossing():
    tr = LaneTracker(CAMS)
    est = tr.process(_view("plus", 2.6))
    est = tr.process(_view("plus", 2.7), dx_m=0.1)
    assert est.junction is not None and est.junction.seen
    assert est.valid and abs(est.heading_rad) < math.radians(2) and abs(est.offset_m) < 0.03
    assert est.view["left"] is not None and est.view["right"] is not None


# ---------------------------------------------------------------- policy
def test_route_parsing_and_policy():
    assert parse_route("S,L,R") == (["S", "L", "R"], [])
    assert parse_route(" left; right straight ") == (["L", "R", "S"], [])
    r, warn = parse_route("S,X")
    assert r == ["S"] and "X" in warn[0]
    assert default_choice(True, None, True) == "S"  # unknown far side: try straight
    assert default_choice(True, False, True) == "L"
    assert default_choice(False, False, True) == "R"
    assert default_choice(False, False, False) is None
    p = RoutePolicy("R,S")
    assert p.choose(True, True, True)[0] == "R"
    ch, note = p.choose(True, False, True)  # S wanted at a T-end: falls back to the default (left)
    assert ch == "L" and "not offered" in note
    assert p.choose(True, True, False)[0] == "S"  # list used up: default
    q = RoutePolicy("L")
    assert q.choose(False, None, False) == ("S", "only way S")  # a gap is not a choice ...
    assert q.choose(True, True, True)[0] == "L" and q.i == 1  # ... so the entry is kept for the junction


def test_runtime_reads_route_env(monkeypatch, tmp_path):
    ctrl = ROOT / "webots" / "controllers" / "butlerbot_controller"
    monkeypatch.syspath_prepend(str(ctrl))
    for m in ("twin_publisher", "controller_rowfit"):
        sys.modules.pop(m, None)
    import twin_publisher

    monkeypatch.setattr(twin_publisher, "_PROJECT_ENV_FILE", str(tmp_path / "none.env"))
    import controller_rowfit

    monkeypatch.setenv("RBM_ROUTE", "L,R")
    monkeypatch.setenv("RBM_MAX_BLIND_M", "2.5")
    rt = controller_rowfit.RowfitRuntime()
    assert rt.ctl.policy.route == ["L", "R"] and rt.ctl.max_blind_m == 2.5
    assert "RBM_ROUTE=L,R" in rt.route_line() and "2.50 m" in rt.route_line()
    monkeypatch.setenv("RBM_MAX_BLIND_M", "-1")
    monkeypatch.delenv("RBM_ROUTE")
    rt = controller_rowfit.RowfitRuntime()
    assert rt.ctl.max_blind_m == 4.0 and rt.route_warnings and rt.ctl.policy.route == []
    cmd = rt.command(0.008, yaw=0.0)
    for k in ("junction_type", "junction_m", "openings", "route_choice", "blind_m"):
        assert k in cmd


# ---------------------------------------------------------------- closed loop
@pytest.mark.parametrize("course,route,exit_", [
    ("plus", "L", "north"), ("t_end", "R", "south"), ("t_left", None, "east"), ("t_right", "R", "south"),
])
def test_closed_loop_routes(course, route, exit_):
    r = sim.run(course, route=route, max_s=45.0)
    assert r["ok"] and r["exit"] == exit_, (r["exit"], r["junctions"], r["braked_at"])
    assert r["max_lateral_m"] < 0.06
    assert all(abs(p["true_err_deg"]) < 3.0 for p in r["pivots"])


def test_closed_loop_large_gap_crossed_blind():
    r = sim.run("gap45", max_s=60.0)
    assert r["ok"] and r["exit"] == "east"
    j = r["junctions"][0]
    assert j["kind"] == "plus" and j["openings"] == "L S? R" and j["choice"] == "S"
    assert 3.0 < j["blind_m"] < 4.0 and r["max_lateral_m"] < 0.05
    assert any(e.startswith("GAP reacquire") for e in r["events"])


def test_gap_gives_up_after_max_blind_and_looks_around():
    r = sim.run("gap45", max_blind=2.0, max_s=60.0)
    assert not r["finished"] and r["final_state"] == "STOPPED"
    ev = r["events"]
    assert any(e.startswith("GAP no lane after 2.00 m") for e in ev)
    assert any(e.startswith("LOOK_AROUND done") for e in ev)
    assert r["braked_at"]["state"] == "LOOK_AROUND"
