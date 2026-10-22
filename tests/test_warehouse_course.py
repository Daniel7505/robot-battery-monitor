"""Default warehouse lap: three left pivots on the unset route, one perimeter."""

from __future__ import annotations

from pathlib import Path

from optimizer.warehouse_course import (
    DEFAULT_LAP_NAME,
    load_missions,
    parse_route_from_console,
    resolve_course,
    route_turn_count,
    track_relpath,
)
from src.route_planner import RoadGraph
from src.track_geometry import load_track

ROOT = Path(__file__).resolve().parents[1]
BACK_LOOP = "S,S,S,S,R,S,S,S,L,S,L"


def test_world_track_default_lap_is_three_lefts():
    rel = track_relpath(ROOT)
    assert rel == "tracks/warehouse.json"
    assert f"# TRACK_FILE {rel}" in (ROOT / "webots/worlds/butlerbot_warehouse.wbt").read_text(
        encoding="utf-8"
    )
    assert parse_route_from_console("ROUTE RBM_ROUTE unset -> default") is None
    course = resolve_course("")
    assert course is not None
    same = resolve_course("ROUTE RBM_ROUTE unset -> default")
    assert same is not None and same.name == course.name and same.turns == course.turns

    assert course.name == DEFAULT_LAP_NAME
    assert course.route == "unset"
    assert course.from_log is False
    assert course.turns == 3
    assert course.corners == 2
    assert course.length_m == 201.0
    assert course.stop_xy == (18.0, -6.0)
    assert [(t.direction, t.source, t.x, t.y, t.at_m) for t in course.lap_turns] == [
        ("left", "junction", 58.5, 0.0, 58.5),
        ("left", "tape", 58.5, 48.0, 106.5),
        ("left", "tape", 18.0, 48.0, 147.0),
    ]
    assert all(abs(t.x - 18.0) > 0.1 or abs(t.y - (-12.0)) > 0.1 for t in course.lap_turns)
    text = course.describe()
    assert text == "3 left (RBM_ROUTE unset, straight-else-left, stop at 201 m)"
    layout = course.layout()
    assert "58.5 m, left (junction) at (58.5, 0.0)" in layout
    assert "48.0 m, left (tape) at (58.5, 48.0)" in layout
    assert "40.5 m, left (tape) at (18.0, 48.0)" in layout
    assert "stop (18.0, -6.0)" in layout

    logged = resolve_course(f"ROUTE RBM_ROUTE={BACK_LOOP} -> receiving_2")
    assert logged is not None and logged.from_log
    assert logged.name == "back_loop_to_receiving"
    assert logged.route == BACK_LOOP
    assert logged.turns == 3
    assert logged.length_m == 148.5
    assert logged.layout() == ""
    assert "back_loop_to_receiving" in logged.describe()
    assert route_turn_count(logged.route) == 3

    net = load_track("warehouse")
    plan = RoadGraph(net).follow(BACK_LOOP)
    assert plan.ok and plan.exit == "receiving_2"
    missions = load_missions(ROOT)
    assert sum(1 for mission in missions if mission.get("name") == "back_loop_to_receiving") == 1
