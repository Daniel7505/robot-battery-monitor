"""Baseline warehouse lap: three route turns, from the world track."""

from __future__ import annotations

import math
from pathlib import Path

from optimizer.warehouse_course import (
    BASELINE_MISSION,
    load_missions,
    parse_route_from_console,
    resolve_course,
    route_turn_count,
    track_relpath,
)
from src.route_planner import RoadGraph
from src.track_geometry import load_track

ROOT = Path(__file__).resolve().parents[1]


def _geom_corners(net, edge) -> int:
    road = net.roads[edge.road]
    spec = road.spec
    lo, hi = min(edge.s0, edge.s1), max(edge.s0, edge.s1)
    arc = 0.0
    n = 0
    for j in range(1, len(spec.waypoints)):
        arc += math.dist(spec.waypoints[j - 1], spec.waypoints[j])
        if (
            j < len(spec.waypoints) - 1
            and lo + 1e-6 < arc < hi - 1e-6
            and abs(road._phi[j]) > math.radians(10)
        ):
            n += 1
    return n


def test_world_track_baseline_has_three_turns():
    rel = track_relpath(ROOT)
    assert rel == "tracks/warehouse.json"
    assert f"# TRACK_FILE {rel}" in (ROOT / "webots/worlds/butlerbot_warehouse.wbt").read_text(
        encoding="utf-8"
    )
    course = resolve_course("")
    assert course is not None
    assert course.name == BASELINE_MISSION
    assert course.turns == 3
    assert course.corners == 3
    assert course.route == "S,S,S,S,R,S,S,S,L,S,L"
    assert route_turn_count(course.route) == 3
    assert course.length_m == 148.5
    assert parse_route_from_console("ROUTE RBM_ROUTE unset -> default") is None
    logged = resolve_course(f"ROUTE RBM_ROUTE={course.route} -> receiving_2")
    assert logged is not None and logged.from_log and logged.turns == 3

    net = load_track("warehouse")
    graph = RoadGraph(net)
    plan = graph.follow(course.route)
    assert plan.ok and plan.exit == "receiving_2"
    lr = [d for d in plan.decisions if d.choice in {"L", "R"} and d.uses_entry]
    assert len(lr) == 3
    assert [(d.choice, round(d.xy[0], 2), round(d.xy[1], 2)) for d in lr] == [
        ("R", 58.5, 0.0),
        ("L", 18.0, 0.0),
        ("L", 4.0, 0.0),
    ]
    assert sum(_geom_corners(net, edge) for edge in plan.edges) == 3
    missions = load_missions(ROOT)
    assert sum(1 for m in missions if int(m["corners"]) == 3) == 1
