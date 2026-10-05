"""Multi-trip shopping list planner + state machine (no Webots)."""
from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

from src.inventory_pick import (  # noqa: E402
    ST_PICKING,
    ST_TO_PICK,
    ST_TO_SHIP,
    apply_pick_speed_cap,
    build_inventory_groups,
    demo_group_specs,
)
from src.shopping_list import (  # noqa: E402
    DEFAULT_TROLLEY_CAPACITY,
    ORDER_AISLE,
    ORDER_NEAREST,
    ST_DONE,
    ST_UNLOADING,
    MultiTripState,
    ShoppingLine,
    attach_trip_routes,
    demo_shopping_lines,
    plan_shopping_trips,
    plan_to_json,
    shopping_efficiency_metrics,
    trolley_capacity_units,
)
import warehouse_builder as wb  # noqa: E402


@pytest.fixture(scope="module")
def lay():
    return wb.Layout(wb.WarehouseSpec())


@pytest.fixture(scope="module")
def groups(lay):
    return build_inventory_groups(lay)


def test_capacity_derived_from_bed():
    assert DEFAULT_TROLLEY_CAPACITY == 2
    assert trolley_capacity_units() == 2


def test_demo_seeds_enough_skus_for_multi_trip(lay, groups):
    specs = demo_group_specs(lay)
    assert len(specs) >= 6
    assert sum(1 for s in specs if s.get("demo")) == 1
    assert len(groups) == len(specs)
    lines = demo_shopping_lines(groups)
    plan = plan_shopping_trips(lines, groups)
    assert plan.total_units == len(lines)
    assert 3 <= plan.n_trips <= 4
    assert plan.capacity == 2
    assert all(t.units <= plan.capacity for t in plan.trips)
    assert sum(t.units for t in plan.trips) == plan.total_units


def test_aisle_order_then_pack(groups):
    lines = [
        ShoppingLine(groups[2].sku, 1),
        ShoppingLine(groups[0].sku, 1),
        ShoppingLine(groups[1].sku, 1),
    ]
    plan = plan_shopping_trips(lines, groups, capacity=2, order=ORDER_AISLE)
    # stops within each trip follow aisle ascending
    flat = [s for t in plan.trips for s in t.stops]
    assert flat == sorted(flat, key=lambda s: (s.aisle, s.block, s.sku))
    assert plan.n_trips == 2  # 3 units / cap 2


def test_nearest_order(groups):
    lines = demo_shopping_lines(groups)[:4]
    start = (0.0, 0.0)
    plan = plan_shopping_trips(lines, groups, capacity=2, order=ORDER_NEAREST, start_xy=start)
    assert plan.order_mode == ORDER_NEAREST
    assert plan.n_trips == 2
    # first stop should be nearest to start among the four
    first = plan.trips[0].stops[0]
    from math import hypot

    dists = sorted(
        (
            hypot(g.approach_xy[0] - start[0], g.approach_xy[1] - start[1]),
            g.sku,
        )
        for g in groups
        if g.sku in {ln.sku for ln in lines}
    )
    assert first.sku == dists[0][1]


def test_unknown_sku_raises(groups):
    with pytest.raises(ValueError, match="not in inventory"):
        plan_shopping_trips([ShoppingLine("NOPE", 1)], groups)


def test_attach_routes(lay, groups):
    from src.route_planner import RoadGraph
    from src.track_geometry import TrackNetwork

    plan = plan_shopping_trips(demo_shopping_lines(groups), groups)
    net = TrackNetwork.from_dict(lay.track_dict(), "warehouse")
    g = RoadGraph(net)
    attach_trip_routes(plan, g)
    assert all(t.route and t.length_m and t.length_m > 0 for t in plan.trips)
    # follow each route to shipping
    for t in plan.trips:
        f = g.follow(t.route)
        assert f.exit == "shipping_1", (t.trip_index, f.exit, f.note)


def test_efficiency_stub(groups):
    plan = plan_shopping_trips(demo_shopping_lines(groups), groups)
    m = shopping_efficiency_metrics(plan)
    assert m["kind"] == "shopping_efficiency_stub"
    assert m["n_trips"] == plan.n_trips
    assert m["capacity"] == 2
    assert "note" in m


def test_multi_trip_state_machine_fill_ship_unload(groups):
    # tiny plan: 3 units → 2 trips at cap 2
    lines = [ShoppingLine(g.sku, 1) for g in groups[:3]]
    plan = plan_shopping_trips(lines, groups, capacity=2, order=ORDER_AISLE)
    assert plan.n_trips == 2
    mt = MultiTripState(plan=plan, pick_radius_m=1.0, pick_dwell_s=0.5, unload_dwell_s=0.3)

    def drive_pick(stop_xy):
        cmd = mt.update(stop_xy, 0.1)
        assert cmd.state in (ST_TO_PICK, ST_PICKING)
        # arrive
        for _ in range(30):
            cmd = mt.update(stop_xy, 0.1)
            if cmd.pick_now:
                break
        assert cmd.pick_now
        # finish dwell
        for _ in range(30):
            cmd = mt.update(stop_xy, 0.1)
            if cmd.state in (ST_TO_PICK, ST_TO_SHIP):
                break
        return cmd

    # trip 1: two stops
    t1 = plan.trips[0]
    for i, stop in enumerate(t1.stops):
        cmd = drive_pick(stop.approach_xy)
        if i < len(t1.stops) - 1:
            assert cmd.state == ST_TO_PICK
        else:
            assert cmd.state == ST_TO_SHIP and cmd.v_cap is None
    assert mt.load_units == 2
    # ship + unload
    cmd = mt.mark_shipped()
    assert cmd.state == ST_UNLOADING
    cleared = False
    for _ in range(20):
        cmd = mt.update((0.0, 0.0), 0.1)
        if mt.clear_load:
            cleared = True
        if cmd.state == ST_TO_PICK and mt.trip_i == 1:
            break
    assert cleared and mt.load_units == 0 and mt.trip_i == 1
    # trip 2: one stop then ship then done
    t2 = plan.trips[1]
    cmd = drive_pick(t2.stops[0].approach_xy)
    assert cmd.state == ST_TO_SHIP
    mt.mark_shipped()
    for _ in range(20):
        cmd = mt.update((0.0, 0.0), 0.1)
        if cmd.state == ST_DONE:
            break
    assert cmd.state == ST_DONE
    assert mt.units_picked_total == 3


def test_builder_emits_shopping_list_and_trip_missions():
    res = wb.build_all(write=False)
    shop = res["track"]["shopping_list"]
    assert shop["n_trips"] >= 3
    assert shop["capacity"] == 2
    names = {m["name"] for m in res["track"]["missions"]}
    for i in range(1, shop["n_trips"] + 1):
        assert f"shopping_list_trip_{i}" in names
    # each trip mission has a route
    for i in range(1, shop["n_trips"] + 1):
        m = next(x for x in res["track"]["missions"] if x["name"] == f"shopping_list_trip_{i}")
        assert m["route"] and m["to"] == "shipping_1"
    # inventory grew
    assert len(res["track"]["inventory"]) >= 6


def test_controller_shopping_env_resolve(monkeypatch):
    mod_path = ROOT / "webots" / "controllers" / "butlerbot_controller" / "controller_inventory.py"
    spec = importlib.util.spec_from_file_location("controller_inventory_shop", mod_path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    plan = {
        "trips": [{"trip_index": 1}, {"trip_index": 2}, {"trip_index": 3}, {"trip_index": 4}],
    }
    assert mod.shopping_list_from_env({mod.SHOP_ENV: ""}) is None
    assert mod.shopping_list_from_env({mod.SHOP_ENV: "demo"}) == "demo"
    i, why, run_all = mod.resolve_shopping_trip("demo", plan)
    assert i == 0 and not run_all and "trip 1" in why
    i, why, run_all = mod.resolve_shopping_trip("all", plan)
    assert i == 0 and run_all
    i, why, run_all = mod.resolve_shopping_trip("3", plan)
    assert i == 2 and not run_all
    i, why, run_all = mod.resolve_shopping_trip("9", plan)
    assert i is None


def test_plan_json_roundtrip(groups):
    plan = plan_shopping_trips(demo_shopping_lines(groups), groups)
    data = plan_to_json(plan)
    assert data["n_trips"] == plan.n_trips
    assert len(data["trips"]) == plan.n_trips
    assert data["efficiency"]["n_trips"] == plan.n_trips


def test_pick_cap_still_releases_under_multi(groups):
    lines = [ShoppingLine(groups[0].sku, 1)]
    plan = plan_shopping_trips(lines, groups, capacity=2)
    mt = MultiTripState(plan=plan, pick_radius_m=1.0, pick_dwell_s=0.4)
    ax, ay = plan.trips[0].stops[0].approach_xy
    mt.update((ax, ay), 0.1)
    cap, holding, saved = apply_pick_speed_cap(0.0, None, holding=False, saved_cap=None)
    for _ in range(20):
        cmd = mt.update((ax, ay), 0.1)
        if cmd.state == ST_TO_SHIP:
            break
    cap2, holding2, _ = apply_pick_speed_cap(cmd.v_cap, cap, holding=holding, saved_cap=saved)
    assert cmd.v_cap is None and cap2 is None and not holding2
