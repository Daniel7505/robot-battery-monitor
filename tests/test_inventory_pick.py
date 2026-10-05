"""Pure inventory / trolley / one-SKU pretend-pick logic (no Webots)."""
from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

from src.inventory_pick import (  # noqa: E402
    DEFAULT_PICK_DWELL_S,
    DEFAULT_PICK_RADIUS_M,
    ST_DONE,
    ST_PICKING,
    ST_TO_PICK,
    ST_TO_SHIP,
    TrolleyGeom,
    build_inventory_groups,
    demo_group_specs,
    demo_sku,
    find_group,
    inventory_to_json,
    apply_pick_speed_cap,
    make_pick_state,
    sku_id,
)
from src.track_geometry import TrackNetwork, _poly_dist  # noqa: E402
import warehouse_builder as wb  # noqa: E402


@pytest.fixture(scope="module")
def lay():
    return wb.Layout(wb.WarehouseSpec())


def test_sku_id_stable():
    assert sku_id(5, 0, "left") == "SKU-A05-B0-L"
    assert sku_id(7, 1, "right") == "SKU-A07-B1-R"


def test_demo_group_is_on_odd_aisle(lay):
    specs = demo_group_specs(lay)
    demo = next(s for s in specs if s.get("demo"))
    assert (demo["aisle"] - lay.sp.main_aisle) % 2 == 1
    assert demo["aisle"] != lay.ship_aisle
    assert len(specs) >= 1


def test_inventory_groups_derived_from_layout(lay):
    groups = build_inventory_groups(lay)
    assert groups
    demo = next(g for g in groups if g.meta.get("demo"))
    assert demo.sku == sku_id(demo.aisle, demo.block, demo.side)
    assert demo.n_cubes == lay.sp.inv_cubes_per_group
    assert demo.approach_xy[1] == pytest.approx(lay.aisle_y[demo.aisle])
    # cubes sit on the rack face, not in the aisle corridor
    face = lay.aisle_y[demo.aisle] + lay.sp.aisle_w_m / 2  # left face
    for c in demo.cubes:
        assert c.cy > face  # inset into the rack (+y)
        assert c.cz > 0.5  # on the first beam shelf, not the floor


def test_inventory_cubes_clear_the_lane(lay):
    net = TrackNetwork.from_dict(lay.track_dict(), "warehouse")
    for g in build_inventory_groups(lay):
        for c in g.cubes:
            for road in net.roads:
                w = max(road.spec.widths) / 2
                assert _poly_dist(road.centerline, c.cx, c.cy) > w + 0.5, (c.def_name, road.spec.name)


def test_inventory_json_roundtrip(lay):
    groups = build_inventory_groups(lay)
    rows = inventory_to_json(groups)
    assert demo_sku(groups) == next(r["sku"] for r in rows if r["demo"])
    assert find_group(groups, rows[0]["sku"]).sku == rows[0]["sku"]
    assert find_group(groups, "no-such") is None


def test_trolley_hitch_behind_robot():
    tg = TrolleyGeom()
    x, y, yaw = tg.world_pose((0.0, 0.0), 0.0)
    assert x == pytest.approx(-tg.hitch_behind_m)
    assert y == pytest.approx(0.0) and yaw == pytest.approx(0.0)
    # facing +y (π/2): trolley is at −y
    x2, y2, yaw2 = tg.world_pose((10.0, 5.0), math.pi / 2)
    assert x2 == pytest.approx(10.0)
    assert y2 == pytest.approx(5.0 - tg.hitch_behind_m)
    assert yaw2 == pytest.approx(math.pi / 2)
    assert tg.hitch_behind_m == pytest.approx(
        tg.chassis_length_m / 2 + tg.hitch_clear_m + tg.tongue_m + tg.length_m / 2
    )


def test_pick_state_machine_one_sku(lay):
    groups = build_inventory_groups(lay)
    demo = next(g for g in groups if g.meta.get("demo"))
    ps = make_pick_state(demo, pick_radius_m=1.0, pick_dwell_s=1.0)
    ax, ay = demo.approach_xy
    # far away: still driving
    cmd = ps.update((0.0, 0.0), 0.1)
    assert cmd.state == ST_TO_PICK and cmd.v_cap is None and not cmd.pick_now
    # arrive
    cmd = ps.update((ax, ay), 0.1)
    assert cmd.state == ST_PICKING and cmd.v_cap == 0.0
    # mid-dwell fires pick once
    fired = 0
    for _ in range(20):
        cmd = ps.update((ax, ay), 0.1)
        if cmd.pick_now:
            fired += 1
        if cmd.state == ST_TO_SHIP:
            break
    assert fired == 1 and ps.picked and cmd.state == ST_TO_SHIP and cmd.v_cap is None
    ps.mark_shipped()
    assert ps.state == ST_DONE


def test_pick_dwell_defaults():
    assert DEFAULT_PICK_RADIUS_M > 0.5
    assert DEFAULT_PICK_DWELL_S >= 1.0


def test_builder_emits_inventory_and_pick_mission():
    res = wb.build_all(write=False)
    inv = res["track"]["inventory"]
    assert inv and any(r["demo"] for r in inv)
    names = {m["name"] for m in res["track"]["missions"]}
    assert "pick_one_sku_to_shipping" in names
    pick = next(m for m in res["track"]["missions"] if m["name"] == "pick_one_sku_to_shipping")
    odd = next(m for m in res["track"]["missions"] if m["name"] == "odd_aisle_to_shipping")
    # same spine as odd_aisle so one RBM_ROUTE already reaches the demo SKU
    assert pick["route"] == odd["route"] and pick["to"] == "shipping_1"
    demo = next(r for r in inv if r["demo"])
    assert demo["approach_xy"][1] == pytest.approx(res["layout"].aisle_y[demo["aisle"]])


def test_controller_resolve_sku(monkeypatch):
    import importlib.util

    mod_path = ROOT / "webots" / "controllers" / "butlerbot_controller" / "controller_inventory.py"
    spec = importlib.util.spec_from_file_location("controller_inventory", mod_path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    catalog = [
        {"sku": "SKU-A05-B0-L", "demo": True},
        {"sku": "SKU-A03-B0-L", "demo": False},
    ]
    sku, why = mod.resolve_sku("demo", catalog)
    assert sku == "SKU-A05-B0-L" and "demo" in why
    sku, why = mod.resolve_sku("SKU-A03-B0-L", catalog)
    assert sku == "SKU-A03-B0-L"
    sku, why = mod.resolve_sku("nope", catalog)
    assert sku is None and "not in inventory" in why
    assert mod.pick_sku_from_env({"RBM_PICK_SKU": ""}) is None
    assert mod.pick_sku_from_env({"RBM_PICK_SKU": "demo"}) == "demo"



def test_apply_pick_speed_cap_releases_sticky_zero():
    """Bug Dan hit: pick set obstacle_v_cap=0 and never cleared → v*=0 forever."""
    cap, holding, saved = apply_pick_speed_cap(0.0, None, holding=False, saved_cap=None)
    assert cap == 0.0 and holding and saved is None
    # resume (cmd v_cap None) must restore, not leave 0 sticky
    cap2, holding2, saved2 = apply_pick_speed_cap(None, cap, holding=holding, saved_cap=saved)
    assert cap2 is None and not holding2 and saved2 is None


def test_apply_pick_speed_cap_restores_prior_obstacle_cap():
    cap, holding, saved = apply_pick_speed_cap(0.0, 0.20, holding=False, saved_cap=None)
    assert cap == 0.0 and holding and saved == 0.20
    cap2, holding2, saved2 = apply_pick_speed_cap(None, cap, holding=holding, saved_cap=saved)
    assert cap2 == 0.20 and not holding2


def test_pick_state_resume_clears_v_cap(lay):
    groups = __import__("src.inventory_pick", fromlist=["build_inventory_groups"]).build_inventory_groups(lay)
    demo = next(g for g in groups if g.meta.get("demo"))
    ps = make_pick_state(demo, pick_radius_m=1.0, pick_dwell_s=0.5)
    ax, ay = demo.approach_xy
    cmd = ps.update((ax, ay), 0.1)
    assert cmd.state == "PICKING" and cmd.v_cap == 0.0
    # hold through dwell
    for _ in range(20):
        cmd = ps.update((ax, ay), 0.1)
        if cmd.state == "DRIVING_TO_SHIP":
            break
    assert cmd.state == "DRIVING_TO_SHIP" and cmd.v_cap is None
    # glue must release
    cap, holding, saved = apply_pick_speed_cap(0.0, None, holding=False, saved_cap=None)
    cap, holding, saved = apply_pick_speed_cap(cmd.v_cap, cap, holding=holding, saved_cap=saved)
    assert cap is None and not holding
