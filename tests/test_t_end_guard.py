"""Dan's t_end report (2026-10-02): a T-end must never be driven straight through,
and a run must not silently use a track / world left over from an earlier run."""
import importlib.util
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
CTRL = ROOT / "webots" / "controllers" / "butlerbot_controller"

from src.junction_vision import Junction  # noqa: E402
from src.rowfit_control import RowfitController  # noqa: E402
from src.track_geometry import referee_for, track_key  # noqa: E402

_spec = importlib.util.spec_from_file_location("rowfit_sim", ROOT / "scripts" / "rowfit_sim.py")
sim = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sim)
T_END_WBT = str(ROOT / "webots" / "worlds" / "butlerbot_t_end.wbt")


def _decide(kind, straight, route):
    c = RowfitController()
    c.set_route(route)
    c.junction = Junction(kind=kind, near_m=1.0, center_m=1.65, far_m=None, left=True,
                          straight=straight, right=True, confidence=0.9, seen=True)
    c.jn_near_m, c.jn_center_m, c.jn_count = 1.0, 1.65, 1
    c._junction_maybe_decide(None, 0.0)
    return c


def test_t_end_route_s_falls_back_to_left_never_straight():
    for straight in (False, None, True):  # even if the classifier's S flag is wrong
        c = _decide("t_end", straight, ["S"])
        assert c.route_choice == "L" and c.state == "CORNER_APPROACH", (straight, c.state)
        assert "not offered" in c.junctions[-1]["note"]
    assert _decide("t_end", None, ["R"]).route_choice == "R"
    assert _decide("t_end", None, []).route_choice == "L"  # default: straight else left
    assert _decide("plus", True, ["S"]).state == "GAP_CROSS"  # a real crossing still goes straight


def test_t_end_closed_loop_with_route_s_turns_left():
    r = sim.run("t_end", route="S", max_s=45.0)
    assert r["ok"] and r["exit"] == "north", (r["exit"], r["junctions"])
    j = r["junctions"][0]
    assert j["kind"] == "t_end" and j["choice"] == "L" and "not offered" in j["note"]


def test_leftover_rbm_track_loses_to_the_loaded_world():
    assert track_key("tracks/t_end.json") == track_key("t_end") == track_key(r"C:\r\tracks\t_end.json")
    r = referee_for(T_END_WBT, env={"RBM_TRACK": "plus"})
    assert r.name == "t_end" and r.warning and "RBM_TRACK=plus" in r.warning and "set RBM_TRACK=" in r.warning
    r = referee_for(T_END_WBT, env={"RBM_TRACK": "t_end"})
    assert r.name == "t_end" and r.warning is None and r.source == "RBM_TRACK"
    r = referee_for(None, env={"RBM_TRACK": "plus"})  # no world tag: RBM_TRACK is all we have
    assert r.name == "plus" and r.warning is None


def test_controller_warns_and_logs_world_track_route(tmp_path):
    env = dict(os.environ)
    env.pop("RBM_LANE_MODE", None)
    env.update(
        PYTHONPATH=str(ROOT / "tests" / "fake_webots"),
        RBM_LOG_DIR=str(tmp_path),
        TWIN_DASHBOARD_URL="http://127.0.0.1:9",
        FAKE_WEBOTS_STEPS="60",
        FAKE_WEBOTS_WORLD=T_END_WBT,
        RBM_TRACK="plus",
        RBM_ROUTE="S",
        RBM_WORLD_FILE=str(ROOT / "webots" / "worlds" / "butlerbot_plus.wbt"),
    )
    out = subprocess.run([sys.executable, "butlerbot_controller.py"], cwd=CTRL, env=env,
                         capture_output=True, text=True, timeout=120)
    text = out.stdout + out.stderr
    assert "FINISH REFEREE track t_end" in text, text
    assert "WARNING RBM_TRACK=plus does not match the loaded world butlerbot_t_end.wbt" in text
    assert "WARNING Webots loaded butlerbot_t_end.wbt but the launcher asked for butlerbot_plus.wbt" in text
    import csv

    rows = list(csv.DictReader((tmp_path / "lane-vision-runs.csv").open()))
    assert len(rows) == 1
    m = rows[0]
    assert m["world"] == "butlerbot_t_end.wbt" and m["track"] == "t_end" and m["route"] == "S"
    assert "RBM_TRACK=plus" in m["warnings"]
    lv = list(csv.DictReader((tmp_path / "lane-vision.csv").open()))
    assert lv and lv[0]["run_id"] == m["run_id"]
