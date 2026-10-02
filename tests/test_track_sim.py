"""rowfit_sim on waypoint tracks (tracks/*.json): lane keep, corners, pivots."""
import importlib.util
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location("rowfit_sim", ROOT / "scripts" / "rowfit_sim.py")
sim = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sim)


def test_widen_track_is_followed_and_width_tracked():
    r = sim.run("widen")
    assert r["finished"] and r["end"]["s_m"] >= 11.0
    assert r["max_ct_m"] < 0.05 and r["left_lane_at"] is None
    assert r["lane_width_worst"]["err_m"] < 0.08  # measured width follows 1.30 -> 1.60 -> 1.30
    assert r["pivots"] == [] and r["corner_first_seen"] is None  # a widening lane is not a corner


@pytest.mark.parametrize("track,turn", [("corner90", 90.0), ("corner90_right", -90.0), ("corner90_wide", 90.0)])
def test_sharp_corner_pivot_and_finish(track, turn):
    r = sim.run(track, max_s=40.0)
    assert r["finished"] and r["end"]["s_m"] >= 7.9 and r["braked_at"] is None
    assert r["left_lane_at"] is None and r["max_lateral_m"] < 0.06
    assert r["corner_first_seen"]["dir"] == ("left" if turn > 0 else "right")
    assert len(r["pivots"]) == 1
    p = r["pivots"][0]
    assert abs(p["true_err_deg"]) < 3.0 and p["time_s"] < 4.0
    assert abs(p["lateral_m"]) < 0.06  # pivoted on the corner's centre line
    assert abs(r["end_yaw_deg"] - turn) < 3.0
    ev = r["events"]
    assert [e.split()[0] for e in ev][:4] == ["CORNER", "PIVOT", "PIVOT", "REACQUIRE"]
    # the paint ends 0.5 m past the finish: that may show up as a lane_lost junction ahead, nothing else
    assert not [e for e in ev[4:] if e.startswith("JUNCTION") and "lane_lost" not in e], ev


def test_corner_mix_rounded_left_then_sharp_right():
    r = sim.run("corner_mix", max_s=50.0)
    assert r["finished"] and r["braked_at"] is None and r["left_lane_at"] is None
    assert len(r["pivots"]) == 1 and r["pivots"][0]["dir"] == -1  # only the sharp one
    assert r["pivots"][0]["start"]["y"] > 3.5  # ... at (4, 4), not at the rounded corner
    assert r["max_lateral_m"] < 0.16  # the rounded corner, unchanged from before


def test_pivot_with_tyre_scrub_and_imu_drift():
    r = sim.run("corner90", max_s=40.0, turn_eff=0.8, imu_noise_deg=0.3, imu_drift_dps=0.5)
    assert r["finished"] and len(r["pivots"]) == 1 and abs(r["pivots"][0]["true_err_deg"]) < 4.0


def test_corner90_without_imu_stops_at_the_corner():
    """No yaw to the controller: approach, then brake on the pivot point (inside the lane)."""
    r = sim.run("corner90", max_s=30.0, corners=False)
    assert not r["finished"] and r["pivots"] == []
    assert r["braked_at"]["state"] == "CORNER_APPROACH"
    assert 3.9 <= r["braked_at"]["x"] <= 4.1 and r["left_lane_at"] is None
