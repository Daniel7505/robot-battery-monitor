"""rowfit_sim on waypoint tracks (tracks/*.json): the phase-1 baseline runs."""
import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location("rowfit_sim", ROOT / "scripts" / "rowfit_sim.py")
sim = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sim)


def test_widen_track_is_followed_and_width_tracked():
    r = sim.run("widen")
    assert r["finished"] and r["end"]["s_m"] >= 11.0
    assert r["max_ct_m"] < 0.05 and r["left_lane_at"] is None
    assert r["lane_width_worst"]["err_m"] < 0.08  # measured width follows 1.30 -> 1.60 -> 1.30


def test_corner90_baseline_runs_and_reports_where_it_loses_the_lane():
    """Current rowfit has no corner logic: it is NOT expected to finish yet.
    This only checks the sim plumbing and that the bot never leaves the lane
    (it brakes instead)."""
    r = sim.run("corner90", max_s=30.0)
    assert r["track_length_m"] == 8.0
    assert r["left_lane_at"] is None
    if not r["finished"]:
        assert r["braked_at"] is not None or r["first_held_at"] is not None
