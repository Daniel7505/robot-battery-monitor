from pathlib import Path

from src.design_agent import Mission, propose
from src.twin_energy import estimate_mission_energy, paper_energy, straight_wh_per_m


def test_straight_wh_per_m_is_about_a_quarter_watt_hour():
    # 40 W / 0.44 m/s / 3600 ≈ 0.025 Wh/m
    assert 0.020 < straight_wh_per_m() < 0.030


def test_s_mission_uses_twin_not_five_minute_crawl():
    twin = estimate_mission_energy(25.0, track="s")
    paper = paper_energy(25.0, draw_w=34.0, time_s=300.0)
    assert twin["source"] == "v1_wheeled_energy_baseline"
    assert twin["energy_wh"] < paper["energy_wh"]
    assert twin["energy_wh"] < 2.0
    assert twin["energy_wh"] > 0.3


def test_override_json_wins(tmp_path: Path):
    p = tmp_path / "twin_energy.json"
    p.write_text('{"wh_per_m": 0.1, "source": "measure_track_run"}', encoding="utf-8")
    out = estimate_mission_energy(10.0, track="s", override_path=p)
    assert out["source"] == "measure_track_run"
    assert out["energy_wh"] == 1.0


def test_propose_exposes_paper_and_twin(tmp_path: Path):
    from src.parts_db import connect

    db = tmp_path / "parts.db"
    connect(db).close()
    result = propose(Mission(track_length_m=25, max_minutes=5), db_path=db)
    top = result["candidates"][0]
    t = top["totals"]
    assert t["energy_source"] == "v1_wheeled_energy_baseline"
    assert t["energy_wh_paper"] is not None
    assert t["energy_wh_est"] < t["energy_wh_paper"]
