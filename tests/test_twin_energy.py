from pathlib import Path

import pytest

from src.design_agent import Mission, propose
from src.hardware_profile import clear_profile_cache
from src.twin_energy import (
    TEN_FT_M,
    brushed_drive_point,
    estimate_mission_energy,
    format_mission_energy_line,
    paper_energy,
    shift_energy_wh,
    straight_wh_per_m,
)


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


def test_payload_is_ignored_until_a_bom_build_is_selected():
    # No bom_build: the V1 straight figure, even if a payload is passed in.
    assert straight_wh_per_m(0.44, 10) == pytest.approx(straight_wh_per_m(), rel=1e-9)


def test_brushed_model_matches_the_light_table_and_the_shift_total():
    # Published light build at 0.44 m/s: drive ~3.43 W, 0.0066 Wh/10 ft.
    point = brushed_drive_point(
        mass_kg=6.142,
        wheel_radius_m=0.04,
        speed_m_s=0.44,
        voltage_v=24,
        no_load_rpm=200,
        no_load_current_a=0.1,
        stall_current_a=3.0,
        stall_torque_nm=2.256,
    )
    assert point["torque_nm"] == pytest.approx(0.0181, abs=0.0002)
    assert point["duty"] == pytest.approx(0.533, abs=0.01)
    assert point["current_a"] == pytest.approx(0.123, abs=0.005)
    assert point["drive_w"] == pytest.approx(3.43, abs=0.05)
    assert point["drive_wh_per_10ft"] == pytest.approx(0.0066, abs=0.00015)
    assert point["extra_wh_per_10ft_per_kg_payload"] == pytest.approx(0.000219, abs=2e-5)
    energy = shift_energy_wh(
        idle_w=9.96,
        shift_h=6,
        distance_m=3000,
        drive_wh_per_10ft=point["drive_wh_per_10ft"],
        extra_wh_per_10ft_per_kg=point["extra_wh_per_10ft_per_kg_payload"],
        payload_kg=10,
    )
    # idle 9.96×6 plus 3 km of drive and 10 kg payload, rounded to 0.01 Wh.
    assert energy == 68.41


def test_shift_energy_rounds_to_the_hundredth_before_a_compare():
    raw = 9.96 * 6 + (3000 / TEN_FT_M) * (0.006597 + 10 * 0.000219)
    assert raw != round(raw, 2)
    assert shift_energy_wh(
        idle_w=9.96,
        shift_h=6,
        distance_m=3000,
        drive_wh_per_10ft=0.006597,
        extra_wh_per_10ft_per_kg=0.000219,
        payload_kg=10,
    ) == round(raw, 2)


def _use_bom(build):
    from src.config import config

    old = config._config.get("bom_build", None)
    config._config["bom_build"] = build
    clear_profile_cache()
    return old


def _restore_bom(old):
    from src.config import config

    if old is None:
        config._config.pop("bom_build", None)
    else:
        config._config["bom_build"] = old
    clear_profile_cache()


def test_bom_wh_per_m_grows_with_payload():
    old = _use_bom("light")
    try:
        empty = straight_wh_per_m(0.44, 0)
        ten = straight_wh_per_m(0.44, 10)
        twenty = straight_wh_per_m(0.44, 20)
        assert empty < ten < twenty
        # Table row, not the V1 0.025 figure and not the 1.35 turn factor.
        assert empty == pytest.approx(0.02576 / TEN_FT_M, rel=1e-4)
        assert ten == pytest.approx(empty + 10 * (0.000219 / TEN_FT_M), rel=1e-3)
        one_ten_ft = estimate_mission_energy(TEN_FT_M, track="s", speed_m_s=0.44, payload_kg=0)
        assert one_ten_ft["source"] == "bom_light"
        assert one_ten_ft["energy_wh"] == pytest.approx(0.02576, abs=0.001)
    finally:
        _restore_bom(old)


def test_measured_override_beats_the_bom_table(tmp_path: Path):
    old = _use_bom("light")
    try:
        path = tmp_path / "twin_energy.json"
        path.write_text('{"wh_per_m": 0.1, "source": "measure_track_run"}', encoding="utf-8")
        out = estimate_mission_energy(10.0, track="s", override_path=path, payload_kg=10)
        assert out["source"] == "measure_track_run"
        assert out["energy_wh"] == 1.0
        # The model line ignores the file so measured Wh can sit beside it.
        line = format_mission_energy_line(10.0, payload_kg=0, wh_measured=1.0)
        assert line.startswith("MISSION_ENERGY build=light ")
        assert "wh_measured=1.00" in line
        assert "wh_model=" in line
        assert "wh_model=1.0" not in line
    finally:
        _restore_bom(old)


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
