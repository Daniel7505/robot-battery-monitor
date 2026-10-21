"""Mission energy model uses ctrl_bom_build when config bom_build is default."""

from __future__ import annotations

from src.mission_energy_log import MissionEnergyLog, resolve_energy_bom_build, _drive_breakdown


def test_resolve_energy_bom_build_prefers_controller_stamp():
    assert resolve_energy_bom_build("light", "default") == "light"
    assert resolve_energy_bom_build(None, "heavy") == "heavy"


def test_segment_model_nonzero_when_ctrl_bom_build_light():
    terms = _drive_breakdown(
        3.048,
        0.44,
        5.0,
        0.0,
        0.0,
        bom_build="light",
    )
    assert terms["wh_roll"] > 0.0
    assert terms["wh_visc"] > 0.0


def test_summary_line_includes_model_when_ctrl_bom_build_set(monkeypatch):
    log = MissionEnergyLog()
    log.reset()
    log._armed = True
    log._build = "default"
    log._ctrl_bom_build = "light"
    log._mission_dist = 200.0
    log._mission_dt = 600.0
    log._payload_kg = 0.0
    log._mission_wh_elec = 3.5
    log._mission_elec_known = True
    log._mission_wh_shaft = 1.0
    log._mission_shaft_known = True
    line = log._summary_line()
    assert "wh_model_drive=" in line
    assert "err_pct=" in line
    assert "err_pct=na" not in line.split("err_pct=")[1][:8]
