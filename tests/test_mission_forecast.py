from src.mission_forecast import forecast_twin_loop


def test_forecast_full_loop_comfortable_battery():
    fc = forecast_twin_loop(
        battery_pct=85.0,
        capacity_wh=480,
        current_phase="standby",
    )
    assert fc["ok"] is True
    assert fc["can_complete_loop"] is True
    assert fc["loop_wh_remaining"] > 0
    assert fc["margin_pct"] > 10


def test_forecast_insufficient_battery():
    fc = forecast_twin_loop(
        battery_pct=0.08,
        capacity_wh=480,
        current_phase="standby",
    )
    assert fc["ok"] is True
    assert fc["can_complete_loop"] is False
    assert fc["feasibility_status"] == "insufficient"


def test_forecast_from_manipulate_phase():
    fc = forecast_twin_loop(
        battery_pct=40.0,
        capacity_wh=480,
        current_phase="manipulate",
    )
    assert fc["ok"] is True
    assert len(fc["phase_breakdown"]) >= 1
    assert fc["phase_breakdown"][0]["phase"] == "manipulate"