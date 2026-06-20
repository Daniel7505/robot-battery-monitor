from src.energy_predictor import EnergyPredictor
from src.mission_tasks import anticipated_phases
from src.power_allocator import PowerAllocator


def test_predictor_builds_confidence_with_history():
    pred = EnergyPredictor()
    for w in [30, 31, 30.5, 29.8, 30.2]:
        pred.update(w)
    result = pred.forecast(
        battery_pct=90,
        capacity_wh=1000,
        task_id="idle",
        task_remaining_s=60,
        blend_progress=1.0,
        current_draw_w=30,
    )
    assert result["confidence_pct"] >= 50
    assert result["predicted_draw_w"] > 0
    assert result["mission_energy_ok"] is not None


def test_horizon_forecast_30_and_60_seconds():
    pred = EnergyPredictor()
    for w in [28, 29, 30, 31, 32]:
        pred.update(w)
    result = pred.forecast(
        battery_pct=85,
        capacity_wh=1000,
        task_id="moving",
        task_remaining_s=45,
        blend_progress=0.8,
        current_draw_w=32,
    )
    assert result["forecast_30s"]["t_s"] == 30
    assert result["forecast_60s"]["t_s"] == 60
    assert result["forecast_30s"]["draw_low_w"] <= result["forecast_30s"]["draw_w"]
    assert result["forecast_60s"]["draw_high_w"] >= result["forecast_60s"]["draw_w"]
    assert len(result["horizon_points"]) == 6
    assert result["risk_level"] in ("low", "medium", "high", "critical")


def test_improved_runtime_with_confidence_interval():
    pred = EnergyPredictor()
    for w in [40, 41, 42, 43, 44]:
        pred.update(w)
    result = pred.forecast(
        battery_pct=70,
        capacity_wh=1000,
        task_id="high_load",
        task_remaining_s=30,
        blend_progress=1.0,
        current_draw_w=44,
    )
    assert result["runtime_min_blended"] > 0
    assert result["runtime_min_low"] is not None
    assert result["runtime_min_high"] is not None
    assert result["runtime_min_high"] >= result["runtime_min_low"]
    assert result["battery_pct_at_60s"] < 70


def test_locomotion_outlook_anticipates_transition():
    pred = EnergyPredictor()
    pred.update(28)
    result = pred.forecast(
        battery_pct=80,
        capacity_wh=1000,
        task_id="idle",
        task_remaining_s=18,
        blend_progress=1.0,
        current_draw_w=28,
    )
    outlook = result["locomotion_outlook"]
    assert outlook["outlook"]
    assert outlook["transition_in_s"] == 18
    assert result["anticipated_phases"]


def test_anticipated_phases_returns_probabilities():
    phases = anticipated_phases("idle")
    assert phases
    assert sum(p["probability_pct"] for p in phases) > 0
    assert all("expected_draw_w" in p for p in phases)


def test_dynamic_budget_tightens_on_high_predicted_load():
    channels = [{"id": "Legs", "max_draw_w": 35}, {"id": "Compute", "max_draw_w": 15}]
    alloc = PowerAllocator(channels, system_budget_w=72)
    prediction = {
        "predicted_draw_w": 80,
        "confidence_pct": 85,
        "mission_energy_ok": True,
    }
    result = alloc.allocate("high_load", {"Legs": 35, "Compute": 13}, prediction=prediction)
    assert result["budget_w"] < result["base_budget_w"]
    assert result["dynamic_budget_applied"] is True


def test_dynamic_budget_reduced_when_mission_energy_marginal():
    channels = [{"id": "Legs", "max_draw_w": 35}]
    alloc = PowerAllocator(channels, system_budget_w=72)
    prediction = {
        "predicted_draw_w": 40,
        "confidence_pct": 70,
        "mission_energy_ok": False,
        "mission_battery_pct_at_end": 8,
    }
    result = alloc.allocate("moving", {"Legs": 25}, prediction=prediction)
    assert result["budget_w"] < result["base_budget_w"]