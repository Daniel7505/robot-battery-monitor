from src.power_requirements import PowerRequirements
from src.lru_monitor import LRUMonitor

_CHANNELS = [
    {"id": "Legs", "max_draw_w": 35, "nominal_voltage": 48},
    {"id": "Arms", "max_draw_w": 25, "nominal_voltage": 48},
    {"id": "Torso", "max_draw_w": 20, "nominal_voltage": 48},
    {"id": "Compute", "max_draw_w": 15, "nominal_voltage": 24},
]


def test_startup_cost_reduces_battery():
    req = PowerRequirements(_CHANNELS, system_budget_w=72)
    pct, info = req.apply_startup(92.0, 1000)
    assert info["applied"] is True
    assert pct < 92.0
    pct2, info2 = req.apply_startup(pct, 1000)
    assert info2["applied"] is False


def test_task_lru_requirements_compliance():
    lru = LRUMonitor(_CHANNELS, system_budget_w=72)
    lru_result = lru.evaluate(
        {"Legs": 20, "Arms": 10, "Torso": 8, "Compute": 9},
        {},
    )
    req = PowerRequirements(_CHANNELS, system_budget_w=72)
    result = req.evaluate("moving", lru_result["lrus"], total_draw_w=47, task_budget_w=66)
    assert result["task"] == "moving"
    assert result["eps"]["id"] == "eps"
    assert len(result["lru_requirements"]) >= 4


def test_eps_violation_on_over_budget():
    lru = LRUMonitor(_CHANNELS, system_budget_w=72)
    lru_result = lru.evaluate(
        {"Legs": 30, "Arms": 22, "Torso": 18, "Compute": 12},
        {},
    )
    req = PowerRequirements(_CHANNELS, system_budget_w=72)
    result = req.evaluate("high_load", lru_result["lrus"], total_draw_w=82, task_budget_w=72)
    assert not result["overall_compliant"]
    assert result["violations"]