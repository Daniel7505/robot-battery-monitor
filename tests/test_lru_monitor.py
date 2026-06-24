from src.lru_monitor import LRUMonitor
from src.safety_monitor import SafetyMonitor

_CHANNELS = [
    {"id": "Legs", "max_draw_w": 35, "nominal_voltage": 48},
    {"id": "Arms", "max_draw_w": 25, "nominal_voltage": 48},
    {"id": "Torso", "max_draw_w": 20, "nominal_voltage": 48},
    {"id": "Compute", "max_draw_w": 15, "nominal_voltage": 24},
]


def test_lru_groups_include_eps_and_hierarchy():
    mon = LRUMonitor(_CHANNELS, system_budget_w=72)
    result = mon.evaluate(
        {"Legs": 20, "Arms": 10, "Torso": 8, "Compute": 9},
        {},
    )
    assert len(result["lrus"]) >= 5
    labels = {l["label"] for l in result["lrus"]}
    assert any("LRUA" in lb for lb in labels)
    assert any("EPS" in lb for lb in labels)
    assert result["hierarchy"]


def test_lru_over_draw_fault():
    mon = LRUMonitor(_CHANNELS)
    result = mon.evaluate({"Legs": 38}, {})
    assert result["degradation_level"] in ("degraded", "critical")
    assert "locomotion" in result["over_draw_lrus"]


def test_lru_spike_detection():
    mon = LRUMonitor(_CHANNELS)
    mon.evaluate({"Arms": 10}, {})
    result = mon.evaluate({"Arms": 20}, {})
    assert "arms" in result["spike_lrus"]


def test_lru_low_voltage_warning_under_heavy_load():
    mon = LRUMonitor(_CHANNELS)
    result = mon.evaluate({"Compute": 14.5}, {})
    compute = next(l for l in result["lrus"] if l["id"] == "compute")
    assert compute["estimated_voltage"] < compute["nominal_voltage"]
    assert compute["status"] in ("warning", "fault", "ok")


def test_lru_throttle_factors_prioritize_arms():
    mon = LRUMonitor(_CHANNELS)
    factors = mon.throttle_factors("degraded", ["arms"])
    assert factors.get("Arms", 1.0) <= factors.get("Compute", 1.0)


def test_safety_includes_lru_data():
    safety = SafetyMonitor(_CHANNELS, system_budget_w=72)
    result = safety.evaluate(
        battery_pct=80,
        requested={"Legs": 30},
        allocated={"Legs": 30},
        allocation={"budget_w": 72, "utilization_pct": 50, "status": "ok"},
    )
    assert "lru" in result
    assert len(result["lru"]["lrus"]) >= 4
    assert "degradation_level" in result