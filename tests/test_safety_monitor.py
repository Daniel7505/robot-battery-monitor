from src.safety_monitor import SafetyMonitor

_CHANNELS = [
    {"id": "Legs", "max_draw_w": 35, "nominal_voltage": 48},
    {"id": "Arms", "max_draw_w": 25, "nominal_voltage": 48},
    {"id": "Torso", "max_draw_w": 20, "nominal_voltage": 48},
    {"id": "Compute", "max_draw_w": 15, "nominal_voltage": 24},
]


def _allocation(budget=72, utilization=50):
    return {
        "budget_w": budget,
        "utilization_pct": utilization,
        "status": "ok",
    }


def test_low_battery_warning():
    mon = SafetyMonitor(_CHANNELS, system_budget_w=72)
    result = mon.evaluate(
        battery_pct=18,
        requested={"Legs": 5, "Arms": 7, "Torso": 5, "Compute": 9},
        allocated={"Legs": 5, "Arms": 7, "Torso": 5, "Compute": 9},
        allocation=_allocation(),
        task_id="idle",
    )
    assert result["status"] in ("warning", "fault")
    assert any("Low battery" in w for w in result["warnings"])
    assert result["alerts"]


def test_critical_battery_fault_and_throttle():
    mon = SafetyMonitor(_CHANNELS, system_budget_w=72)
    result = mon.evaluate(
        battery_pct=8,
        requested={"Legs": 10, "Arms": 8, "Torso": 6, "Compute": 9},
        allocated={"Legs": 10, "Arms": 8, "Torso": 6, "Compute": 9},
        allocation=_allocation(),
    )
    assert result["status"] == "fault"
    assert result["throttle_required"]
    assert result["throttle_factor"] <= 0.70


def test_power_spike_detection():
    mon = SafetyMonitor(_CHANNELS, system_budget_w=72)
    mon.evaluate(
        battery_pct=80,
        requested={"Legs": 10},
        allocated={"Legs": 10},
        allocation=_allocation(),
    )
    result = mon.evaluate(
        battery_pct=80,
        requested={"Legs": 20},
        allocated={"Legs": 20},
        allocation=_allocation(),
    )
    assert "Legs" in result["spike_channels"]
    assert result["throttle_required"]


def test_over_draw_fault():
    mon = SafetyMonitor(_CHANNELS, system_budget_w=72)
    result = mon.evaluate(
        battery_pct=80,
        requested={"Legs": 40},
        allocated={"Legs": 40},
        allocation=_allocation(),
    )
    assert result["status"] == "fault"
    assert "Legs" in result["over_draw_channels"]


def test_thermal_rises_with_draw():
    mon = SafetyMonitor(_CHANNELS, system_budget_w=72)
    low = mon.evaluate(
        battery_pct=80,
        requested={"Legs": 5, "Arms": 5, "Torso": 5, "Compute": 5},
        allocated={"Legs": 5, "Arms": 5, "Torso": 5, "Compute": 5},
        allocation=_allocation(),
        tick_seconds=3,
    )
    for _ in range(8):
        high = mon.evaluate(
            battery_pct=80,
            requested={"Legs": 30, "Arms": 22, "Torso": 18, "Compute": 12},
            allocated={"Legs": 30, "Arms": 22, "Torso": 18, "Compute": 12},
            allocation=_allocation(utilization=95),
            tick_seconds=3,
        )
    assert high["thermal_c"] > low["thermal_c"]


def test_safety_reports_lru_groups():
    mon = SafetyMonitor(_CHANNELS, system_budget_w=72)
    result = mon.evaluate(
        battery_pct=80,
        requested={"Legs": 8, "Arms": 6, "Torso": 5, "Compute": 6},
        allocated={"Legs": 8, "Arms": 6, "Torso": 5, "Compute": 6},
        allocation=_allocation(),
    )
    assert result["degradation_level"] in ("normal", "caution")
    assert len(result["lru"]["lrus"]) >= 4
    assert "requirements" in result


def test_apply_throttle_reduces_draw():
    mon = SafetyMonitor(_CHANNELS, system_budget_w=72)
    allocated = {"Legs": 20, "Arms": 15, "Torso": 10, "Compute": 10}
    safety = {"throttle_required": True, "throttle_factor": 0.85, "spike_channels": ["Legs"]}
    throttled = mon.apply_throttle(allocated, safety)
    assert sum(throttled.values()) < sum(allocated.values())
    assert throttled["Legs"] < allocated["Legs"]