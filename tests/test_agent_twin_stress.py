from src.onboard_agent import OnboardAgent


def _alloc(**kw):
    base = {
        "task": "moving",
        "total_allocated_w": 48,
        "utilization_pct": 78,
        "status": "warning",
        "throttled_channels": [],
        "budget_w": 55,
    }
    base.update(kw)
    return base


def test_twin_stress_rule_fires_during_drive_transit():
    agent = OnboardAgent()
    result = agent.evaluate(
        battery_pct=75,
        task_id="moving",
        allocation=_alloc(),
        safety={"status": "ok", "thermal_c": 38, "thermal_status": "normal", "alerts": []},
        prediction={"risk_level": "medium", "mission_energy_ok": True},
        mission={"task": "moving"},
        readings={},
        twin_context={"phase": "drive_transit", "gait": "drive", "source": "webots"},
    )
    actions = {r.action for r in result.recommendations}
    assert "safety_alert" in actions
    assert "throttle_system" in actions
    assert any(r.channel == "Legs" for r in result.recommendations if r.action == "throttle_channel")


def test_should_auto_apply_when_twin_active():
    agent = OnboardAgent()
    agent._cfg["auto_apply_throttle"] = False
    agent._cfg["twin_auto_apply"] = True
    assert agent.should_auto_apply(twin_active=True) is True
    assert agent.should_auto_apply(twin_active=False) is False