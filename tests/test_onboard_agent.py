from src.onboard_agent import OnboardAgent, AgentRecommendation


def _base_allocation(**overrides):
    base = {
        "task": "balanced",
        "total_allocated_w": 38.0,
        "utilization_pct": 65,
        "status": "ok",
        "throttled_channels": [],
        "budget_w": 55,
    }
    base.update(overrides)
    return base


def _base_safety(**overrides):
    base = {
        "status": "ok",
        "thermal_c": 35.0,
        "thermal_status": "normal",
        "alerts": [],
        "faults": [],
        "warnings": [],
        "spike_channels": [],
        "throttle_required": False,
        "lru": {"lrus": []},
    }
    base.update(overrides)
    return base


def test_agent_nominal_no_recommendations():
    agent = OnboardAgent()
    result = agent.evaluate(
        battery_pct=85,
        task_id="idle",
        allocation=_base_allocation(),
        safety=_base_safety(),
        prediction={"risk_level": "low", "mission_energy_ok": True},
        mission={"task": "idle"},
        readings={},
    )
    assert result.posture == "normal"
    assert result.recommendations == []


def test_low_battery_suggests_task_and_throttle():
    agent = OnboardAgent()
    result = agent.evaluate(
        battery_pct=18,
        task_id="high_load",
        allocation=_base_allocation(task="high_load", utilization_pct=95),
        safety=_base_safety(status="warning", warnings=["Low battery: 18.0%"]),
        prediction={"risk_level": "medium", "mission_energy_ok": True},
        mission={"task": "high_load"},
        readings={},
    )
    actions = {r.action for r in result.recommendations}
    assert "suggest_task" in actions
    assert "throttle_system" in actions
    tasks = [r.task for r in result.recommendations if r.task]
    assert "balanced" in tasks


def test_critical_battery_posture():
    agent = OnboardAgent()
    result = agent.evaluate(
        battery_pct=9,
        task_id="moving",
        allocation=_base_allocation(),
        safety=_base_safety(status="fault", faults=["Critical battery: 9.0%"]),
        prediction={"risk_level": "critical", "mission_energy_ok": False},
        mission={"task": "moving"},
        readings={},
    )
    assert result.posture == "critical"
    assert any(r.task == "idle" for r in result.recommendations)


def test_lru_fault_throttles_channels():
    agent = OnboardAgent()
    result = agent.evaluate(
        battery_pct=70,
        task_id="balanced",
        allocation=_base_allocation(),
        safety=_base_safety(
            lru={
                "lrus": [
                    {
                        "id": "arms",
                        "label": "Arms",
                        "status": "fault",
                        "channels": ["Arms"],
                    }
                ]
            }
        ),
        prediction={"risk_level": "low", "mission_energy_ok": True},
        mission={"task": "balanced"},
        readings={},
    )
    throttles = [r for r in result.recommendations if r.action == "throttle_channel"]
    assert any(r.channel == "Arms" for r in throttles)


def test_auto_apply_throttle():
    agent = OnboardAgent()
    agent._cfg["auto_apply_throttle"] = True
    rec = AgentRecommendation(
        action="throttle_channel",
        priority="medium",
        reason="test",
        channel="Legs",
        factor=0.80,
        rule_id="test",
    )
    result = agent.evaluate(
        battery_pct=70,
        task_id="idle",
        allocation=_base_allocation(),
        safety=_base_safety(),
        prediction={"risk_level": "low", "mission_energy_ok": True},
        mission={},
        readings={},
    )
    result.recommendations.append(rec)
    out, applied = agent.apply_throttle({"Legs": 20.0, "Arms": 8.0}, result)
    assert out["Legs"] == 16.0
    assert applied


def test_status_dict_includes_log():
    agent = OnboardAgent()
    agent.evaluate(
        battery_pct=8,
        task_id="moving",
        allocation=_base_allocation(),
        safety=_base_safety(status="fault"),
        prediction={"risk_level": "critical", "mission_energy_ok": False},
        mission={},
        readings={},
    )
    status = agent.status_dict(agent.evaluate(
        battery_pct=8,
        task_id="moving",
        allocation=_base_allocation(),
        safety=_base_safety(status="fault"),
        prediction={"risk_level": "critical", "mission_energy_ok": False},
        mission={},
        readings={},
    ))
    assert status["enabled"] is True
    assert status["recommendation_count"] >= 1
    assert len(status["recent_log"]) >= 1