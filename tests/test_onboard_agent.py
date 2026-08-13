from src.onboard_agent import AgentRecommendation, AgentResult, OnboardAgent


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


def test_prediction_risk_quiet_on_healthy_pack_comfortable_loop():
    """95% pack + comfortable loop must not raise AGENT INTERVENING."""
    agent = OnboardAgent()
    result = agent.evaluate(
        battery_pct=95,
        task_id="moving",
        allocation=_base_allocation(task="moving"),
        safety=_base_safety(),
        prediction={"risk_level": "high", "mission_energy_ok": True},
        mission={
            "task": "moving",
            "loop_forecast": {
                "ok": True,
                "margin_pct": 96.0,
                "can_complete_loop": True,
                "feasibility_status": "comfortable",
            },
        },
        readings={},
        twin_context={"phase": "teleop", "gait": "drive", "source": "webots"},
    )
    assert "prediction_risk" not in result.rules_fired
    assert not any(r.action == "throttle_system" for r in result.recommendations)
    status = agent.status_dict(result)
    assert status["intervening"] is False


def test_prediction_risk_quiet_on_healthy_pack_without_loop():
    agent = OnboardAgent()
    result = agent.evaluate(
        battery_pct=95,
        task_id="moving",
        allocation=_base_allocation(task="moving"),
        safety=_base_safety(),
        prediction={"risk_level": "high", "mission_energy_ok": True},
        mission={"task": "moving"},
        readings={},
        twin_context={"phase": "teleop", "gait": "drive", "source": "webots"},
    )
    assert "prediction_risk" not in result.rules_fired
    assert agent.status_dict(result)["intervening"] is False


def test_prediction_risk_still_fires_when_pack_is_thin():
    agent = OnboardAgent()
    result = agent.evaluate(
        battery_pct=18,
        task_id="moving",
        allocation=_base_allocation(task="moving"),
        safety=_base_safety(status="warning"),
        prediction={"risk_level": "high", "mission_energy_ok": True},
        mission={"task": "moving"},
        readings={},
        twin_context={"phase": "teleop", "gait": "drive", "source": "webots"},
    )
    assert "prediction_risk" in result.rules_fired
    assert any(r.action == "safety_alert" for r in result.recommendations)


def test_prediction_risk_fires_when_loop_cannot_finish():
    agent = OnboardAgent()
    result = agent.evaluate(
        battery_pct=80,
        task_id="moving",
        allocation=_base_allocation(task="moving"),
        safety=_base_safety(),
        prediction={"risk_level": "high", "mission_energy_ok": True},
        mission={
            "task": "moving",
            "loop_forecast": {
                "ok": True,
                "margin_pct": 6.0,
                "can_complete_loop": False,
                "loop_wh_remaining": 0.45,
                "energy_wh_remaining": 0.12,
                "finish_battery_pct": 2.0,
                "feasibility_status": "insufficient",
            },
        },
        readings={},
        twin_context={"phase": "drive_transit", "gait": "drive", "source": "webots"},
    )
    assert "prediction_risk" in result.rules_fired


def test_intervening_ignores_medium_applied_throttle():
    rec = AgentRecommendation(
        action="throttle_system",
        priority="medium",
        reason="gentle trim",
        factor=0.90,
        applied=True,
    )
    result = AgentResult(
        posture="advisory",
        summary="gentle trim",
        recommendations=[rec],
        applied_actions=["system ×90%"],
    )
    status = result.to_status(True, [])
    assert status["intervening"] is False
    assert status["controlling"] is True


def test_high_utilization_quiet_on_healthy_teleop():
    agent = OnboardAgent()
    result = agent.evaluate(
        battery_pct=94,
        task_id="moving",
        allocation=_base_allocation(task="moving", utilization_pct=92, total_allocated_w=50.0),
        safety=_base_safety(),
        prediction={"risk_level": "low", "mission_energy_ok": True},
        mission={"task": "moving"},
        readings={},
        twin_context={"phase": "teleop", "gait": "drive", "source": "webots"},
    )
    assert "high_utilization" not in result.rules_fired
    assert agent.status_dict(result)["intervening"] is False


def test_high_utilization_still_fires_without_teleop_phase():
    agent = OnboardAgent()
    result = agent.evaluate(
        battery_pct=94,
        task_id="moving",
        allocation=_base_allocation(task="moving", utilization_pct=92, total_allocated_w=50.0),
        safety=_base_safety(),
        prediction={"risk_level": "low", "mission_energy_ok": True},
        mission={"task": "moving"},
        readings={},
    )
    assert "high_utilization" in result.rules_fired


def test_lane_keep_rule_logs_when_armed():
    agent = OnboardAgent()
    result = agent.evaluate(
        battery_pct=94,
        task_id="moving",
        allocation=_base_allocation(task="moving"),
        safety=_base_safety(),
        prediction={"risk_level": "low", "mission_energy_ok": True},
        mission={"task": "moving"},
        readings={},
        twin_context={
            "phase": "teleop",
            "gait": "drive",
            "source": "webots",
            "lane_keep": True,
            "lane_sensors": {"left_yellow": 0.6, "right_yellow": 0.0, "finish_red": 0.0},
        },
    )
    assert "lane_keep" in result.rules_fired
    assert agent.status_dict(result)["intervening"] is False


def test_lane_keep_red_is_not_silent():
    agent = OnboardAgent()
    result = agent.evaluate(
        battery_pct=94,
        task_id="moving",
        allocation=_base_allocation(task="moving"),
        safety=_base_safety(),
        prediction={"risk_level": "low", "mission_energy_ok": True},
        mission={"task": "moving"},
        readings={},
        twin_context={
            "phase": "teleop",
            "source": "webots",
            "lane_keep": True,
            "lane_sensors": {"left_yellow": 0.0, "right_yellow": 0.0, "finish_red": 0.8},
        },
    )
    assert any("red" in (r.message or "").lower() for r in result.recommendations)


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